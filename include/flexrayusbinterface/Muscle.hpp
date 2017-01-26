#pragma once

#include <functional>
#include <future>

#include <boost/optional.hpp>

enum class Completion
{
  Completed,
  Preempted
};

template <typename T>
class Slot
{
public:
  using result_t = Completion;
  using data_t = boost::optional<std::pair<std::promise<result_t>, T>>;

  auto enqueue(T t) -> std::future<result_t>
  {
    std::lock_guard<std::mutex> _{ guard.get() };
    data_t& data = this->data;
    if (data)
      data->first.set_value(Completion::Preempted);

    data = std::make_pair(std::promise<result_t>{}, std::move(t));
    return data->first.get_future();
  }

  Slot(std::mutex& guard, data_t& data) : guard{ guard }, data{ data }
  {
  }
  Slot(Slot&&) = default;
  Slot& operator=(Slot&&) = default;

private:
  Slot(Slot const&) = delete;
  Slot& operator=(Slot const&) = delete;

  std::reference_wrapper<std::mutex> guard;
  std::reference_wrapper<data_t> data;
};

/**
 * This class represents the connection between an object `now` that is readily
 * available and another object `later` which will become available in the
 * future in the case when operations on `now` may affect the completion
 * (availability) of `later`.
 *
 * By default (when template parameter `Strict` is `true`), this class only
 * provides access to the two values once `later` becomes available. See method
 * `get` for details.
 *
 * There is a second mode of this class (when template parameter `Strict` is
 * `true` in which the two values can be decoupled and maintaining the
 * relationship between them becomes the user's responsibility. See method
 * `unsafe_get` for details.
 */
template <typename Now, typename Later, typename Strict = std::true_type>
class Entangled
{
public:
  Entangled(Now now, std::future<Later> later) : now{ std::move(now) }, later{ std::move(later) }
  {
  }

  /**
   * This method decouples the two objects making the user responsible for any
   * misuse caused by operating on `now` while `later` has not yet been
   * computed.
   *
   * This method is only available for the non-strict version of the class
   * (when template parameter `Strict` is `false`) and its name contains the
   * word `unsafe` so that the user should consider the relation between the
   * two objects closely before using it.
   */
  template <typename T = Strict>
  auto unsafe_get() && ->
      typename std::enable_if<std::is_same<T, Strict>::value && !T::value, std::pair<Now, std::future<Later>>>::type
  {
    return { std::move(now), std::move(later) };
  }

  /**
   * Blocks until the `later` object becomes available, when returns both
   * objects.
   */
  auto get() && -> std::pair<Now, Later>
  {
    return { std::move(now), std::move(later).get() };
  }

  auto status() const -> std::future_status
  {
    return later.wait_for(std::chrono::seconds{ 0 });
  }

private:
  Now now;
  std::future<Later> later;
};

/**
 *
 */
template <typename T>
class CompletionGuard
{
public:
  using completion_t = typename Slot<T>::result_t;

  template <typename... Args>
  auto enqueue(Args&&... args) && -> Entangled<CompletionGuard, completion_t>
  {
    auto fut = slot.enqueue(std::forward<Args>(args)...);
    return { CompletionGuard{ std::move(slot) }, std::move(fut) };
  }

  operator Slot<T>() &&
  {
    return std::move(slot);
  }

  CompletionGuard(Slot<T> slot) : slot{ std::move(slot) }
  {
  }

  CompletionGuard(CompletionGuard&& other) = default;
  CompletionGuard& operator=(CompletionGuard&& other) = default;

private:
  Slot<T> slot;

  CompletionGuard(CompletionGuard const&) = delete;
  CompletionGuard& operator=(CompletionGuard const&) = delete;
};
