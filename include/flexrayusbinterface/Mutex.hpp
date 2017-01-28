#pragma once
#include <functional>
#include <mutex>

template <typename T, typename MutexT = std::mutex>
class Mutex
{
public:
  class Guard : public std::reference_wrapper<T>
  {
  public:
    Guard(T& data, MutexT& mutex) : std::reference_wrapper<T>{ data }, lock{ mutex }
    {
    }

    Guard(Guard&& other) = default;
    Guard& operator=(Guard&& other) = default;
  private:

    Guard(Guard&) = delete;
    Guard& operator=(Guard&) = delete;
    
    std::unique_lock<MutexT> lock;
  };

  template<typename... Args>
  explicit Mutex(Args&&... args) : data{std::forward<Args>(args)...} {}

  auto lock() -> Guard
  {
    return { data, mtx };
  }

private:
  T data;
  mutable MutexT mtx;
};
