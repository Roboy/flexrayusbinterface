#pragma once
#include <functional>

template <typename C>
class reversed_container
{
private:
  std::reference_wrapper<C> inner;

public:
  explicit reversed_container(C& c) : inner{ c }
  {
  }

  auto begin() -> decltype(this->inner.get().rbegin())
  {
    return inner.get().rbegin();
  }

  auto end() -> decltype(this->inner.get().rend())
  {
    return inner.get().rend();
  }

  auto rbegin() -> decltype(this->inner.get().begin())
  {
    return inner.get().begin();
  }

  auto rend() -> decltype(this->inner.get().end())
  {
    return inner.get().end();
  }

  auto cbegin() const -> decltype(this->inner.get().crbegin())
  {
    return inner.get().crbegin();
  }

  auto cend() const -> decltype(this->inner.get().crend())
  {
    return inner.get().crend();
  }

  auto crbegin() const -> decltype(this->inner.get().cbegin())
  {
    return inner.get().cbegin();
  }

  auto crend() const -> decltype(this->inner.get().cend())
  {
    return inner.get().cend();
  }
};

template<typename C>
auto reversed(C& c) -> reversed_container<C> {
    return reversed_container<C>{c};
}
