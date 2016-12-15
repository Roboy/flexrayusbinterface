#pragma once
#include <cstddef>
#include <functional>

template <size_t BytesAvailable, typename... Chunks>
class Message;

template <int N>
using const_char_arr_ignore_null_t = char const[N - 1];

template <size_t N>
using const_char_arr = char const[N];

template <size_t BytesAvailable, typename U, typename W, typename... Chunks>
class Message<BytesAvailable, U, W, Chunks...>
{
  using prev_t = Message<BytesAvailable + sizeof(U), W, Chunks...>;

  template <typename T>
  using next_t = Message<BytesAvailable - sizeof(T), T, U, W, Chunks...>;

  prev_t prev;
  std::reference_wrapper<U const> chunk;

  template <typename T>
  auto add_with_cref(T const& new_chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { *this, new_chunk };
  }

public:
  static constexpr size_t size = sizeof(U) + prev_t::size;

  Message(prev_t prev, U const& chunk) : prev{ prev }, chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T&& new_chunk) -> next_t<typename std::decay<T>::type>
  {
      static_assert(!std::is_rvalue_reference<decltype(std::forward<T>(new_chunk))>::value, "Cannot bind to temporaries.");
      return add_with_cref(new_chunk);
  }

  template <size_t N>
  auto adds(const_char_arr<N> const& str) -> next_t<const_char_arr_ignore_null_t<N>>
  {
    static_assert(BytesAvailable >= sizeof(const_char_arr_ignore_null_t<N>), "There is not enough room in the message.");
    return { *this, *reinterpret_cast<const_char_arr_ignore_null_t<N>*>(&str) };
  }

  template <typename Buffer>
  auto write_data(Buffer& buffer) -> Buffer&
  {
    if (buffer && prev.write_data(buffer))
      buffer.write(reinterpret_cast<char const*>(&chunk.get()), sizeof(U));
    return buffer;
  }

  template <typename Buffer>
  auto write(Buffer& buffer) -> Buffer&
  {
    if (write_data(buffer))
      for (auto i = 0; (i < BytesAvailable) && buffer.put(0); ++i)
        ;
    return buffer;
  }
};

template <size_t BytesAvailable, typename U>
class Message<BytesAvailable, U>
{
  template <typename T>
  using next_t = Message<BytesAvailable - sizeof(T), T, U>;

  std::reference_wrapper<U const> chunk;

  template <typename T>
  auto add_with_cref(T const& new_chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { *this, new_chunk };
  }

public:
  static constexpr size_t size = sizeof(U);

  Message(U const& chunk) : chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T&& new_chunk) -> next_t<typename std::decay<T>::type>
  {
      static_assert(!std::is_rvalue_reference<decltype(std::forward<T>(new_chunk))>::value, "Cannot bind to temporaries.");
      return add_with_cref(new_chunk);
  }

  template <size_t N>
  auto adds(const_char_arr<N> const& str) -> next_t<const_char_arr_ignore_null_t<N>>
  {
    static_assert(BytesAvailable >= sizeof(const_char_arr_ignore_null_t<N>), "There is not enough room in the message.");
    return { *this, *reinterpret_cast<const_char_arr_ignore_null_t<N>*>(&str) };
  }

  template <typename Buffer>
  auto write_data(Buffer& buffer) -> Buffer&
  {
    buffer.write(reinterpret_cast<char const*>(&chunk.get()), sizeof(U));
    return buffer;
  }

  template <typename Buffer>
  auto write(Buffer& buffer) -> Buffer&
  {
    write_data(buffer);
    for (auto i = 0; i < BytesAvailable; ++i)
      buffer.put(0);
    return buffer;
  }
};

template <size_t BytesAvailable>
class Message<BytesAvailable>
{
  template <typename T>
  using next_t = Message<BytesAvailable - sizeof(T), T>;

  template <typename T>
  auto add_with_cref(T const& new_chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { new_chunk };
  }

public:
  static constexpr size_t size = 0;

  template <typename T>
  auto add(T&& new_chunk) -> next_t<typename std::decay<T>::type>
  {
      static_assert(!std::is_rvalue_reference<decltype(std::forward<T>(new_chunk))>::value, "Cannot bind to temporaries.");
      return add_with_cref(new_chunk);
  }

  template <size_t N>
  auto adds(const_char_arr<N> const& str) -> next_t<const_char_arr_ignore_null_t<N>>
  {
    static_assert(BytesAvailable >= sizeof(const_char_arr_ignore_null_t<N>), "There is not enough room in the message.");
    return { *reinterpret_cast<const_char_arr_ignore_null_t<N>*>(&str) };
  }

  template <typename Buffer>
  auto write(Buffer& buffer) -> Buffer&
  {
    for (auto i = 0; i < BytesAvailable; ++i)
      buffer.put(0);
    return buffer;
  }
};

template <size_t BytesAvailable, typename... Chunks>
class Parser;

template <size_t BytesAvailable, typename U, typename W, typename... Chunks>
class Parser<BytesAvailable, U, W, Chunks...>
{
  using prev_t = Parser<BytesAvailable + sizeof(U), W, Chunks...>;

  template <typename T>
  using next_t = Parser<BytesAvailable - sizeof(T), T, U, W, Chunks...>;

  prev_t prev;
  std::reference_wrapper<U> chunk;

public:
  static constexpr size_t size = sizeof(U) + prev_t::size;

  Parser(prev_t prev, U& chunk) : prev{ prev }, chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
  }

  template <typename Buffer>
  auto read_data(Buffer& buffer) -> Buffer&
  {
    prev.read_data(buffer);
    U u;
    if (buffer.read(reinterpret_cast<char*>(&u), sizeof(U)))
      std::swap(chunk.get(), u);
    return buffer;
  }

  template <typename Buffer>
  auto read(Buffer& buffer) -> Buffer&
  {
    read_data(buffer);
    buffer.ignore(BytesAvailable);
    return buffer;
  }
};

template <size_t BytesAvailable, typename U>
class Parser<BytesAvailable, U>
{
  template <typename T>
  using next_t = Parser<BytesAvailable - sizeof(T), T, U>;

  std::reference_wrapper<U> chunk;

public:
  static constexpr size_t size = sizeof(U);

  Parser(U& chunk) : chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
  }

  template <typename Buffer>
  auto read_data(Buffer& buffer) -> Buffer&
  {
    U u;
    if (buffer.read(reinterpret_cast<char*>(&u), sizeof(U)))
      std::swap(chunk.get(), u);
    return buffer;
  }

  template <typename Buffer>
  auto read(Buffer& buffer) -> Buffer&
  {
    read_data(buffer);
    buffer.ignore(BytesAvailable);
    return buffer;
  }
};

template <size_t BytesAvailable>
class Parser<BytesAvailable>
{
  template <typename T>
  using next_t = Parser<BytesAvailable - sizeof(T), T>;

public:
  static constexpr size_t size = 0;

  template <typename T>
  auto add(T& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable >= sizeof(T), "There is not enough room in the message.");
    return { chunk };
  }

  template <typename Buffer>
  auto read(Buffer& buffer) -> Buffer&
  {
    buffer.ignore(BytesAvailable);
    return buffer;
  }
};
