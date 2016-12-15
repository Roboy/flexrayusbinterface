#pragma once

#include <boost/optional.hpp>
#include <mapbox/variant.hpp>

#include <bitset>
#include "flexrayusbinterface/Spi.hpp"

struct UsbError
{
  FT_STATUS status;
};

template <size_t BytesAvailable, typename... Chunks>
class Message;

template <size_t BytesAvailable, typename U, typename W, typename... Chunks>
class Message<BytesAvailable, U, W, Chunks...>
{
  using prev_t = Message<BytesAvailable + sizeof(U), W, Chunks...>;

  template <typename T>
  using next_t = Message<BytesAvailable - sizeof(T), T, U, W, Chunks...>;

  prev_t prev;
  std::reference_wrapper<U const> chunk;

public:
  static constexpr size_t size = sizeof(U) + prev_t::size;

  Message(prev_t prev, U const& chunk) : prev{ prev }, chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T const& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
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

public:
  static constexpr size_t size = sizeof(U);

  Message(U const& chunk) : chunk{ chunk }
  {
  }

  template <typename T>
  auto add(T const& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
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

public:
  static constexpr size_t size = 0;

  template <typename T>
  auto add(T const& chunk) -> next_t<T>
  {
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { chunk };
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
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
  }

  template <typename Buffer>
  auto read_data(Buffer& buffer) -> Buffer&
  {
    prev.read_data(buffer);
    U u;
    if (buffer.read(reinterpret_cast<char*>(&u), sizeof(U)))
      chunk.get() = u;
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
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { *this, chunk };
  }

  template <typename Buffer>
  auto read_data(Buffer& buffer) -> Buffer&
  {
    U u;
    if (buffer.read(reinterpret_cast<char*>(&u), sizeof(U)))
      chunk.get() = u;
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
    static_assert(BytesAvailable > sizeof(T), "There is not enough room in the message.");
    return { chunk };
  }

  template <typename Buffer>
  auto read(Buffer& buffer) -> Buffer&
  {
    buffer.ignore(BytesAvailable);
    return buffer;
  }
};

class UsbChannel
{
  FT_HANDLE handle;
  inline UsbChannel(FT_HANDLE handle) : handle(handle)
  {
  }
  friend class MpsseChannel;

  template <typename... Args>
  using variant = mapbox::util::variant<Args...>;

public:
  class OpenChannel;
  class Device;
  class Connected;

  class MpsseChannel
  {
    FT_HANDLE handle;
    inline MpsseChannel(FT_HANDLE handle) : handle{ handle }
    {
    }
    friend class OpenChannel;

  public:
    auto configure_spi(uint32_t clock_divisor = 2) -> boost::optional<UsbChannel>;
  };

  class OpenChannel
  {
    FT_HANDLE handle;
    inline OpenChannel(FT_HANDLE handle) : handle{ handle }
    {
    }
    friend class Device;

  public:
    auto configure_mpsse() -> boost::optional<MpsseChannel>;
  };

  class Device
  {
    Device() = default;
    friend class Connected;

  public:
    auto open() const -> boost::optional<OpenChannel>;
  };

  class Connected
  {
    uint32_t number_of_devices;
    inline Connected(uint32_t number_of_devices) : number_of_devices{ number_of_devices }
    {
    }
    friend class UsbChannel;

  public:
    auto get_device() -> boost::optional<Device>;
  };

  static auto connect() -> boost::optional<Connected>;

  FT_STATUS write(std::vector<WORD> const& data) const;
  auto read(std::string buffer) const -> variant<std::string, UsbError>;
  auto bytes_available() const -> variant<DWORD, UsbError>;
};
