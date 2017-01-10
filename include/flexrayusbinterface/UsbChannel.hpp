#pragma once

#include <bitset>
#include <vector>

#include <boost/optional.hpp>
#include <mapbox/variant.hpp>

#include "flexrayusbinterface/Spi.hpp"

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

  void write(std::vector<WORD> const& data) const;
  auto read(std::string buffer) const -> variant<std::string, FtResult>;
  auto bytes_available() const -> variant<DWORD, FtResult>;
};
