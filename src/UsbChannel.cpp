#include "flexrayusbinterface/UsbChannel.hpp"

auto UsbChannel::connect() -> boost::optional<Connected>
{
  uint32_t number_of_devices;
  if (CheckDeviceConnected(&number_of_devices))
    return Connected(number_of_devices);
  return boost::none;
}

auto UsbChannel::Connected::get_device() -> boost::optional<Device>
{
  if (GetDeviceInfo(&number_of_devices))
    return Device();
  return boost::none;
}

auto UsbChannel::Device::open(DWORD in, DWORD out) const -> boost::optional<OpenChannel>
{
  FT_HANDLE handle;
  if (OpenPortAndConfigureMPSSE(&handle, in, out))
    return OpenChannel(handle);
  return boost::none;
}

auto UsbChannel::OpenChannel::configure_mpsse() -> boost::optional<MpsseChannel>
{
  if (TestMPSSE(handle))
    return MpsseChannel(handle);
  return boost::none;
}

auto UsbChannel::MpsseChannel::configure_spi(uint32_t clock_divisor) -> boost::optional<UsbChannel>
{
  if (ConfigureSPI(handle, clock_divisor))
    return UsbChannel(handle);
  return boost::none;
}

FT_STATUS UsbChannel::write(std::vector<WORD> const& data) const
{
  return SPI_WriteBuffer(handle, const_cast<WORD*>(&data[0]), data.size());
}

auto UsbChannel::read(std::vector<char> buffer) const -> variant<std::vector<char>, UsbError>
{
  DWORD num_bytes;
  if (auto status = FT_Read(handle, &buffer.front(), buffer.size() * sizeof(char), &num_bytes))
    return UsbError{ status };
  buffer.resize(num_bytes);
  return buffer;
}

auto UsbChannel::bytes_available() const -> variant<DWORD, UsbError>
{
  DWORD num_bytes;
  if (FT_STATUS status = FT_GetQueueStatus(handle, &num_bytes))
    return UsbError{ status };
  return num_bytes;
}
