#include "flexrayusbinterface/UsbChannel.hpp"

auto UsbChannel::connect() -> boost::optional<Connected>
{
  uint32_t number_of_devices;
  if (CheckDeviceConnected(&number_of_devices) == FtResult::Message::OK)
    return Connected(number_of_devices);
  return boost::none;
}

auto UsbChannel::Connected::get_device() -> boost::optional<Device>
{
  if (GetDeviceInfo(&number_of_devices) == FtResult::Message::OK)
    return Device();
  return boost::none;
}

auto UsbChannel::Device::open() const -> boost::optional<OpenChannel>
{
  FT_HANDLE handle;
  if (OpenPortAndConfigureMPSSE(&handle) == FtResult::Message::OK)
    return OpenChannel(handle);
  return boost::none;
}

auto UsbChannel::OpenChannel::configure_mpsse() -> boost::optional<MpsseChannel>
{
  if (TestMPSSE(handle) == FtResult::Message::OK)
    return MpsseChannel(handle);
  return boost::none;
}

auto UsbChannel::MpsseChannel::configure_spi(uint32_t clock_divisor) -> boost::optional<UsbChannel>
{
  if (ConfigureSPI(handle, clock_divisor) == FtResult::Message::OK)
    return UsbChannel(handle);
  return boost::none;
}

void UsbChannel::write(std::vector<WORD> const& data) const
{
  SPI_WriteBuffer(handle, const_cast<WORD*>(&data[0]), data.size());
}

auto UsbChannel::read(std::string buffer) const -> variant<std::string, FtResult>
{
  DWORD num_bytes;
  TRY(FtResult{ FT_Read(handle, &buffer.front(), buffer.size() * sizeof(char), &num_bytes) });
  buffer.resize(num_bytes);
  return buffer;
}

auto UsbChannel::bytes_available() const -> variant<DWORD, FtResult>
{
  DWORD num_bytes;
  TRY(FtResult { FT_GetQueueStatus(handle, &num_bytes) });
  return num_bytes;
}
