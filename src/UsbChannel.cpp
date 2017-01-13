#include "flexrayusbinterface/UsbChannel.hpp"

#include "Spi.hpp"

auto UsbChannel::open(std::string serial_number, uint32_t spi_clock_divisor) -> variant<UsbChannel, FtResult>
{
  FT_HANDLE handle;
  TRY(Open(serial_number, spi_clock_divisor, &handle));
  return UsbChannel{ handle };
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
  TRY(FtResult{ FT_GetQueueStatus(handle, &num_bytes) });
  return num_bytes;
}
