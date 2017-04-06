#pragma once

#include <bitset>
#include <memory>
#include <vector>

#include "flexrayusbinterface/Types.hpp"
#include "flexrayusbinterface/FtResult.hpp"

class UsbChannel
{
  FT_HANDLE handle;

  inline UsbChannel(FT_HANDLE handle) : handle{ handle }
  {
  }

public:
  static auto open(std::string serial_number, uint32_t spi_clock_divisor = 2) -> variant<UsbChannel, FtResult>;

  void write(std::vector<WORD> const& data) const;
  auto read(std::string buffer) const -> variant<std::string, FtResult>;
  auto bytes_available() const -> variant<DWORD, FtResult>;
};
