#include <bitset>
#include <iostream>
#include <vector>

#include "ftd2xx.h"

#include "flexrayusbinterface/FtResult.hpp"

std::vector<FT_DEVICE_LIST_INFO_NODE> getConnectedDevices()
{
  DWORD num_devs;
  if (FtResult{ FT_CreateDeviceInfoList(&num_devs) } == FtResult::Message::OK)
  {
    std::vector<FT_DEVICE_LIST_INFO_NODE> devices(num_devs);
    if (FtResult{ FT_GetDeviceInfoList(&devices.front(), &num_devs) } == FtResult::Message::OK)
    {
      devices.resize(num_devs);
      return devices;
    }
  }
  return {};
}

int main()
{
  auto devices = getConnectedDevices();
  std::cout << devices.size() << " FTDI devices found\n\n";
  for (auto const& info : devices)
  {
    std::cout << "FTDI device serial number: " << info.SerialNumber << '\n';
    std::cout << "description: " << info.Description << '\n';
    std::cout << "type: " << info.Type << '\n';
    std::cout << "flags: " << std::bitset<8*sizeof(info.Flags)>(info.Flags) << "\n\n";
  }
}
