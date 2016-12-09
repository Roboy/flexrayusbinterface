#include "flexrayusbinterface/UsbChannel.hpp"

auto UsbChannel::connect() -> boost::optional<Connected> {
    uint32_t numberOfConnectedDevices;
    if (CheckDeviceConnected(&numberOfConnectedDevices))
        return Connected();
    return boost::none;
}

auto UsbChannel::Connected::get_device() const -> boost::optional<Device> {
    uint32_t numberOfConnectedDevices;
    if (GetDeviceInfo(&numberOfConnectedDevices))
        return Device();
    return boost::none;
}

auto UsbChannel::Device::open(DWORD in, DWORD out) const -> boost::optional<OpenChannel> {
    FT_HANDLE handle;
    if (OpenPortAndConfigureMPSSE(&handle, in, out))
        return OpenChannel(handle);
    return boost::none;
}

auto UsbChannel::OpenChannel::configure_mpsse() -> boost::optional<MpsseChannel> {
    if (TestMPSSE(&handle))
        return MpsseChannel(handle);
    return boost::none;
}

auto UsbChannel::MpsseChannel::configure_spi(uint32_t clock_divisor) -> boost::optional<UsbChannel> {
    if (ConfigureSPI(&handle, clock_divisor))
        return UsbChannel(handle);
    return boost::none;
}

FT_STATUS UsbChannel::write(std::vector<WORD> const& data) const {
    return SPI_WriteBuffer(handle, const_cast<WORD*>(&data[0]), data.size() * sizeof(WORD));
}

auto UsbChannel::read(std::vector<uint8_t> buffer) const -> variant<std::vector<uint8_t>, FT_STATUS> {
    DWORD num_bytes;
    if (auto status = FT_Read(handle, &buffer.front(), buffer.size(), &num_bytes))
        return status;
    buffer.resize(num_bytes);
    return buffer;
}

auto UsbChannel::bytes_available() const -> variant<DWORD, FT_STATUS> {
    DWORD num_bytes;
    if (FT_STATUS status = FT_GetQueueStatus(handle, &num_bytes)) {
        return status;
    } else {
        return num_bytes;
    }
}
