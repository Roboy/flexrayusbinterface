#include "flexrayusbinterface/Spi.hpp"

#include <cstdlib>
#include <sstream>
#include <vector>

#include "flexrayusbinterface/Message.hpp"
#include "flexrayusbinterface/UsbChannel.hpp"
#include "ftd2xx.h"

// necessary for MPSSE command
static const BYTE MSB_RISING_EDGE_CLOCK_BYTE_OUT = '\x10';
static const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_OUT = '\x11';
static const BYTE MSB_RISING_EDGE_CLOCK_BIT_OUT = '\x12';
static const BYTE MSB_FALLING_EDGE_CLOCK_BIT_OUT = '\x13';
static const BYTE MSB_RISING_EDGE_CLOCK_BYTE_IN = '\x20';
static const BYTE MSB_RISING_EDGE_CLOCK_BIT_IN = '\x22';
static const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_IN = '\x24';
static const BYTE MSB_FALLING_EDGE_CLOCK_BIT_IN = '\x26';
static const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE = '\x31';
static const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE = '\x34';
static const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BIT = '\x36';
static const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BIT = '\x33';

FtResult CheckDeviceConnected(DWORD* NumDevs)
{
  // -----------------------------------------------------------
  // Does an FTDI device exist?
  // -----------------------------------------------------------

  TRY(FtResult{ FT_CreateDeviceInfoList(NumDevs) }.or_else([](FtResult error) {
    return error;  // Exit with error
  }));
  if (*NumDevs < 1)  // Exit if we don't see any
  {
    return FtResult::Message::OTHER_ERROR;  // Exit with error
  }
  return FtResult::Message::OK;
}

FtResult GetDeviceInfo(DWORD* NumDevs)
{
  FT_DEVICE_LIST_INFO_NODE* devInfo;

  // ------------------------------------------------------------
  // If yes then print details of devices
  // ------------------------------------------------------------
  devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * (*NumDevs));
  TRY(FtResult{ FT_GetDeviceInfoList(devInfo, NumDevs) });
  return FtResult::Message::OK;
}

FtResult OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle)
{
  // Configure port parameters
  DWORD InTransferSize = USBINSIZE;
  DWORD OutTransferSize = USBOUTSIZE;
  DWORD dwNumBytesToRead, dwNumBytesRead;
  BYTE byInputBuffer[DATASETSIZE * 2];  // Local buffer to hold data read from the FT2232H

  // -----------------------------------------------------------
  // Open the port on first device located
  // -----------------------------------------------------------
  TRY(FtResult{ FT_Open(0, ftHandle) });

  // ------------------------------------------------------------
  // Configure MPSSE and test for synchronisation
  // ------------------------------------------------------------

  TRY(FtResult{ FT_ResetDevice(*ftHandle) });                        // Reset USB device
  TRY(FtResult{ FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead) });  // Purge USB receive buffer
                                                                     // first by
  // reading out all old data from FT2232H
  // receive buffer
  //
  // Get the number of bytes in the FT2232H receive buffer
  if ((dwNumBytesToRead > 0))  // Read out the data from FT2232H receive buffer if not empty
    TRY(FtResult{ FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead) });

  // request transfer sizes to 64K
  // Set USB request transfer sizes...page 73 d2XX programmer guide
  // (only dwInTransferSize has been implemented)
  TRY(FtResult{ FT_SetUSBParameters(*ftHandle, InTransferSize, OutTransferSize) });
  TRY(FtResult{ FT_SetChars(*ftHandle, false, 0, false, 0) });  // Disable event and error characters
  TRY(FtResult{ FT_SetTimeouts(*ftHandle, 300, 300) });         // Sets the read and write timeouts in
                                                                // milliseconds
  TRY(FtResult{ FT_SetLatencyTimer(*ftHandle, 1) });            // Set the latency timer to 1mS (default is
                                                                // 16mS)
  TRY(FtResult{ FT_SetBitMode(*ftHandle, 0x0, 0x00) });         // Reset controller
  TRY(FtResult{ FT_SetBitMode(*ftHandle, 0x0, 0x02) });         // Enable MPSSE mode

  usleep(1);  // Wait for all the USB stuff to complete and work

  return FtResult::Message::OK;
}

FtResult TestMPSSE(FT_HANDLE ftHandle)
{
  auto disable_mpsse = [&ftHandle](FtResult error) {
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return error;
  };
  BYTE byOutputBuffer[DATASETSIZE * 2];
  BYTE byInputBuffer[DATASETSIZE * 2];
  DWORD dwNumBytesToSend = 0;
  DWORD dwNumBytesSent = 0, dwNumBytesRead = 0, dwNumBytesToRead = 0;

  /*
   *  Now the MPSSE is ready for commands, each command consists of an op-code
   * followed by any necessary parameters or
   *  data. Each op-code is sent using FT_Write call
   */

  /* Synchronisation and Bad communication detection */
  // Enable internal loop-back
  byOutputBuffer[dwNumBytesToSend++] = 0x84;  // Enable loopback
  TRY(FtResult{ FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent) }.or_else(
      disable_mpsse));   // Send off the loopback command
  dwNumBytesToSend = 0;  // Reset output buffer pointer

  // Check the receive buffer - it should be empty
  TRY(FtResult{ FT_GetQueueStatus(ftHandle, &dwNumBytesToRead) }.or_else(disable_mpsse));  // Get the number
                                                                                           // of bytes in the
                                                                                           // FT2232H receive
                                                                                           // buffer
  if (dwNumBytesToRead != 0)
    return disable_mpsse(FtResult::Message::OTHER_ERROR);

  // send bad op-code to check every thing is working correctly
  byOutputBuffer[dwNumBytesToSend++] = 0xAB;  // Add bogus command ‘0xAB’ to the queue
  TRY(FtResult{ FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent) }.or_else(
      disable_mpsse));   // Send off the BAD command
  dwNumBytesToSend = 0;  // Reset output buffer pointer

  for (uint32_t counter = 0; (counter < 100) && (dwNumBytesToRead == 0); ++counter)
    TRY(FtResult{ FT_GetQueueStatus(ftHandle, &dwNumBytesToRead) }.or_else(disable_mpsse));  // Get the number of
                                                                                             // bytes in the device
                                                                                             // input buffer

  if (!dwNumBytesToRead)
    return FtResult::Message::OTHER_ERROR;
  TRY(FtResult{ FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead) }.or_else(
      disable_mpsse));  // Read the data from input buffer
  auto received_echo = std::adjacent_find(&byInputBuffer[0], std::next(&byInputBuffer[0], dwNumBytesRead),
                                          [](BYTE left, BYTE right) { return left == 0xFA && right == 0xAB; });

  if (received_echo)
  {
    byOutputBuffer[dwNumBytesToSend++] = '\x85';  // Command to turn off loop back of TDI/TDO connection
    TRY(FtResult{ FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent) }.or_else(
        disable_mpsse));  // Send off the loopback command
    return FtResult::Message::OK;
  }
  else
  {
    FT_SetBitMode(ftHandle, 0x0, 0x00);     // Reset the port to disable MPSSE
    FT_Close(ftHandle);                     // Close the USB port
    return FtResult::Message::OTHER_ERROR;  // Exit with error
  }
}

FtResult ConfigureSPI(FT_HANDLE ftHandle, DWORD dwClockDivisor)
{
  DWORD dwNumBytesSent;
  auto cleanup = [&ftHandle](FtResult error) {
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return error;
  };

  // ------------------------------------------------------------
  // Configure MPSSE for SPI communications
  // ------------------------------------------------------------
  // The SK clock frequency can be worked out as follows with divide by 5 set as
  // off:: SK frequency = 60MHz /((1 + [(1 + (0xValueH*256) OR 0xValueL])*2)

  {
    std::stringstream ss;
    // Ensure disable clock divide by 5 for 60Mhz master clock
    // Ensure turn off adaptive clocking
    // disable 3 phase data clock
    Message<3>{}.adds("\x8A\x97\x8D").write(ss);
    auto output = ss.str();
    TRY(FtResult{ FT_Write(ftHandle, &output.front(), output.size(), &dwNumBytesSent) }.or_else(
        cleanup));  // Send out the commands
  }

  {
    BYTE divisor_l = dwClockDivisor & '\xFF';
    BYTE divisor_h = dwClockDivisor >> 8;
    std::stringstream ss;
    Message<7>{}
        .adds("\x80"     // Set directions of lower 8 pins, set value on bits set as output
              "\x08"     // Set SCK, DO, DI low and CS high
              "\x0b"     // Set SK,DO,GPIOL0 pins as output =1, other pins as input=0
              "\x86")    // Command to set clock divisor
        .add(divisor_l)  // Set 0xValueL of clock divisor
        .add(divisor_h)  // Set 0xValueH of clock divisor
        .adds("\x85")    // Command to turn off loop back of TDI/TDO connection
        .write(ss);
    auto output = ss.str();
    TRY(FtResult{ FT_Write(ftHandle, &output.front(), output.size(), &dwNumBytesSent) }.or_else(
        cleanup));  // Send out the commands
  }
  usleep(100);  // Delay for 100us
  return FtResult::Message::OK;
}

template <typename FirstIterator, typename EndIterator>
static std::string encode(FirstIterator fst, EndIterator const end)
{
  std::stringstream ss;
  char low;
  char high;
  auto message = Message<17>{}
                     .adds("\x80\x00\x0b\x80\x00\x0b")
                     .add(MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE)
                     .adds("\x01\x00")
                     .add(high)
                     .add(low)
                     .adds("\x80\x00\x0b\x80\x08\x0b");
  std::for_each(fst, end, [&](WORD word) {
    high = word >> 8;
    low = word & 0xff;
    message.write(ss);
  });
  return ss.str();
}

void SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords)
{
  DWORD dwNumBytesSent = 0;
  auto message = encode(buffer, buffer + numwords);

  auto write = [&]() { return FtResult{ FT_Write(ftHandle, &message.front(), message.size(), &dwNumBytesSent) }; };

  while (write() != FtResult::Message::OK)
    ;
}
