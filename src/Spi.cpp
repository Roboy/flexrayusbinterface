#include "flexrayusbinterface/Spi.hpp"

#include <cstdlib>
#include <vector>

#include <ros/console.h>
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
  ROS_DEBUG("Checking for FTDI devices...");

  TRY(FtResult{ FT_CreateDeviceInfoList(NumDevs) }.or_else([](FtResult error) {
    ROS_ERROR_STREAM("Error in getting the number of devices: " << error.str());
    return error;  // Exit with error
  }));
  if (*NumDevs < 1)  // Exit if we don't see any
  {
    ROS_WARN("There are no FTDI devices installed");
    return FtResult::Message::OTHER_ERROR;  // Exit with error
  }
  ROS_DEBUG_STREAM(*NumDevs << " FTDI devices found-the count includes "
                               "individual ports on a single chip");
  return FtResult::Message::OK;
}

FtResult GetDeviceInfo(DWORD* NumDevs)
{
  FT_DEVICE_LIST_INFO_NODE* devInfo;
  auto log_error = [](FtResult result, char const* message = "Could not find info for devices: ") {
    ROS_ERROR_STREAM(message << result.str());
    return result;
  };

  // ------------------------------------------------------------
  // If yes then print details of devices
  // ------------------------------------------------------------
  devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * (*NumDevs));
  TRY(FtResult{ FT_GetDeviceInfoList(devInfo, NumDevs) }.or_else(log_error));

  ROS_INFO_STREAM(*NumDevs << " devices ");
  for (unsigned int i = 0; i < *NumDevs; i++)
  {
    ROS_INFO_STREAM(" Dev: " << i);
    ROS_INFO_STREAM(" Flags=0x" << devInfo[i].Flags);
    ROS_INFO_STREAM(" Type=0x" << devInfo[i].Type);
    ROS_INFO_STREAM(" ID=0x" << devInfo[i].ID);
    ROS_INFO_STREAM(" LocId=0x" << devInfo[i].LocId);
    ROS_INFO_STREAM(" SerialNumber=" << devInfo[i].SerialNumber);
    ROS_INFO_STREAM(" Description=" << devInfo[i].Description);
    ROS_INFO_STREAM(" ftHandle=0x" << devInfo[i].ftHandle);
  }
  return FtResult::Message::OK;
}

FtResult OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle)
{
  // Configure port parameters
  ROS_DEBUG("Configuring port for MPSSE use");
  auto log_error = [](FtResult result, char const* message = "Error in initializing the MPSSE on device. Error: ") {
    ROS_ERROR_STREAM(message << result.str());
    return result;
  };

  DWORD InTransferSize = USBINSIZE;
  DWORD OutTransferSize = USBOUTSIZE;
  DWORD dwNumBytesToRead, dwNumBytesRead;
  BYTE byInputBuffer[DATASETSIZE * 2];  // Local buffer to hold data read from the FT2232H

  // -----------------------------------------------------------
  // Open the port on first device located
  // -----------------------------------------------------------
  TRY(FtResult{ FT_Open(0, ftHandle) }.or_else(log_error, "Failed to open: "));

  ROS_DEBUG("Port opened");

  // ------------------------------------------------------------
  // Configure MPSSE and test for synchronisation
  // ------------------------------------------------------------

  TRY(FtResult{ FT_ResetDevice(*ftHandle) }.or_else(log_error));                        // Reset USB device
  TRY(FtResult{ FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead) }.or_else(log_error));  // Purge USB receive buffer
                                                                                        // first by
  // reading out all old data from FT2232H
  // receive buffer
  //
  // Get the number of bytes in the FT2232H receive buffer
  if ((dwNumBytesToRead > 0))  // Read out the data from FT2232H receive buffer if not empty
    TRY(FtResult{ FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead) });
  else
    ROS_WARN("Buffer empty");

  // FtResult{FT_SetUSBParameters(ftHandle, 65536, 65535)}.or_else(log_error);  // Set USB
  // request transfer sizes to 64K
  TRY(FtResult{ FT_SetUSBParameters(*ftHandle, InTransferSize, OutTransferSize) }.or_else(
      log_error));  // Set USB request transfer
                    // sizes...page 73 d2XX
                    // programmer guide (only
                    // dwInTransferSize has been
                    // implemented)
                    /*  FT_STATUS FT_SetUSBParameters(FT_HANDLE ftHandle, DWORD dwInTransferSize,
                     DWORD dwOutTransferSize)
                     (only dwInTransferSize currently supported..test this...see d2XX programmers
                     guide p 73)
                     */
  TRY(FtResult{ FT_SetChars(*ftHandle, false, 0, false, 0) }.or_else(log_error));  // Disable event and error characters
  /*  FT_STATUS FT_SetChars (FT_HANDLE ftHandle, UCHAR uEventCh, UCHAR
   uEventChEn,UCHAR uErrorCh, UCHAR uErrorChEn)
   (uEventCh = event character, uEventChEn = enable event character insertion,
   uErrorCh = error character, ...)
   */
  // FtResult{FT_SetTimeouts(ftHandle, 3000, 3000)}.or_else(log_error);         // Sets the read
  // and write timeouts in milliseconds
  TRY(FtResult{ FT_SetTimeouts(*ftHandle, 300, 300) }.or_else(log_error));  // Sets the read and write timeouts in
                                                                            // milliseconds
  /*  FT_STATUS FT_SetTimeouts (FT_HANDLE ftHandle, DWORD dwReadTimeout, DWORD
   dwWriteTimeout)
   (dwReadTimeout, dwWriteTimeout in ms)
   */
  TRY(FtResult{ FT_SetLatencyTimer(*ftHandle, 1) }.or_else(log_error));  // Set the latency timer to 1mS (default is
                                                                         // 16mS)
  /*  FT_STATUS FT_SetLatencyTimer (FT_HANDLE ftHandle, UCHAR ucTimer)
   (ucTimer = require latency value in milliseconds [range of 2-255 page 68 of
   d2XX programmers guide?])
   */
  // FtResult{FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x00, 0x00)}.or_else(log_error);
  // Turn on flow control to synchronize IN requests
  /*  FT_STATUS FT_SetFlowControl (FT_HANDLE ftHandle, USHORT usFlowControl,
   UCHAR uXon, UCHAR uXoff)
   (usFlowControl must be either FT_FLOW_NONE, FT_FLOW_RTS_CTS, FT_FLOW_DTR_DSR
   or FT_FLOW_XON_XOFF)
   (uXon = character to signal Xon [Only used is set to FT_FLOW_XON_XOFF])
   (uXoff = character to signal Xoff [Only used is set to FT_FLOW_XON_XOFF])
   */
  TRY(FtResult{ FT_SetBitMode(*ftHandle, 0x0, 0x00) }.or_else(log_error));  // Reset controller
  /*  FT_STATUS FT_SetBitmode (FT_HANDLE ftHandle, UCHAR ucMask, UCHAR ucMode)
   (ucMask sets which bits are inputs [=0] or outputs [=1])
   (ucMode can one of the following:
   0x0 = Reset
   0x1 = Asynchronous Bit Bang
   0x2 = MPSSE (FT2232, FT2232H, FT4232H and FT232H devices only)
   0x4 = Synchronous Bit Bang (FT232R, FT245R, FT2232, FT2232H, FT4232H and
   FT232H devices only)
   0x8 = MCU Host Bus Emulation Mode (FT2232, FT2232H, FT4232H and FT232H
   devices only)
   0x10 = Fast Opto-Isolated Serial Mode (FT2232, FT2232H, FT4232H and FT232H
   devices only)
   0x20 = CBUS Bit Bang Mode (FT232R and FT232H devices only)
   0x40 = Single Channel Synchronous 245 FIFO Mode (FT2232H and FT232H devices
   only)
   )

   */
  TRY(FtResult{ FT_SetBitMode(*ftHandle, 0x0, 0x02) }.or_else(log_error));  // Enable MPSSE mode

  usleep(1);  // Wait for all the USB stuff to complete and work

  ROS_INFO("MPSSE ready for commands");
  return FtResult::Message::OK;
}

FtResult TestMPSSE(FT_HANDLE ftHandle)
{
  auto log_error = [&ftHandle](FtResult error, char const* message = "") {
    ROS_FATAL_STREAM(message << error.str());
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
      log_error));       // Send off the loopback command
  dwNumBytesToSend = 0;  // Reset output buffer pointer

  // Check the receive buffer - it should be empty
  TRY(FtResult{ FT_GetQueueStatus(ftHandle, &dwNumBytesToRead) }.or_else(log_error));  // Get the number
                                                                                       // of bytes in the
                                                                                       // FT2232H receive
                                                                                       // buffer
  if (dwNumBytesToRead != 0)
    return log_error(FtResult::Message::OTHER_ERROR, "Error - MPSSE receive buffer should be empty, try to run again!");

  ROS_DEBUG("Internal loop-back configured and receive buffer is empty");

  // send bad op-code to check every thing is working correctly
  byOutputBuffer[dwNumBytesToSend++] = 0xAB;  // Add bogus command ‘0xAB’ to the queue
  TRY(FtResult{ FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent) }.or_else(
      log_error));       // Send off the BAD command
  dwNumBytesToSend = 0;  // Reset output buffer pointer

  for (uint32_t counter = 0; (counter < 100) && (dwNumBytesToRead == 0); ++counter)
    TRY(FtResult{ FT_GetQueueStatus(ftHandle, &dwNumBytesToRead) }.or_else(log_error));  // Get the number of
                                                                                         // bytes in the device
                                                                                         // input buffer

  if (!dwNumBytesToRead)
    return FtResult::Message::OTHER_ERROR;
  TRY(FtResult{ FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead) }.or_else(
      log_error));  // Read the data from input buffer
  auto received_echo = std::adjacent_find(&byInputBuffer[0], std::next(&byInputBuffer[0], dwNumBytesRead),
                                          [](BYTE left, BYTE right) { return left == 0xFA && right == 0xAB; });

  if (received_echo)
  {
    ROS_DEBUG("MPSSE synchronised.");
    byOutputBuffer[dwNumBytesToSend++] = '\x85';  // Command to turn off loop back of TDI/TDO connection
    TRY(FtResult{ FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent) }.or_else(
        log_error));  // Send off the loopback command
    return FtResult::Message::OK;
  }
  else
  {
    ROS_ERROR("Error in synchronizing the MPSSE");
    FT_SetBitMode(ftHandle, 0x0, 0x00);     // Reset the port to disable MPSSE
    FT_Close(ftHandle);                     // Close the USB port
    return FtResult::Message::OTHER_ERROR;  // Exit with error
  }
}

FtResult ConfigureSPI(FT_HANDLE ftHandle, DWORD dwClockDivisor)
{
  DWORD dwNumBytesSent;
  auto cleanup = [&ftHandle](FtResult error) {
    ROS_ERROR_STREAM("Error configuring SPI, Error code " << error.str());
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
  ROS_DEBUG("SPI initialisation successful");
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
  static_assert(message.size == 17, "The encoding is incorrect!");
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

  auto write = [&]() {
    FtResult result{ FT_Write(ftHandle, &message.front(), message.size(), &dwNumBytesSent) };
    return result.or_else([](FtResult error) {
      ROS_ERROR_STREAM(" something wrong with FT_Write call, Error code " << error.str());
      return error;
    });
  };

  while (write() != FtResult::Message::OK);
}
