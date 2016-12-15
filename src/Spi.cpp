#include "flexrayusbinterface/Spi.hpp"

#include <cstdlib>
#include <vector>

#include <ros/console.h>
#include "flexrayusbinterface/UsbChannel.hpp"
#include "flexrayusbinterface/Message.hpp"
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

bool CheckDeviceConnected(DWORD* NumDevs)
{
  // -----------------------------------------------------------
  // Does an FTDI device exist?
  // -----------------------------------------------------------
  ROS_DEBUG("Checking for FTDI devices...");

  FT_STATUS ftStatus;
  ftStatus = FT_CreateDeviceInfoList(NumDevs);  // Get the number of FTDI devices
  if (ftStatus != FT_OK)                        // Did the command execute OK?
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_ERROR_STREAM("Error in getting the number of devices, Error code: " << errorMessage);
    return false;  // Exit with error
  }
  if (*NumDevs < 1)  // Exit if we don't see any
  {
    ROS_WARN("There are no FTDI devices installed");
    return false;  // Exit with error
  }
  ROS_DEBUG_STREAM(*NumDevs << " FTDI devices found-the count includes "
                               "individual ports on a single chip");
  return true;
}

bool GetDeviceInfo(DWORD* NumDevs)
{
  FT_DEVICE_LIST_INFO_NODE* devInfo;

  // ------------------------------------------------------------
  // If yes then print details of devices
  // ------------------------------------------------------------
  devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * (*NumDevs));
  FT_STATUS ftStatus;
  ftStatus = FT_GetDeviceInfoList(devInfo, NumDevs);

  ROS_INFO_STREAM(*NumDevs << " devices ");
  if (ftStatus == FT_OK)
  {
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
    return true;
  }
  else
  {
    ROS_WARN("Could not find info for devices");
    return false;  // return with error
  }
}

bool OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle)
{
  DWORD InTransferSize = USBINSIZE;
  DWORD OutTransferSize = USBOUTSIZE;
  DWORD dwNumBytesToRead, dwNumBytesRead;
  BYTE byInputBuffer[DATASETSIZE * 2];  // Local buffer to hold data read from the FT2232H

  // -----------------------------------------------------------
  // Open the port on first device located
  // -----------------------------------------------------------
  FT_STATUS ftStatus;
  ftStatus = FT_Open(0, ftHandle);
  if (ftStatus != FT_OK)
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_ERROR_STREAM("Open Failed with error " << errorMessage);
    return false;  // Exit with error
  }
  else
    ROS_DEBUG("Port opened");

  // ------------------------------------------------------------
  // Configure MPSSE and test for synchronisation
  // ------------------------------------------------------------

  // Configure port parameters
  ROS_DEBUG("Configuring port for MPSSE use");
  ftStatus |= FT_ResetDevice(*ftHandle);                        // Reset USB device
  ftStatus |= FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead);  // Purge USB receive buffer first by
                                                                // reading out all old data from FT2232H
                                                                // receive buffer
  // Get the number of bytes in the FT2232H receive buffer
  if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))  // Read out the data from FT2232H receive buffer if not empty
    FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
  else
    ROS_WARN("Buffer empty");

  // ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);  // Set USB
  // request transfer sizes to 64K
  ftStatus |= FT_SetUSBParameters(*ftHandle, InTransferSize, OutTransferSize);  // Set USB request transfer
                                                                                // sizes...page 73 d2XX
                                                                                // programmer guide (only
                                                                                // dwInTransferSize has been
                                                                                // implemented)
  /*  FT_STATUS FT_SetUSBParameters(FT_HANDLE ftHandle, DWORD dwInTransferSize,
   DWORD dwOutTransferSize)
   (only dwInTransferSize currently supported..test this...see d2XX programmers
   guide p 73)
   */
  ftStatus |= FT_SetChars(*ftHandle, false, 0, false, 0);  // Disable event and error characters
  /*  FT_STATUS FT_SetChars (FT_HANDLE ftHandle, UCHAR uEventCh, UCHAR
   uEventChEn,UCHAR uErrorCh, UCHAR uErrorChEn)
   (uEventCh = event character, uEventChEn = enable event character insertion,
   uErrorCh = error character, ...)
   */
  // ftStatus |= FT_SetTimeouts(ftHandle, 3000, 3000);         // Sets the read
  // and write timeouts in milliseconds
  ftStatus |= FT_SetTimeouts(*ftHandle, 300, 300);  // Sets the read and write timeouts in milliseconds
  /*  FT_STATUS FT_SetTimeouts (FT_HANDLE ftHandle, DWORD dwReadTimeout, DWORD
   dwWriteTimeout)
   (dwReadTimeout, dwWriteTimeout in ms)
   */
  ftStatus |= FT_SetLatencyTimer(*ftHandle, 1);  // Set the latency timer to 1mS (default is 16mS)
                                                 /*  FT_STATUS FT_SetLatencyTimer (FT_HANDLE ftHandle, UCHAR ucTimer)
                                                  (ucTimer = require latency value in milliseconds [range of 2-255 page 68 of
                                                  d2XX programmers guide?])
                                                  */
  // ftStatus |= FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x00, 0x00);
  // Turn on flow control to synchronize IN requests
  /*  FT_STATUS FT_SetFlowControl (FT_HANDLE ftHandle, USHORT usFlowControl,
   UCHAR uXon, UCHAR uXoff)
   (usFlowControl must be either FT_FLOW_NONE, FT_FLOW_RTS_CTS, FT_FLOW_DTR_DSR
   or FT_FLOW_XON_XOFF)
   (uXon = character to signal Xon [Only used is set to FT_FLOW_XON_XOFF])
   (uXoff = character to signal Xoff [Only used is set to FT_FLOW_XON_XOFF])
   */
  ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x00);  // Reset controller
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
  ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x02);  // Enable MPSSE mode
  if (ftStatus != FT_OK)
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_ERROR_STREAM("Error in initializing the MPSSE on device. Error: " << errorMessage);
    FT_Close(ftHandle);
    return false;  // Exit with error
  }
  usleep(1);  // Wait for all the USB stuff to complete and work

  ROS_INFO("MPSSE ready for commands");
  return true;
}

bool TestMPSSE(FT_HANDLE ftHandle)
{
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
  FT_STATUS ftStatus;
  ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);  // Send off the loopback command
  dwNumBytesToSend = 0;                                                              // Reset output buffer pointer

  // Check the receive buffer - it should be empty
  ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);  // Get the number
                                                              // of bytes in the
                                                              // FT2232H receive
                                                              // buffer
  if (dwNumBytesToRead != 0)
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_FATAL_STREAM("Error - MPSSE receive buffer should be empty, Error Code: " << errorMessage << ", try to run "
                                                                                                     "again!");
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return false;                        // Exit with error
  }
  else
    ROS_DEBUG("Internal loop-back configured and receive buffer is empty");

  // send bad op-code to check every thing is working correctly
  byOutputBuffer[dwNumBytesToSend++] = 0xAB;  // Add bogus command ‘0xAB’ to the queue
  ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);  // Send off the BAD command
  dwNumBytesToSend = 0;                                                              // Reset output buffer pointer

  uint32_t counter = 0;
  do
  {
    ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);  // Get the number of
                                                                // bytes in the device
                                                                // input buffer
    counter++;
  } while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK) &&
           (counter < 100));  // wait for bytes to return, an error or Timeout

  bool bCommandEchod = false;
  if (dwNumBytesToRead)
  {
    ftStatus = FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);  // Read the data from input buffer
    for (DWORD dwCount = 0; dwCount < dwNumBytesRead - 1;
         dwCount++)  // Check if Bad command and echo command are received
    {
      if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount + 1] == 0xAB))
      {
        bCommandEchod = true;
        break;
      }
    }
  }

  if (bCommandEchod == false)
  {
    ROS_ERROR("Error in synchronizing the MPSSE");
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return false;                        // Exit with error
  }
  else
  {
    ROS_DEBUG("MPSSE synchronised.");
    byOutputBuffer[dwNumBytesToSend++] = '\x85';  // Command to turn off loop back of TDI/TDO connection
    ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);  // Send off the loopback command
    dwNumBytesToSend = 0;                                                              // Reset output buffer pointer
    return true;
  }
}

bool ConfigureSPI(FT_HANDLE ftHandle, DWORD dwClockDivisor)
{
  BYTE byOutputBuffer[DATASETSIZE * 2];
  DWORD dwNumBytesToSend = 0;
  DWORD dwNumBytesSent;

  // ------------------------------------------------------------
  // Configure MPSSE for SPI communications
  // ------------------------------------------------------------
  // The SK clock frequency can be worked out as follows with divide by 5 set as
  // off:: SK frequency = 60MHz /((1 + [(1 + (0xValueH*256) OR 0xValueL])*2)

  byOutputBuffer[dwNumBytesToSend++] = '\x8A';  // Ensure disable clock divide by 5 for 60Mhz master clock
  byOutputBuffer[dwNumBytesToSend++] = '\x97';  // Ensure turn off adaptive clocking
  byOutputBuffer[dwNumBytesToSend++] = '\x8D';  // disable 3 phase data clock
  FT_STATUS ftStatus;
  ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);  // Send out the commands
  if (ftStatus != FT_OK)
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_ERROR_STREAM("Error configuring SPI, Error code " << errorMessage);
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return false;
  }

  dwNumBytesToSend = 0;                         // Clear output buffer
  byOutputBuffer[dwNumBytesToSend++] = '\x80';  // Set directions of lower 8 pins, set value on bits set as output
  byOutputBuffer[dwNumBytesToSend++] = '\x08';  // Set SCK, DO, DI low and CS high
  byOutputBuffer[dwNumBytesToSend++] = '\x0b';  // Set SK,DO,GPIOL0 pins as output =1, other pins as input=0
  byOutputBuffer[dwNumBytesToSend++] = '\x86';  // Command to set clock divisor
  byOutputBuffer[dwNumBytesToSend++] = (BYTE)(dwClockDivisor & '\xFF');  // Set 0xValueL of clock divisor
  byOutputBuffer[dwNumBytesToSend++] = (BYTE)(dwClockDivisor >> 8);      // Set 0xValueH of clock divisor
  byOutputBuffer[dwNumBytesToSend++] = '\x85';  // Command to turn off loop back of TDI/TDO connection
  ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);  // Send out the commands
  if (ftStatus != FT_OK)
  {
    char errorMessage[256];
    getErrorMessage(ftStatus, errorMessage);
    ROS_ERROR_STREAM("Error configuring SPI " << errorMessage);
    FT_SetBitMode(ftHandle, 0x0, 0x00);  // Reset the port to disable MPSSE
    FT_Close(ftHandle);                  // Close the USB port
    return false;
  }
  dwNumBytesToSend = 0;  // Clear output buffer
  usleep(100);           // Delay for 100us
  ROS_DEBUG("SPI initialisation successful");
  return true;
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
  for (; fst != end; ++fst)
  {
    auto word = WORD{ *fst };
    high = word >> 8;
    low = word & 0xff;
    message.write(ss);
  }
  return ss.str();
}

BOOL SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata)
{
  DWORD dwNumBytesSent;

  auto message = encode(&bdata, (&bdata) + 1);
  FT_STATUS ftStatus;
  ftStatus = FT_Write(ftHandle, &message.front(), message.size(), &dwNumBytesSent);
  // send out MPSSE command to MPSSE engine
  ROS_DEBUG_STREAM(dwNumBytesSent << " bytes sent through SPI");
  return ftStatus;
}

FT_STATUS SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords)
{
  DWORD dwNumBytesSent = 0;
  auto message = encode(buffer, buffer+numwords);

  FT_STATUS ftStatus = FT_OK;
  do
  {
    ftStatus = FT_Write(ftHandle, &message.front(), message.size(),
                        &dwNumBytesSent);  // send out MPSSE command to MPSSE engine
    if (ftStatus != FT_OK)
    {
      char errorMessage[256];
      getErrorMessage(ftStatus, errorMessage);
      ROS_ERROR_STREAM(" something wrong with FT_Write call, Error code " << errorMessage);
    }
  } while (ftStatus != FT_OK);
  return ftStatus;
}

static std::vector<std::string> errorMessages = { "FT_OK - it's all good",
                                                  "FT_INVALID_HANDLE",
                                                  "FT_DEVICE_NOT_FOUND",
                                                  "FT_DEVICE_NOT_OPENED",
                                                  "FT_IO_ERROR",
                                                  "FT_INSUFFICIENT_RESOURCES",
                                                  "FT_INVALID_PARAMETER",
                                                  "FT_INVALID_BAUD_RATE",
                                                  "FT_DEVICE_NOT_OPENED_FOR_ERASE",
                                                  "FT_DEVICE_NOT_OPENED_FOR_WRITE",
                                                  "FT_FAILED_TO_WRITE_DEVICE",
                                                  "FT_EEPROM_READ_FAILED",
                                                  "FT_EEPROM_WRITE_FAILED",
                                                  "FT_EEPROM_ERASE_FAILED",
                                                  "FT_EEPROM_NOT_PRESENT",
                                                  "FT_EEPROM_NOT_PROGRAMMED",
                                                  "FT_INVALID_ARGS",
                                                  "FT_NOT_SUPPORTED",
                                                  "FT_OTHER_ERROR",
                                                  "FT_DEVICE_LIST_NOT_READY" };

void getErrorMessage(FT_STATUS status, char* msg)
{
  snprintf(msg, 256, "%s", errorMessages[status].c_str());
}
