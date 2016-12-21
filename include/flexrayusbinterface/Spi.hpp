#pragma once

#include "flexrayusbinterface/FtResult.hpp"

#define NUM_SPI_FRAMES 310
/*! \def DATASETSIZE
 * \brief number of words to exchange per SPI frame (taken from CommunicationData.h)
 */
#define DATASETSIZE NUM_SPI_FRAMES
/*! \def USBOUTSIZE
 * \brief size of USB out buffer in bytes (64 byte aligned)
 */
#define USBOUTSIZE (((DATASETSIZE * 17) / 64) + 1) * 64
/*! \def USBINSIZE
 * \brief size of USB in buffer in bytes (64 byte aligned)
 */
#define USBINSIZE (((DATASETSIZE * 2) / 64) + 1) * 64

/**
 * find connected devices
 * @param NumDevs gets filled with number of devices
 * @return (true) if any devices found (false) if no devices found
 */
FtResult CheckDeviceConnected(DWORD* NumDevs);
/**
 * get device info
 * @param NumDevs devices to get info on
 * @return (true) if devices found (false) if devices not found
 */
FtResult GetDeviceInfo(DWORD* NumDevs);
/**
 * Open device and configure the MPSSE controller
 * @param ftHandle Handle of the device
 * @param InTransferSize  Transfer size for USB IN request.
 * @param OutTransferSize Transfer size for USB OUT request
 * @return (true) on success (false) on failure
 */
FtResult OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle);
/**
 * test the MPSSE controller
 * @param ftHandle Handle of the device
 * @return (true) MPSSE synchronized (false) Error in synchronizing the MPSSE
 */
FtResult TestMPSSE(FT_HANDLE ftHandle);
/**
 * Configure the MPSSE controller into an SPI module
 * @param ftHandle Handle of the device
 * @param dwClockDivisor
 * @return (true) SPI initialization successful (false) Error configuring SPI
 */
FtResult ConfigureSPI(FT_HANDLE ftHandle, DWORD dwClockDivisor);
/**
 * Write a number of words to the SPI bus
 * @param ftHandle Handle of the device
 * @param buffer data
 * @param numwords number of words
 */
void SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);
