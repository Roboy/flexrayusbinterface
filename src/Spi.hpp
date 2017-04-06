#pragma once

#include <string>
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


FtResult Open(std::string serial_number, DWORD spi_clock_divisor, FT_HANDLE* ftHandle);
/**
 * Write a number of words to the SPI bus
 * @param ftHandle Handle of the device
 * @param buffer data
 * @param numwords number of words
 */
void SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);
