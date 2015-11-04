/*
 * flexRayUSBInterface.h
 *
 *  Created on: 17 Dec 2013
 *      Author: alex
 */

#ifndef _FLEXRAYUSBINTERFACE_H_
#define _FLEXRAYUSBINTERFACE_H_

#include <QObject>

#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QMap>
#include <QFile>

#include <stdint.h>


#include <QElapsedTimer>

#include "CommunicationData.h" //TODO make this proper link into myode project
#include "ftd2xx.h"

//declare for MPSSE command
const BYTE MSB_RISING_EDGE_CLOCK_BYTE_OUT = '\x10';
const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_OUT = '\x11';
const BYTE MSB_RISING_EDGE_CLOCK_BIT_OUT = '\x12';
const BYTE MSB_FALLING_EDGE_CLOCK_BIT_OUT = '\x13';
const BYTE MSB_RISING_EDGE_CLOCK_BYTE_IN = '\x20';
const BYTE MSB_RISING_EDGE_CLOCK_BIT_IN = '\x22';
const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_IN = '\x24';
const BYTE MSB_FALLING_EDGE_CLOCK_BIT_IN = '\x26';
const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE = '\x31';
const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE = '\x34';
const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BIT = '\x36';
const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BIT = '\x33';

#define DATASETSIZE 	NUM_SPI_FRAMES				// number of words to exchange per SPI frame (taken from CommunicationData.h)
#define NUMINTS  	1000							// number of exchanges (for testing)
#define USBOUTSIZE 	(((DATASETSIZE*17)/64)+1)*64	// size of USB out buffer in bytes (64 byte aligned)
#define USBINSIZE (((DATASETSIZE*2)/64)+1)*64		// size of USB in buffer in bytes (64 byte aligned)

DWORD SPI_CSEnable(BYTE* OutputBuffer, DWORD* NumBytesToSend);
DWORD SPI_CSDisable(BYTE* OutputBuffer, DWORD* NumBytesToSend, bool end);
bool CheckDeviceConnected(DWORD* NumDevs);
bool GetDeviceInfo(DWORD* NumDevs);
bool OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle, DWORD InTransferSize, DWORD OutTransferSize);
bool TestMPSSE(FT_HANDLE* ftHandle);
bool ConfigureSPI(FT_HANDLE* ftHandle, DWORD dwClockDivisor);
BOOL SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata);
FT_STATUS SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);


//for a fast 1 DOF system we only use 2 drivers with a 1ms control loop,
//otherwise the CAN bandwidth is not sufficient.

namespace eu
{
namespace myode
{
namespace myorobot
{


class FlexRayUSBInterface : public QObject
{
     Q_OBJECT

 public:
     FlexRayUSBInterface(int cycleTimeInMilliSeconds);
     void close();
     int exchangeData();
     virtual ~FlexRayUSBInterface();
     int mode() const { return m_mode; }

     //void handleRxData(canNotifyData * rxNotifyData);
 public slots:

    // void start();

 	 void testConnection(comsCommandFrame  * CommandFrame1);
 	 void stop();
 	 void sendAllCommandFrames(comsCommandFrame * completeCommandFrames, control_Parameters_t * ControlParams);
 	 void sendCommandFrame( comsCommandFrame  * CommandFrame1, comsCommandFrame  * CommandFrame2,   control_Parameters_t * ControlParams);
     //void detachflexRayBus();
     //void getReference(float ref);
    // void enableLogging (bool enabled);

 	 signals:
 	 void newGanglionDataReady(ganglionData_t *, unsigned short); //emits pointer to complete ganglion Data, and a mask identify the
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 //ganglions that are actively broadcasting on the bus
 	 /* @brief signals that flexRay connection was successful
 	  *
 	  */
 	 void  FlexRayReady(bool isReady);

 private slots:
     void cyclicProcessor(void);

 private:
     FT_HANDLE ftHandle; 			// Handle of the FTDI device
 	 FT_STATUS ftStatus; 			// Result of each D2XX call
     int mFTDIReady;

     void initSafeCommandFrame();
     DWORD SPI_CSEnable(BYTE* OutputBuffer, DWORD* NumBytesToSend);
     DWORD SPI_CSDisable(BYTE* OutputBuffer, DWORD* NumBytesToSend, bool end);
     bool CheckDeviceConnected(DWORD* NumDevs);
     bool GetDeviceInfo(DWORD* NumDevs);
     bool OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle, DWORD InTransferSize, DWORD OutTransferSize);
     bool TestMPSSE(FT_HANDLE* ftHandle);
     bool ConfigureSPI(FT_HANDLE* ftHandle, DWORD dwClockDivisor);
     BOOL SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata);
     FT_STATUS SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);

     //communication data for USB SPI FLEXRAY stream
     WORD dataset[DATASETSIZE];
     WORD InputBuffer[DATASETSIZE];

     //upstream from ganglions to PC

     ganglionData_t GanglionData[NUMBER_OF_GANGLIONS];
     unsigned short activeGanglionsMask;


     QMutex dataExchangeMutex;

     unsigned int m_cycleCount;
     int mNewSendDataReady;
     int mCycleTime;
     int m_mode;
     int count;
     QThread *timerThread;
    // QThread  *userInterfaceThread;
     qint64 nanoSec;


 };

}}}

#endif /*_FLEXRAYUSBINTERFACE_H_*/
