#pragma once

/*! \file FlexRayUSBInterface.h
 \brief motor control via Flexray
 
 This header declares functions for motor control via Flexray.
 */

#include <QObject>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QElapsedTimer>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include "CommunicationData.h"
#include "ftd2xx.h"

//declare for MPSSE command
const BYTE MSB_RISING_EDGE_CLOCK_BYTE_OUT = '\x10'; /*! \var int MSB_RISING_EDGE_CLOCK_BYTE_OUT */
const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_OUT = '\x11';	/*! \var int MSB_FALLING_EDGE_CLOCK_BYTE_OUT */
const BYTE MSB_RISING_EDGE_CLOCK_BIT_OUT = '\x12';  /*! \var int MSB_RISING_EDGE_CLOCK_BIT_OUT */
const BYTE MSB_FALLING_EDGE_CLOCK_BIT_OUT = '\x13'; /*! \var int MSB_FALLING_EDGE_CLOCK_BIT_OUT */
const BYTE MSB_RISING_EDGE_CLOCK_BYTE_IN = '\x20';  /*! \var int MSB_RISING_EDGE_CLOCK_BYTE_IN */
const BYTE MSB_RISING_EDGE_CLOCK_BIT_IN = '\x22';   /*! \var int MSB_RISING_EDGE_CLOCK_BIT_IN */
const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_IN = '\x24'; /*! \var int MSB_FALLING_EDGE_CLOCK_BYTE_IN */
const BYTE MSB_FALLING_EDGE_CLOCK_BIT_IN = '\x26';  /*! \var int MSB_FALLING_EDGE_CLOCK_BIT_IN */
const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE = '\x31';	/*! \var int MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE */
const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE = '\x34';	/*! \var int MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE */
const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BIT = '\x36';	/*! \var int MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BIT */
const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BIT = '\x33';	/*! \var int MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BIT */

/*! \def DATASETSIZE 
 * \brief number of words to exchange per SPI frame (taken from CommunicationData.h)
 */
#define DATASETSIZE NUM_SPI_FRAMES 
/*! \def NUMINTS 
 * \brief number of exchanges (for testing)
 */
#define NUMINTS 1000	
/*! \def USBOUTSIZE 
 * \brief size of USB out buffer in bytes (64 byte aligned)
 */
#define USBOUTSIZE (((DATASETSIZE*17)/64)+1)*64
/*! \def USBINSIZE 
 * \brief size of USB in buffer in bytes (64 byte aligned)
 */
#define USBINSIZE (((DATASETSIZE*2)/64)+1)*64

//for a fast 1 DOF system we only use 2 drivers with a 1ms control loop,
//otherwise the CAN bandwidth is not sufficient.

class FlexRayUSBInterface : public QObject
{
    Q_OBJECT
    
  public:
    /**
     * Constructor 
     * @param cycleTimeInMilliSeconds cycle time in milli-seconds
     */
    FlexRayUSBInterface(int cycleTimeInMilliSeconds);
    /**
     * closes FTDI connection 
     */
    void close();
    /**
     * send and receive FlexRay bus data.
     * @return (-1) FTDI USB interface not ready (1) success
     */
    int exchangeData();
    /**
     * FlexRayUSBInterface Destructor
     */
    virtual ~FlexRayUSBInterface();
    /**
     * get mode
     * @return private m_mode
     */
    int mode() const { return m_mode; }
    
  public slots:
    /**
     * nothing implemented yet
     */
    void testConnection(comsCommandFrame  * CommandFrame1);
    /**
     * closing FTDI USB to FlexRay Interface
     */
    void stop();
    /**
     * slot calls data exchange method
     * @param completeCommandFrames for ganglion 1,2,3,4,5,6
     *	@param ControlParams motor control parameters
     */
    void sendAllCommandFrames(comsCommandFrame * completeCommandFrames, control_Parameters_t * ControlParams);
    /**
     * slot calls data exchange method
     * @param CommandFrame1 for ganglion 1,2 and 3
     * @param CommandFrame1 for ganglion 4,5 and 6
     *	@param ControlParams motor control parameters
     */
    void sendCommandFrame( comsCommandFrame  * CommandFrame1, comsCommandFrame  * CommandFrame2,   control_Parameters_t * ControlParams);
    
    signals:
    /* signals that flexRay connection was successful
     * \brief emits pointer to complete ganglion Data, and a mask identify the
     *	ganglions that are actively broadcasting on the bus
     */
    void newGanglionDataReady(ganglionData_t *, unsigned short); 
    /* signals that flexRay is ready
     */
    void  FlexRayReady(bool isReady);
    
  private slots:
    /**
     * calls private method exchangeData()
     * @see exchangeData
     */
    void cyclicProcessor(void);
    
  private:
    void initSafeCommandFrame();
    /**
     * routine is used to enable slave
     * @param OutputBuffer points to output buffer
     * @param NumBytesToSend number of bytes to send
     * @return NumBytesToSend number of bytes to send
     */
    DWORD SPI_CSEnable(BYTE* OutputBuffer, DWORD* NumBytesToSend);
    /**
     * routine is used to disable slave
     * @param OutputBuffer points to output buffer
     * @param NumBytesToSend number of bytes to send
     * @param end (true) disable slave
     * @return NumBytesToSend number of bytes to send
     */
    DWORD SPI_CSDisable(BYTE* OutputBuffer, DWORD* NumBytesToSend, bool end);
    /**
     * find connected devices
     * @param NumDevs gets filled with number of devices
     * @return (true) if any devices found (false) if no devices found
     */
    bool CheckDeviceConnected(DWORD* NumDevs);
    /**
     * get device info
     * @param NumDevs devices to get info on
     * @return (true) if devices found (false) if devices not found
     */
    bool GetDeviceInfo(DWORD* NumDevs);
    /**
     * Open device and configure the MPSSE controller
     * @param ftHandle Handle of the device
     * @param InTransferSize  Transfer size for USB IN request.
     * @param OutTransferSize Transfer size for USB OUT request
     * @return (true) on success (false) on failure
     */
    bool OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle, DWORD InTransferSize, DWORD OutTransferSize);
    /**
     * test the MPSSE controller
     * @param ftHandle Handle of the device
     * @return (true) MPSSE synchronized (false) Error in synchronizing the MPSSE
     */
    bool TestMPSSE(FT_HANDLE* ftHandle);
    /**
     * Configure the MPSSE controller into an SPI module
     * @param ftHandle Handle of the device
     * @param dwClockDivisor 
     * @return (true) SPI initialization successful (false) Error configuring SPI
     */
    bool ConfigureSPI(FT_HANDLE* ftHandle, DWORD dwClockDivisor);
    /**
     * Send a single byte through the SPI bus
     * @param ftHandle Handle of the device
     * @param bdata Byte to be send
     * @return (true) success (false) failure
     */
    BOOL SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata);
    /**
     * Write a number of words to the SPI bus
     * @param ftHandle Handle of the device
     * @param buffer data
     * @param numwords number of words
     * @return FT_STATUS
     */
    FT_STATUS SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);
    
    //! Handle of the FTDI device
    FT_HANDLE ftHandle;
    //! Result of each D2XX call
    FT_STATUS ftStatus; 
    //! indication of FTDI ready
    int mFTDIReady; 
    
    //! communication data for USB SPI FLEXRAY stream
    WORD dataset[DATASETSIZE];	
    WORD InputBuffer[DATASETSIZE];
    
    //! upstream from ganglions to PC
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