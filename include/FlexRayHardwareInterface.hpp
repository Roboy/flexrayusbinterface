/* 
 * File:   flexRayCommunication.hpp
 * Author: letrend
 *
 * Created on November 18, 2015, 4:04 PM
 */

#pragma once

// comment if no hardware available
#define HARDWARE

#include <ftd2xx.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "CommunicationData.h"
#include <ros/console.h>

#define NUM_SPI_FRAMES 310
/*! \def DATASETSIZE 
 * \brief number of words to exchange per SPI frame (taken from CommunicationData.h)
 */
#define DATASETSIZE NUM_SPI_FRAMES 
/*! \def USBOUTSIZE 
 * \brief size of USB out buffer in bytes (64 byte aligned)
 */
#define USBOUTSIZE (((DATASETSIZE*17)/64)+1)*64
/*! \def USBINSIZE 
 * \brief size of USB in buffer in bytes (64 byte aligned)
 */
#define USBINSIZE (((DATASETSIZE*2)/64)+1)*64

class FlexRayHardwareInterface{
public:
    /**
     * Constructor 
     */
    FlexRayHardwareInterface();
    /**
     * connect to flexray
     * @return status
     */
    bool connect();
    /**
     * use this to initialize the motors
     */
    void initializeMotors();
    
    /**
     * This function initializes the position controller
     * @param Pgain
     * @param IGain
     * @param Dgain
     * @param forwardGain
     * @param deadBand
     * @param integral
     * @param IntegralPosMin
     * @param IntegralPosMax
     * @param spPosMin
     * @param spPosMax
     */
    void initPositionControl(float Pgain=10000.0, float IGain=0.0, float Dgain=0.0, float forwardGain=0.0, 
    float deadBand=0.0, float integral=0.0, float IntegralPosMin=0.0, float IntegralPosMax=0.0, 
    float spPosMin=-100.0, float spPosMax=100.0);
    
    /**
     * This function initializes the velocity controller
     * @param Pgain
     * @param IGain
     * @param Dgain
     * @param forwardGain
     * @param deadBand
     * @param integral
     * @param IntegralPosMin
     * @param IntegralPosMax
     * @param spPosMin
     * @param spPosMax
     */
    void initVelocityControl(float Pgain=100.0, float IGain=0.0, float Dgain=0.0, float forwardGain=0.0, 
    float deadBand=0.0, float integral=0.0, float IntegralPosMin=0.0, float IntegralPosMax=0.0, 
    float spPosMin=-100.0, float spPosMax=100.0);
    
    /**
     * This function initializes the force controller
     * @param Pgain
     * @param IGain
     * @param Dgain
     * @param forwardGain
     * @param deadBand
     * @param integral
     * @param IntegralPosMin
     * @param IntegralPosMax
     * @param spPosMin
     * @param spPosMax
     */
    void initForceControl(float Pgain=100.0, float IGain=0.0, float Dgain=0.0, float forwardGain=0.0, 
    float deadBand=0.0, float integral=0.0, float IntegralPosMin=0.0, float IntegralPosMax=0.0, 
    float spPosMin=-100.0, float spPosMax=100.0, float torqueConstant=1.0 , char springType=SoftSpring);
        
    /**
     * This function exchanges data between interface and motors
     */
    void exchangeData();
    
    /**
     * This function updates the commandframes
     */
    void updateCommandFrame();
    
    /**
     * This function checks the number of ones set in a bitmask
     * @param i bitmask
     * @return number of ones set
     */
    uint32_t NumberOfSetBits(uint32_t i);
    
    uint32_t checkNumberOfConnectedGanglions();

    //! upstream from ganglions to PC
    ganglionData_t GanglionData[NUMBER_OF_GANGLIONS]; 
    //! bitmask with active ganglions
    unsigned short activeGanglionsMask;
    //! number of connected ganglions
    uint32_t numberOfGanglionsConnected;
    //! command frames containing motor control parameters for 3 ganglia, respectively
    comsCommandFrame commandframe0[3], commandframe1[3];
    //! control parameters for motor [initialization, run, disable]
    control_Parameters_t controlparams;
    //! Result of each D2XX call
    FT_STATUS ftStatus; 
private:
    //! Handle of the FTDI device
    FT_HANDLE m_ftHandle;
    //! number of devices connected
    uint m_numberOfConnectedDevices;
    //! Value of clock divisor, SCL Frequency = 60/((1+value)*2) = MHz i.e., value of 2 = 10MHz, or 29 = 1Mhz
    uint m_clockDivisor = 2;	
    //! FTDI ready
    bool m_FTDIReady;
    //! this will be send via flexray
    WORD dataset[DATASETSIZE];	
    //! this will contain data coming from flexray
    WORD InputBuffer[DATASETSIZE];	
    
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
    
    // necessary for MPSSE command
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
    
    void getErrorMessage(FT_STATUS status, char* msg){
        snprintf(msg,256,"%s", errorMessages[status].c_str());
    }
    char errorMessage[256];
    std::vector<std::string> errorMessages = {
        "FT_OK - it's all good",
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
	"FT_DEVICE_LIST_NOT_READY"
    };
    
    struct defaultParams{
        sint32 tag = 0;			// sint32
        sint32 outputPosMax = 1000;	// sint32			
        sint32 outputNegMax = -1000;	// sint32
        float32 spPosMax = 100.0;	// float32
        float32 spNegMax = -100.0;	// float32
        float32 timePeriod = 100;		// float32		//in us	// set time period to avoid error case
        float32 radPerEncoderCount = 2*3.14159265359/(2000.0*53.0);	// float32
        float32 polyPar[4] = {0,1,0,0};		// float32
        float32 torqueConstant = 1.0;	// float32

        float32 integral = 0;           // float32
        float32 pgain = 10000.0;	// float32
        float32 igain = 0.0;            // float32
        float32 dgain = 0.0;            // float32
        float32 forwardGain = 0;	// float32
        float32 deadBand = 0;           // float32
        float32 lastError = 0;          // float32
        float32 IntegralPosMax = 0;	// float32
        float32 IntegralNegMax = 0;	// float32
    };
    
    enum{
        SoftSpring,
        MiddleSpring,
        HardSpring
    };
};

