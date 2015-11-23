#include "FlexRayHardwareInterface.hpp"



int FlexRayHardwareInterface::exchangeData()
{
    DWORD dwNumInputBuffer=0;
    DWORD dwNumBytesRead;
    
    if (m_FTDIReady)
    {
	// SEND DATA
	m_ftStatus = SPI_WriteBuffer(m_ftHandle, &dataset[0], DATASETSIZE);		// send data
	if (m_ftStatus != FT_OK)
	{
	    LOG(ERROR) << "Failed to Send a byte through SPI, Error Code: " << m_ftStatus;
	    //FT_SetBitMode(ftHandle, 0x0, 0x00); 			// Reset the port to disable MPSSE
	    //FT_Close(ftHandle);					// Close the USB port
	}
	else
	{
            //std::cout<<"Data successfully sent via USB!"<<std::endl;
	    // WAIT FOR DATA TO ARRIVE
	    while(dwNumInputBuffer!=DATASETSIZE*2)
	    {
		m_ftStatus = FT_GetQueueStatus(m_ftHandle, &dwNumInputBuffer); // get the number of bytes in the device receive buffer
		//std::cout << t << ": " << dwNumInputBuffer << std::endl;
	    }
	    
	    // RECEIVE DATA
	    if(dwNumInputBuffer > DATASETSIZE*2)					// to prevent segfaults
		dwNumInputBuffer = DATASETSIZE*2;
	    FT_Read(m_ftHandle, &InputBuffer[0], dwNumInputBuffer, &dwNumBytesRead); 	// read bytes into word locations
	    
            dwNumInputBuffer = 0;							// reset the byte counter
	    //	std::cout<<"Data read!"<<std::endl;
	    
	    //now make a copy of relevant signal and emit signal with data
	    memcpy( GanglionData, InputBuffer, sizeof(GanglionData)); //ganglion data
	    
	    activeGanglionsMask=InputBuffer[sizeof(GanglionData)>>1];   //active ganglions, generated from usbFlexRay interface
	}
	return 1;
    }
    else
    {
	LOG(ERROR) << "FTDI USB interface not ready.";
	return -1;
    }
}

FT_STATUS FlexRayHardwareInterface::SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords)
{
    DWORD dwNumBytesSent=0;
    DWORD dwNumBytesToSend=0;
    BYTE OutputBuffer[USBOUTSIZE];
    FT_STATUS ftStatus; 			// Result of each D2XX call
    WORD bdata;
    
    // Bytes actually sent = data words*2 + 3 Commands*data words + 3 CSenable*data words + 6 CSdisable*data words => 14* Data words
    for(unsigned int i=0;i<numwords;i++)
    {
	bdata = buffer[i];
	dwNumBytesToSend = SPI_CSEnable(&OutputBuffer[0], &dwNumBytesToSend);
	OutputBuffer[dwNumBytesToSend++] = MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE;//MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE;//
	OutputBuffer[dwNumBytesToSend++] = 1;				// data length in bytes (low byte)  } +1!!
	OutputBuffer[dwNumBytesToSend++] = 0;				// data length in bytes (high byte) }
	OutputBuffer[dwNumBytesToSend++] = (BYTE)(bdata >> 8);		// output high byte
	OutputBuffer[dwNumBytesToSend++] = (BYTE)(bdata & 0xff);	// output low byte
	dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
    }
    //dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
    //printf("%i NumBytesToSend", sizeof(OutputBuffer));
    do
    {
	ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);	//send out MPSSE command to MPSSE engine
	if(ftStatus != FT_OK)
	    LOG(ERROR) << " something wrong with FT_Write call, Error code " << ftStatus;
	//else
	//	std::cout << dwNumBytesSent <<" bytes sent through SPI" << std::endl;
    }while(ftStatus!=FT_OK);
    
    dwNumBytesToSend = 0;								//Clear output buffer
    return ftStatus;
}

DWORD FlexRayHardwareInterface::SPI_CSEnable(BYTE* OutputBuffer, DWORD* NumBytesToSend)
{
    DWORD dwNumBytesToSend;
    dwNumBytesToSend = *NumBytesToSend;
    
    for(int loop=0;loop<2;loop++) //one 0x80 command can keep 0.2us, do 5 times to stay in this situation for 1us
    {
	OutputBuffer[dwNumBytesToSend++] = '\x80';	// GPIO command for ADBUS
	OutputBuffer[dwNumBytesToSend++] = '\x00';	// set CS, MOSI and SCL low
	OutputBuffer[dwNumBytesToSend++] = '\x0b';	// bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }
    return dwNumBytesToSend;
}

DWORD FlexRayHardwareInterface::SPI_CSDisable(BYTE* OutputBuffer, DWORD* NumBytesToSend, bool end)
{
    DWORD dwNumBytesToSend;
    dwNumBytesToSend = *NumBytesToSend;
    
    // one 0x80 command can keep 0.2us, do 5 times to stay in this situation for 1us (leave CS low)
    //for(int loop=0;loop<2;loop++)
    {
	OutputBuffer[dwNumBytesToSend++] = '\x80';	// GPIO command for ADBUS
	OutputBuffer[dwNumBytesToSend++] = '\x00';	// set CS, MOSI and SCL low
	OutputBuffer[dwNumBytesToSend++] = '\x0b';	// bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }
    
    if(end==true)
    {
	// finally pull CS high
	OutputBuffer[dwNumBytesToSend++] = '\x80';	// GPIO command for ADBUS
	OutputBuffer[dwNumBytesToSend++] = '\x08';	// set CS high, MOSI and SCL low
	OutputBuffer[dwNumBytesToSend++] = '\x0b';	// bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }
    
    return dwNumBytesToSend;
}

bool FlexRayHardwareInterface::CheckDeviceConnected(DWORD* NumDevs)
{
    FT_STATUS ftStatus;
    
    // -----------------------------------------------------------
    // Does an FTDI device exist?
    // -----------------------------------------------------------
    LOG(INFO) << "Checking for FTDI devices...";
    
    ftStatus = FT_CreateDeviceInfoList(NumDevs);				// Get the number of FTDI devices
    if (ftStatus != FT_OK)							// Did the command execute OK?
    {
	LOG(ERROR) << "Error in getting the number of devices, Error code: " << ftStatus;
	return false;								// Exit with error
    }
    if (*NumDevs < 1)								// Exit if we don't see any
    {
	LOG(WARNING) << "There are no FTDI devices installed";
	return false;								// Exit with error
    }
    LOG(INFO) << *NumDevs << " FTDI devices found-the count includes individual ports on a single chip";
    return true;
}

bool FlexRayHardwareInterface::GetDeviceInfo(DWORD* NumDevs)
{
    FT_STATUS ftStatus;
    FT_DEVICE_LIST_INFO_NODE *devInfo;
    
    // ------------------------------------------------------------
    // If yes then print details of devices
    // ------------------------------------------------------------
    devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*(*NumDevs));
    ftStatus = FT_GetDeviceInfoList(devInfo,NumDevs);
    if (ftStatus == FT_OK)
    {
	for (unsigned int i = 0; i < *NumDevs; i++)
	{
	    LOG(INFO) << " Dev: " << i;
	    LOG(INFO) << " Flags=0x" << devInfo[i].Flags;
	    LOG(INFO) << " Type=0x" << devInfo[i].Type;
	    LOG(INFO) << " ID=0x" << devInfo[i].ID;
	    LOG(INFO) << " LocId=0x" << devInfo[i].LocId;
	    LOG(INFO) << " SerialNumber=" << devInfo[i].SerialNumber;
	    LOG(INFO) << " Description=" << devInfo[i].Description;
	    LOG(INFO) << " ftHandle=0x" << devInfo[i].ftHandle;
	}
	return true;
    }
    else
    {
	LOG(WARNING) << "Could not find info for devices";
	return false;								// return with error
    }
}

bool FlexRayHardwareInterface::OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle, DWORD InTransferSize, DWORD OutTransferSize)
{
    FT_STATUS ftStatus;
    DWORD dwNumBytesToRead, dwNumBytesRead;
    BYTE byInputBuffer[DATASETSIZE*2]; 			// Local buffer to hold data read from the FT2232H
    
    // -----------------------------------------------------------
    // Open the port on first device located
    // -----------------------------------------------------------
    ftStatus = FT_Open(0, ftHandle);
    if (ftStatus != FT_OK)
    {
	LOG(ERROR) << "Open Failed with error " << ftStatus;
	return false;								// Exit with error
    }
    else
	LOG(INFO) << "Port opened";
    
    // ------------------------------------------------------------
    // Configure MPSSE and test for synchronisation
    // ------------------------------------------------------------
    
    // Configure port parameters
    LOG(INFO) << "Configuring port for MPSSE use";
    ftStatus |= FT_ResetDevice(*ftHandle);				//Reset USB device
    ftStatus |= FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead); 	// Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
    // Get the number of bytes in the FT2232H receive buffer
    if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))		//Read out the data from FT2232H receive buffer if not empty
	FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
    else
	LOG(WARNING) << "Buffer empty";
    
    //ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535); 	// Set USB request transfer sizes to 64K
    ftStatus |= FT_SetUSBParameters(*ftHandle, InTransferSize, OutTransferSize); 	// Set USB request transfer sizes...page 73 d2XX programmer guide (only dwInTransferSize has been implemented)
    /*	FT_STATUS FT_SetUSBParameters(FT_HANDLE ftHandle, DWORD dwInTransferSize, DWORD dwOutTransferSize)
     (only dwInTransferSize currently supported..test this...see d2XX programmers guide p 73)
     */
    ftStatus |= FT_SetChars(*ftHandle, false, 0, false, 0);		// Disable event and error characters
    /*	FT_STATUS FT_SetChars (FT_HANDLE ftHandle, UCHAR uEventCh, UCHAR uEventChEn,UCHAR uErrorCh, UCHAR uErrorChEn)
     (uEventCh = event character, uEventChEn = enable event character insertion, uErrorCh = error character, ...)
     */
    //ftStatus |= FT_SetTimeouts(ftHandle, 3000, 3000);			// Sets the read and write timeouts in milliseconds
    ftStatus |= FT_SetTimeouts(*ftHandle, 300, 300);			// Sets the read and write timeouts in milliseconds
    /*	FT_STATUS FT_SetTimeouts (FT_HANDLE ftHandle, DWORD dwReadTimeout, DWORD dwWriteTimeout)
     (dwReadTimeout, dwWriteTimeout in ms)
     */
    ftStatus |= FT_SetLatencyTimer(*ftHandle, 1);				// Set the latency timer to 1mS (default is 16mS)
    /*	FT_STATUS FT_SetLatencyTimer (FT_HANDLE ftHandle, UCHAR ucTimer)
     (ucTimer = require latency value in milliseconds [range of 2-255 page 68 of d2XX programmers guide?])
     */
    //ftStatus |= FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x00, 0x00);
    //Turn on flow control to synchronize IN requests
    /*	FT_STATUS FT_SetFlowControl (FT_HANDLE ftHandle, USHORT usFlowControl, UCHAR uXon, UCHAR uXoff)
     (usFlowControl must be either FT_FLOW_NONE, FT_FLOW_RTS_CTS, FT_FLOW_DTR_DSR or FT_FLOW_XON_XOFF)
     (uXon = character to signal Xon [Only used is set to FT_FLOW_XON_XOFF])
     (uXoff = character to signal Xoff [Only used is set to FT_FLOW_XON_XOFF])
     */
    ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x00);				//Reset controller
    /*	FT_STATUS FT_SetBitmode (FT_HANDLE ftHandle, UCHAR ucMask, UCHAR ucMode)
     (ucMask sets which bits are inputs [=0] or outputs [=1])
     (ucMode can one of the following:
     0x0 = Reset
     0x1 = Asynchronous Bit Bang
     0x2 = MPSSE (FT2232, FT2232H, FT4232H and FT232H devices only)
     0x4 = Synchronous Bit Bang (FT232R, FT245R, FT2232, FT2232H, FT4232H and FT232H devices only)
     0x8 = MCU Host Bus Emulation Mode (FT2232, FT2232H, FT4232H and FT232H devices only)
     0x10 = Fast Opto-Isolated Serial Mode (FT2232, FT2232H, FT4232H and FT232H devices only)
     0x20 = CBUS Bit Bang Mode (FT232R and FT232H devices only)
     0x40 = Single Channel Synchronous 245 FIFO Mode (FT2232H and FT232H devices only)
     )
     
     */
    ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x02);				//Enable MPSSE mode
    if (ftStatus != FT_OK)
    {
	LOG(ERROR) << "Error in initializing the MPSSE on device. Error: " << ftStatus;
	FT_Close(ftHandle);
	return false;											// Exit with error
    }
    usleep(1); 													// Wait for all the USB stuff to complete and work
    
    LOG(INFO) << "MPSSE ready for commands";
    return true;
}

bool FlexRayHardwareInterface::TestMPSSE(FT_HANDLE* ftHandle)
{
    FT_STATUS ftStatus;
    BYTE byOutputBuffer[DATASETSIZE*2];
    BYTE byInputBuffer[DATASETSIZE*2];
    DWORD dwNumBytesToSend = 0;
    DWORD dwNumBytesSent, dwNumBytesRead, dwNumBytesToRead;
    
    /*
     *	Now the MPSSE is ready for commands, each command consists of an op-code followed by any necessary parameters or
     *	data. Each op-code is sent using FT_Write call
     */
    
    /* Synchronisation and Bad communication detection */
    // Enable internal loop-back
    byOutputBuffer[dwNumBytesToSend++] = 0x84;						// Enable loopback
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); 	// Send off the loopback command
    dwNumBytesToSend = 0; 									// Reset output buffer pointer
    
    // Check the receive buffer - it should be empty
    ftStatus = FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead); 				// Get the number of bytes in the FT2232H receive buffer
    if (dwNumBytesToRead != 0)
    {
	LOG(ERROR) << "Error - MPSSE receive buffer should be empty, Error Code: " << ftStatus;
	FT_SetBitMode(*ftHandle, 0x0, 0x00); 					// Reset the port to disable MPSSE
	FT_Close(*ftHandle);							// Close the USB port
	return false;								// Exit with error
    }
    else
	LOG(DEBUG) <<  "Internal loop-back configured and receive buffer is empty";
    
    // send bad op-code to check every thing is working correctly
    byOutputBuffer[dwNumBytesToSend++] = 0xAB;						//Add bogus command ‘0xAB’ to the queue
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); 	// Send off the BAD command
    dwNumBytesToSend = 0;									// Reset output buffer pointer
    
    do
    {
	ftStatus = FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead);		// Get the number of bytes in the device input buffer
    } while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));			// wait for bytes to return, an error or Timeout
    bool bCommandEchod = false;
    ftStatus = FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); 	// Read the data from input buffer
    for (DWORD dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)			//Check if Bad command and echo command are received
    {
	if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount+1] == 0xAB))
	{
	    bCommandEchod = true;
	    break;
	}
    }
    
    if (bCommandEchod == false)
    {
	LOG(ERROR) << "Error in synchronizing the MPSSE";
	FT_SetBitMode(*ftHandle, 0x0, 0x00); 					// Reset the port to disable MPSSE
	FT_Close(*ftHandle);							// Close the USB port
	return false;								// Exit with error
    }
    else
    {
	LOG(DEBUG) << "MPSSE synchronised.";
	byOutputBuffer[dwNumBytesToSend++] = '\x85';  						// Command to turn off loop back of TDI/TDO connection
	ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); 	// Send off the loopback command
	dwNumBytesToSend = 0; 									// Reset output buffer pointer
	return true;
    }
    
}

bool FlexRayHardwareInterface::ConfigureSPI(FT_HANDLE* ftHandle, DWORD dwClockDivisor)
{
    FT_STATUS ftStatus;
    BYTE byOutputBuffer[DATASETSIZE*2];
    DWORD dwNumBytesToSend = 0;
    DWORD dwNumBytesSent;
    
    // ------------------------------------------------------------
    // Configure MPSSE for SPI communications
    // ------------------------------------------------------------
    // The SK clock frequency can be worked out as follows with divide by 5 set as off:: SK frequency = 60MHz /((1 + [(1 + (0xValueH*256) OR 0xValueL])*2)
    
    byOutputBuffer[dwNumBytesToSend++] = '\x8A'; 						// Ensure disable clock divide by 5 for 60Mhz master clock
    byOutputBuffer[dwNumBytesToSend++] = '\x97';						// Ensure turn off adaptive clocking
    byOutputBuffer[dwNumBytesToSend++] = '\x8D'; 						// disable 3 phase data clock
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); 	// Send out the commands
    if(ftStatus!=FT_OK)
    {
	LOG(ERROR) << "Error configuring SPI, Error code " << ftStatus;
	FT_SetBitMode(*ftHandle, 0x0, 0x00); 						// Reset the port to disable MPSSE
	FT_Close(*ftHandle);										// Close the USB port
	return false;
    }
    
    dwNumBytesToSend = 0;												// Clear output buffer
    byOutputBuffer[dwNumBytesToSend++] = '\x80'; 						// Set directions of lower 8 pins, set value on bits set as output
    byOutputBuffer[dwNumBytesToSend++] = '\x08'; 						// Set SCK, DO, DI low and CS high
    byOutputBuffer[dwNumBytesToSend++] = '\x0b'; 						// Set SK,DO,GPIOL0 pins as output =1, other pins as input=0
    byOutputBuffer[dwNumBytesToSend++] = '\x86'; 						// Command to set clock divisor
    byOutputBuffer[dwNumBytesToSend++] = (BYTE)(dwClockDivisor & '\xFF'); 			// Set 0xValueL of clock divisor
    byOutputBuffer[dwNumBytesToSend++] = (BYTE)(dwClockDivisor >> 8); 			// Set 0xValueH of clock divisor
    byOutputBuffer[dwNumBytesToSend++] = '\x85';  						// Command to turn off loop back of TDI/TDO connection
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);	// Send out the commands
    if(ftStatus!=FT_OK)
    {
	LOG(ERROR) << "Error configuring SPI" << ftStatus;
	FT_SetBitMode(*ftHandle, 0x0, 0x00); 					// Reset the port to disable MPSSE
	FT_Close(*ftHandle);							// Close the USB port
	return false;
    }
    dwNumBytesToSend = 0;									// Clear output buffer
    usleep(100);										// Delay for 100us
    LOG(INFO) <<  "SPI initialisation successful";
    return true;
}

BOOL FlexRayHardwareInterface::SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata)
{
    DWORD dwNumBytesSent;
    DWORD dwNumBytesToSend=0;
    BYTE OutputBuffer[512];
    FT_STATUS ftStatus; 			// Result of each D2XX call
    
    dwNumBytesToSend = SPI_CSEnable(&OutputBuffer[0], &dwNumBytesToSend);
    OutputBuffer[dwNumBytesToSend++] = MSB_FALLING_EDGE_CLOCK_BYTE_OUT;	// Byte out falling edge msb first
    OutputBuffer[dwNumBytesToSend++] = 1;					// data length in bytes (low byte)  } +1!!
    OutputBuffer[dwNumBytesToSend++] = 0;					// data length in bytes (high byte) }
    OutputBuffer[dwNumBytesToSend++] = bdata >> 8;		//output high byte
    OutputBuffer[dwNumBytesToSend++] = bdata & 0xff;	//output low byte
    dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
    ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
    //send out MPSSE command to MPSSE engine
    LOG(DEBUG) << dwNumBytesSent << " bytes sent through SPI";
    return ftStatus;
}