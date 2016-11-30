#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <string.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <utility>

// ros
#include <ros/console.h>

FlexRayHardwareInterface::FlexRayHardwareInterface()
{
  motorState.fill(1);
  ROS_INFO("Trying to connect to FlexRay");
  while (!connect())
  {
    ROS_INFO("retrying...");
    usleep(1000000);  // sleep for a second
  }
  command.params.tag = 0;               // sint32
  command.params.outputPosMax = 1000;   // sint32
  command.params.outputNegMax = -1000;  // sint32
  command.params.timePeriod = 100;      // float32      //in us set time period to avoid error case

  command.params.radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);  // float32
  command.params.params.pidParameters.lastError = 0;                        // float32

  for (auto& frame : command.frame)
  {
    for (uint32_t j = 0; j < 4; j++)
    {
      frame.sp[j] = 0;
    }
  }
  initForceControl();

  auto ganglions = exchangeData();
  ROS_INFO_STREAM(ganglions.count() << " ganglions are connected via flexray, activeGanglionsMask " << ganglions);
};

bool FlexRayHardwareInterface::connect()
{
  //! number of devices connected
  uint32_t numberOfConnectedDevices;

  if (CheckDeviceConnected(&numberOfConnectedDevices) == true)
  {
    if (GetDeviceInfo(&numberOfConnectedDevices) == true)
    {
      if (OpenPortAndConfigureMPSSE(&m_ftHandle, USBINSIZE, USBOUTSIZE) == true)
      {
        bool mpssetested = false;
        uint32_t tries = 0;
        do
        {
          mpssetested = TestMPSSE(&m_ftHandle);
          if (mpssetested)
          {
            bool spiconfigured = false;
            do
            {
              //! Value of clock divisor, SCL Frequency = 60/((1+value)*2) = MHz i.e., value of 2 = 10MHz, or 29 = 1Mhz
              constexpr uint32_t clockDivisor = 2;
              spiconfigured = ConfigureSPI(&m_ftHandle, clockDivisor);
            } while (spiconfigured == false);
            ROS_INFO("connection OK");
            return true;
          }
          ROS_INFO("Testing MPSSE failed, retrying!");
          tries++;
        } while (mpssetested == false && tries < 3);  // TestMPSSE
      }                                               // OpenPortAndConfigureMPSSE
      else
      {
        ROS_ERROR("open port failed\n--Perhaps the kernel automatically loaded "
                  "another driver for the FTDI USB device, from command line "
                  "try: \nsudo rmmod ftdi_sio \n sudo rmmod usbserial\n or "
                  "maybe run with sudo");
      }
    }  // GetDeviceInfo
    else
    {
      ROS_ERROR("device info failed");
    }
  }  // CheckDeviceConnected
  else
  {
    ROS_ERROR("device not connected");
  }
  return false;
};

void FlexRayHardwareInterface::relaxSpring(uint32_t ganglion_id, uint32_t motor_id, int controlmode)
{
  initVelocityControl(ganglion_id, motor_id);
  uint32_t count = 0;
  float aux = 0;
  uint32_t p = 0;
  float vel;
  float tendonDisplacement_t[3];
  float tendonDisplacement_t2[10000];

  vel = 0;
  command.frame[ganglion_id].sp[motor_id] = vel;
  exchangeData();
  usleep(300000);
  exchangeData();
  tendonDisplacement_t[0] =
      GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;  // tendon displacemnte iniziale
  ROS_INFO("Displacement_t0 %.5f     ", tendonDisplacement_t[0]);
  uint32_t i = 1;
  for (i = 1; i < 3; i++)
  {
    vel = 3;
    command.frame[ganglion_id].sp[motor_id] = vel;
    exchangeData();
    usleep(500000);
    exchangeData();
    tendonDisplacement_t[i] = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
    ROS_INFO("Displacement_t%d %.5f ", i, tendonDisplacement_t[i]);
  }

  uint32_t t = 0;
  tendonDisplacement_t2[0] = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
  ROS_INFO("Displacement_t20 %.5f ", tendonDisplacement_t2[t]);

  if (tendonDisplacement_t[2] < tendonDisplacement_t[0])
    vel = 3;
  else if (tendonDisplacement_t[1] < tendonDisplacement_t[2])
    vel = -3;
  else
    vel = 0;

  do
  {
    command.frame[ganglion_id].sp[motor_id] = vel;
    exchangeData();
    usleep(300000);
    exchangeData();
    tendonDisplacement_t2[t + 1] = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
    t++;
    ROS_INFO("Displacement_t2 %d %.5f ", t, tendonDisplacement_t2[t]);

    if (tendonDisplacement_t2[t] > tendonDisplacement_t2[t - 1])
      p++;

    else if (tendonDisplacement_t2[t] == tendonDisplacement_t2[t - 1])
    {
      if (count > 0 && tendonDisplacement_t2[t] == aux)
        count++;
      else if (count == 0)
      {
        aux = tendonDisplacement_t2[t];
        count++;
      }
      else
        count = 0;
    }

  } while (count != 4 && p != 3);

  if (p == 3)
    relaxSpring(ganglion_id, motor_id, controlmode);

  else
  {
    vel = 0;
    command.frame[ganglion_id].sp[motor_id] = vel;
    exchangeData();
    sleep(2);

    command.params.tag = 1;
    command.frame[0].OperationMode[0] = Initialise;
    ROS_INFO("Tag = %d ", command.params.tag);

    exchangeData();
    command.params.tag = 0;
    command.frame[0].OperationMode[0] = Initialise;
    ROS_INFO("Tag = %d ", command.params.tag);
    exchangeData();

    switch (controlmode)
    {
      case 1:
        initPositionControl(ganglion_id, motor_id);
        break;
      case 2:
        initVelocityControl(ganglion_id, motor_id);
        break;
      case 3:
        initForceControl(ganglion_id, motor_id);
        break;
    }
  }
}

void FlexRayHardwareInterface::initPositionControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                   float deadBand, float integral, float IntegralPosMin,
                                                   float IntegralPosMax, float spPosMin, float spPosMax)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].ControlMode[motor] = Position;
      command.frame[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
    }
  }
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].OperationMode[motor] = Run;
      float pos = GanglionData[i].muscleState[motor].actuatorPos;
      command.frame[i].sp[motor] = pos * command.params.radPerEncoderCount;
    }
  }
  exchangeData();
}

void FlexRayHardwareInterface::initPositionControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  command.frame[ganglion].ControlMode[motor] = Position;
  command.frame[ganglion].OperationMode[motor] = Initialise;
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
  exchangeData();
  float pos = GanglionData[ganglion].muscleState[motor].actuatorPos * command.params.radPerEncoderCount;
  command.frame[ganglion].OperationMode[motor] = Run;
  command.frame[ganglion].sp[motor] = pos;
  exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                   float deadBand, float integral, float IntegralPosMin,
                                                   float IntegralPosMax, float spPosMin, float spPosMax)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].ControlMode[motor] = Velocity;
      command.frame[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
    }
  }
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].OperationMode[motor] = Run;
      command.frame[i].sp[motor] = 0;
    }
  }
  exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  command.frame[ganglion].ControlMode[motor] = Velocity;
  command.frame[ganglion].OperationMode[motor] = Initialise;
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
  exchangeData();
  command.frame[ganglion].OperationMode[motor] = Run;
  command.frame[ganglion].sp[motor] = 0;
  exchangeData();
}

void FlexRayHardwareInterface::initForceControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                float deadBand, float integral, float IntegralPosMin,
                                                float IntegralPosMax, float spPosMin, float spPosMax,
                                                float torqueConstant, SpringElasticity springType)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  float polyPar[4];

  switch (springType)
  {
    // f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4
    case SpringElasticity::Soft:  // D311-spring 0.621249
      polyPar[0] = 0;
      polyPar[1] = 0.237536;
      polyPar[2] = -0.000032;
      polyPar[3] = 0;
      break;
    case SpringElasticity::Medium:
      polyPar[0] = 1.604382;
      polyPar[1] = 0.508932;
      polyPar[2] = -0.000117;
      polyPar[3] = 0;
      break;
    case SpringElasticity::Hard:
      polyPar[0] = 1.186483;
      polyPar[1] = 0.938377;
      polyPar[2] = -0.000274;
      polyPar[3] = 0.000001;
      break;
  }

  command.params.polyPar[0] = polyPar[0];  // float32
  command.params.polyPar[1] = polyPar[1];
  command.params.polyPar[2] = polyPar[2];
  command.params.polyPar[3] = polyPar[3];
  command.params.torqueConstant = torqueConstant;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].ControlMode[motor] = Force;
      command.frame[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;
    }
  }
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      command.frame[i].OperationMode[motor] = Run;
      command.frame[i].sp[motor] = 0;
    }
  }
  exchangeData();
}

void FlexRayHardwareInterface::initForceControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                float Dgain, float forwardGain, float deadBand, float integral,
                                                float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                float spPosMax, float torqueConstant, SpringElasticity springType)
{
  command.params.spPosMax = spPosMax;  // float32
  command.params.spNegMax = spPosMin;  // float32

  command.params.params.pidParameters.integral = integral;              // float32
  command.params.params.pidParameters.pgain = Pgain;                    // float32
  command.params.params.pidParameters.igain = IGain;                    // float32
  command.params.params.pidParameters.dgain = Dgain;                    // float32
  command.params.params.pidParameters.forwardGain = forwardGain;        // float32
  command.params.params.pidParameters.deadBand = deadBand;              // float32
  command.params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  command.params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  float polyPar[4];

  switch (springType)
  {
    // f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4
    case SpringElasticity::Soft:  // D311-spring
      polyPar[0] = 0;
      polyPar[1] = 0.237536;
      polyPar[2] = -0.000032;
      polyPar[3] = 0;
      break;
    case SpringElasticity::Medium:
      polyPar[0] = 1.604382;
      polyPar[1] = 0.508932;
      polyPar[2] = -0.000117;
      polyPar[3] = 0;
      break;
    case SpringElasticity::Hard:
      polyPar[0] = 1.186483;
      polyPar[1] = 0.938377;
      polyPar[2] = -0.000274;
      polyPar[3] = 0.000001;
      break;
  }

  command.params.polyPar[0] = polyPar[0];  // float32
  command.params.polyPar[1] = polyPar[1];
  command.params.polyPar[2] = polyPar[2];
  command.params.polyPar[3] = polyPar[3];
  command.params.torqueConstant = torqueConstant;  // float32

  // initialize PID controller in motordriver boards
  command.frame[ganglion].ControlMode[motor] = Force;
  command.frame[ganglion].OperationMode[motor] = Initialise;
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;
  exchangeData();
  command.frame[ganglion].OperationMode[motor] = Run;
  command.frame[ganglion].sp[motor] = 0;
  exchangeData();
}

std::bitset<NUMBER_OF_GANGLIONS> FlexRayHardwareInterface::exchangeData()
{
  uint32_t activeGanglionsMask = 0;
  FT_STATUS ftStatus;
  std::array<WORD, DATASETSIZE> buffer;
  std::memcpy(&buffer, &command, sizeof(command));

  ftStatus = SPI_WriteBuffer(m_ftHandle, &buffer[0], DATASETSIZE);  // send data
  if (ftStatus != FT_OK)
  {
    ROS_ERROR_STREAM("Failed to Send a byte through SPI, Error Code: " << ftStatus);
  }
  else
  {
    DWORD dwNumInputBuffer = 0;
    DWORD dwNumBytesRead;

    // WAIT FOR DATA TO ARRIVE
    ROS_DEBUG("waiting for data");
    while (dwNumInputBuffer != DATASETSIZE * 2)
    {
      // get the number of bytes in the device receive buffer
      ftStatus = FT_GetQueueStatus(m_ftHandle, &dwNumInputBuffer);
    }
    if (ftStatus != FT_OK)
    {
      char errorMessage[256];
      getErrorMessage(ftStatus, errorMessage);
      ROS_ERROR_STREAM("exchange data failed with error " << errorMessage);
    }
    ROS_DEBUG("reading data");
    FT_Read(m_ftHandle, &buffer[0], dwNumInputBuffer, &dwNumBytesRead);
    std::memcpy(&GanglionData, &buffer, sizeof(GanglionData));

    // active ganglions, generated from usbFlexRay interface
    activeGanglionsMask = buffer[sizeof(GanglionData) >> 1];
  }
  uint32_t m = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      int8_t status;
      if (GanglionData[ganglion].muscleState[motor].actuatorCurrent != 0)
      {
        status = 1;
      }
      else
      {
        status = 0;
      }
      motorState[m] = status;
      m++;
    }
  }
  return activeGanglionsMask;
}

double FlexRayHardwareInterface::measureConnectionTime()
{
  std::ofstream file;
  file.open("measureConnectionTime.log");
  auto start = std::chrono::high_resolution_clock::now();
  for (uint32_t i = 0; i < 1000; i++)
  {
    exchangeData();
  }
  auto elapsed = start - std::chrono::high_resolution_clock::now();
  auto average = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() / 1000;
  file << "Measured runtime for 1000 updateCommandFrame() + exchangeData() "
          "calls\n";
  file << "average time: " << average << " seconds" << std::endl;
  file.close();
  return average;
}
