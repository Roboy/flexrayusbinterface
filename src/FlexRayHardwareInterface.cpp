#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <utility>

// ros
#include <ros/console.h>

#include "common_utilities/timer.hpp"

FlexRayHardwareInterface::FlexRayHardwareInterface()
{
  motorState.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION, 1);
  motorControllerType.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION, 0);
#ifdef HARDWARE
  ROS_INFO("Trying to connect to FlexRay");
  while (!connect())
  {
    ROS_INFO("retrying...");
    usleep(1000000);  // sleep for a second
  }
#else
  ROS_DEBUG("No Hardware mode enabled");
  activeGanglionsMask = 0b111111;
  numberOfGanglionsConnected = 6;
#endif
  initializeMotors();
};

FlexRayHardwareInterface::~FlexRayHardwareInterface()
{
}

bool FlexRayHardwareInterface::connect()
{
  if (CheckDeviceConnected(&m_numberOfConnectedDevices) == true)
  {
    if (GetDeviceInfo(&m_numberOfConnectedDevices) == true)
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
              spiconfigured = ConfigureSPI(&m_ftHandle, m_clockDivisor);
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

void FlexRayHardwareInterface::initializeMotors()
{
  controlparams.tag = 0;               // sint32
  controlparams.outputPosMax = 1000;   // sint32
  controlparams.outputNegMax = -1000;  // sint32
  controlparams.timePeriod = 100;      // float32      //in us set time period to avoid error case

  controlparams.radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);  // float32
  controlparams.params.pidParameters.lastError = 0;                        // float32

  for (uint32_t i = 0; i < 3; i++)
  {
    for (uint32_t j = 0; j < 4; j++)
    {
      commandframe0[i].sp[j] = 0;
      commandframe1[i].sp[j] = 0;
    }
  }
  updateCommandFrame();
  initForceControl();

#ifdef HARDWARE
  checkNumberOfConnectedGanglions();
  updateMotorState();
#else
  ROS_DEBUG("NO HARDWARE MODE");
  numberOfGanglionsConnected = 6;
#endif
  ROS_INFO("%d ganglions are connected via flexray, activeGanglionsMask %c", numberOfGanglionsConnected,
           activeGanglionsMask);
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
  if (ganglion_id < 3)
    commandframe0[ganglion_id].sp[motor_id] = vel;
  else
    commandframe1[ganglion_id - 3].sp[motor_id] = vel;
  updateCommandFrame();
  exchangeData();
  usleep(300000);
  updateCommandFrame();
  exchangeData();
  tendonDisplacement_t[0] =
      GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;  // tendon displacemnte iniziale
  ROS_INFO("Displacement_t0 %.5f     ", tendonDisplacement_t[0]);
  uint32_t i = 1;
  for (i = 1; i < 3; i++)
  {
    vel = 3;
    if (ganglion_id < 3)
      commandframe0[ganglion_id].sp[motor_id] = vel;
    else
      commandframe1[ganglion_id - 3].sp[motor_id] = vel;
    updateCommandFrame();
    exchangeData();
    usleep(500000);
    updateCommandFrame();
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
    if (ganglion_id < 3)
      commandframe0[ganglion_id].sp[motor_id] = vel;
    else
      commandframe1[ganglion_id - 3].sp[motor_id] = vel;
    updateCommandFrame();
    exchangeData();
    usleep(300000);
    updateCommandFrame();
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
    if (ganglion_id < 3)
      commandframe0[ganglion_id].sp[motor_id] = vel;
    else
      commandframe1[ganglion_id - 3].sp[motor_id] = vel;
    updateCommandFrame();
    exchangeData();
    sleep(2);

    controlparams.tag = 1;
    commandframe0[0].OperationMode[0] = Initialise;
    ROS_INFO("Tag = %d ", controlparams.tag);

    updateCommandFrame();
    exchangeData();
    updateMotorState();
    controlparams.tag = 0;
    commandframe0[0].OperationMode[0] = Initialise;
    ROS_INFO("Tag = %d ", controlparams.tag);
    updateCommandFrame();
    exchangeData();
    updateMotorState();

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
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].ControlMode[motor] = Position;
      commandframe0[i].OperationMode[motor] = Initialise;
      commandframe1[i].ControlMode[motor] = Position;
      commandframe1[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
      motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
    }
  }
  updateCommandFrame();
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].OperationMode[motor] = Run;
      float pos = GanglionData[i].muscleState[motor].actuatorPos;
      commandframe0[i].sp[motor] = pos * controlparams.radPerEncoderCount;
      commandframe1[i].OperationMode[motor] = Run;
      pos = GanglionData[i + 3].muscleState[motor].actuatorPos;
      commandframe1[i].sp[motor] = pos * controlparams.radPerEncoderCount;
    }
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::initPositionControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  if (ganglion < 3)
  {
    commandframe0[ganglion].ControlMode[motor] = Position;
    commandframe0[ganglion].OperationMode[motor] = Initialise;
  }
  else
  {
    commandframe1[ganglion - 3].ControlMode[motor] = Position;
    commandframe1[ganglion - 3].OperationMode[motor] = Initialise;
  }
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
  updateCommandFrame();
  exchangeData();
  float pos = GanglionData[ganglion].muscleState[motor].actuatorPos * controlparams.radPerEncoderCount;
  if (ganglion < 3)
  {
    commandframe0[ganglion].OperationMode[motor] = Run;
    commandframe0[ganglion].sp[motor] = pos;
  }
  else
  {
    commandframe1[ganglion - 3].OperationMode[motor] = Run;
    commandframe1[ganglion - 3].sp[motor] = pos;
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                   float deadBand, float integral, float IntegralPosMin,
                                                   float IntegralPosMax, float spPosMin, float spPosMax)
{
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].ControlMode[motor] = Velocity;
      commandframe0[i].OperationMode[motor] = Initialise;
      commandframe1[i].ControlMode[motor] = Velocity;
      commandframe1[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
      motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
    }
  }
  updateCommandFrame();
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].OperationMode[motor] = Run;
      commandframe0[i].sp[motor] = 0;
      commandframe1[i].OperationMode[motor] = Run;
      commandframe1[i].sp[motor] = 0;
    }
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  // initialize PID controller in motordriver boards
  if (ganglion < 3)
  {
    commandframe0[ganglion].ControlMode[motor] = Velocity;
    commandframe0[ganglion].OperationMode[motor] = Initialise;
  }
  else
  {
    commandframe1[ganglion].ControlMode[motor] = Velocity;
    commandframe1[ganglion].OperationMode[motor] = Initialise;
  }
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
  updateCommandFrame();
  exchangeData();
  if (ganglion < 3)
  {
    commandframe0[ganglion].OperationMode[motor] = Run;
    commandframe0[ganglion].sp[motor] = 0;
  }
  else
  {
    commandframe1[ganglion - 3].OperationMode[motor] = Run;
    commandframe1[ganglion - 3].sp[motor] = 0;
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::initForceControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                float deadBand, float integral, float IntegralPosMin,
                                                float IntegralPosMax, float spPosMin, float spPosMax,
                                                float torqueConstant, SpringElasticity springType)
{
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

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

  controlparams.polyPar[0] = polyPar[0];  // float32
  controlparams.polyPar[1] = polyPar[1];
  controlparams.polyPar[2] = polyPar[2];
  controlparams.polyPar[3] = polyPar[3];
  controlparams.torqueConstant = torqueConstant;  // float32

  // initialize PID controller in motordriver boards
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].ControlMode[motor] = Force;
      commandframe0[i].OperationMode[motor] = Initialise;
      commandframe1[i].ControlMode[motor] = Force;
      commandframe1[i].OperationMode[motor] = Initialise;
      motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;
      motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;
    }
  }
  updateCommandFrame();
  exchangeData();
  for (uint32_t i = 0; i < NUMBER_OF_GANGLIONS / 2; i++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      commandframe0[i].OperationMode[motor] = Run;
      commandframe0[i].sp[motor] = 0;
      commandframe1[i].OperationMode[motor] = Run;
      commandframe1[i].sp[motor] = 0;
    }
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::initForceControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                float Dgain, float forwardGain, float deadBand, float integral,
                                                float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                float spPosMax, float torqueConstant, SpringElasticity springType)
{
  controlparams.spPosMax = spPosMax;  // float32
  controlparams.spNegMax = spPosMin;  // float32

  controlparams.params.pidParameters.integral = integral;              // float32
  controlparams.params.pidParameters.pgain = Pgain;                    // float32
  controlparams.params.pidParameters.igain = IGain;                    // float32
  controlparams.params.pidParameters.dgain = Dgain;                    // float32
  controlparams.params.pidParameters.forwardGain = forwardGain;        // float32
  controlparams.params.pidParameters.deadBand = deadBand;              // float32
  controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

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

  controlparams.polyPar[0] = polyPar[0];  // float32
  controlparams.polyPar[1] = polyPar[1];
  controlparams.polyPar[2] = polyPar[2];
  controlparams.polyPar[3] = polyPar[3];
  controlparams.torqueConstant = torqueConstant;  // float32

  // initialize PID controller in motordriver boards
  if (ganglion < 3)
  {
    commandframe0[ganglion].ControlMode[motor] = Force;
    commandframe0[ganglion].OperationMode[motor] = Initialise;
  }
  else
  {
    commandframe1[ganglion - 3].ControlMode[motor] = Force;
    commandframe1[ganglion - 3].OperationMode[motor] = Initialise;
  }
  motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;

  updateCommandFrame();
  exchangeData();
  if (ganglion < 3)
  {
    commandframe0[ganglion].OperationMode[motor] = Run;
    commandframe0[ganglion].sp[motor] = 0;
  }
  else
  {
    commandframe1[ganglion - 3].OperationMode[motor] = Run;
    commandframe1[ganglion - 3].sp[motor] = 0;
  }
  updateCommandFrame();
  exchangeData();
}

void FlexRayHardwareInterface::exchangeData()
{
#ifdef HARDWARE
  FT_STATUS ftStatus;
  ftStatus = SPI_WriteBuffer(m_ftHandle, &dataset[0], DATASETSIZE);  // send data
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
      ftStatus = FT_GetQueueStatus(m_ftHandle, &dwNumInputBuffer);  // get the number of bytes in the
                                                                    // device receive buffer
    }
    if (ftStatus != FT_OK)
    {
      char errorMessage[256];
      getErrorMessage(ftStatus, errorMessage);
      ROS_ERROR_STREAM("exchange data failed with error " << errorMessage);
    }
    ROS_DEBUG("reading data");
    FT_Read(m_ftHandle, &InputBuffer[0], dwNumInputBuffer, &dwNumBytesRead);  // read bytes into word locations

    // now make a copy of relevant signal
    memcpy(GanglionData, InputBuffer, sizeof(GanglionData));  // ganglion data

    activeGanglionsMask =
        InputBuffer[sizeof(GanglionData) >> 1];  // active ganglions, generated from usbFlexRay interface
  }
#else
  ROS_DEBUG("NO HARDWARE");
#endif
  updateMotorState();  // ogni volta che scrivo e leggo faccio un update del
                       // motor state (cambia da 0 a 1, da 0 a 1)
}

void FlexRayHardwareInterface::updateCommandFrame()
{
  memcpy((void *)&dataset[0], commandframe0, sizeof(comsCommandFrame) * GANGLIONS_PER_CONTROL_FRAME);
  memcpy((void *)&dataset[sizeof(comsCommandFrame) * GANGLIONS_PER_CONTROL_FRAME / 2], commandframe1,
         sizeof(comsCommandFrame) * GANGLIONS_PER_CONTROL_FRAME);
  memcpy((void *)&dataset[72], &controlparams, sizeof(control_Parameters_t));
}

/**
 * This function counts the number of bits set in a bitmask
 * @param i bitmask
 * @return number of ones set
 */
static uint32_t NumberOfSetBits(uint32_t i)
{
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

uint32_t FlexRayHardwareInterface::checkNumberOfConnectedGanglions()
{
  exchangeData();
#ifdef HARDWARE
  numberOfGanglionsConnected = NumberOfSetBits(activeGanglionsMask);
#endif
  return numberOfGanglionsConnected;
}

void FlexRayHardwareInterface::updateMotorState()
{
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
}

double FlexRayHardwareInterface::measureConnectionTime()
{
  std::ofstream file;
  file.open("measureConnectionTime.log");
  Timer timer;
  timer.start();
  for (uint32_t i = 0; i < 1000; i++)
  {
    updateCommandFrame();
    exchangeData();
  }
  double elapsedTime = timer.stop() / 1000.0;
  file << "Measured runtime for 1000 updateCommandFrame() + exchangeData() "
          "calls\n";
  file << "average time: " << elapsedTime << " seconds" << std::endl;
  file.close();
  return elapsedTime;
}

float FlexRayHardwareInterface::recordTrajectories(float samplingTime, float recordTime,
                                                   std::vector<std::vector<float>> &trajectories,
                                                   std::vector<int> &idList, std::vector<int> &controlmode,
                                                   std::string name)
{
  // this will be filled with the trajectories
  trajectories.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);

  // make a backup of control modes so after recording they can be restored
  std::vector<int8_t> motorControllerType_backup = motorControllerType;
  std::vector<float> setPoints_backup(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
  uint32_t i = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        setPoints_backup[i] = commandframe0[ganglion].sp[motor];
      else
        setPoints_backup[i] = commandframe1[ganglion].sp[motor];
      i++;
    }
  }

  // force control is probably the most convenient mode for recording
  initForceControl();
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = 6.0f;
      else
        commandframe1[ganglion - 3].sp[motor] = 6.0f;
    }
  }
  updateCommandFrame();
  exchangeData();

  // samplingTime milli -> seconds
  samplingTime /= 1000.0f;

  double elapsedTime = 0.0, dt;
  long sample = 0;

  // start recording
  Timer timer;
  timer.start();
  while (elapsedTime < recordTime)
  {
    dt = elapsedTime;
    exchangeData();
    for (uint32_t m = 0; m < idList.size(); m++)
    {
      if (motorState[idList[m]] == 1)
      {  // only record if motor is available
        if (controlmode[m] == 1)
          trajectories.at(idList[m]).push_back(GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .actuatorPos *
                                               controlparams.radPerEncoderCount);
        else if (controlmode[m] == 2)
          trajectories.at(idList[m]).push_back(GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .actuatorVel *
                                               controlparams.radPerEncoderCount);
        else if (controlmode[m] == 3)
          trajectories.at(idList[m]).push_back(GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                                   .tendonDisplacement);
      }
    }
    sample++;
    elapsedTime = timer.elapsedTime();
    dt = elapsedTime - dt;
    // if faster than sampling time sleep for difference
    if (dt < samplingTime)
    {
      usleep((samplingTime - dt) * 1000000.0);
      elapsedTime = timer.elapsedTime();
    }
  }
  // done recording
  std::ofstream outfile;
  if (name.empty())
  {
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    char str[200];
    sprintf(str, "/home/letrend/workspace/roboy-ros-control/src/"
                 "flexrayusbinterface/recording_%s.log",
            asctime(timeinfo));
    name = str;
  }

  outfile.open(name);
  if (outfile.is_open())
  {
    outfile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
    outfile << "roboybehavior name=" << name << " behaviorid=\"200\"> " << std::endl;
    char motorname[10];
    for (uint32_t m = 0; m < idList.size(); m++)
    {
      sprintf(motorname, "motor%d", idList[m]);
      outfile << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\"" << controlmode[m] << "\" samplerate=\""
              << samplingTime << "\">" << std::endl;
      outfile << "<waypointlist>" << std::endl;
      for (uint32_t i = 0; i < trajectories[idList[m]].size(); i++)
        outfile << trajectories[idList[m]][i] << " ";
      outfile << "</waypointlist>" << std::endl;
      outfile << "</trajectory>" << std::endl;
    }
    outfile << "</roboybehavior>" << std::endl;
    outfile.close();
  }

  // set force to zero
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = 0.0f;
      else
        commandframe1[ganglion - 3].sp[motor] = 0.0f;
    }
  }
  updateCommandFrame();
  exchangeData();

  // restore controller types
  uint32_t m = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      switch (motorControllerType_backup[m])
      {
        case Position:
          initPositionControl(ganglion, motor);
          break;
        case Velocity:
          initVelocityControl(ganglion, motor);
          break;
        case Force:
          initForceControl(ganglion, motor);
          break;
      }
      m++;
    }
  }
  // restore setpoints
  i = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = setPoints_backup[i];
      else
        commandframe1[ganglion - 3].sp[motor] = setPoints_backup[i];
      i++;
    }
  }
  updateCommandFrame();
  exchangeData();

  // return average sampling time in milliseconds
  return elapsedTime / (double)sample * 1000.0f;
}

float FlexRayHardwareInterface::recordTrajectories(float samplingTime, std::vector<std::vector<float>> &trajectories,
                                                   std::vector<int> &idList, std::vector<int> &controlmode,
                                                   RecordingAction recording)
{
  // this will be filled with the trajectories
  trajectories.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);

  // make a backup of control modes so after recording they can be restored
  std::vector<int8_t> motorControllerType_backup = motorControllerType;
  std::vector<float> setPoints_backup(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
  uint32_t i = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        setPoints_backup[i] = commandframe0[ganglion].sp[motor];
      else
        setPoints_backup[i] = commandframe1[ganglion].sp[motor];
      i++;
    }
  }

  // force control is probably the most convenient mode for recording
  initForceControl();
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = 6.0f;
      else
        commandframe1[ganglion - 3].sp[motor] = 6.0f;
    }
  }
  updateCommandFrame();
  exchangeData();

  // samplingTime milli -> seconds
  samplingTime /= 1000.0f;

  double elapsedTime = 0.0, dt;
  long sample = 0;

  // start recording
  Timer timer;
  timer.start();
  while (recording != RecordingAction::Stop)
  {
    dt = elapsedTime;
    if (recording == RecordingAction::Play)
    {
      exchangeData();
      for (uint32_t m = 0; m < idList.size(); m++)
      {
        if (motorState[idList[m]] == 1)
        {  // only record if motor is available
          if (controlmode[m] == 1)
            trajectories.at(idList[m]).push_back(GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                                     .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                                     .actuatorPos *
                                                 controlparams.radPerEncoderCount);
          else if (controlmode[m] == 2)
            trajectories.at(idList[m]).push_back(GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                                     .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                                     .actuatorVel *
                                                 controlparams.radPerEncoderCount);
          else if (controlmode[m] == 3)
          {
            float polyPar[4];
            polyPar[0] = 0;
            polyPar[1] = 0.237536;
            polyPar[2] = -0.000032;
            polyPar[3] = 0;
            float tendonDisplacement = GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                           .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                           .tendonDisplacement;
            float force = polyPar[0] + polyPar[1] * tendonDisplacement + polyPar[2] * powf(tendonDisplacement, 2.0f) +
                          polyPar[3] * powf(tendonDisplacement, 3.0f);
            trajectories.at(idList[m]).push_back(force);
          }
        }
      }
      sample++;
    }
    elapsedTime = timer.elapsedTime();
    dt = elapsedTime - dt;
    // if faster than sampling time sleep for difference
    if (dt < samplingTime)
    {
      usleep((samplingTime - dt) * 1000000.0);
      elapsedTime = timer.elapsedTime();
    }
  }

  // set force to zero
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = 0.0f;
      else
        commandframe1[ganglion - 3].sp[motor] = 0.0f;
    }
  }
  updateCommandFrame();
  exchangeData();

  // restore controller types
  uint32_t m = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++)
    {
      if (motorState[m] == 1)
      {
        switch (motorControllerType_backup[m])
        {
          case Position:
            initPositionControl(ganglion, motor);
            break;
          case Velocity:
            initVelocityControl(ganglion, motor);
            break;
          case Force:
            initForceControl(ganglion, motor);
            break;
        }
      }
      m++;
    }
  }
  // restore setpoints
  i = 0;
  for (uint32_t ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++)
  {
    for (uint32_t motor = 0; motor < 4; motor++)
    {
      if (ganglion < 3)
        commandframe0[ganglion].sp[motor] = setPoints_backup[i];
      else
        commandframe1[ganglion - 3].sp[motor] = setPoints_backup[i];
      i++;
    }
  }
  updateCommandFrame();
  exchangeData();

  // return average sampling time in milliseconds
  return elapsedTime / (double)sample * 1000.0f;
}
