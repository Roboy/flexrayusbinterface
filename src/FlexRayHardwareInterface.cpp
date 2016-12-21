#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <string.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <utility>

#include "flexrayusbinterface/Message.hpp"
// ros
#include <ros/console.h>

FlexRayHardwareInterface::FlexRayHardwareInterface(UsbChannel channel) : usb(channel)
{
  command.params.tag = 0;               // sint32
  command.params.outputPosMax = 1000;   // sint32
  command.params.outputNegMax = -1000;  // sint32
  command.params.timePeriod = 100;      // float32      //in us set time period to avoid error case

  command.params.radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);  // float32
  command.params.params.pidParameters.lastError = 0;                        // float32

  for (auto& frame : command.frame)
    std::fill_n(&frame.sp[0], 4, 0);
  initForceControl();

  auto ganglions = exchangeData();
  ROS_INFO_STREAM(ganglions.count() << " ganglions are connected via flexray, activeGanglionsMask " << ganglions);
};

auto FlexRayHardwareInterface::connect() -> boost::optional<FlexRayHardwareInterface>
{
  if (auto connection = UsbChannel::connect())
  {
    if (auto device = connection->get_device())
    {
      if (auto channel = device->open())
      {
        for (auto tries = 0; tries < 3; ++tries)
        {
          if (auto mpsse = channel->configure_mpsse())
          {
            for (;;)
            {
              if (auto usb = mpsse->configure_spi())
              {
                ROS_INFO("connection OK");
                return FlexRayHardwareInterface(*usb);
              }
            }
          }
          else
          {
            ROS_INFO("Testing MPSSE failed, retrying!");
          }
        }
      }
      else
      {
        ROS_ERROR("open port failed\n--Perhaps the kernel automatically loaded "
                  "another driver for the FTDI USB device, from command line "
                  "try: \nsudo rmmod ftdi_sio \n sudo rmmod usbserial\n or "
                  "maybe run with sudo");
      }
    }
    else
    {
      ROS_ERROR("device info failed");
    }
  }
  else
  {
    ROS_ERROR("device not connected");
  }
  return boost::none;
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
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax);
  init(Position);
}

void FlexRayHardwareInterface::initPositionControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax);
  init(Position, ganglion, motor);
}

void FlexRayHardwareInterface::initVelocityControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                   float deadBand, float integral, float IntegralPosMin,
                                                   float IntegralPosMax, float spPosMin, float spPosMax)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax);
  init(Velocity);
}

void FlexRayHardwareInterface::initVelocityControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                   float Dgain, float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax);
  init(Velocity, ganglion, motor);
}

void FlexRayHardwareInterface::initForceControl(float Pgain, float IGain, float Dgain, float forwardGain,
                                                float deadBand, float integral, float IntegralPosMin,
                                                float IntegralPosMax, float spPosMin, float spPosMax,
                                                float torqueConstant, SpringElasticity springType)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax,
            torqueConstant, springType);
  init(Force);
}

void FlexRayHardwareInterface::initForceControl(uint32_t ganglion, uint32_t motor, float Pgain, float IGain,
                                                float Dgain, float forwardGain, float deadBand, float integral,
                                                float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                float spPosMax, float torqueConstant, SpringElasticity springType)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax,
            torqueConstant, springType);

  init(Force, ganglion, motor);
}

std::bitset<NUMBER_OF_GANGLIONS> FlexRayHardwareInterface::exchangeData()
{
  uint32_t activeGanglionsMask = 0;
  std::vector<WORD> buffer(DATASETSIZE, 0);
  std::copy_n(static_cast<WORD*>(static_cast<void*>(&command)), sizeof(command) / sizeof(WORD), std::begin(buffer));

  usb.write(buffer);
  // WAIT FOR DATA TO ARRIVE
  ROS_DEBUG("waiting for data");
  while (usb.bytes_available().match([](DWORD bytes) { return bytes; },
                                     [](FtResult error) {
                                       ROS_ERROR_STREAM("exchange data failed with error " << error.str());
                                       return 0;
                                     }) < DATASETSIZE * sizeof(WORD))
  {
  }
  ROS_DEBUG("reading data");
  usb.read(std::string(DATASETSIZE * sizeof(WORD), '\0'))
      .match(
          [this, &activeGanglionsMask](std::string& data) {
            std::stringstream buffer;
            WORD ganglions;
            buffer.str(data);
            Parser<DATASETSIZE * sizeof(WORD)>{}.add(GanglionData).add(ganglions).read(buffer);
            activeGanglionsMask = ganglions;
          },
          [](FtResult) {});
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

void FlexRayHardwareInterface::setParams(float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
                                         float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
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
}

void FlexRayHardwareInterface::setParams(float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
                                         float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                         float spPosMax, float torqueConstant, SpringElasticity springType)
{
  setParams(Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax, spPosMin, spPosMax);

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
}

void FlexRayHardwareInterface::init(comsControllerMode mode)
{
  // initialize PID controller in motordriver boards
  for (auto& frame : command.frame)
  {
    std::fill(std::begin(frame.ControlMode), std::end(frame.ControlMode), mode);
    std::fill(std::begin(frame.OperationMode), std::end(frame.OperationMode), Initialise);
  }
  exchangeData();
  for (auto& frame : command.frame)
  {
    std::fill(std::begin(frame.ControlMode), std::end(frame.ControlMode), 0);
    std::fill(std::begin(frame.OperationMode), std::end(frame.OperationMode), Run);
  }
  exchangeData();
}

void FlexRayHardwareInterface::init(comsControllerMode mode, uint32_t ganglion, uint32_t motor)
{
  // initialize PID controller in motordriver boards
  command.frame[ganglion].ControlMode[motor] = mode;
  command.frame[ganglion].OperationMode[motor] = Initialise;
  exchangeData();
  command.frame[ganglion].OperationMode[motor] = Run;
  command.frame[ganglion].sp[motor] = 0;
  exchangeData();
}
