#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

#include "Spi.hpp"
#include "flexrayusbinterface/Message.hpp"

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

  exchangeData();
};

auto FlexRayHardwareInterface::connect() -> variant<FlexRayHardwareInterface, FtResult>
{
  return UsbChannel::open("FTVDIMQW")
      .match(
          [](UsbChannel usb) -> variant<FlexRayHardwareInterface, FtResult> { return FlexRayHardwareInterface(usb); },
          [](FtResult error) -> variant<FlexRayHardwareInterface, FtResult> { return error; });
};


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
  while (usb.bytes_available().match([](DWORD bytes) { return bytes; }, [](FtResult) { return 0; }) <
         DATASETSIZE * sizeof(WORD))
  {
  }
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

std::chrono::duration<double> FlexRayHardwareInterface::measureConnectionTime(uint32_t iterations)
{
  auto start = std::chrono::high_resolution_clock::now();
  for (uint32_t i = 0; i < iterations; i++)
  {
    exchangeData();
  }
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  return elapsed / iterations;
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
