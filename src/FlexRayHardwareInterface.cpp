#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <thread>
#include <utility>

#include "flexrayusbinterface/Protocol.hpp"
#include "flexrayusbinterface/Range.hpp"

FlexRayHardwareInterface::FlexRayHardwareInterface(UsbChannel&& channel)
  : protocol{ std::move(channel) }
  , slots(protocol.NumberOfGanglions)
  , worker{ std::async(std::launch::async,
                       [&](std::future<void> stop) {
                         while (stop.wait_for(std::chrono::seconds{ 0 }) != std::future_status::ready)
                         {
                           protocol.exchange_data();
                         }
                       },
                       stop_work.get_future()) }
{
  for (std::size_t i = 0; i < protocol.NumberOfGanglions; ++i)
  {
    auto& muscle_slots = slots[i];
    for (std::size_t j = 0; j < protocol.MusclesPerGanglion; ++j)
      muscle_slots.emplace_back(std::move(*protocol.get_slot(i, j)));
  }
  initForceControl();
};

FlexRayHardwareInterface::~FlexRayHardwareInterface()
{
  stop_work.set_value();
}

void FlexRayHardwareInterface::initPositionControl(bool set_tag, float Pgain, float IGain, float Dgain,
                                                   float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  init(Position, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin,
                           IntegralPosMax, spPosMin, spPosMax));
}

void FlexRayHardwareInterface::initPositionControl(uint32_t ganglion, uint32_t motor, bool set_tag, float Pgain,
                                                   float IGain, float Dgain, float forwardGain, float deadBand,
                                                   float integral, float IntegralPosMin, float IntegralPosMax,
                                                   float spPosMin, float spPosMax)
{
  init(Position, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin,
                           IntegralPosMax, spPosMin, spPosMax),
       ganglion, motor);
}

void FlexRayHardwareInterface::initVelocityControl(bool set_tag, float Pgain, float IGain, float Dgain,
                                                   float forwardGain, float deadBand, float integral,
                                                   float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                   float spPosMax)
{
  init(Velocity, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin,
                           IntegralPosMax, spPosMin, spPosMax));
}

void FlexRayHardwareInterface::initVelocityControl(uint32_t ganglion, uint32_t motor, bool set_tag, float Pgain,
                                                   float IGain, float Dgain, float forwardGain, float deadBand,
                                                   float integral, float IntegralPosMin, float IntegralPosMax,
                                                   float spPosMin, float spPosMax)
{
  init(Velocity, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin,
                           IntegralPosMax, spPosMin, spPosMax),
       ganglion, motor);
}

void FlexRayHardwareInterface::initForceControl(bool set_tag, float Pgain, float IGain, float Dgain, float forwardGain,
                                                float deadBand, float integral, float IntegralPosMin,
                                                float IntegralPosMax, float spPosMin, float spPosMax,
                                                float torqueConstant, SpringElasticity springType)
{
  init(Force, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax,
                        spPosMin, spPosMax, torqueConstant, springType));
}

void FlexRayHardwareInterface::initForceControl(uint32_t ganglion, uint32_t motor, bool set_tag, float Pgain,
                                                float IGain, float Dgain, float forwardGain, float deadBand,
                                                float integral, float IntegralPosMin, float IntegralPosMax,
                                                float spPosMin, float spPosMax, float torqueConstant,
                                                SpringElasticity springType)
{
  init(Force, setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax,
                        spPosMin, spPosMax, torqueConstant, springType),
       ganglion, motor);
}

control_Parameters_t FlexRayHardwareInterface::setParams(bool set_tag, float Pgain, float IGain, float Dgain,
                                                         float forwardGain, float deadBand, float integral,
                                                         float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                         float spPosMax)
{
  control_Parameters_t params;
  params.tag = set_tag ? 1 : 0;  // sint32
  params.outputPosMax = 1000;    // sint32
  params.outputNegMax = -1000;   // sint32
  params.timePeriod = 100;       // float32      //in us set time period to avoid error case

  params.radPerEncoderCount = radPerEncoderCount;  // float32
  params.params.pidParameters.lastError = 0;       // float32

  params.spPosMax = spPosMax;  // float32
  params.spNegMax = spPosMin;  // float32

  params.params.pidParameters.integral = integral;              // float32
  params.params.pidParameters.pgain = Pgain;                    // float32
  params.params.pidParameters.igain = IGain;                    // float32
  params.params.pidParameters.dgain = Dgain;                    // float32
  params.params.pidParameters.forwardGain = forwardGain;        // float32
  params.params.pidParameters.deadBand = deadBand;              // float32
  params.params.pidParameters.IntegralPosMax = IntegralPosMax;  // float32
  params.params.pidParameters.IntegralNegMax = IntegralPosMin;  // float32

  return params;
}

control_Parameters_t FlexRayHardwareInterface::setParams(bool set_tag, float Pgain, float IGain, float Dgain,
                                                         float forwardGain, float deadBand, float integral,
                                                         float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                                         float spPosMax, float torqueConstant,
                                                         SpringElasticity springType)
{
  auto params = setParams(set_tag, Pgain, IGain, Dgain, forwardGain, deadBand, integral, IntegralPosMin, IntegralPosMax,
                          spPosMin, spPosMax);

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

  params.polyPar[0] = polyPar[0];  // float32
  params.polyPar[1] = polyPar[1];
  params.polyPar[2] = polyPar[2];
  params.polyPar[3] = polyPar[3];
  params.torqueConstant = torqueConstant;  // float32

  return params;
}

void FlexRayHardwareInterface::init(comsControllerMode mode, control_Parameters_t params)
{
  using guard_t = CompletionGuard<Protocol<>::input_t>;
  std::vector<Entangled<guard_t, guard_t::completion_t>> tmp_slots;

  for (auto& muscle_slots : slots)
    for (auto& slot : muscle_slots)
      tmp_slots.emplace_back(std::move(slot).enqueue(Enqueue{ mode, params }));

  for (auto& muscle_slots : reversed(slots))
    for (auto& slot : reversed(muscle_slots))
    {
      slot = std::move(tmp_slots.back()).get().first;
      tmp_slots.pop_back();
    }
}

void FlexRayHardwareInterface::init(comsControllerMode mode, control_Parameters_t params, uint32_t ganglion,
                                    uint32_t motor)
{
  slots[ganglion][motor] = std::move(slots[ganglion][motor]).enqueue(Enqueue{ mode, params }).get().first;
}
