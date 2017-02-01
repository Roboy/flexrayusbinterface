#pragma once

// std
#include <array>
#include <bitset>
#include <chrono>
#include <string>
#include <vector>

#include "flexrayusbinterface/CommunicationData.h"

#include "flexrayusbinterface/Protocol.hpp"

class FlexRayHardwareInterface
{
public:
  enum class SpringElasticity
  {
    Soft,
    Medium,
    Hard
  };

  enum class Controller : unsigned int
  {
    Raw = comsControllerMode::Raw,
    Torque = comsControllerMode::Torque,
    Velocity = comsControllerMode::Velocity,
    Position = comsControllerMode::Position,
    Force = comsControllerMode::Force
  };

  /**
  *This function initializes position control for all motors on all ganglia. Pleaser refer to myorobotics
  * documentation for details on control parameters.
  */
  void initPositionControl(bool set_tag = false, float Pgain = 100.0, float IGain = 0.0, float Dgain = 0.0,
                           float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0,
                           float IntegralPosMin = 0.0, float IntegralPosMax = 0.0, float spPosMin = -100.0,
                           float spPosMax = 100.0);
  /**
   * This function initializes position control for one specific motor in a ganglion.
   */
  void initPositionControl(uint32_t ganglion, uint32_t motor, bool set_tag = false, float Pgain = 1000.0,
                           float IGain = 0.0, float Dgain = 0.0, float forwardGain = 0.0, float deadBand = 0.0,
                           float integral = 0.0, float IntegralPosMin = 0.0, float IntegralPosMax = 0.0,
                           float spPosMin = -100.0, float spPosMax = 100.0);
  /**
   * This function initializes velocity control for all motors on all ganglia. Pleaser refer to myorobotics
   * documentation for details on control parameters.
   */
  void initVelocityControl(bool set_tag = false, float Pgain = 200.0, float IGain = 0.0, float Dgain = 0.0,
                           float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0,
                           float IntegralPosMin = 0.0, float IntegralPosMax = 0.0, float spPosMin = -100.0,
                           float spPosMax = 100.0);
  /**
   * This function initializes velocity control for one specific motor in a ganglion.
   */
  void initVelocityControl(uint32_t ganglion, uint32_t motor, bool set_tag = false, float Pgain = 200.0,
                           float IGain = 0.0, float Dgain = 0.0, float forwardGain = 0.0, float deadBand = 0.0,
                           float integral = 0.0, float IntegralPosMin = 0.0, float IntegralPosMax = 0.0,
                           float spPosMin = -100.0, float spPosMax = 100.0);

  /**
   * This function initializes force control for all motors on all ganglia. Pleaser refer to myorobotics
   * documentation for details on control parameters.
   */
  void initForceControl(bool set_tag = false, float Pgain = 70.0, float IGain = 0.0, float Dgain = 0.0,
                        float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                        float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0,
                        float torqueConstant = 1.0, SpringElasticity springType = SpringElasticity::Soft);
  /**
   * This function initializes force control for one specific motor in a ganglion.
   */
  void initForceControl(uint32_t ganglion, uint32_t motor, bool set_tag = false, float Pgain = 70.0, float IGain = 0.0,
                        float Dgain = 0.0, float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0,
                        float IntegralPosMin = 0.0, float IntegralPosMax = 0.0, float spPosMin = -100.0,
                        float spPosMax = 100.0, float torqueConstant = 1.0,
                        SpringElasticity springType = SpringElasticity::Soft);

  inline auto read_muscle(uint32_t ganglion, uint32_t muscle) -> muscleState_t
  {
    return protocol.read_muscle(ganglion, muscle);
  }

  inline auto connected_ganglions() -> std::bitset<Protocol<>::NumberOfGanglions>
  {
    return protocol.connected_ganglions();
  }

  inline void set(uint32_t ganglion, uint32_t motor, Controller controller, float param)
  {
    slots[ganglion][motor] = std::move(slots[ganglion][motor])
                                 .enqueue(Send{ Send::Run{ param }, static_cast<comsControllerMode>(controller) })
                                 .get()
                                 .first;
  }

  virtual ~FlexRayHardwareInterface();

  FlexRayHardwareInterface(UsbChannel&& channel);

  static constexpr float radPerEncoderCount{2 * 3.14159265359 / (2000.0 * 53.0)};
private:

  /**
   * connect to flexray
   * @return success
   */

  control_Parameters_t setParams(bool set_tag, float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
                                 float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                 float spPosMax);

  control_Parameters_t setParams(bool set_tag, float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
                                 float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
                                 float spPosMax, float torqueConstant, SpringElasticity springType);

  void init(comsControllerMode mode, control_Parameters_t params);
  void init(comsControllerMode mode, control_Parameters_t params, uint32_t ganglion, uint32_t motor);

  //! Handle of the FTDI device
  Protocol<> protocol;
  std::vector<std::vector<CompletionGuard<Protocol<>::input_t>>> slots;
  std::promise<void> stop_work;
  std::future<void> worker;
};
