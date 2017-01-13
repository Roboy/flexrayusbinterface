#pragma once

// std
#include <array>
#include <bitset>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

#include "flexrayusbinterface/CommunicationData.h"

#include "flexrayusbinterface/UsbChannel.hpp"


class FlexRayHardwareInterface
{
public:
  enum class SpringElasticity
  {
    Soft,
    Medium,
    Hard
  };

  /**
  * This function resets the sensor displacement of one motor
  */
  void relaxSpring(uint32_t ganglion_id, uint32_t motor_id, int controlmode);

  /**
  *This function initializes position control for all motors on all ganglia. Pleaser refer to myorobotics
  * documentation for details on control parameters.
  */
  void initPositionControl(float Pgain = 100.0, float IGain = 0.0, float Dgain = 0.0, float forwardGain = 0.0,
                           float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                           float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0);
  /**
   * This function initializes position control for one specific motor in a ganglion.
   */
  void initPositionControl(uint32_t ganglion, uint32_t motor, float Pgain = 1000.0, float IGain = 0.0,
                           float Dgain = 0.0, float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0,
                           float IntegralPosMin = 0.0, float IntegralPosMax = 0.0, float spPosMin = -100.0,
                           float spPosMax = 100.0);
  /**
   * This function initializes velocity control for all motors on all ganglia. Pleaser refer to myorobotics
   * documentation for details on control parameters.
   */
  void initVelocityControl(float Pgain = 200.0, float IGain = 0.0, float Dgain = 0.0, float forwardGain = 0.0,
                           float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                           float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0);
  /**
   * This function initializes velocity control for one specific motor in a ganglion.
   */
  void initVelocityControl(uint32_t ganglion, uint32_t motor, float Pgain = 200.0, float IGain = 0.0, float Dgain = 0.0,
                           float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0,
                           float IntegralPosMin = 0.0, float IntegralPosMax = 0.0, float spPosMin = -100.0,
                           float spPosMax = 100.0);

  /**
   * This function initializes force control for all motors on all ganglia. Pleaser refer to myorobotics
   * documentation for details on control parameters.
   */
  void initForceControl(float Pgain = 70.0, float IGain = 0.0, float Dgain = 0.0, float forwardGain = 0.0,
                        float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                        float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0,
                        float torqueConstant = 1.0, SpringElasticity springType = SpringElasticity::Soft);
  /**
   * This function initializes force control for one specific motor in a ganglion.
   */
  void initForceControl(uint32_t ganglion, uint32_t motor, float Pgain = 70.0, float IGain = 0.0, float Dgain = 0.0,
                        float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                        float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0,
                        float torqueConstant = 1.0, SpringElasticity springType = SpringElasticity::Soft);

  /**
   * This function exchanges data between interface and motors
   * @return the mask of connected ganglia
   */
  std::bitset<NUMBER_OF_GANGLIONS> exchangeData();

  /**
   * Measure connection time via multiple calls to exchangeData()
   */
  std::chrono::duration<double> measureConnectionTime(uint32_t iterations);

  struct
  {
    //! command frames containing motor control parameters for the ganglia
    std::array<comsCommandFrame, NUMBER_OF_GANGLIONS> frame;
    //! control parameters for motor
    control_Parameters_t params;
  } command;

  //! upstream from ganglions to PC
  std::array<ganglionData_t, NUMBER_OF_GANGLIONS> GanglionData;

  static auto connect() -> variant<FlexRayHardwareInterface, FtResult>;

private:
  FlexRayHardwareInterface(UsbChannel channel);

  /**
   * connect to flexray
   * @return success
   */

  void setParams(float Pgain, float IGain, float Dgain, float forwardGain,
          float deadBand, float integral, float IntegralPosMin,
          float IntegralPosMax, float spPosMin, float spPosMax);

  void setParams(float Pgain, float IGain, float Dgain, float forwardGain,
          float deadBand, float integral, float IntegralPosMin,
          float IntegralPosMax, float spPosMin, float spPosMax,
          float torqueConstant, SpringElasticity springType);

  void init(comsControllerMode mode);
  void init(comsControllerMode mode, uint32_t ganglion, uint32_t motor);

  //! Handle of the FTDI device
  UsbChannel usb;
};
