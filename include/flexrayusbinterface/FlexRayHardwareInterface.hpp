#pragma once

// std
#include <cstdint>
#include <string>
#include <vector>

#include "flexrayusbinterface/CommunicationData.h"
#include "flexrayusbinterface/Spi.hpp"
#include "ftd2xx.h"

class FlexRayHardwareInterface
{
public:
  /** Constructor */
  FlexRayHardwareInterface();
  /** Destructor */
  ~FlexRayHardwareInterface();
  /**
   * connect to flexray
   * @return success
   */
  bool connect();
  /**
   * use this to initialize the motors
   */
  void initializeMotors();

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
                        float torqueConstant = 1.0, char springType = SoftSpring);
  /**
   * This function initializes force control for one specific motor in a ganglion.
   */
  void initForceControl(uint32_t ganglion, uint32_t motor, float Pgain = 70.0, float IGain = 0.0, float Dgain = 0.0,
                        float forwardGain = 0.0, float deadBand = 0.0, float integral = 0.0, float IntegralPosMin = 0.0,
                        float IntegralPosMax = 0.0, float spPosMin = -100.0, float spPosMax = 100.0,
                        float torqueConstant = 1.0, char springType = SoftSpring);
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

  /**
   * Checks the number of connected ganglia
   * @return number of connected ganglia
   */
  uint32_t checkNumberOfConnectedGanglions();

  /**
   * Checks which motors are ready and updates motorState
   */
  void updateMotorState();

  /**
   * Measure connection time via multiple calls to exchangeData()
   */
  double measureConnectionTime();

  /**
   * Records trajectories of all available motors
   * @param samplingTime - sampling time in milliseconds
   * @param recordTime - time span to record in seconds
   * @param trajectories - reference will be filled with trajectories
   * @param idList - record from these motors
   * @param controlMode - what values should be recorded
   */
  float recordTrajectories(float samplingTime, float recordTime, std::vector<std::vector<float>>& trajectories,
                           std::vector<int>& idList, std::vector<int>& controlMode, std::string name = "");
  /**
   * Records trajectories of all available motors
   * @param samplingTime - sampling time in milliseconds
   * @param trajectories - reference will be filled with trajectories
   * @param idList - record from these motors
   * @param controlMode - what values should be recorded
   * @param stop_recording - pointer to steering bool
   */
  float recordTrajectories(float samplingTime, std::vector<std::vector<float>>& trajectories, std::vector<int>& idList,
                           std::vector<int>& controlMode, int8_t* recording);

  //! upstream from ganglions to PC
  ganglionData_t GanglionData[NUMBER_OF_GANGLIONS];
  //! number of connected ganglions
  uint32_t numberOfGanglionsConnected;
  //! command frames containing motor control parameters for 3 ganglia
  comsCommandFrame commandframe0[3];
  //! command frames containing motor control parameters for 3 ganglia
  comsCommandFrame commandframe1[3];
  //! control parameters for motor
  control_Parameters_t controlparams;

private:
  //! bitmask with active ganglions
  unsigned short activeGanglionsMask;
  //! vector containing a status for each motor
  std::vector<int8_t> motorState;
  //! vector containing the controller type for each motor
  std::vector<int8_t> motorControllerType;

  //! Handle of the FTDI device
  FT_HANDLE m_ftHandle;
  //! number of devices connected
  uint32_t m_numberOfConnectedDevices;
  //! Value of clock divisor, SCL Frequency = 60/((1+value)*2) = MHz i.e., value of 2 = 10MHz, or 29 = 1Mhz
  uint32_t m_clockDivisor = 2;
  //! this will be send via flexray
  WORD dataset[DATASETSIZE];
  //! this will contain data coming from flexray
  WORD InputBuffer[DATASETSIZE];

  enum
  {
    SoftSpring,
    MiddleSpring,
    HardSpring
  };
  enum SteeringCommand
  {
    STOP_TRAJECTORY = 0,
    PLAY_TRAJECTORY,
    PAUSE_TRAJECTORY,
    REWIND_TRAJECTORY
  };
};
