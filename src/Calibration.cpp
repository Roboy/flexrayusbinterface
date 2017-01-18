#include <chrono>
#include <thread>

#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

void relaxSpring(FlexRayHardwareInterface flex_ray, uint32_t ganglion_id, uint32_t motor_id, int controlmode)
{
  flex_ray.initVelocityControl(ganglion_id, motor_id);
  uint32_t count = 0;
  float aux = 0;
  uint32_t p = 0;
  float vel;
  float tendonDisplacement_t[3];
  float tendonDisplacement_t2[10000];

  flex_ray.set(ganglion_id, motor_id, FlexRayHardwareInterface::Controller::Velocity, 0);
  flex_ray.exchangeData();
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  flex_ray.exchangeData();
  tendonDisplacement_t[0] =
      flex_ray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;  // tendon displacemnte iniziale
  uint32_t i = 1;
  for (i = 1; i < 3; i++)
  {
    flex_ray.set(ganglion_id, motor_id, FlexRayHardwareInterface::Controller::Velocity, 3);
    flex_ray.exchangeData();
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    flex_ray.exchangeData();
    tendonDisplacement_t[i] = flex_ray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
  }

  uint32_t t = 0;
  tendonDisplacement_t2[0] = flex_ray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;

  if (tendonDisplacement_t[2] < tendonDisplacement_t[0])
    vel = 3;
  else if (tendonDisplacement_t[1] < tendonDisplacement_t[2])
    vel = -3;
  else
    vel = 0;

  do
  {
    flex_ray.set(ganglion_id, motor_id, FlexRayHardwareInterface::Controller::Velocity, vel);
    flex_ray.exchangeData();
    std::this_thread::sleep_for(std::chrono::milliseconds{300});
    flex_ray.exchangeData();
    tendonDisplacement_t2[t + 1] = flex_ray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
    t++;

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
    relaxSpring(flex_ray, ganglion_id, motor_id, controlmode);

  else
  {
    vel = 0;
    flex_ray.set(ganglion_id, motor_id, FlexRayHardwareInterface::Controller::Velocity, vel);
    flex_ray.exchangeData();
    std::this_thread::sleep_for(std::chrono::seconds{2});

    flex_ray.command.params.tag = 1;
    flex_ray.command.frame[0].OperationMode[0] = Initialise;

    flex_ray.exchangeData();
    flex_ray.command.params.tag = 0;
    flex_ray.command.frame[0].OperationMode[0] = Initialise;
    flex_ray.exchangeData();

    switch (controlmode)
    {
      case 1:
        flex_ray.initPositionControl(ganglion_id, motor_id);
        break;
      case 2:
        flex_ray.initVelocityControl(ganglion_id, motor_id);
        break;
      case 3:
        flex_ray.initForceControl(ganglion_id, motor_id);
        break;
    }
  }
}
