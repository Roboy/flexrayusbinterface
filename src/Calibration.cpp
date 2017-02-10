#include <chrono>
#include <thread>

#include "flexrayusbinterface/FlexRayBus.hpp"
#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

void relaxSpring(FlexRayHardwareInterface& flex_ray, uint32_t ganglion_id, uint32_t motor_id)
{
  uint32_t count = 0;
  float aux = 0;
  uint32_t p = 0;
  float vel;
  float tendonDisplacement_t[3];
  float tendonDisplacement_t2[10000];

  flex_ray.set(ganglion_id, motor_id, ControlMode::Velocity, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds{ 300 });
  tendonDisplacement_t[0] =
      flex_ray.read_muscle(ganglion_id, motor_id).get<muscleState_t>().tendonDisplacement / 32768.0f;  // tendon displacemnte iniziale
  uint32_t i = 1;
  for (i = 1; i < 3; i++)
  {
    flex_ray.set(ganglion_id, motor_id, ControlMode::Velocity, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds{ 500 });
    tendonDisplacement_t[i] = flex_ray.read_muscle(ganglion_id, motor_id).get<muscleState_t>().tendonDisplacement / 32768.0f;
  }

  uint32_t t = 0;
  tendonDisplacement_t2[0] = flex_ray.read_muscle(ganglion_id, motor_id).get<muscleState_t>().tendonDisplacement / 32768.0f;

  if (tendonDisplacement_t[2] < tendonDisplacement_t[0])
    vel = 3;
  else if (tendonDisplacement_t[1] < tendonDisplacement_t[2])
    vel = -3;
  else
    vel = 0;

  do
  {
    flex_ray.set(ganglion_id, motor_id, ControlMode::Velocity, vel);
    std::this_thread::sleep_for(std::chrono::milliseconds{ 300 });
    tendonDisplacement_t2[t + 1] = flex_ray.read_muscle(ganglion_id, motor_id).get<muscleState_t>().tendonDisplacement / 32768.0f;
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
    relaxSpring(flex_ray, ganglion_id, motor_id);

  else
  {
    vel = 0;
    flex_ray.set(ganglion_id, motor_id, ControlMode::Velocity, vel);
    std::this_thread::sleep_for(std::chrono::seconds{ 2 });
  }
}
