#pragma once

// std
#include <array>
#include <bitset>
#include <chrono>
#include <string>
#include <vector>

#include "flexrayusbinterface/FlexRayBus.hpp"
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

  inline auto read_muscle(uint32_t ganglion, uint32_t muscle) -> muscleState_t
  {
    return protocol->read_muscle(ganglion, muscle);
  }

  inline auto connected_ganglions() -> std::bitset<Protocol<>::NumberOfGanglions>
  {
    return protocol->connected_ganglions();
  }

  inline void set(uint32_t ganglion, uint32_t motor, ControlMode controller, float param)
  {
    slots[ganglion][motor] = std::move(slots[ganglion][motor])
                                 .enqueue(Send{ Send::Run{ param }, static_cast<comsControllerMode>(controller) })
                                 .get()
                                 .first;
  }

  virtual ~FlexRayHardwareInterface();

  FlexRayHardwareInterface(FlexRayHardwareInterface&&) = default;
  FlexRayHardwareInterface& operator=(FlexRayHardwareInterface&&) = default;

  static auto connect(FlexRayBus& bus) -> variant<FlexRayHardwareInterface, FtResult>;


private:
  FlexRayHardwareInterface(UsbChannel&& channel, FlexRayBus&& bus);
  void init(comsControllerMode mode, control_Parameters_t params, uint32_t ganglion, uint32_t motor);

  std::unique_ptr<Protocol<>> protocol;
  FlexRayBus bus;
  std::vector<std::vector<CompletionGuard<Protocol<>::input_t>>> slots;
  std::unique_ptr<std::promise<void>> stop_work;
  std::future<void> worker;
};
