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

  enum class CommandResult
  {
    Ok,
    GanglionNotAttached,
    MuscleDoesNotExist,
    ControllerNotInitialized
  };

  inline auto read_muscle(uint32_t ganglion, uint32_t muscle) -> muscleState_t
  {
    return protocol->read_muscle(ganglion, muscle);
  }

  inline auto connected_ganglions() -> std::bitset<Protocol<>::NumberOfGanglions>
  {
    return protocol->connected_ganglions();
  }

  inline CommandResult set(uint32_t ganglion, uint32_t muscle, ControlMode controller, float param)
  {
    if (ganglion > Protocol<>::NumberOfGanglions || muscle > Protocol<>::MusclesPerGanglion)
      return CommandResult::MuscleDoesNotExist;
    if (!protocol->connected_ganglions()[ganglion])
      return CommandResult::GanglionNotAttached;

    auto slot_ident = slots[ganglion].find(muscle);
    if (slot_ident == std::end(slots[ganglion]))
      return CommandResult::ControllerNotInitialized;

    slot_ident->second = std::move(slot_ident->second)
                             .enqueue(Send{ Send::Run{ param }, static_cast<comsControllerMode>(controller) })
                             .get()
                             .first;

    return CommandResult::Ok;
  }

  inline CommandResult set(std::string const& muscle_name, ControlMode controller, float param)
  {
    auto muscle_location = bus.muscles.find(muscle_name);

    if (muscle_location == std::end(bus.muscles))
      return CommandResult::MuscleDoesNotExist;

    auto ganglion = std::get<0>(muscle_location->second);
    auto muscle = std::get<1>(muscle_location->second);

    return set(ganglion, muscle, controller, param);
  }

  virtual ~FlexRayHardwareInterface();

  FlexRayHardwareInterface(FlexRayHardwareInterface&&) = default;
  FlexRayHardwareInterface& operator=(FlexRayHardwareInterface&&) = default;

  static auto connect(FlexRayBus&& bus) -> variant<FlexRayHardwareInterface, std::pair<FlexRayBus, FtResult>>;

private:
  FlexRayHardwareInterface(UsbChannel&& channel, FlexRayBus&& bus);
  void init(comsControllerMode mode, control_Parameters_t params, CompletionGuard<Protocol<>::input_t>& muscle);

  std::unique_ptr<Protocol<>> protocol;
  FlexRayBus bus;
  std::array<std::unordered_map<uint32_t, CompletionGuard<Protocol<>::input_t>>, Protocol<>::NumberOfGanglions> slots;
  std::unique_ptr<std::promise<void>> stop_work;
  std::future<void> worker;
};
