#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <thread>
#include <utility>

#include "flexrayusbinterface/Protocol.hpp"
#include "flexrayusbinterface/Range.hpp"

FlexRayHardwareInterface::FlexRayHardwareInterface(UsbChannel&& usb_channel, FlexRayBus&& flex_bus)
  : protocol{ new Protocol<>{ std::move(usb_channel) } }
  , bus{ std::move(flex_bus) }
  , stop_work{ new std::promise<void>{} }
{
  Protocol<>* _protocol = protocol.get();
  worker = std::async(std::launch::async,
                      [_protocol](std::future<void> stop) {
                        while (stop.wait_for(std::chrono::seconds{ 0 }) != std::future_status::ready)
                        {
                          _protocol->exchange_data();
                        }
                      },
                      stop_work->get_future());

  for (auto&& muscle_iter : bus.muscles)
  {
    auto ganglion_id = std::get<0>(muscle_iter.second);
    auto muscle_id = std::get<1>(muscle_iter.second);
    Muscle& muscle = std::get<2>(muscle_iter.second);

    auto slot = protocol->get_slot(ganglion_id, muscle_id);
    if (!slot)
      continue;

    auto& slot_guard = slots[ganglion_id].emplace(muscle_id, std::move(*slot)).first->second;

    if (auto& controller = std::get<0>(muscle.controllers))
      init(comsControllerMode::Position, (*controller).parameters(), slot_guard);
    if (auto& controller = std::get<1>(muscle.controllers))
      init(comsControllerMode::Velocity, (*controller).parameters(), slot_guard);
    if (auto& controller = std::get<2>(muscle.controllers))
      init(comsControllerMode::Force, (*controller).parameters(), slot_guard);
  }
}

FlexRayHardwareInterface::~FlexRayHardwareInterface()
{
  if (stop_work)
    stop_work->set_value();
}

auto FlexRayHardwareInterface::connect(FlexRayBus&& bus)
    -> variant<FlexRayHardwareInterface, std::pair<FlexRayBus, FtResult>>
{
  return UsbChannel::open(bus.serial_number)
      .match(
          [&bus](UsbChannel& usb) -> variant<FlexRayHardwareInterface, std::pair<FlexRayBus, FtResult>> {
            return FlexRayHardwareInterface{ std::move(usb), std::move(bus) };
          },
          [&bus](FtResult err) -> variant<FlexRayHardwareInterface, std::pair<FlexRayBus, FtResult>> {
            return std::make_pair(std::move(bus), err);
          });
}

void FlexRayHardwareInterface::init(comsControllerMode mode, control_Parameters_t params,
                                    CompletionGuard<Protocol<>::input_t>& muscle)
{
  muscle = std::move(muscle).enqueue(Enqueue{ mode, params }).get().first;
}
