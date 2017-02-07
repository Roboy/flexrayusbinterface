#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

#include <thread>
#include <utility>

#include "flexrayusbinterface/Protocol.hpp"
#include "flexrayusbinterface/Range.hpp"

FlexRayHardwareInterface::FlexRayHardwareInterface(UsbChannel&& usb_channel, FlexRayBus&& flex_bus)
  : protocol{ new Protocol<>{ std::move(usb_channel) } }
  , bus{ std::move(flex_bus) }
  , slots(protocol->NumberOfGanglions)
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
  for (std::size_t i = 0; i < protocol->NumberOfGanglions; ++i)
  {
    auto& muscle_slots = slots[i];
    for (std::size_t j = 0; j < protocol->MusclesPerGanglion; ++j)
      muscle_slots.emplace_back(std::move(*protocol->get_slot(i, j)));
  }

  for (auto&& ganglion_iter : bus.ganglions) {
    for (auto&& muscle_iter : ganglion_iter.second.muscles)
    {
      auto ganglion_id = ganglion_iter.first;
      auto muscle_id = muscle_iter.first;
      Muscle& muscle = muscle_iter.second;
      if (auto& controller = std::get<0>(muscle.controllers))
      {
        init(comsControllerMode::Position, (*controller).parameters(), ganglion_id, muscle_id);
      }
      if (auto& controller = std::get<1>(muscle.controllers))
      {
        init(comsControllerMode::Velocity, (*controller).parameters(), ganglion_id, muscle_id);
      }
      if (auto& controller = std::get<2>(muscle.controllers))
      {
        init(comsControllerMode::Force, (*controller).parameters(), ganglion_id, muscle_id);
      }
    }
  }
}

FlexRayHardwareInterface::~FlexRayHardwareInterface()
{
  if (stop_work)
    stop_work->set_value();
}

auto FlexRayHardwareInterface::connect(FlexRayBus& bus) -> variant<FlexRayHardwareInterface, FtResult>
{
  return UsbChannel::open(bus.serial_number)
      .match(
          [&bus](UsbChannel& usb) -> variant<FlexRayHardwareInterface, FtResult> {
            return FlexRayHardwareInterface{ std::move(usb), std::move(bus) };
          },
          [](FtResult err) -> variant<FlexRayHardwareInterface, FtResult> { return err; });
}

void FlexRayHardwareInterface::init(comsControllerMode mode, control_Parameters_t params, uint32_t ganglion,
                                    uint32_t motor)
{
  slots[ganglion][motor] = std::move(slots[ganglion][motor]).enqueue(Enqueue{ mode, params }).get().first;
}
