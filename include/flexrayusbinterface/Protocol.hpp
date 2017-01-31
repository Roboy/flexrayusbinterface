#pragma once

#include <sstream>
#include <thread>

#include <units.h>
#include <boost/optional.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "flexrayusbinterface/CommunicationData.h"
#include "flexrayusbinterface/Message.hpp"
#include "flexrayusbinterface/Muscle.hpp"
#include "flexrayusbinterface/Types.hpp"
#include "flexrayusbinterface/UsbChannel.hpp"

struct Send
{
  float set_point;
  comsControllerMode controller;
};

struct Enqueue
{
  control_Parameters_t params;
};

template <std::size_t NumberOfGanglions = 6, std::size_t MusclesPerGanglion = 4, std::size_t DatasetSize = 310>
class Protocol
{
public:
  static_assert(NumberOfGanglions > 0, "The protocol must declare at least 1 ganglion.");
  using input_t = variant<Send, Enqueue>;
  using output_t = std::string;
  using message_t = std::string;
  using channel_t = UsbChannel;

  template <typename... Args>
  explicit Protocol(Args&&... args) : channel{ std::forward<Args>(args)... }
  {
  }

  auto get_slot(size_t ganglion_number, size_t muscle_number) -> boost::optional<CompletionGuard<input_t>>
  {
    if (ganglion_number > NumberOfGanglions || muscle_number > MusclesPerGanglion)
      return boost::none;

    auto& slot_input = input[ganglion_number][muscle_number];

    if (static_cast<bool>(input))
      return boost::none;

    slot_input.reset(
        new Mutex<Slot<input_t>::data_t>{ boost::in_place(std::make_pair(std::promise<Completion>{}, input_t{})) });
    return { *slot_input };
  }

  auto read_muscle(size_t ganglion_number, size_t muscle_number) -> muscleState_t
  {
    boost::shared_lock<boost::shared_mutex> _{ read_mutex };
    return ganglion_data[ganglion_number].muscleState[muscle_number];
  }

  auto exchange_data() -> void
  {
    channel.write(create_message());
    // WAIT FOR DATA TO ARRIVE
    while (channel.bytes_available().match([](DWORD bytes) { return bytes; }, [](FtResult) { return 0; }) <
           DatasetSize * sizeof(WORD))
    {
      std::this_thread::yield();
    }

    channel.read(std::string(DatasetSize * sizeof(WORD), '\0'))
        .match(
            [this](std::string& data) {
              std::stringstream buffer;
              WORD ganglions;
              buffer.str(data);
              {
                std::unique_lock<boost::shared_mutex> _{ read_mutex };
                Parser<DatasetSize * sizeof(WORD)>{}.add(ganglion_data).add(ganglions).read(buffer);
              }
              activeGanglionsMask = ganglions;
            },
            [](FtResult) {});
  }

private:
  using time_point_t = std::chrono::time_point<std::chrono::steady_clock>;

  auto create_message() -> std::vector<WORD>
  {
    // Handle the completion of current dynamic slot
    if (static_cast<bool>(dynamic_slot_completion) &&
        (std::chrono::steady_clock::now() - dynamic_slot_completion.get().template get<time_point_t>() >=
         dynamic_wait_time))
    {
      dynamic_slot_completion.get().template get<std::promise<Completion>>().set_value(Completion::Completed);
      comsCommandFrame& frame = command.frame[dynamic_slot_completion.get().template get<3>()];
      auto muscle = dynamic_slot_completion.get().template get<4>();
      frame.OperationMode[muscle] = dynamic_slot_completion.get().template get<1>();
      frame.ControlMode[muscle] = dynamic_slot_completion.get().template get<2>();
      dynamic_slot_completion = boost::none;
    }

    // Handle the messages that need to be sent
    for (std::size_t ganglion_id = 0; ganglion_id < NumberOfGanglions; ++ganglion_id)
    {
      for (std::size_t muscle_id = 0; muscle_id < MusclesPerGanglion; ++muscle_id)
      {
        auto& slot_input = input[ganglion_id][muscle_id];
        if (!static_cast<bool>(slot_input))
          continue;
        auto guard = slot_input->lock();
        if (!static_cast<bool>(guard.get()))
          continue;
        auto& frame = command.frame[ganglion_id];
        auto& message = *guard.get();
        message.second.match(
            [&](Send& cmd) mutable {
              frame.sp[muscle_id] = cmd.set_point;
              frame.ControlMode[muscle_id] = cmd.controller;
              frame.OperationMode[muscle_id] = comsOperationMode::Run;
              message.first.set_value(Completion::Completed);
              guard.get() = boost::none;
            },
            [&](Enqueue& cmd) {
              if (static_cast<bool>(dynamic_slot_completion))
                return;
              command.params = cmd.params;
              dynamic_slot_completion = std::make_tuple(std::move(message.first), frame.OperationMode[muscle_id],
                                                        frame.ControlMode[muscle_id], ganglion_id, muscle_id,
                                                        std::chrono::steady_clock::now());
              guard.get() = boost::none;
            });
      }
    }

    std::vector<WORD> buffer(DatasetSize, 0);
    std::copy_n(reinterpret_cast<WORD*>(&command), sizeof(command) / sizeof(WORD), std::begin(buffer));

    return buffer;
  }

  std::array<std::array<std::unique_ptr<Mutex<Slot<input_t>::data_t>>, MusclesPerGanglion>, NumberOfGanglions> input;

  struct
  {
    //! command frames containing motor control parameters for the ganglia
    std::array<comsCommandFrame, NumberOfGanglions> frame;
    //! control parameters for motor
    control_Parameters_t params;
  } command;

  boost::shared_mutex read_mutex;
  std::array<ganglionData_t, NumberOfGanglions> ganglion_data;

  std::bitset<NumberOfGanglions> activeGanglionsMask{ 0 };

  channel_t channel;
  boost::optional<std::tuple<std::promise<Completion>, comsOperationMode, comsControllerMode, int, int, time_point_t>>
      dynamic_slot_completion;
  static constexpr std::chrono::milliseconds dynamic_wait_time{ 10 };
};
