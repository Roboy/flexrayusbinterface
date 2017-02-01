#pragma once

#include <sstream>
#include <thread>
#include <tuple>

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
  struct Run
  {
    float pos;
  };
  struct Disable
  {
  };
  variant<Run, Disable> command;
  comsControllerMode controller;
};

struct Enqueue
{
  comsControllerMode controller;
  control_Parameters_t params;
};

template <std::size_t N = 6, std::size_t M = 4, std::size_t DatasetSize = 310>
class Protocol
{
public:
  static constexpr std::size_t NumberOfGanglions{ N };
  static constexpr std::size_t MusclesPerGanglion{ M };

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

    if (static_cast<bool>(slot_input))
      return boost::none;

    auto slot_channel = make_channel<input_t>();
    slot_input = std::move(slot_channel.second);
    return CompletionGuard<input_t>{ std::move(slot_channel.first) };
  }

  auto read_muscle(size_t ganglion_number, size_t muscle_number) -> muscleState_t
  {
    boost::shared_lock<boost::shared_mutex> _{ read_mutex };
    return ganglion_data[ganglion_number].muscleState[muscle_number];
  }

  auto connected_ganglions() -> std::bitset<NumberOfGanglions>
  {
    boost::shared_lock<boost::shared_mutex> _{ read_mutex };
    return activeGanglionsMask;
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

              std::unique_lock<boost::shared_mutex> _{ read_mutex };
              Parser<DatasetSize * sizeof(WORD)>{}.add(ganglion_data).add(ganglions).read(buffer);
              activeGanglionsMask = ganglions;
            },
            [](FtResult) {});
  }

private:
  using time_point_t = std::chrono::time_point<std::chrono::steady_clock>;
  using completion_data_t =
      std::tuple<std::promise<Completion>, comsOperationMode, comsControllerMode, int, int, time_point_t>;

  auto create_message() -> std::vector<WORD>
  {
    // Handle the completion of current dynamic slot
    if (static_cast<bool>(dynamic_slot_completion) &&
        (std::chrono::steady_clock::now() - std::get<5>(dynamic_slot_completion.get()) >= dynamic_wait_time()))
    {
      std::get<0>(*dynamic_slot_completion).set_value(Completion::Completed);
      comsCommandFrame& frame = command.frame[std::get<3>(*dynamic_slot_completion)];
      auto muscle = std::get<4>(*dynamic_slot_completion);
      frame.OperationMode[muscle] = std::get<1>(*dynamic_slot_completion);
      frame.ControlMode[muscle] = std::get<2>(*dynamic_slot_completion);
      dynamic_slot_completion = boost::none;
    }

    // Handle the messages that need to be sent
    for (int ganglion_id = 0; ganglion_id < NumberOfGanglions; ++ganglion_id)
    {
      for (int muscle_id = 0; muscle_id < MusclesPerGanglion; ++muscle_id)
      {
        if (static_cast<bool>(dynamic_slot_completion) && std::get<3>(*dynamic_slot_completion) == ganglion_id &&
            std::get<4>(*dynamic_slot_completion) == muscle_id)
          continue;
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
              cmd.command.match(
                  [&](Send::Run& run) {
                    frame.sp[muscle_id] = run.pos;
                    frame.OperationMode[muscle_id] = comsOperationMode::Run;
                  },
                  [&](Send::Disable) { frame.OperationMode[muscle_id] = comsOperationMode::Disable; });
              frame.ControlMode[muscle_id] = cmd.controller;
              message.first.set_value(Completion::Completed);
              guard.get() = boost::none;
            },
            [&](Enqueue& cmd) {
              if (static_cast<bool>(dynamic_slot_completion))
                return;
              command.params = cmd.params;
              dynamic_slot_completion =
                  completion_data_t{ std::move(message.first),
                                     static_cast<comsOperationMode>(frame.OperationMode[muscle_id]),
                                     static_cast<comsControllerMode>(frame.ControlMode[muscle_id]),
                                     ganglion_id,
                                     muscle_id,
                                     std::chrono::steady_clock::now() };
              guard.get() = boost::none;
            });
      }
    }
    std::vector<WORD> buffer(DatasetSize, 0);
    std::copy_n(reinterpret_cast<WORD*>(&command), sizeof(command) / sizeof(WORD), std::begin(buffer));

    return buffer;
  }

  std::array<std::array<std::unique_ptr<Mutex<typename Slot<input_t>::data_t>>, MusclesPerGanglion>, NumberOfGanglions>
      input;

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
  boost::optional<completion_data_t> dynamic_slot_completion;

  static constexpr auto dynamic_wait_time() -> std::chrono::milliseconds
  {
    return std::chrono::milliseconds{ 20 };
  }
};
