#pragma once

#include <string>
#include <unordered_map>

#include <boost/optional.hpp>
#include "units.h"

#include "CommunicationData.h"
#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

template <typename... T>
class TupleOfOptional
{
public:
  template <typename U>
  using optional = boost::optional<U>;

  template <typename U>
  auto get() -> optional<U>
  {
    return std::get<optional<U>>(data);
  }

  template <typename U>
  void set(U&& new_val)
  {
    std::get<optional<U>>(data) = std::forward<U>(new_val);
  }

private:
  std::tuple<optional<T>...> data;
};

template <typename LengthU, typename ForceU>
struct Spring
{
  using force = ForceU;
  using length = LengthU;
  using force_per_length = units::compound_unit<force, units::inverse<length>>;
  using force_per_area = units::compound_unit<force, units::inverse<units::squared<length>>>;
  using force_per_volume = units::compound_unit<force, units::inverse<units::cubed<length>>>;

  using length_t = units::unit_t<length>;
  using force_t = units::unit_t<force>;
  using force_per_length_t = units::unit_t<force_per_length>;
  using force_per_area_t = units::unit_t<force_per_area>;
  using force_per_volume_t = units::unit_t<force_per_volume>;

  force_t constant;
  force_per_length_t linear;
  force_per_area_t quadratic;
  force_per_volume_t cubic;

  auto tension(length_t displacement) const -> force_t
  {
    return constant + linear * displacement + quadratic * units::math::pow<2>(displacement) +
           cubic * units::math::pow<3>(displacement);
  }
};

template <FlexRayHardwareInterface::Controller C>
struct control_unit_traits;

template <>
struct control_unit_traits<FlexRayHardwareInterface::Controller::Raw>
{
  using unit = units::dimensionless::dimensionless;
};

template <>
struct control_unit_traits<FlexRayHardwareInterface::Controller::Position>
{
  using unit = units::angle::radian;
};

template <>
struct control_unit_traits<FlexRayHardwareInterface::Controller::Velocity>
{
  using unit = units::angular_velocity::radians_per_second;
};

template <>
struct control_unit_traits<FlexRayHardwareInterface::Controller::Torque>
{
  using unit = units::torque::newton_meter;
};

template <>
struct control_unit_traits<FlexRayHardwareInterface::Controller::Force>
{
  using unit = units::force::newton;
};

template <FlexRayHardwareInterface::Controller C>
using control_unit = typename control_unit_traits<C>::unit;

template <FlexRayHardwareInterface::Controller C>
class Controller
{
  static constexpr FlexRayHardwareInterface::Controller controller = C;
  using control_t = units::unit_t<control_unit<C>>;
  using torque_t =
      units::unit_t<units::compound_unit<units::torque::newton_meter, units::inverse<units::current::ampere>>>;
  using spring_t = Spring<units::length::meter, units::force::newton>;

  float Pgain;
  float Igain;
  float Dgain;
  float forward_gain;
  float dead_band;
  float integral;
  float integral_pos_min;
  float integral_pos_max;
  control_t sp_pos_min;
  control_t sp_pos_max;
  torque_t torque_constant;
  spring_t spring;

  Controller(float Pgain, float Igain, float Dgain, float forward_gain, float dead_band, float integral,
             float integral_pos_min, float integral_pos_max, control_t sp_pos_min, control_t sp_pos_max,
             torque_t torque_constant, spring_t spring)
    : Pgain{ Pgain }
    , Igain{ Igain }
    , Dgain{ Dgain }
    , forward_gain{ forward_gain }
    , dead_band{ dead_band }
    , integral{ integral }
    , integral_pos_min{ integral_pos_min }
    , integral_pos_max{ integral_pos_max }
    , sp_pos_min{ sp_pos_min }
    , sp_pos_max{ sp_pos_max }
    , torque_constant{ torque_constant }
    , spring{ spring }
  {
  }

  auto parameters() const -> control_Parameters_t
  {
    control_Parameters_t p;
    p.spNegMax = sp_pos_min.template to<decltype(p.spNegMax)>();
    p.spPosMax = sp_pos_max.template to<decltype(p.spPosMax)>();

    p.params.pidParameters.integral = integral;
    p.params.pidParameters.pgain = Pgain;
    p.params.pidParameters.igain = Igain;
    p.params.pidParameters.dgain = Dgain;
    p.params.pidParameters.forwardGain = forward_gain;
    p.params.pidParameters.deadBand = dead_band;
    p.params.pidParameters.IntegralPosMax = integral_pos_max;
    p.params.pidParameters.IntegralNegMax = integral_pos_min;

    p.polyPar[0] = spring.constant.template to<decltype(p.polyPar[0])>();
    p.polyPar[1] = spring.linear.template to<decltype(p.polyPar[1])>();
    p.polyPar[2] = spring.quadratic.template to<decltype(p.polyPar[2])>();
    p.polyPar[3] = spring.cubic.template to<decltype(p.polyPar[3])>();

    return p;
  }
};

using RawCtrl = Controller<FlexRayHardwareInterface::Controller::Raw>;

using PositionCtrl = Controller<FlexRayHardwareInterface::Controller::Position>;

using VelocityCtrl = Controller<FlexRayHardwareInterface::Controller::Velocity>;

using TorqueCtrl = Controller<FlexRayHardwareInterface::Controller::Torque>;

using ForceCtrl = Controller<FlexRayHardwareInterface::Controller::Force>;

struct Ganglion
{
  using muscle_id_t = uint8_t;
  using muscle_t = TupleOfOptional<RawCtrl, PositionCtrl, VelocityCtrl, ForceCtrl>;
  using container_t = std::unordered_map<muscle_id_t, muscle_t>;

  container_t muscles;
};

struct FlexRayBus
{
  using ganglion_id_t = uint8_t;
  using container_t = std::unordered_map<ganglion_id_t, Ganglion>;

  std::string serial_number;
  units::frequency::hertz_t rate;

  container_t ganglions;

  inline FlexRayBus(std::string serial_number, units::frequency::hertz_t rate)
    : serial_number{ std::move(serial_number) }, rate{ rate }
  {
  }
};
