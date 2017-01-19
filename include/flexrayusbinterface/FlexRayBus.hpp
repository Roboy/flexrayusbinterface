#pragma once

#include <string>
#include <unordered_map>

#include <boost/optional.hpp>
#include "units.h"

#include "CommunicationData.h"

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

struct RawCtrl
{
  control_Parameters_t params;
};

struct PositionCtrl
{
  control_Parameters_t params;
};

struct VelocityCtrl
{
  control_Parameters_t params;
};

struct ForceCtrl
{
  control_Parameters_t params;
};

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
  std::string description;
  units::frequency::hertz_t rate;

  container_t ganglions;
};
