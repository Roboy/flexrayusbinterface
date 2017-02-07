#include "yaml-cpp/yaml.h"

#include "flexrayusbinterface/FlexRayBus.hpp"
#include "flexrayusbinterface/Types.hpp"

namespace YAML
{
template <>
struct convert<Spring<>>
{
  /* static Node encode(Spring<> const& rhs) {
      Node node;
      return node;
  } */

  static bool decode(Node const& node, Spring<>& rhs)
  {
    rhs = Spring<>{ Spring<>::force_t{ node["constant"].as<double>() },
                    Spring<>::force_per_length_t{ node["linear"].as<double>() },
                    Spring<>::force_per_area_t{ node["quadratic"].as<double>() },
                    Spring<>::force_per_volume_t{ node["cubic"].as<double>() } };
    return true;
  }
};

template <>
struct convert<PositionCtrl>
{
  /* static Node encode(PositionCtrl const& rhs) {
      Node node;
      return node;
  } */
  using Inner = PositionCtrl;

  static bool decode(Node const& node, Inner& rhs)
  {
    rhs = Inner{ node["output_pos_max"].as<int32_t>(),
                 node["output_neg_max"].as<int32_t>(),
                 node["time_period"].as<float>(),
                 node["rad_per_encoder_count"].as<float>(),
                 node["P_gain"].as<float>(),
                 node["I_gain"].as<float>(),
                 node["D_gain"].as<float>(),
                 node["forward_gain"].as<float>(),
                 node["dead_band"].as<float>(),
                 node["integral"].as<float>(),
                 node["integral_pos_min"].as<float>(),
                 node["integral_pos_max"].as<float>(),
                 Inner::control_t{ node["sp_pos_min"].as<double>() },
                 Inner::control_t{ node["sp_pos_max"].as<double>() },
                 Inner::torque_t{ 1.0 },
                 Inner::spring_t{} };
    return true;
  }
};

template <>
struct convert<VelocityCtrl>
{
  /* static Node encode(VelocityCtrl const& rhs) {
      Node node;
      return node;
  } */
  using Inner = VelocityCtrl;

  static bool decode(Node const& node, Inner& rhs)
  {
    rhs = Inner{ node["output_pos_max"].as<int32_t>(),
                 node["output_neg_max"].as<int32_t>(),
                 node["time_period"].as<float>(),
                 node["rad_per_encoder_count"].as<float>(),
                 node["P_gain"].as<float>(),
                 node["I_gain"].as<float>(),
                 node["D_gain"].as<float>(),
                 node["forward_gain"].as<float>(),
                 node["dead_band"].as<float>(),
                 node["integral"].as<float>(),
                 node["integral_pos_min"].as<float>(),
                 node["integral_pos_max"].as<float>(),
                 Inner::control_t{ node["sp_pos_min"].as<double>() },
                 Inner::control_t{ node["sp_pos_max"].as<double>() },
                 Inner::torque_t{ 1.0 },
                 Inner::spring_t{} };
    return true;
  }
};

template <>
struct convert<ForceCtrl>
{
  /* static Node encode(ForceCtrl const& rhs) {
      Node node;
      return node;
  } */
  using Inner = ForceCtrl;

  static bool decode(Node const& node, Inner& rhs)
  {
    rhs = Inner{ node["output_pos_max"].as<int32_t>(),
                 node["output_neg_max"].as<int32_t>(),
                 node["time_period"].as<float>(),
                 node["rad_per_encoder_count"].as<float>(),
                 node["P_gain"].as<float>(),
                 node["I_gain"].as<float>(),
                 node["D_gain"].as<float>(),
                 node["forward_gain"].as<float>(),
                 node["dead_band"].as<float>(),
                 node["integral"].as<float>(),
                 node["integral_pos_min"].as<float>(),
                 node["integral_pos_max"].as<float>(),
                 Inner::control_t{ node["sp_pos_min"].as<double>() },
                 Inner::control_t{ node["sp_pos_max"].as<double>() },
                 Inner::torque_t{ node["torque"].as<double>() },
                 node["spring"].as<Inner::spring_t>() };
    return true;
  }
};

template <>
struct convert<Muscle>
{
  /* static Node encode(Muscle const& rhs) {
      Node node;
      return node;
  } */

  static bool decode(Node const& node, Muscle& rhs)
  {
    rhs.id = node["id"].as<std::string>();
    if (node["Force"])
      rhs.add_controller(node["Force"].as<ForceCtrl>());
    if (node["Velocity"])
      rhs.add_controller(node["Velocity"].as<VelocityCtrl>());
    if (node["Position"])
      rhs.add_controller(node["Position"].as<PositionCtrl>());
    return true;
  }
};

template <>
struct convert<Ganglion>
{
  /* static Node encode(Ganglion const& rhs) {
      Node node;
      return node;
  } */

  static bool decode(Node const& node, Ganglion& rhs)
  {
    for (int i = 0; i < 4; ++i)
    {
      auto& muscle = node[std::string{ "muscle " } + std::to_string(i)];
      if (muscle)
        rhs.muscles[i] = muscle.as<Muscle>();
    }
    return true;
  }
};

template <>
struct convert<FlexRayBus>
{
  /* static Node encode(FlexRayBus const& rhs) {
      Node node;
      return node;
  } */

  static bool decode(Node const& node, FlexRayBus& rhs)
  {
    rhs.serial_number = node["serial"].as<std::string>();
    rhs.ganglions.clear();
    rhs.muscles.clear();
    for (int i = 0; i < 6; ++i)
    {
      auto& ganglion = node[std::string{ "ganglion " } + std::to_string(i)];
      if (ganglion)
        rhs.ganglions[i] = ganglion.as<Ganglion>();
    }
    for (auto&& ganglion_it: rhs.ganglions)
        for (auto&& muscle_it: ganglion_it.second.muscles)
            rhs.muscles.emplace(muscle_it.second.id, std::forward_as_tuple(ganglion_it.first, muscle_it.first, std::ref(muscle_it.second)));
    return true;
  }
};
}
