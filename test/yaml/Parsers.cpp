#include "flexrayusbinterface/Parsers.hpp"
#include <gtest/gtest.h>

TEST(ParseFlexRayBus, root)
{
  auto node = YAML::LoadFile("RobotDescription.yaml");
  FlexRayBus bus = node["FlexRay"].as<FlexRayBus>();
  EXPECT_EQ(bus.ganglions.size(), 1);
  EXPECT_EQ(bus.ganglions[0].muscles.size(), 1);
  EXPECT_EQ(bus.serial_number, "FTVXQGER");
  auto i_biceps = bus.muscles.find("left_biceps");
  ASSERT_EQ(bus.muscles.size(), 1);
  EXPECT_NE(i_biceps, bus.muscles.end());
  Muscle& muscle = std::get<2>(i_biceps->second);
  EXPECT_TRUE(static_cast<bool>(std::get<0>(muscle.controllers)));
}
