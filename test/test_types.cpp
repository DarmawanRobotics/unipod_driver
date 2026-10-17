// Helpers that turn ROS parameters and ptz_cmd values into protocol values.
#include <gtest/gtest.h>

#include <limits>

#include "unipod_driver/types.hpp"

using namespace unipod_driver;

TEST(Types, NearestStreamRate)
{
  EXPECT_EQ(nearestStreamRate(0.0), kRateOff);
  EXPECT_EQ(nearestStreamRate(-3.0), kRateOff);
  EXPECT_EQ(nearestStreamRate(std::numeric_limits<double>::quiet_NaN()), kRateOff);
  EXPECT_EQ(nearestStreamRate(1.0), kRate2Hz);
  EXPECT_EQ(nearestStreamRate(10.0), kRate10Hz);
  EXPECT_EQ(nearestStreamRate(30.0), kRate20Hz);
  EXPECT_EQ(nearestStreamRate(40.0), kRate50Hz);
  EXPECT_EQ(nearestStreamRate(1000.0), kRate100Hz);
  EXPECT_DOUBLE_EQ(streamRateHz(kRate10Hz), 10.0);
}

TEST(Types, AxisToSpeed)
{
  EXPECT_EQ(axisToSpeed(0.2, 100), 20);
  EXPECT_EQ(axisToSpeed(-1.0, 100), -100);
  EXPECT_EQ(axisToSpeed(5.0, 100), 100);     // clamped axis
  EXPECT_EQ(axisToSpeed(1.0, 250), 100);     // clamped max
  EXPECT_EQ(axisToSpeed(0.5, 40), 20);
  EXPECT_EQ(axisToSpeed(std::numeric_limits<double>::infinity(), 100), 0);
}

TEST(Types, AxisToZoom)
{
  EXPECT_EQ(axisToZoom(0.0), 0);
  EXPECT_EQ(axisToZoom(0.4), 0);
  EXPECT_EQ(axisToZoom(0.8), 1);
  EXPECT_EQ(axisToZoom(-1.0), -1);
  EXPECT_EQ(axisToZoom(std::numeric_limits<double>::quiet_NaN()), 0);
}
