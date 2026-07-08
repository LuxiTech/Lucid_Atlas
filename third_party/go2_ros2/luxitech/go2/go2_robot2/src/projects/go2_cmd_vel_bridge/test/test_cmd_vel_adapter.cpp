#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "go2_cmd_vel_bridge/cmd_vel_adapter.hpp"

namespace
{

using go2_cmd_vel_bridge::CmdVelAdapter;
using go2_cmd_vel_bridge::VelocityLimits;
using go2_cmd_vel_bridge::kSportApiIdMove;
using go2_cmd_vel_bridge::kSportApiIdStopMove;

geometry_msgs::msg::Twist makeTwist(double x, double y, double yaw)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = x;
  twist.linear.y = y;
  twist.angular.z = yaw;
  return twist;
}

TEST(CmdVelAdapterTest, ConvertsNonZeroCmdVelToMoveRequest)
{
  const CmdVelAdapter adapter;

  const auto request = adapter.toRequest(makeTwist(0.25, -0.1, 0.5), 7);

  EXPECT_EQ(request.header.identity.id, 7);
  EXPECT_EQ(request.header.identity.api_id, kSportApiIdMove);
  EXPECT_EQ(request.parameter, "{\"x\":0.25,\"y\":-0.1,\"z\":0.5}");
}

TEST(CmdVelAdapterTest, ClampsVelocityToConfiguredLimits)
{
  const CmdVelAdapter adapter(VelocityLimits{
    0.5,
    0.5,
    1.0,
    1.0e-4,
  });

  const auto request = adapter.toRequest(makeTwist(2.0, -2.0, 4.0), 8);

  EXPECT_EQ(request.header.identity.api_id, kSportApiIdMove);
  EXPECT_EQ(request.parameter, "{\"x\":0.5,\"y\":-0.5,\"z\":1}");
}

TEST(CmdVelAdapterTest, ConvertsZeroCmdVelToStopMoveRequest)
{
  const CmdVelAdapter adapter;

  const auto request = adapter.toRequest(makeTwist(0.0, 0.0, 0.0), 9);

  EXPECT_EQ(request.header.identity.id, 9);
  EXPECT_EQ(request.header.identity.api_id, kSportApiIdStopMove);
  EXPECT_TRUE(request.parameter.empty());
}

TEST(CmdVelAdapterTest, AppliesDeadbandBeforeChoosingMove)
{
  const CmdVelAdapter adapter(VelocityLimits{
    0.8,
    0.6,
    1.8,
    0.01,
  });

  const auto request = adapter.toRequest(makeTwist(0.005, -0.004, 0.006), 10);

  EXPECT_EQ(request.header.identity.api_id, kSportApiIdStopMove);
}

TEST(CmdVelAdapterTest, TreatsNonFiniteValuesAsZero)
{
  const CmdVelAdapter adapter;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  const auto request = adapter.toRequest(makeTwist(nan, inf, -inf), 11);

  EXPECT_EQ(request.header.identity.api_id, kSportApiIdStopMove);
  EXPECT_TRUE(request.parameter.empty());
}

}  // namespace
