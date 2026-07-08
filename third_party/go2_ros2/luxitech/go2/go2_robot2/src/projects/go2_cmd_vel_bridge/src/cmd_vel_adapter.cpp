#include "go2_cmd_vel_bridge/cmd_vel_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace go2_cmd_vel_bridge
{

CmdVelAdapter::CmdVelAdapter(VelocityLimits limits)
: limits_(limits)
{
  limits_.max_linear_x = std::abs(limits_.max_linear_x);
  limits_.max_linear_y = std::abs(limits_.max_linear_y);
  limits_.max_angular_z = std::abs(limits_.max_angular_z);
  limits_.deadband = std::abs(limits_.deadband);
}

unitree_api::msg::Request CmdVelAdapter::toRequest(
  const geometry_msgs::msg::Twist & twist,
  const int64_t request_id) const
{
  const double linear_x = clamp(sanitize(twist.linear.x), limits_.max_linear_x);
  const double linear_y = clamp(sanitize(twist.linear.y), limits_.max_linear_y);
  const double angular_z = clamp(sanitize(twist.angular.z), limits_.max_angular_z);

  if (isZeroCommand(linear_x, linear_y, angular_z)) {
    return makeStopRequest(request_id);
  }

  return makeMoveRequest(linear_x, linear_y, angular_z, request_id);
}

unitree_api::msg::Request CmdVelAdapter::makeMoveRequest(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const int64_t request_id) const
{
  unitree_api::msg::Request request;
  request.header.identity.id = request_id;
  request.header.identity.api_id = kSportApiIdMove;
  request.parameter = makeMoveParameter(linear_x, linear_y, angular_z);
  return request;
}

unitree_api::msg::Request CmdVelAdapter::makeStopRequest(const int64_t request_id) const
{
  unitree_api::msg::Request request;
  request.header.identity.id = request_id;
  request.header.identity.api_id = kSportApiIdStopMove;
  request.parameter.clear();
  return request;
}

double CmdVelAdapter::clamp(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

double CmdVelAdapter::sanitize(const double value)
{
  return std::isfinite(value) ? value : 0.0;
}

std::string CmdVelAdapter::makeMoveParameter(
  const double linear_x,
  const double linear_y,
  const double angular_z)
{
  std::ostringstream out;
  out << std::setprecision(8)
      << "{\"x\":" << (linear_x == 0.0 ? 0.0 : linear_x)
      << ",\"y\":" << (linear_y == 0.0 ? 0.0 : linear_y)
      << ",\"z\":" << (angular_z == 0.0 ? 0.0 : angular_z) << "}";
  return out.str();
}

bool CmdVelAdapter::isZeroCommand(
  const double linear_x,
  const double linear_y,
  const double angular_z) const
{
  return std::abs(linear_x) <= limits_.deadband &&
         std::abs(linear_y) <= limits_.deadband &&
         std::abs(angular_z) <= limits_.deadband;
}

}  // namespace go2_cmd_vel_bridge
