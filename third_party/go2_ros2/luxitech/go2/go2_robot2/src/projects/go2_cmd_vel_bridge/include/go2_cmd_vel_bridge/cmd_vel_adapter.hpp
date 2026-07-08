#ifndef GO2_CMD_VEL_BRIDGE_CMD_VEL_ADAPTER_HPP_
#define GO2_CMD_VEL_BRIDGE_CMD_VEL_ADAPTER_HPP_

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"

namespace go2_cmd_vel_bridge
{

constexpr int64_t kSportApiIdStopMove = 1003;
constexpr int64_t kSportApiIdMove = 1008;

struct VelocityLimits
{
  double max_linear_x{0.5};
  double max_linear_y{0.5};
  double max_angular_z{1.0};
  double deadband{1.0e-4};
};

class CmdVelAdapter
{
public:
  explicit CmdVelAdapter(VelocityLimits limits = {});

  unitree_api::msg::Request toRequest(
    const geometry_msgs::msg::Twist & twist,
    int64_t request_id) const;

  unitree_api::msg::Request makeMoveRequest(
    double linear_x,
    double linear_y,
    double angular_z,
    int64_t request_id) const;

  unitree_api::msg::Request makeStopRequest(int64_t request_id) const;

private:
  static double clamp(double value, double limit);
  static double sanitize(double value);
  static std::string makeMoveParameter(double linear_x, double linear_y, double angular_z);
  bool isZeroCommand(double linear_x, double linear_y, double angular_z) const;

  VelocityLimits limits_;
};

}  // namespace go2_cmd_vel_bridge

#endif  // GO2_CMD_VEL_BRIDGE_CMD_VEL_ADAPTER_HPP_
