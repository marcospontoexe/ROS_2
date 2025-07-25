#ifndef COMPOSITION__MOVEROBOT_COMPONENT_HPP_
#define COMPOSITION__MOVEROBOT_COMPONENT_HPP_

#include "marcos_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace marcos_components
{

class MoveRobot : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit MoveRobot(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

#endif  // COMPOSITION__MOVEROBOT_COMPONENT_HPP_