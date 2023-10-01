#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "md_lib/md2022.hpp"
#include "rclcpp/rclcpp.hpp"

namespace illias_wheelctrl {
class MovingWheel : public rclcpp::Node {
 public:
  MovingWheel();
  ~MovingWheel(){};

 private:
  // methods
  void set_param();
  void update();
  void cal_cmd();
  void send_cmd();

  // callbacks
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr);

  // variables
  
};
}  // namespace illias_wheelctrl