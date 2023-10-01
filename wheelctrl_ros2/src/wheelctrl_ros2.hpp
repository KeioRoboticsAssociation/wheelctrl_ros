// #pragma once

// #include "general_wheelctrl.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "md_lib/md2022.hpp"
// #include "md_lib/odrive.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rogilink2_interfaces/msg/frame.hpp"
// #include "rogilink2_interfaces/srv/request_add_topic.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"

// namespace illias {
// class WheelCtrlRos2 : rclcpp::Node {
//  public:
//   WheelCtrlRos2();
//   ~WheelCtrlRos2(){};

//  private:
//   void config_param();
//   void update();
//   void send_pos();
// };
// }  // namespace illias