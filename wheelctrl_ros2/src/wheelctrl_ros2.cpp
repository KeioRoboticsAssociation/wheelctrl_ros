#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"

using namespace std::chrono_literals;

class WheelCtrlRos2 : public rclcpp::Node
{
public:
    WheelCtrlRos2() : Node("wheelctrl_ros2")
    {
        RCLCPP_INFO(this->get_logger(), "nyan");
    }

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelCtrlRos2>());
    rclcpp::shutdown();
    return 0;
}