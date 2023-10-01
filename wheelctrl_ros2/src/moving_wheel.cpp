#include "moving_wheel.hpp"
using namespace illias_wheelctrl;

MovingWheel::MovingWheel() : Node("moving_wheel") {
    
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingWheel>());
  rclcpp::shutdown();
  return 0;
}