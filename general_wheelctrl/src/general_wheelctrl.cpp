#include <memory>

#include "rclcpp/rclcpp.hpp"

class SimpleCode : public rclcpp::Node
{
public:
    SimpleCode() : Node("simple")
    {
        RCLCPP_INFO(this->get_logger(), "nyan");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleCode>());
    rclcpp::shutdown();
    return 0;
}