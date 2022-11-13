#include "wheel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"

using namespace std::chrono_literals;

class WheelCtrlRos2 : public rclcpp::Node {
 public:
  WheelCtrlRos2() : Node("wheelctrl_ros2") {
    set_wheel_param();
    set_handles();
    mytimer =
        this->create_wall_timer(5ms, std::bind(&WheelCtrlRos2::update, this));
  }

 private:
  // methods
  // set parameters from yaml file
  void set_wheel_param() {
    // declare_parameters
    // robot_param
    this->declare_parameter("robot_param.name", "undefined");

    // moving_wheel
    this->declare_parameter("moving_wheel.type_name", "undefined");
    this->declare_parameter("moving_wheel.radius", 0);
    this->declare_parameter("moving_wheel.quantity", 0);
    this->declare_parameter("moving_wheel.gear_ratio", 1);
    this->declare_parameter("moving_wheel.gear_ratio_horizonal", 1);
    this->declare_parameter("moving_wheel.coordinates", "undefined");
    this->declare_parameter("moving_wheel.distance", 0);
    this->declare_parameter("moving_wheel.arguments", std::vector<float>(4, 0));
    this->declare_parameter("moving_wheel.length_x", 0);
    this->declare_parameter("moving_wheel.length_y", 0);

    // measuring_wheel
    this->declare_parameter("measuring_wheel.type_name", "undefined");
    this->declare_parameter("measuring_wheel.radius", 0);
    this->declare_parameter("measuring_wheel.quantity", 0);
    this->declare_parameter("measuring_wheel.gear_ratio", 1);
    this->declare_parameter("measuring_wheel.gear_ratio_horizonal", 1);
    this->declare_parameter("measuring_wheel.coordinates", "undefined");
    this->declare_parameter("measuring_wheel.distance", 0);
    this->declare_parameter("measuring_wheel.arguments",
                            std::vector<float>(4, 0));
    this->declare_parameter("measuring_wheel.length_x", 0);
    this->declare_parameter("measuring_wheel.length_y", 0);

    // get parameters
    // robot_name
    robot_name = this->get_parameter("robot_param.name").as_string();

    // moving_wheel
    moving_wheel.type_name =
        this->get_parameter("moving_wheel.type_name").as_string();
    moving_wheel.radius =
        (float)this->get_parameter("moving_wheel.radius").as_double();
    moving_wheel.quantity =
        (float)this->get_parameter("moving_wheel.quantity").as_double();
    moving_wheel.gear_ratio =
        (float)this->get_parameter("moving_wheel.gear_ratio").as_double();
    moving_wheel.gear_ratio_horizonal =
        (float)this->get_parameter("moving_wheel.gear_ratio_horizonal")
            .as_double();
    moving_wheel.coordinate =
        this->get_parameter("moving_wheel.coordinate").as_string();
    moving_wheel.distance =
        (float)this->get_parameter("moving_wheel.distance").as_double();
    moving_wheel.arguments =
        this->get_parameter("moving_wheel.arguments").as_double_array();
    moving_wheel.length_x =
        (float)this->get_parameter("moving_wheel.length_x").as_double();
    moving_wheel.length_y =
        (float)this->get_parameter("moving_wheel.length_y").as_double();

    // measuring wheel
    measuring_wheel.type_name =
        this->get_parameter("measuring_wheel.type_name").as_string();
    measuring_wheel.radius =
        (float)this->get_parameter("measuring_wheel.radius").as_double();
    measuring_wheel.quantity =
        (float)this->get_parameter("measuring_wheel.quantity").as_double();
    measuring_wheel.gear_ratio =
        (float)this->get_parameter("measuring_wheel.gear_ratio").as_double();
    measuring_wheel.gear_ratio_horizonal =
        (float)this->get_parameter("measuring_wheel.gear_ratio_horizonal")
            .as_double();
    measuring_wheel.coordinate =
        this->get_parameter("measuring_wheel.coordinate").as_string();
    measuring_wheel.distance =
        (float)this->get_parameter("measuring_wheel.distance").as_double();
    measuring_wheel.arguments =
        this->get_parameter("measuring_wheel.arguments").as_double_array();
    measuring_wheel.length_x =
        (float)this->get_parameter("measuring_wheel.length_x").as_double();
    measuring_wheel.length_y =
        (float)this->get_parameter("measuring_wheel.length_y").as_double();
  }

  // set publisher and subscriber
  void set_handles() {
    // set moving_wheel
    wheel_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "rogilink2/send", 100);
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 100, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_vel.x = msg->linear.x;
          cmd_vel.y = msg->linear.y;
          cmd_vel.theta = msg->angular.z;
        });

    // set measuring_wheel
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    if (measuring_wheel.type_name == "undefined") {
      RCLCPP_ERROR(this->get_logger(), "please set wheel type!");
    } else if (measuring_wheel.type_name == "omni" ||
               measuring_wheel.type_name == "mechanam") {
      if (measuring_wheel.quantity > 2) {
        wheel_sub.resize(measuring_wheel.quantity);
        sub_data.resize(measuring_wheel.quantity);
        for (int i = 0; i < measuring_wheel.quantity; i++) {
          wheel_sub[i] =
              this->create_subscription<rogilink2_interfaces::msg::Frame>(
                  "rogilink2/receive" + std::to_string(i), 100,
                  [this, i](const rogilink2_interfaces::msg::Frame &msg) {
                    memcpy(&sub_data[i],msg.data.begin(),msg.data.size());
                  });
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "invalid wheel quantity!");
      }
    } else if (measuring_wheel.type_name == "steering") {
      if (measuring_wheel.quantity > 2) {
        wheel_sub.resize(2 * measuring_wheel.quantity);
        sub_data.resize(2 * measuring_wheel.quantity);
        for (int i = 0; i < 2 * measuring_wheel.quantity; i++) {
          wheel_sub[i] =
              this->create_subscription<rogilink2_interfaces::msg::Frame>(
                  "rogilink2/receive" + std::to_string(i), 100,
                  [this](
                      const rogilink2_interfaces::msg::Frame::UniquePtr &msg) {

                  });
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "invalid wheel quantity!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "invalid wheel type!");
    }
  }

  // main routine
  void update(){
    
  };

  // publishers and subscribers
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr wheel_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  std::vector<rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr>
      wheel_sub;

  rclcpp::TimerBase::SharedPtr mytimer;
  W_PARAM moving_wheel;
  W_PARAM measuring_wheel;
  std::string robot_name;
  CMD cmd_vel;
  std::vector<float> sub_data;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelCtrlRos2>());
  rclcpp::shutdown();
  return 0;
}