#include "general_wheelctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "md_lib/odrive.hpp"
#include "md_lib/md2022.hpp"

using namespace chrono_literals;
using namespace illias;

// class WheelCtrlRos2 : public rclcpp::Node {
//   public:
//     WheelCtrlRos2() : Node("wheel_ctrl_ros2") {}
//     void init() {
//       set_param();
//       set_handles();
//       set_subclass();
//       set_initial_pos();
//       update();
//     };

//    private:
//    // setting functions
//     void set_param();
//     void set_handles();
//     void set_subclass(){};
//     void set_initial_pos(){};

//    // main routine functions
//     void update(){};

//     // variables
//     string robot_name;
//     bool sim_mode;
//     U_PARAM moving_param;
//     U_PARAM measuring_param;
//     vector<string> wheel_name;
//     CMD cmd;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
// };

// void WheelCtrlRos2::set_param(){
//     this->declare_parameter("robot_param.name", "undefined");
//     this->declare_parameter("robot_param.mode", "real");

//     this->declare_parameter("moving_wheel.type_name", "undefined");
//     this->declare_parameter("moving_wheel.radius", 1.0);
//     this->declare_parameter("moving_wheel.quantity", 4);
//     this->declare_parameter("moving_wheel.loop_rate", 1.0);
//     this->declare_parameter("moving_wheel.gear_ratio", 1.0);
//     this->declare_parameter("moving_wheel.gear_ratio_steer", 1.0);
//     this->declare_parameter("moving_wheel.distance", vector<double>(8, 1.0));
//     this->declare_parameter("moving_wheel.arguments", vector<double>(8, 1.0));
//     this->declare_parameter("moving_wheel.wheel_name",
//                             vector<string>(8, "undefined"));

//     this->declare_parameter("measuring_wheel.type_name", "undefined");
//     this->declare_parameter("measuring_wheel.radius", 1.0);
//     this->declare_parameter("measuring_wheel.quantity", 4);
//     this->declare_parameter("measuring_wheel.loop_rate", 1.0);
//     this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
//     this->declare_parameter("measuring_wheel.gear_ratio_steer", 1.0);
//     this->declare_parameter("measuring_wheel.distance", vector<double>(8, 1.0));
//     this->declare_parameter("measuring_wheel.arguments", vector<double>(8, 1.0));
//     this->declare_parameter("measuring_wheel.wheel_name",
//                             vector<string>(8, "undefined"));

//     robot_name = this->get_parameter("robot_param.name").as_string();
//     sim_mode = this->get_parameter("robot_param.mode").as_string() == "sim";

//     moving_param.type_name =
//         this->get_parameter("moving_wheel.type_name").as_string();
//     moving_param.radius =
//         (float)this->get_parameter("moving_wheel.radius").as_double();
//     moving_param.quantity =
//         this->get_parameter("moving_wheel.quantity").as_int();
//     moving_param.loop_rate =
//         (float)this->get_parameter("moving_wheel.loop_rate").as_double();
//     moving_param.gear_ratio =
//         (float)this->get_parameter("moving_wheel.gear_ratio").as_double();
//     moving_param.gear_ratio_steer =
//         (float)this->get_parameter("moving_wheel.gear_ratio_steer").as_double();
//     vector<double> move_distance_buff =
//         this->get_parameter("moving_wheel.distance").as_double_array();
//     vector<double> move_arguments_buff =
//         this->get_parameter("moving_wheel.arguments").as_double_array();
//     wheel_name =
//         this->get_parameter("moving_wheel.wheel_name").as_string_array();

//     measuring_param.type_name =
//         this->get_parameter("measuring_wheel.type_name").as_string();
//     measuring_param.radius =
//         (float)this->get_parameter("measuring_wheel.radius").as_double();
//     measuring_param.quantity =
//         this->get_parameter("measuring_wheel.quantity").as_int();
//     measuring_param.loop_rate =
//         (float)this->get_parameter("measuring_wheel.loop_rate").as_double();
//     measuring_param.gear_ratio =
//         (float)this->get_parameter("measuring_wheel.gear_ratio").as_double();
//     measuring_param.gear_ratio_steer =
//         (float)this->get_parameter("measuring_wheel.gear_ratio_steer")
//             .as_double();
//     vector<double> measure_distance_buff =
//         this->get_parameter("measuring_wheel.distance").as_double_array();
//     vector<double> measure_arguments_buff =
//         this->get_parameter("measuring_wheel.arguments").as_double_array();

//     for (int i = 0; i < move_distance_buff.size();i++){
//       W_PARAM buff;
//       buff.distance = move_distance_buff[i];
//       buff.argument = move_arguments_buff[i];
//       moving_param.wheels.push_back(buff);
//     }
//     for (int i = 0; i < measure_distance_buff.size();i++){
//       W_PARAM buff;
//       buff.distance = measure_distance_buff[i];
//       buff.argument = measure_arguments_buff[i];
//       measuring_param.wheels.push_back(buff);
//     }
//     RCLCPP_INFO(this->get_logger(), "%ld", moving_param.wheels.size());
// }

// void WheelCtrlRos2::set_handles(){
//     cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
//           cmd.x = msg->linear.x;
//           cmd.y = msg->linear.y;
//           cmd.w = msg->angular.z;
//           //  RCLCPP_INFO(this->get_logger(),"%f,%f,%f",cmd.x,cmd.y,cmd.w);
//         });
// }

class WheelCtrlRos2 : public rclcpp::Node {
  public:
   WheelCtrlRos2() : Node("wheel_ctrl_ros2") { sim_mode = true; }
   void init() {
     RCLCPP_INFO(this->get_logger(), "ACTIVATED: wheelctrl_ros2");
     set_wheel_parameter();
     set_initial_pos();
     set_subclass();

     RCLCPP_INFO(this->get_logger(), "wheelctrl_ros2 initialized");
     frame_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
         "/rogilink2/send", 10);
     odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
     tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
     cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
         "/cmd_vel", 10,
         std::bind(&WheelCtrlRos2::cmd_callback, this, std::placeholders::_1));

     // if (measuring_wheel.type_name == "steering") {
     //   encoder_sub.resize(measuring_wheel.quantity * 2);
     //   for (int i = 0; i < measuring_wheel.quantity * 2; i++) {
     //     encoder_sub[i] =
     //         this->create_subscription<rogilink2_interfaces::msg::Frame>(
     //             "/rogilink2/recieve_" + measuring_name[i], 10,
     //             [this,
     //               i](const rogilink2_interfaces::msg::Frame::SharedPtr msg)
     //               { memcpy(&encoder[i], msg->data.data(), sizeof(float));
     //             });
     //   }
     // } else {
     //   encoder_sub.resize(measuring_wheel.quantity);
     //   for (int i = 0; i < measuring_wheel.quantity; i++) {
     //     encoder_sub[i] =
     //         this->create_subscription<rogilink2_interfaces::msg::Frame>(
     //             "/rogilink2/recieve_" + measuring_name[i], 10,
     //             [this,
     //               i](const rogilink2_interfaces::msg::Frame::SharedPtr msg)
     //               { memcpy(&encoder[i], msg->data.data(), sizeof(float));
     //             });
     //   }
     // }
     // RCLCPP_INFO(this->get_logger(), "set subscriber");
     mytimer =
         this->create_wall_timer(5ms, std::bind(&WheelCtrlRos2::update, this));
  }

 private:
  void set_wheel_parameter();
  void set_initial_pos();
  void set_subclass();
  void update();
  void pub_rogilink2_frame();
  void pub_odometry();
  void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
  std::shared_ptr<illias::Measuring> measure;
  std::shared_ptr<illias::Moving> moving;
  rclcpp::TimerBase::SharedPtr mytimer;

  // handles of measuring wheel
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  vector<rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr>
      encoder_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // handles of moving wheel
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr frame_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  vector<std::shared_ptr<Md>> drivers;
  illias::U_PARAM moving_wheel;
  illias::U_PARAM measuring_wheel;
  illias::POS current_pos;
  illias::POS current_vel;
  illias::CMD cmd;
  std::vector<float> cmd_rotate;
  std::vector<float> encoder;
  std::string robot_name;
  std::vector<std::string> moving_name;
  std::vector<std::string> measuring_name;

  bool AMCL = false;
  bool sim_mode;
};

void WheelCtrlRos2::set_wheel_parameter(){
  RCLCPP_INFO(this->get_logger(), "set_wheel_parameter");
    this->declare_parameter("robot_param.name", "undefined");
    this->declare_parameter("robot_param.AMCL", false);

    this->declare_parameter("moving_wheel.type_name", "steering");
    this->declare_parameter("moving_wheel.radius", 0.0);
    this->declare_parameter("moving_wheel.quantity", 0);
    this->declare_parameter("moving_wheel.loop_rate", 200.0);
    this->declare_parameter("moving_wheel.gear_ratio", 1.0);
    this->declare_parameter("moving_wheel.gear_ratio_steer", 1.0);
    this->declare_parameter("moving_wheel.distance", vector<double>(8,0.0));
    this->declare_parameter("moving_wheel.arguments", vector<double>(8,
    0.0)); 
    this->declare_parameter("moving_wheel.wheel_name", vector<std::string>(8,
    "undefined"));

    this->declare_parameter("measuring_wheel.type_name", "steering");
    this->declare_parameter("measuring_wheel.radius", 0.0);
    this->declare_parameter("measuring_wheel.quantity", 0);
    this->declare_parameter("measuring_wheel.loop_rate", 200.0);
    this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
    this->declare_parameter("measuring_wheel.gear_ratio_steer", 1.0);
    this->declare_parameter("measuring_wheel.distance", vector<double>(8,
    0.0));
    this->declare_parameter("measuring_wheel.arguments",
                            vector<double>(8, 0.0));
    this->declare_parameter("measuring_wheel.wheel_name",
    vector<std::string>(8, "undefined"));

    robot_name = this->get_parameter("robot_param.name").as_string();
    AMCL = this->get_parameter("robot_param.AMCL").as_bool();

    // moving_wheel
    moving_wheel.type_name =
        this->get_parameter("moving_wheel.type_name").as_string();
    moving_wheel.radius =
        (float)this->get_parameter("moving_wheel.radius").as_double();
    moving_wheel.quantity =
        (float)this->get_parameter("moving_wheel.quantity").as_int();
    moving_wheel.loop_rate =
        (float)this->get_parameter("moving_wheel.loop_rate").as_double();
    moving_wheel.gear_ratio =
        (float)this->get_parameter("moving_wheel.gear_ratio").as_double();
    moving_wheel.gear_ratio_steer =
        (float)this->get_parameter("moving_wheel.gear_ratio_steer")
            .as_double();
    std::vector<double> dist_buff1 =
        this->get_parameter("moving_wheel.distance").as_double_array();
    std::vector<double> arg_buff1 =
        this->get_parameter("moving_wheel.arguments").as_double_array();
    std::vector<std::string> name_buff1 =
        this->get_parameter("moving_wheel.wheel_name").as_string_array();

    // measuring wheel
    measuring_wheel.type_name =
        this->get_parameter("measuring_wheel.type_name").as_string();
    measuring_wheel.radius =
        (float)this->get_parameter("measuring_wheel.radius").as_double();
    measuring_wheel.quantity =
        (float)this->get_parameter("measuring_wheel.quantity").as_int();
    measuring_wheel.loop_rate =
        (float)this->get_parameter("measuring_wheel.loop_rate").as_double();
    measuring_wheel.gear_ratio =
        (float)this->get_parameter("measuring_wheel.gear_ratio").as_double();
    measuring_wheel.gear_ratio_steer =
        (float)this->get_parameter("measuring_wheel.gear_ratio_steer")
            .as_double();
    std::vector<double> dist_buff2 =
        this->get_parameter("measuring_wheel.distance").as_double_array();
    std::vector<double> arg_buff2 =
        this->get_parameter("measuring_wheel.arguments").as_double_array();
    std::vector<std::string> name_buff2 =
        this->get_parameter("measuring_wheel.wheel_name").as_string_array();

    illias::W_PARAM wheel_param;
    for (int i = 0; i < (int)moving_wheel.quantity; i++) {
      wheel_param.distance = (float)dist_buff1[i];
      wheel_param.argument = (float)arg_buff1[i];
      moving_wheel.wheels.push_back(wheel_param);
    }
    for (int i = 0;i<(int)measuring_wheel.quantity;i++){
      wheel_param.distance = (float)dist_buff2[i];
      wheel_param.argument = (float)arg_buff2[i];
      moving_wheel.wheels.push_back(wheel_param);
    }
    RCLCPP_INFO(this->get_logger(), "parameter setting end");
}
void WheelCtrlRos2::set_initial_pos(){
  RCLCPP_INFO(this->get_logger(), "set_initial_pos");
  current_pos.x = 0;
  current_pos.y = 0;
  current_pos.w = 0;
  RCLCPP_INFO(this->get_logger(), "initial_pos setting end");
}

void WheelCtrlRos2::set_subclass() {
  RCLCPP_INFO(this->get_logger(), "set_subclass : measuring");
  
  if(measuring_wheel.type_name == "omni"){
    encoder.resize(measuring_wheel.quantity);
    switch(measuring_wheel.quantity){
      case 2:
        measure = std::make_shared<illias::MeasureOmni2W>(measuring_wheel,
                                                          current_pos);
        break;
      case 3:
        measure = std::make_shared<illias::MeasureOmni3W>(measuring_wheel,
                                                          current_pos);
        break;
      case 4:
        measure = std::make_shared<illias::MeasureOmni4W>(measuring_wheel,
                                                          current_pos);
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "please set proper wheel quantity");
    }
  }else if(measuring_wheel.type_name == "steering"){
    encoder.resize(2 * measuring_wheel.quantity);
    RCLCPP_INFO(this->get_logger(), "steering wheel setting start");
    measure =
        std::make_shared<illias::MeasureSteering>(measuring_wheel, current_pos);
    RCLCPP_INFO(this->get_logger(), "steering wheel setting end");
  } else {
    RCLCPP_INFO(this->get_logger(), "please set collect wheel_type");
  }
  RCLCPP_INFO(this->get_logger(), "subclass setting end : measuring");

  RCLCPP_INFO(this->get_logger(), "set_subclass : moving");
  if(moving_wheel.type_name == "omni"){
    cmd_rotate.resize(moving_wheel.quantity);
    if (!sim_mode) {
      for (int i = 0; i < moving_wheel.quantity;i++){
        drivers.push_back(std::make_shared<ODrive>(this, moving_name[i]));
        RCLCPP_INFO(this->get_logger(), "omni wheel start %d", i);
        drivers.at(i)->init();
        drivers.at(i)->setMode(Md::Mode::Velocity);
        drivers.at(i)->setPosition(0.0);
      } 
    }
    switch(moving_wheel.quantity){
      case 3:
        moving = std::make_shared<illias::MoveOmni3W>(moving_wheel);
        break;
      case 4:
        moving = std::make_shared<illias::MoveOmni4W>(moving_wheel);
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "please set proper wheel quantity");
    }
  } else if (moving_wheel.type_name == "steering") {
    cmd_rotate.resize(2 * moving_wheel.quantity);
    RCLCPP_INFO(this->get_logger(), "hya");
      // RCLCPP_INFO(this->get_logger(), "%f", moving_wheel.wheels[7].distance);
    moving = std::make_shared<illias::MoveSteering>(moving_wheel);
    RCLCPP_INFO(this->get_logger(), "nya");
    if (!sim_mode) {
      for (int i = 0; i < 8; i++) {
        if (i < 4) {
          drivers.push_back(std::make_shared<ODrive>(this, moving_name[i]));
          RCLCPP_INFO(this->get_logger(), "steering wheel start %d", i);
          drivers.at(i)->init();
          drivers.at(i)->setMode(Md::Mode::Velocity);
          drivers.at(i)->setPosition(0.0);
        } else {
          drivers.push_back(std::make_shared<MD2022>(this, moving_name[i]));
          drivers[i]->init();
          drivers[i]->setMode(Md::Mode::Position);
          drivers[i]->setPosition(0.0);
        }
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "please set collect wheel_type");
  }
  RCLCPP_INFO(this->get_logger(), "subclass setting end : moving");
}
void WheelCtrlRos2::update() { }
void WheelCtrlRos2::pub_rogilink2_frame(){}
void WheelCtrlRos2::pub_odometry(){}
void WheelCtrlRos2::cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg){
  // RCLCPP_INFO(this->get_logger(), "%f, %f", msg->linear.x, msg->linear.y);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WheelCtrlRos2>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<WheelCtrlRos2>();
//   node->init();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
// // #include "general_wheelctrl.hpp"
// // #include "geometry_msgs/msg/twist.hpp"
// // #include "nav_msgs/msg/odometry.hpp"
// // #include "rclcpp/rclcpp.hpp"
// // #include "rogilink2_interfaces/msg/frame.hpp"
// // #include "rogilink2_interfaces/srv/request_add_topic.hpp"
// // #include "tf2/LinearMath/Quaternion.h"
// // #include "tf2_ros/transform_broadcaster.h"
// // #include "md_lib/odrive.hpp"
// // #include "md_lib/md2022.hpp"

// // using namespace chrono_literals;

// // class WheelCtrlRos2:public rclcpp::Node{
// //     public:
// //     WheelCtrlRos2():Node("wheel_ctrl_ros2"){

// //     }
// //     void init(){
// //       RCLCPP_INFO(this->get_logger(), "ACTIVATED: wheelctrl_ros2");
// //       set_wheel_parameter();
// //       set_initial_pos();
// //       set_subclass();
// //       RCLCPP_INFO(this->get_logger(), "wheelctrl_ros2 initialized");
// //       frame_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
// //           "/rogilink2/send", 10);
// //       odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
// //       tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
// //       cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
// //           "/cmd_vel", 10,
// //           std::bind(&WheelCtrlRos2::cmd_callback, this, std::placeholders::_1));
// //       // if (measuring_wheel.type_name == "steering") {
// //       //   encoder_sub.resize(measuring_wheel.quantity * 2);
// //       //   for (int i = 0; i < measuring_wheel.quantity * 2; i++) {
// //       //     encoder_sub[i] =
// //       //         this->create_subscription<rogilink2_interfaces::msg::Frame>(
// //       //             "/rogilink2/recieve_" + measuring_name[i], 10,
// //       //             [this,
// //       //              i](const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
// //       //               memcpy(&encoder[i], msg->data.data(), sizeof(float));
// //       //             });
// //       //   }
// //       // } else {
// //       //   encoder_sub.resize(measuring_wheel.quantity);
// //       //   for (int i = 0; i < measuring_wheel.quantity; i++) {
// //       //     encoder_sub[i] =
// //       //         this->create_subscription<rogilink2_interfaces::msg::Frame>(
// //       //             "/rogilink2/recieve_" + measuring_name[i], 10,
// //       //             [this,
// //       //              i](const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
// //       //               memcpy(&encoder[i], msg->data.data(), sizeof(float));
// //       //             });
// //       //   }
// //       // }
// //       mytimer =
// //           this->create_wall_timer(5ms, std::bind(&WheelCtrlRos2::update, this));
// //     }

// //    private:
// //     void set_wheel_parameter();
// //     void set_initial_pos();
// //     void set_subclass();
// //     void update();
// //     void pub_rogilink2_frame();
// //     void pub_odometry();
// //     void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
// //     std::unique_ptr<illias::Measuring> measure;
// //     std::unique_ptr<illias::Moving> moving;
// //     rclcpp::TimerBase::SharedPtr mytimer;

// //     // handles of measuring wheel
// //     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
// //     vector<rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr>
// //         encoder_sub;
// //     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// //     // handles of moving wheel
// //     rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr frame_pub;
// //     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
// //     vector<std::shared_ptr<Md>> drivers;
// //     illias::U_PARAM moving_wheel;
// //     illias::U_PARAM measuring_wheel;
// //     illias::POS current_pos;
// //     illias::POS current_vel;
// //     illias::CMD cmd;
// //     std::shared_ptr<float[]> cmd_rotate;
// //     std::shared_ptr<float[]> encoder;
// //     std::string robot_name;
// //     std::vector<std::string> moving_name;
// //     std::vector<std::string> measuring_name;

// //     bool AMCL = false;
    
// // };

// // void WheelCtrlRos2::set_wheel_parameter(){
// //   this->declare_parameter("robot_param.name", "undefined");
// //   this->declare_parameter("robot_param.AMCL", false);

// //   this->declare_parameter("moving_wheel.type_name", "undefined");
// //   this->declare_parameter("moving_wheel.radius", 0.0);
// //   this->declare_parameter("moving_wheel.quantity", 0);
// //   this->declare_parameter("moving_wheel.loop_rate", 200.0);
// //   this->declare_parameter("moving_wheel.gear_ratio", 1.0);
// //   this->declare_parameter("moving_wheel.gear_ratio_horizonal", 1.0);
// //   this->declare_parameter("moving_wheel.coordinate", "undefined");
// //   this->declare_parameter("moving_wheel.distance", vector<double>(4,0.0));
// //   this->declare_parameter("moving_wheel.arguments", vector<double>(4, 0.0));
// //   this->declare_parameter("moving_wheel.length_x", 1.0);
// //   this->declare_parameter("moving_wheel.length_y", 1.0);
// //   this->declare_parameter("moving_wheel.wheel_name", vector<std::string>(8, "undefined"));

// //   this->declare_parameter("measuring_wheel.type_name", "undefined");
// //   this->declare_parameter("measuring_wheel.radius", 0.0);
// //   this->declare_parameter("measuring_wheel.quantity", 0);
// //   this->declare_parameter("measuring_wheel.loop_rate", 200.0); 
// //   this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
// //   this->declare_parameter("measuring_wheel.gear_ratio_horizonal", 1.0);
// //   this->declare_parameter("measuring_wheel.coordinate", "undefined");
// //   this->declare_parameter("measuring_wheel.distance", vector<double>(4, 0.0));
// //   this->declare_parameter("measuring_wheel.arguments", vector<double>(4, 0.0));
// //   this->declare_parameter("measuring_wheel.length_x", 1.0);
// //   this->declare_parameter("measuring_wheel.length_y", 1.0);
// //   this->declare_parameter("measuring_wheel.wheel_name", vector<std::string>(8, "undefined"));

// //   robot_name = this->get_parameter("robot_param.name").as_string();
// //   AMCL = this->get_parameter("robot_param.AMCL").as_bool();

// //   // moving_wheel
// //   moving_wheel.type_name =
// //       this->get_parameter("moving_wheel.type_name").as_string();
// //   moving_wheel.radius =
// //       (float)this->get_parameter("moving_wheel.radius").as_double();
// //   moving_wheel.quantity =
// //       (float)this->get_parameter("moving_wheel.quantity").as_int();
// //   moving_wheel.loop_rate =
// //       (float)this->get_parameter("moving_wheel.loop_rate").as_double();
// //   moving_wheel.gear_ratio =
// //       (float)this->get_parameter("moving_wheel.gear_ratio").as_double();
// //   moving_wheel.gear_ratio_steer =
// //       (float)this->get_parameter("moving_wheel.gear_ratio_horizonal")
// //           .as_double();
// //   std::vector<double> dist_buff1 =
// //       this->get_parameter("moving_wheel.distance").as_double_array();
// //   std::vector<double> arg_buff1 =
// //       this->get_parameter("moving_wheel.arguments").as_double_array();

// //   // measuring wheel
// //   measuring_wheel.type_name =
// //       this->get_parameter("measuring_wheel.type_name").as_string();
// //   measuring_wheel.radius =
// //       (float)this->get_parameter("measuring_wheel.radius").as_double();
// //   measuring_wheel.quantity =
// //       (float)this->get_parameter("measuring_wheel.quantity").as_int();
// //   measuring_wheel.loop_rate =
// //       (float)this->get_parameter("measuring_wheel.loop_rate").as_double();
// //   measuring_wheel.gear_ratio =
// //       (float)this->get_parameter("measuring_wheel.gear_ratio").as_double();
// //   measuring_wheel.gear_ratio_steer =
// //       (float)this->get_parameter("measuring_wheel.gear_ratio_horizonal")
// //           .as_double();
// //   std::vector<double> dist_buff2 =
// //       this->get_parameter("measuring_wheel.distance").as_double_array();
// //   std::vector<double> arg_buff2 =
// //       this->get_parameter("measuring_wheel.arguments").as_double_array();

// //   illias::W_PARAM wheel_param;
// //   for (int i = 0; i < (int)dist_buff1.size(); i++) {
// //     wheel_param.distance = (float)dist_buff1[i];
// //     wheel_param.argument = (float)arg_buff1[i];
// //     moving_wheel.wheels.push_back(wheel_param);
// //   }
// //   for (int i = 0;i<(int)dist_buff2.size();i++){
// //     wheel_param.distance = (float)dist_buff1[i];
// //     wheel_param.argument = (float)arg_buff1[i];
// //     moving_wheel.wheels.push_back(wheel_param);
// //   }
// // }

// // void WheelCtrlRos2::set_initial_pos() { 
// //   current_pos.x = 0;
// //   current_pos.y = 0;
// //   current_pos.w = 0;
// // }

// // void WheelCtrlRos2::set_subclass(){
// //   RCLCPP_INFO(this->get_logger(), "set_subclass");
// //   if(measuring_wheel.type_name=="omni"){
// //     encoder = std::make_unique<float[]>(measuring_wheel.quantity);
// //     switch (measuring_wheel.quantity) {
// //           case 2:
// //             measure = std::make_unique<illias::MeasureOmni2W>(measuring_wheel,
// //                                                               current_pos);
// //             break;
// //           case 3:
// //             measure = std::make_unique<illias::MeasureOmni3W>(measuring_wheel,
// //                                                               current_pos);
// //             break;
// //           case 4:
// //             measure = std::make_unique<illias::MeasureOmni4W>(measuring_wheel,
// //                                                               current_pos);
// //             break;
// //         }
// //   } else if (measuring_wheel.type_name == "steering") {
// //     encoder = std::make_unique<float[]>(2 * measuring_wheel.quantity);
// //     measure = std::make_unique<illias::MeasureSteering>(measuring_wheel,
// //                                                             current_pos);
// //   } else if (measuring_wheel.type_name == "mechanam") {
// //     encoder = std::make_unique<float[]>(measuring_wheel.quantity);
// //   } else {
// //     RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
// //   }

// //   if (moving_wheel.type_name == "omni") {
// //     cmd_rotate = std::make_unique<float[]>(moving_wheel.quantity);
  
// //     switch (moving_wheel.quantity) {
// //       case 3:
// //         moving = std::make_unique<illias::MoveOmni3W>(moving_wheel);
// //         for (int i = 0; i < 3; i++) {
// //           drivers.push_back(std::make_shared<MD2022>(this, moving_name[i]));
// //           drivers.at(i)->init();
// //           drivers.at(i)->setMode(Md::Mode::Velocity);
// //           drivers.at(i)->setPosition(0.0);
// //         }
// //         break;
// //       case 4:
// //         moving = std::make_unique<illias::MoveOmni4W>(moving_wheel);
// //         for (int i = 0; i < 4; i++) {
// //           drivers.push_back(std::make_shared<MD2022>(this, moving_name[i]));
// //           drivers.at(i)->init();
// //           drivers.at(i)->setMode(Md::Mode::Velocity);
// //           drivers.at(i)->setPosition(0.0);
// //         }
// //         break;
// //     }
// //   } else if (moving_wheel.type_name == "steering") {
// //     cmd_rotate = std::make_unique<float[]>(2 * moving_wheel.quantity);
// //     moving = std::make_unique<illias::MoveSteering>(moving_wheel);
// //     RCLCPP_INFO(this->get_logger(), "steering wheel start!");
// //     // for (int i = 0; i < 8; i++) {
// //     //   if(i<4){
// //     //     drivers.push_back(std::make_shared<ODrive>(this, moving_name[i]));
// //     //     RCLCPP_INFO(this->get_logger(), "steering wheel start %d", i);
// //     //     drivers.at(i)->init();
// //     //     drivers.at(i)->setMode(Md::Mode::Velocity);
// //     //     drivers.at(i)->setPosition(0.0);
// //     //   }else
// //     //   {
// //     //     drivers.push_back(std::make_shared<MD2022>(this, moving_name[i]));
// //     //     drivers[i]->init();
// //     //     drivers[i]->setMode(Md::Mode::Position);
// //     //     drivers[i]->setPosition(0.0);
// //     //   }
// //     // }
// //     RCLCPP_INFO(this->get_logger(), "steering wheel end");
// //     } else if (moving_wheel.type_name == "mechanam") {
// //   } else {
// //     RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
// //   }
// // }

// // void WheelCtrlRos2::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
// //   cmd.x = msg->linear.x;
// //   cmd.y = msg->linear.y;
// //   cmd.w = msg->angular.z;
// //   if(moving_wheel.type_name=="omni"){
// //     moving->cal_cmd(cmd);
// //   } else if (moving_wheel.type_name == "steering") {
// //     moving->cal_cmd(cmd);
// //   } else if (moving_wheel.type_name == "mechanam") {
// //     moving->cal_cmd(cmd);
// //   } else {
// //     RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
// //   }
// // }

// // void WheelCtrlRos2::update(){
// //   RCLCPP_INFO(this->get_logger(), "update");
// //   // get encoder
// //   // if(measuring_wheel.type_name=="omni"){
// //   //   for(int i=0;i<measuring_wheel.quantity;i++){
// //   //     encoder[i] = drivers[i]->getPosition();
// //   //   }
// //   //   measure->cal_disp(encoder,measuring_wheel.quantity);
// //   // } else if (measuring_wheel.type_name == "steering") {
// //   //   for(int i=0;i<2*measuring_wheel.quantity;i++){
// //   //     encoder[i] = drivers[i]->getPosition();
// //   //   }
// //   //   measure->cal_disp(encoder, 8);
// //   // } else if (measuring_wheel.type_name == "mechanam") {
// //   // }

// //   // current_pos = measure->get_current_pos();
// //   // current_vel = measure->get_current_vel();

// //       // copy from moving->wheel_cmd_rot to cmd_rotate
// //   if (moving_wheel.type_name == "omni") {
// //     for(int i=0;i<moving_wheel.quantity;i++){
// //       cmd_rotate[i] = moving->wheel_cmd_rot[i];
// //     }
// //   }
// //   else if (moving_wheel.type_name == "steering") {
// //     RCLCPP_INFO(this->get_logger(), "steering wheel start");
// //     for (int i = 0; i < 2 * moving_wheel.quantity; i++) {
// //       RCLCPP_INFO(this->get_logger(), "hya");
// //       cmd_rotate[i] = moving->wheel_cmd_rot[i];
// //     }
// //   }
// //   RCLCPP_INFO(this->get_logger(), "update end");

// //   //publish
// //   pub_rogilink2_frame();
// //   pub_odometry();
// // }

// // void WheelCtrlRos2::pub_rogilink2_frame(){
// //   auto msg = rogilink2_interfaces::msg::Frame();
// //   if(moving_wheel.type_name=="omni"){
// //     for (int i = 0; i < moving_wheel.quantity; i++) {
// //       drivers[i]->setVelocity(moving->wheel_cmd_rot[i]);
// //     }
// //   } else if (moving_wheel.type_name == "steering") {
// //     for (int i = 0; i < 2*moving_wheel.quantity; i++) {
// //       if(i<4){
// //         RCLCPP_INFO(this->get_logger(), "%f ", moving->wheel_cmd_rot[i]);
// //         // drivers[i]->setVelocity(moving->wheel_cmd_rot[i]);
// //       }else{
// //         // drivers[i]->setPosition(moving->wheel_cmd_rot[i]);
// //         RCLCPP_INFO(this->get_logger(), "%f ", moving->wheel_cmd_rot[i]);
// //       }
// //       RCLCPP_INFO(this->get_logger(), "\n");
// //     }
// //   }
// // }

// // void WheelCtrlRos2::pub_odometry(){
// //   auto msg = nav_msgs::msg::Odometry();
// //   msg.header.stamp = this->now();
// //   msg.header.frame_id = "odom";
// //   msg.child_frame_id = "base_link";
// //   msg.pose.pose.position.x = current_pos.x;
// //   msg.pose.pose.position.y = current_pos.y;
// //   msg.pose.pose.position.z = 0;
// //   msg.pose.pose.orientation.x = 0;
// //   msg.pose.pose.orientation.y = 0;
// //   msg.pose.pose.orientation.z = sin(current_pos.w / 2);
// //   msg.pose.pose.orientation.w = cos(current_pos.w / 2);
// //   msg.twist.twist.linear.x = current_vel.x;
// //   msg.twist.twist.linear.y = current_vel.y;
// //   msg.twist.twist.angular.z = current_vel.w;
// //   odom_pub->publish(msg);
// //   if(!AMCL){
// //     geometry_msgs::msg::TransformStamped transform;
// //     transform.header.stamp = this->now();
// //     transform.header.frame_id = "odom";
// //     transform.child_frame_id = "base_link";
// //     transform.transform.translation.x = current_pos.x;
// //     transform.transform.translation.y = current_pos.y;
// //     transform.transform.translation.z = 0;
// //     tf2::Quaternion q;
// //     q.setRPY(0, 0, current_pos.w);
// //     transform.transform.rotation.x = q.x();
// //     transform.transform.rotation.y = q.y();
// //     transform.transform.rotation.z = q.z();
// //     transform.transform.rotation.w = q.w();
// //     tf_broadcaster->sendTransform(transform);
// //   }
// // }


// // int main(int argc, char *argv[]) {
// //   rclcpp::init(argc, argv);
// //   auto node = std::make_shared<WheelCtrlRos2>();
// //   node->init();
// //   rclcpp::spin(node);
// //   rclcpp::shutdown();
// //   return 0;
// // }