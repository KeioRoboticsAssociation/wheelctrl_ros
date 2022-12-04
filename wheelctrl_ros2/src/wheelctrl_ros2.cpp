#include "general_wheelctrl.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace chrono_literals;

class WheelCtrlRos2:public rclcpp::Node{
    public:
    WheelCtrlRos2():Node("wheel_ctrl_ros2"){
      RCLCPP_INFO(this->get_logger(), "ACTIVATED: wheelctrl_ros2");
      set_wheel_parameter();
      set_initial_pos();
      set_subclass();
      frame_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>("/rogilink2/send", 10);
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&WheelCtrlRos2::cmd_callback, this, std::placeholders::_1));
      if(measuring_wheel.type_name=="steering"){
        encoder_sub.resize(measuring_wheel.quantity * 2);
        for (int i = 0; i < measuring_wheel.quantity*2;i++){
          encoder_sub[i] =
              this->create_subscription<rogilink2_interfaces::msg::Frame>(
                  "/rogilink2/recieve_" + measuring_name[i], 10,
                  [this,i](const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
                    memcpy(&encoder[i], msg->data.data(), sizeof(float));
                  });
        }
      } else {
        encoder_sub.resize(measuring_wheel.quantity);
        for (int i = 0; i < measuring_wheel.quantity;i++){
          encoder_sub[i] =
              this->create_subscription<rogilink2_interfaces::msg::Frame>(
                  "/rogilink2/recieve_" + measuring_name[i], 10,
                  [this,i](const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
                    memcpy(&encoder[i], msg->data.data(), sizeof(float));
                  });
        }
      }
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
     std::unique_ptr<illias::Measuring> measure;
     std::unique_ptr<illias::Moving> moving;
     rclcpp::TimerBase::SharedPtr mytimer;
     
     // handles of measuring wheel
     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
     vector<rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr>
         encoder_sub;
     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

     // handles of moving wheel
     rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr frame_pub;
     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;

     illias::W_PARAM moving_wheel;
     illias::W_PARAM measuring_wheel;
     illias::POS current_pos;
     illias::POS current_vel;
     illias::CMD cmd;
     std::unique_ptr<float[]> cmd_rotate;
     std::unique_ptr<float[]> encoder;
     std::string robot_name;
     std::vector<std::string> moving_name;
     std::vector<std::string> measuring_name;

     bool AMCL = false;
};

void WheelCtrlRos2::set_wheel_parameter(){
  this->declare_parameter("robot_param.name", "undefined");
  this->declare_parameter("robot_param.AMCL", false);

  this->declare_parameter("moving_wheel.type_name", "undefined");
  this->declare_parameter("moving_wheel.radius", 0.0);
  this->declare_parameter("moving_wheel.quantity", 0);
  this->declare_parameter("moving_wheel.loop_rate", 200.0);
  this->declare_parameter("moving_wheel.gear_ratio", 1.0);
  this->declare_parameter("moving_wheel.gear_ratio_horizonal", 1.0);
  this->declare_parameter("moving_wheel.coordinate", "undefined");
  this->declare_parameter("moving_wheel.distance", 0.0);
  this->declare_parameter("moving_wheel.arguments", vector<double>(4, 0.0));
  this->declare_parameter("moving_wheel.length_x", 1.0);
  this->declare_parameter("moving_wheel.length_y", 1.0);
  this->declare_parameter("moving_wheel.wheel_name", vector<std::string>(8, "undefined"));

  this->declare_parameter("measuring_wheel.type_name", "undefined");
  this->declare_parameter("measuring_wheel.radius", 0.0);
  this->declare_parameter("measuring_wheel.quantity", 0);
  this->declare_parameter("measuring_wheel.loop_rate", 200.0); 
  this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
  this->declare_parameter("measuring_wheel.gear_ratio_horizonal", 1.0);
  this->declare_parameter("measuring_wheel.coordinate", "undefined");
  this->declare_parameter("measuring_wheel.distance", 0.0);
  this->declare_parameter("measuring_wheel.arguments", vector<double>(4, 0));
  this->declare_parameter("measuring_wheel.length_x", 1.0);
  this->declare_parameter("measuring_wheel.length_y", 1.0);
  this->declare_parameter("measuring_wheel.wheel_name", vector<std::string>(8, "undefined"));

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
  moving_name = this->get_parameter("moving_wheel.wheel_name").as_string_array(); 

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
  measuring_name = this->get_parameter("measuring_wheel.wheel_name").as_string_array();  

  for (int i = 0; i < (int)moving_wheel.arguments.size();i++){
    moving_wheel.arguments[i] = moving_wheel.arguments[i] * M_PI / 180;
  }
  for (int i = 0; i < (int)measuring_wheel.arguments.size(); i++) {
    measuring_wheel.arguments[i] = measuring_wheel.arguments[i] * M_PI / 180;
  }
}

void WheelCtrlRos2::set_initial_pos() { 
  current_pos.x = 0;
  current_pos.y = 0;
  current_pos.theta = 0;
}

void WheelCtrlRos2::set_subclass(){
  if(measuring_wheel.type_name=="omni"){
    encoder = std::make_unique<float[]>(measuring_wheel.quantity);
    switch (measuring_wheel.quantity) {
      case 2:
        measure = std::make_unique<illias::MeasureOmni2W>(measuring_wheel,
                                                          current_pos);
        break;
      case 3:
        measure = std::make_unique<illias::MeasureOmni3W>(measuring_wheel,
                                                          current_pos);
        break;
      case 4:
        measure = std::make_unique<illias::MeasureOmni4W>(measuring_wheel,
                                                          current_pos);
        break;
    }
  } else if (measuring_wheel.type_name == "steering") {
    encoder = std::make_unique<float[]>(2 * measuring_wheel.quantity);
    measure =
        std::make_unique<illias::MeasureSteering>(measuring_wheel, current_pos);
  } else if (measuring_wheel.type_name == "mechanam") {
    encoder = std::make_unique<float[]>(measuring_wheel.quantity);
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
  }

  if (moving_wheel.type_name == "omni") {
    cmd_rotate = std::make_unique<float[]>(moving_wheel.quantity);
    switch (moving_wheel.quantity) {
      case 3:
        moving = std::make_unique<illias::MoveOmni3W>(moving_wheel);
        break;
      case 4:
        moving = std::make_unique<illias::MoveOmni4W>(moving_wheel);
    }
  } else if (moving_wheel.type_name == "steering") {
    cmd_rotate = std::make_unique<float[]>(2 * moving_wheel.quantity);
    moving = std::make_unique<illias::MoveSteering>(moving_wheel);
  } else if (moving_wheel.type_name == "mechanam") {
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
  }
}

void WheelCtrlRos2::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd.x = msg->linear.x;
  cmd.y = msg->linear.y;
  cmd.theta = msg->angular.z;
  if(moving_wheel.type_name=="omni"){
    moving->cal_cmd(cmd);
  } else if (moving_wheel.type_name == "steering") {
    moving->cal_cmd(cmd, current_pos.theta);
  } else if (moving_wheel.type_name == "mechanam") {
    moving->cal_cmd(cmd);
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
  }
}

void WheelCtrlRos2::update(){
  current_pos = measure->get_current_pos();
  current_vel = measure->get_current_vel();

  //copy from moving->wheel_cmd_rotate to cmd_rotate
  if(moving_wheel.type_name=="omni"){
    for(int i=0;i<moving_wheel.quantity;i++){
      cmd_rotate[i] = moving->wheel_cmd_rotate[i];
    }
  } else if(moving_wheel.type_name=="steering"){
    for(int i=0;i<2*moving_wheel.quantity;i++){
      cmd_rotate[i] = moving->wheel_cmd_rotate[i];
    }
  }

  //publish
  pub_rogilink2_frame();
  pub_odometry();
}

void WheelCtrlRos2::pub_rogilink2_frame(){
  auto msg = rogilink2_interfaces::msg::Frame();
  if(moving_wheel.type_name=="omni"){
    for (int i = 0; i < moving_wheel.quantity; i++) {
      msg.name = moving_name[i];
      memcpy(&msg.data, &moving->wheel_cmd_rotate[i], sizeof(float));
      frame_pub->publish(msg);
    }
  } else if (moving_wheel.type_name == "steering") {
    for (int i = 0; i < 2*moving_wheel.quantity; i++) {
      msg.name = moving_name[i];
      memcpy(&msg.data, &moving->wheel_cmd_rotate[i], sizeof(float));
      frame_pub->publish(msg);
    }
  }
}

void WheelCtrlRos2::pub_odometry(){
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = current_pos.x;
  msg.pose.pose.position.y = current_pos.y;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = sin(current_pos.theta / 2);
  msg.pose.pose.orientation.w = cos(current_pos.theta / 2);
  msg.twist.twist.linear.x = current_vel.x;
  msg.twist.twist.linear.y = current_vel.y;
  msg.twist.twist.angular.z = current_vel.theta;
  odom_pub->publish(msg);
  if(!AMCL){
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = current_pos.x;
    transform.transform.translation.y = current_pos.y;
    transform.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pos.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(transform);
  }
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelCtrlRos2>());
  rclcpp::shutdown();
  return 0;
}

// #include "wheel.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "rogilink2_interfaces/msg/frame.hpp"
// #include "rogilink2_interfaces/srv/request_add_topic.hpp"
// #include "general_wheelctrl.hpp"

// using namespace std::chrono_literals;
// using namespace illias;

// class WheelCtrlRos2 : public rclcpp::Node {
//  public:
//   WheelCtrlRos2() : Node("wheelctrl_ros2") {
//     set_wheel_param();
//     set_class();
//     set_handles();
//     mytimer =
//         this->create_wall_timer(5ms, std::bind(&WheelCtrlRos2::update,
//         this));
//   }

//  private:
//   // methods
//   // set parameters from yaml file
//   void set_wheel_param() {
//     // declare_parameters
//     // robot_param
//     this->declare_parameter("robot_param.name", "undefined");

//     // moving_wheel
//     this->declare_parameter("moving_wheel.type_name", "undefined");
//     this->declare_parameter("moving_wheel.radius", 0);
//     this->declare_parameter("moving_wheel.quantity", 0);
//     this->declare_parameter("moving_wheel.gear_ratio", 1);
//     this->declare_parameter("moving_wheel.gear_ratio_horizonal", 1);
//     this->declare_parameter("moving_wheel.coordinates", "undefined");
//     this->declare_parameter("moving_wheel.distance", 0);
//     this->declare_parameter("moving_wheel.arguments", std::vector<float>(4,
//     0)); this->declare_parameter("moving_wheel.length_x", 0);
//     this->declare_parameter("moving_wheel.length_y", 0);

//     // measuring_wheel
//     this->declare_parameter("measuring_wheel.type_name", "undefined");
//     this->declare_parameter("measuring_wheel.radius", 0);
//     this->declare_parameter("measuring_wheel.quantity", 0);
//     this->declare_parameter("measuring_wheel.gear_ratio", 1);
//     this->declare_parameter("measuring_wheel.gear_ratio_horizonal", 1);
//     this->declare_parameter("measuring_wheel.coordinates", "undefined");
//     this->declare_parameter("measuring_wheel.distance", 0);
//     this->declare_parameter("measuring_wheel.arguments",
//                             std::vector<float>(4, 0));
//     this->declare_parameter("measuring_wheel.length_x", 0);
//     this->declare_parameter("measuring_wheel.length_y", 0);

//     // get parameters
//     // robot_name
//     robot_name = this->get_parameter("robot_param.name").as_string();

//     // moving_wheel
//     moving_wheel.type_name =
//         this->get_parameter("moving_wheel.type_name").as_string();
//     moving_wheel.radius =
//         (float)this->get_parameter("moving_wheel.radius").as_double();
//     moving_wheel.quantity =
//         (float)this->get_parameter("moving_wheel.quantity").as_double();
//     moving_wheel.gear_ratio =
//         (float)this->get_parameter("moving_wheel.gear_ratio").as_double();
//     moving_wheel.gear_ratio_horizonal =
//         (float)this->get_parameter("moving_wheel.gear_ratio_horizonal")
//             .as_double();
//     moving_wheel.coordinate =
//         this->get_parameter("moving_wheel.coordinate").as_string();
//     moving_wheel.distance =
//         (float)this->get_parameter("moving_wheel.distance").as_double();
//     moving_wheel.arguments =
//         this->get_parameter("moving_wheel.arguments").as_double_array();
//     moving_wheel.length_x =
//         (float)this->get_parameter("moving_wheel.length_x").as_double();
//     moving_wheel.length_y =
//         (float)this->get_parameter("moving_wheel.length_y").as_double();

//     // measuring wheel
//     measuring_wheel.type_name =
//         this->get_parameter("measuring_wheel.type_name").as_string();
//     measuring_wheel.radius =
//         (float)this->get_parameter("measuring_wheel.radius").as_double();
//     measuring_wheel.quantity =
//         (float)this->get_parameter("measuring_wheel.quantity").as_double();
//     measuring_wheel.gear_ratio =
//         (float)this->get_parameter("measuring_wheel.gear_ratio").as_double();
//     measuring_wheel.gear_ratio_horizonal =
//         (float)this->get_parameter("measuring_wheel.gear_ratio_horizonal")
//             .as_double();
//     measuring_wheel.coordinate =
//         this->get_parameter("measuring_wheel.coordinate").as_string();
//     measuring_wheel.distance =
//         (float)this->get_parameter("measuring_wheel.distance").as_double();
//     measuring_wheel.arguments =
//         this->get_parameter("measuring_wheel.arguments").as_double_array();
//     measuring_wheel.length_x =
//         (float)this->get_parameter("measuring_wheel.length_x").as_double();
//     measuring_wheel.length_y =
//         (float)this->get_parameter("measuring_wheel.length_y").as_double();
//   }

//   // set publisher and subscriber
//   void set_handles() {
//     // set moving_wheel
//     wheel_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
//         "rogilink2/send", 100);
//     sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
//         "cmd_vel", 100, [this](const geometry_msgs::msg::Twist::SharedPtr
//         msg) {
//           cmd_vel.x = msg->linear.x;
//           cmd_vel.y = msg->linear.y;
//           cmd_vel.theta = msg->angular.z;
//         });

//     // set measuring_wheel
//     pub_POS = this->create_publisher<nav_msgs::msg::odometry>("POS", 100);
//     if (measuring_wheel.type_name == "undefined") {
//       RCLCPP_ERROR(this->get_logger(), "please set wheel type!");
//     } else if (measuring_wheel.type_name == "omni" ||
//                measuring_wheel.type_name == "mechanam") {
//       if (measuring_wheel.quantity > 2) {
//         wheel_sub.resize(measuring_wheel.quantity);
//         for (int i = 0; i < measuring_wheel.quantity; i++) {
//           wheel_sub[i] =
//               this->create_subscription<rogilink2_interfaces::msg::Frame>(
//                   "rogilink2/receive" + std::to_string(i), 100,
//                   [this, i](const rogilink2_interfaces::msg::Frame &msg) {
//                     memcpy(&encoder[i],msg.data.begin(),msg.data.size());
//                   });
//         }
//       } else {
//         RCLCPP_INFO(this->get_logger(), "invalid wheel quantity!");
//       }
//     } else if (measuring_wheel.type_name == "steering") {
//       if (measuring_wheel.quantity > 2) {
//         wheel_sub.resize(2 * measuring_wheel.quantity);
//         for (int i = 0; i < 2 * measuring_wheel.quantity; i++) {
//           wheel_sub[i] =
//               this->create_subscription<rogilink2_interfaces::msg::Frame>(
//                   "rogilink2/receive" + std::to_string(i), 100,
//                   [this](const rogilink2_interfaces::msg::Frame &msg) {
//                     memcpy(&encoder[i], msg.data.begin(), msg.data.size());
//                   });
//         }
//       } else {
//         RCLCPP_INFO(this->get_logger(), "invalid wheel quantity!");
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "invalid wheel type!");
//     }
//   }

//   // set class
//   void set_class() {
//     if (moving_wheel.type_name == "mechanam")
//     {
//       move = new MoveMechanam(moving_wheel);
//     } else if (moving_wheel.type_name == "omni_3W") {
//       move = new MoveOmni3W(moving_wheel);
//     } else if (moving_wheel.type_name == "omni_4W") {
//       move = new MoveOmni4W(moving_wheel);
//     } else if (moving_wheel.type_name == "steeing") {
//       move = new MoveSteering(moving_wheel);
//     }

//     if (measuring_wheel.type_name == "mechanam") {
//       measure = new MeasureMechanam(measuring_wheel);
//     } else if (measuring_wheel.type_name == "omni_3W") {
//       measure = new MeasureOmni3W(measuring_wheel);
//     } else if (measuring_wheel.type_name == "omni_4W") {
//       measure = new MeasureOmni4W(measuring_wheel);
//     } else if (measuring_wheel.type_name == "steeing") {
//       measure = new MeasureSteering(measuring_wheel);
//     } else if (measuring_wheel.type_name == "omni_2W"){
//       measure = new MeasureOmni2W(measuring_wheel);
//     }

//     encoder = new float[measuring_wheel.quantity];
//     wheel_cmd = new float[moving_wheel.quantity];
//   }
//   // main routine
//    void update() {
//     measure->cal_disp(encoder,);
//     move->cal_cmd();

//    };

//    // publishers and subscribers
//    rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr wheel_pub;
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
//    rclcpp::Publisher<nav_msgs::msg::POSetry>::SharedPtr pub_POS;
//    std::vector<
//        rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr>
//        wheel_sub;

//    rclcpp::TimerBase::SharedPtr mytimer;
//    W_PARAM moving_wheel;
//    W_PARAM measuring_wheel;
//    std::string robot_name;
//    POS cmd_vel;
//    float *encoder;
//    float *wheel_cmd;

//    Moving *move;
//    Measuring *measure;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<WheelCtrlRos2>());
//   rclcpp::shutdown();
//   return 0;
// }