#include "general_wheelctrl.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"

class WheelCtrlRos2:public rclcpp::Node{
    public:
    WheelCtrlRos2():Node("wheel_ctrl_ros2"){
      RCLCPP_INFO(this->get_logger(), "ACTIVATED: wheelctrl_ros2");
      set_wheel_parameter();
      set_initial_pos();
      set_subclass();
    }

    private:
     void set_wheel_parameter();
     void set_initial_pos();
     void set_subclass();
     std::unique_ptr<illias::Measuring> measure;
     std::unique_ptr<illias::Moving> move;
     illias::W_PARAM moving_wheel;
     illias::W_PARAM measuring_wheel;
     illias::POS current_pos;
     std::string robot_name;
};

void WheelCtrlRos2::set_wheel_parameter(){
  this->declare_parameter("robot_param.name", "undefined");

  this->declare_parameter("moving_wheel.type_name", "undefined");
  this->declare_parameter("moving_wheel.radius", 0.0);
  this->declare_parameter("moving_wheel.quantity", 0);
  this->declare_parameter("moving_wheel.gear_ratio", 1.0);
  this->declare_parameter("moving_wheel.gear_ratio_horizonal", 1.0);
  this->declare_parameter("moving_wheel.coordinate", "undefined");
  this->declare_parameter("moving_wheel.distance", 0.0);
  this->declare_parameter("moving_wheel.arguments", vector<double>(4, 0.0));
  this->declare_parameter("moving_wheel.length_x", 1.0);
  this->declare_parameter("moving_wheel.length_y", 1.0);

  this->declare_parameter("measuring_wheel.type_name", "undefined");
  this->declare_parameter("measuring_wheel.radius", 0.0);
  this->declare_parameter("measuring_wheel.quantity", 0);
  this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
  this->declare_parameter("measuring_wheel.gear_ratio_horizonal", 1.0);
  this->declare_parameter("measuring_wheel.coordinate", "undefined");
  this->declare_parameter("measuring_wheel.distance", 0.0);
  this->declare_parameter("measuring_wheel.arguments", vector<double>(4, 0));
  this->declare_parameter("measuring_wheel.length_x", 1.0);
  this->declare_parameter("measuring_wheel.length_y", 1.0);

  robot_name = this->get_parameter("robot_param.name").as_string();

  // moving_wheel
  moving_wheel.type_name =
      this->get_parameter("moving_wheel.type_name").as_string();
  moving_wheel.radius =
      (float)this->get_parameter("moving_wheel.radius").as_double();
  moving_wheel.quantity =
      (float)this->get_parameter("moving_wheel.quantity").as_int();
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
      (float)this->get_parameter("measuring_wheel.quantity").as_int();
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
    switch (measuring_wheel.quantity)
    {
    case 2:
      measure =
          std::make_unique<illias::MeasureOmni2W>(measuring_wheel,
          current_pos);
      break;
    case 3:
      measure =
          std::make_unique<illias::MeasureOmni3W>(measuring_wheel,
          current_pos);
      break;
    case 4:
      measure =
          std::make_unique<illias::MeasureOmni4W>(measuring_wheel,
          current_pos);
      break;
    }
  } else if (measuring_wheel.type_name == "steering") {
    measure =
    std::make_unique<illias::MeasureSteering>(measuring_wheel,current_pos);
  } else if (measuring_wheel.type_name == "mechanam") {
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
  }

  RCLCPP_INFO(this->get_logger(),"nyannyan");
  if (moving_wheel.type_name == "omni") {
    switch (moving_wheel.quantity) {
      case 3:
        move = std::make_unique<illias::MoveOmni3W>(moving_wheel);
        break;
      case 4:
        RCLCPP_INFO(this->get_logger(), "hya");
        move = std::make_unique<illias::MoveOmni4W>(moving_wheel);
    }
  } else if (moving_wheel.type_name == "steering") {
    move = std::make_unique<illias::MoveSteering>(moving_wheel);
  } else if (moving_wheel.type_name == "mechanam") {
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid wheel type");
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