#include "general_wheelctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "rogilink2_interfaces/srv/request_add_topic.hpp"
#include "sensor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace chrono_literals;
using namespace illias;

class WheelCtrlRos2 : public rclcpp::Node {
 public:
  WheelCtrlRos2() : Node("wheel_ctrl_ros2") { sim_mode = false; }
  void init() {
    RCLCPP_INFO(this->get_logger(), "ACTIVATED: wheelctrl_ros2");
    set_wheel_parameter();
    set_subclass();
    set_handles();
    if (!sim_mode) {
      set_initial_pos();
    }
    RCLCPP_INFO(this->get_logger(), "wheelctrl_ros2 initialized");

    mytimer =
        this->create_wall_timer(10ms, std::bind(&WheelCtrlRos2::update, this));
  }

 private:
  void set_wheel_parameter();
  void set_subclass();
  void set_handles();
  void set_initial_pos();

  void update();
  void get_tf();
  void pub_rogilink2_frame();
  void pub_odometry();

  std::shared_ptr<illias::Measuring> measure;
  std::shared_ptr<illias::Moving> moving;
  rclcpp::TimerBase::SharedPtr mytimer;
  std::unique_ptr<Sensor> photo;

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
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  bool AMCL = false;
  bool sim_mode;
  bool tf_flag;

  std::string odom_frame_id;
  std::string base_frame_id;
};

void WheelCtrlRos2::set_wheel_parameter() {
  RCLCPP_INFO(this->get_logger(), "set_wheel_parameter");
  this->declare_parameter("robot_param.name", "undefined");
  this->declare_parameter("robot_param.AMCL", false);
  this->declare_parameter("robot_param.tf", true);
  this->declare_parameter("robot_param.odom_frame_id", "odom");
  this->declare_parameter("robot_param.base_frame_id", "base_link");

  this->declare_parameter("moving_wheel.type_name", "steering");
  this->declare_parameter("moving_wheel.radius", 0.0);
  this->declare_parameter("moving_wheel.quantity", 0);
  this->declare_parameter("moving_wheel.loop_rate", 200.0);
  this->declare_parameter("moving_wheel.gear_ratio", 1.0);
  this->declare_parameter("moving_wheel.gear_ratio_steer", 1.0);
  this->declare_parameter("moving_wheel.distance", vector<double>(8, 0.0));
  this->declare_parameter("moving_wheel.arguments", vector<double>(8, 0.0));
  this->declare_parameter("moving_wheel.wheel_name",
                          vector<std::string>(8, "undefined"));

  this->declare_parameter("measuring_wheel.type_name", "steering");
  this->declare_parameter("measuring_wheel.radius", 0.0);
  this->declare_parameter("measuring_wheel.quantity", 0);
  this->declare_parameter("measuring_wheel.loop_rate", 200.0);
  this->declare_parameter("measuring_wheel.gear_ratio", 1.0);
  this->declare_parameter("measuring_wheel.gear_ratio_steer", 1.0);
  this->declare_parameter("measuring_wheel.distance", vector<double>(8, 0.0));
  this->declare_parameter("measuring_wheel.arguments", vector<double>(8, 0.0));
  this->declare_parameter("measuring_wheel.wheel_name",
                          vector<std::string>(8, "undefined"));

  robot_name = this->get_parameter("robot_param.name").as_string();
  AMCL = this->get_parameter("robot_param.AMCL").as_bool();
  tf_flag = this->get_parameter("robot_param.tf").as_bool();
  odom_frame_id = this->get_parameter("robot_param.odom_frame_id").as_string();
  base_frame_id = this->get_parameter("robot_param.base_frame_id").as_string();
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
      (float)this->get_parameter("moving_wheel.gear_ratio_steer").as_double();
  std::vector<double> dist_buff1 =
      this->get_parameter("moving_wheel.distance").as_double_array();
  std::vector<double> arg_buff1 =
      this->get_parameter("moving_wheel.arguments").as_double_array();
  moving_name =
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
  measuring_name =
      this->get_parameter("measuring_wheel.wheel_name").as_string_array();

  illias::W_PARAM wheel_param;
  for (int i = 0; i < (int)moving_wheel.quantity; i++) {
    wheel_param.distance = (float)dist_buff1[i];
    wheel_param.argument = (float)arg_buff1[i] * M_PI / 180;
    moving_wheel.wheels.push_back(wheel_param);
  }
  for (int i = 0; i < (int)measuring_wheel.quantity; i++) {
    wheel_param.distance = (float)dist_buff2[i];
    wheel_param.argument = (float)arg_buff2[i] * M_PI / 180;
    measuring_wheel.wheels.push_back(wheel_param);
  }
  RCLCPP_INFO(this->get_logger(), "parameter setting end");
}

void WheelCtrlRos2::set_subclass() {
  RCLCPP_INFO(this->get_logger(), "set_subclass : measuring");

  if (measuring_wheel.type_name == "omni") {
    encoder.resize(measuring_wheel.quantity);
    switch (measuring_wheel.quantity) {
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
  } else if (measuring_wheel.type_name == "steering") {
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
  if (moving_wheel.type_name == "omni") {
    cmd_rotate.resize(moving_wheel.quantity);
    if (!sim_mode) {
      for (int i = 0; i < moving_wheel.quantity; i++) {
        drivers.push_back(
            std::make_shared<MD2022>(this, moving_name[i], 100, 100));
        RCLCPP_INFO(this->get_logger(), "omni wheel start %d", i);
        drivers.at(i)->init();
        drivers.at(i)->setMode(Md::Mode::Velocity);
        drivers.at(i)->setVelocity(0.0);
      }
    }
    switch (moving_wheel.quantity) {
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
          drivers.push_back(
              std::make_shared<ODrive>(this, moving_name[i], 100, 6));
          RCLCPP_INFO(this->get_logger(), "steering wheel start %d", i);
          drivers.at(i)->init();
          drivers.at(i)->setMode(Md::Mode::Velocity);
          drivers.at(i)->setPosition(0.0);
        } else {
          drivers.push_back(
              std::make_shared<MD2022>(this, moving_name[i], 100, 100));
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

void WheelCtrlRos2::set_handles() {
  RCLCPP_INFO(this->get_logger(), "set_handles");
  frame_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
      "/rogilink2/send", 10);
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom0", 10);
  if (tf_flag) {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd.x = msg->linear.x;
        cmd.y = msg->linear.y;
        cmd.w = msg->angular.z;
        // RCLCPP_INFO(this->get_logger(), "cmd_vel : %f, %f, %f", cmd.x, cmd.y,
        //             cmd.w);
      });

  if (!sim_mode) {
    int sub_num = measuring_wheel.type_name == "steering"
                      ? 2 * measuring_wheel.quantity
                      : measuring_wheel.quantity;
    encoder_sub.resize(sub_num);
    for (int i = 0; i < sub_num; i++) {
      encoder_sub[i] =
          this->create_subscription<rogilink2_interfaces::msg::Frame>(
              "rogilink2/recieve_" + measuring_name[i], 10,
              [this, i](const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
                memcpy(&encoder[i], msg->data.data(), sizeof(float));
              });
    }
  }
  photo = std::make_unique<Sensor>(this, "steer_sensor");
  RCLCPP_INFO(this->get_logger(), "set_handles end");
}

void WheelCtrlRos2::set_initial_pos() {
  RCLCPP_INFO(this->get_logger(), "set_initial_pos");
  current_pos.x = 0;
  current_pos.y = 0;
  current_pos.w = 0;

  // set steer angle
  if (moving_wheel.type_name == "steering") {
    photo->start();
    for (int i = 0; i < 4; i++) {
      drivers[i + 4]->setMode(Md::Mode::Velocity);
      drivers[i + 4]->setVelocity(0.1);
      std::this_thread::sleep_for(10ms);
    }
    int completed_count = 0;
    float offset[4] = {5.0 / 12 - 0.008, -1.0 / 3 - 0.005, 1.0 / 3 - 0.002,
                       -5.0 / 12 - 0.005};
    while (completed_count != 0b1111) {
      std::this_thread::sleep_for(10ms);
      rclcpp::spin_some(this->shared_from_this());
      std::vector<int> val = photo->read();
      if (val.size() != 4) {
        RCLCPP_INFO(this->get_logger(), "No sensor data");
        continue;
      }
      // RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d", val[0], val[1],
      // val[2],
      //             val[3]);

      for (int i = 0; i < 4; i++) {
        if (completed_count & (1 << i)) continue;
        // cmd_rotate[i] = 0;
        if (val[i] <= 1024) {
          drivers[i + 4]->setVelocity(0);
          drivers[i + 4]->setMode(Md::Mode::Idle);

          completed_count |= (1 << i);
        }
      }
    }
    std::this_thread::sleep_for(100ms);
    for (int i = 0; i < 4; i++) {
      drivers[i + 4]->resetEncoder(offset[i] * moving_wheel.gear_ratio_steer);
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      drivers[i + 4]->setPosition(0);
      drivers[i + 4]->setMode(Md::Mode::Position);
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    photo->stop();
  }
  RCLCPP_INFO(this->get_logger(), "set_initial_pos end");
}

void WheelCtrlRos2::update() {
  // RCLCPP_INFO(this->get_logger(),"update");
  if (!sim_mode) {
    // get current possition
    if (measuring_wheel.type_name == "steering") {
      for (int i = 0; i < 2 * measuring_wheel.quantity; i++) {
        encoder[i] =
            i < 4 ? drivers[i]->getVelocity() : drivers[i]->getPosition();
      }
    } else {
      for (int i = 0; i < measuring_wheel.quantity; i++) {
        encoder[i] = drivers[i]->getVelocity();
      }
    }
    get_tf();
    measure->cal_disp(encoder);
    current_pos = measure->get_current_pos();
    current_vel = measure->get_current_vel();
    // RCLCPP_INFO(this->get_logger(), "pos: %f,%f,%f", current_pos.x,
    //             current_pos.y, current_pos.w);
  }
  // set cmd
  //  RCLCPP_INFO(this->get_logger(), "%f,%f,%f", cmd.x, cmd.y, cmd.w);
  // cmd.x = 1;
  // cmd.y = 1;
  // cmd.w = 0;
  moving->cal_cmd(cmd, false);
  // set wheel_cmd
  float cmd_num = moving_wheel.type_name == "steering"
                      ? 2 * moving_wheel.quantity
                      : moving_wheel.quantity;
  for (int i = 0; i < cmd_num; i++) {
    cmd_rotate[i] = moving->wheel_cmd_rot[i];
  }
  // RCLCPP_INFO(this->get_logger(), "vel: %f,%f,%f,%f", cmd_rotate[0],
  //             cmd_rotate[1], cmd_rotate[2], cmd_rotate[3]);
  // RCLCPP_INFO(this->get_logger(), "arg: %f,%f,%f,%f", cmd_rotate[4],
  //             cmd_rotate[5], cmd_rotate[6], cmd_rotate[7]);
  // publish
  if (!sim_mode) {
    pub_rogilink2_frame();
    pub_odometry();
  }
}

void WheelCtrlRos2::pub_rogilink2_frame() {
  if (moving_wheel.type_name == "steering") {
    for (int i = 0; i < 2 * moving_wheel.quantity; i++) {
      if (i < 4) {
        drivers[i]->setVelocity(moving->wheel_cmd_rot[i]);
      } else {
        drivers[i]->setPosition(moving->wheel_cmd_rot[i]);
      }
    }
  } else {
    for (int i = 0; i < moving_wheel.quantity; i++) {
      drivers[i]->setVelocity(moving->wheel_cmd_rot[i]);
    }
  }
}
void WheelCtrlRos2::get_tf() {
  geometry_msgs::msg::TransformStamped tf;

  try {
    // 何故かrclcpp::Time(0)じゃないと変数でodom_frame_idとbase_frame_idを指定できない
    tf = tf_buffer_->lookupTransform(odom_frame_id, base_frame_id,
                                     rclcpp::Time(0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s",
                ex.what());
    return;
  }
  POS tf_pos;
  tf_pos.x = tf.transform.translation.x;
  tf_pos.y = tf.transform.translation.y;

  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  tf_pos.w = (float)yaw;

  measure->set_past_pos(tf_pos);
}

void WheelCtrlRos2::pub_odometry() {
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = odom_frame_id;
  msg.child_frame_id = base_frame_id;
  msg.pose.pose.position.x = current_pos.x;
  msg.pose.pose.position.y = current_pos.y;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = sin(current_pos.w / 2);
  msg.pose.pose.orientation.w = cos(current_pos.w / 2);
  msg.twist.twist.linear.x = current_vel.x;
  msg.twist.twist.linear.y = current_vel.y;
  msg.twist.twist.angular.z = current_vel.w;
  odom_pub->publish(msg);
  if (tf_flag) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame_id;
    transform.child_frame_id = base_frame_id;
    transform.transform.translation.x = current_pos.x;
    transform.transform.translation.y = current_pos.y;
    transform.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pos.w);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(transform);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WheelCtrlRos2>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
