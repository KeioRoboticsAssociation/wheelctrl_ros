#ifndef MEASURE_WHEEL_ODOM_PUBLISHER_H
#define MEASURE_WHEEL_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <string>

const float PI = 3.1415926535;

class Measure_Wheel_Odom_Publisher
{
public:
    Measure_Wheel_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &c_w_distance_a, const float &c_w_distance_b,
                                const std::string &vertical_axis,const std::string &base_farme_id, const float &wheel_diameter, const float &initial_position_x, const float &initial_position_y, const float &initial_position_theta);
    ~Measure_Wheel_Odom_Publisher(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher odom_pub;
    ros::Subscriber sub_wheel;
    ros::Subscriber sub_IMU;

    tf::TransformBroadcaster odom_broadcaster;

    //Configurations
    int loop_rate_;
//    float WHEEL_WIDTH;
//    float WHEEL_HEIGHT;
    float CENTER_WHEEL_DISTANCE_A;
    float CENTER_WHEEL_DISTANCE_B;
    std::string VERTICAL_AXIS;
    std::string base_frame_id_;
    float wheel_diameter_;
    float initial_position_x_;
    float initial_position_y_;
    float initial_position_theta_;

    //variables
    float x, y, theta;
    float vx, vy, omega;
    float old_vx, old_vy, old_omega;
    float wheel_speed[2]; // 0:右前左後, 1:左前右後（機械からみて）
    float rotate_speed;

    //Methods
    void init_variables();
    void Wheel_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
    void update();
};

#endif