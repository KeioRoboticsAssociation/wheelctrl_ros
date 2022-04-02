#ifndef MEASURE_WHEEL_ODOM_PUBLISHER_H
#define MEASURE_WHEEL_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <string>

class Measure_Wheel_Odom_Publisher
{
public:
    Measure_Wheel_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &c_w_distance,
                                const std::string &vertical_axis,const std::string &base_farme_id);
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
    float CENTER_WHEEL_DISTANCE;
    std::string VERTICAL_AXIS;
    std::string base_frame_id_;

    //variables
    float x, y, theta;
    float vx, vy, omega;
    float old_vx, old_vy, old_omega;
    float wheel_speed[2]; // 0:前後, 1:左右　（機械からみて）
    float rotate_speed;

    //Methods
    void init_variables();
    void Wheel_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
    void update();
};

#endif