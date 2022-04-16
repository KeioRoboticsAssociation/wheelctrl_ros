#ifndef CMDVEL2OMNI_H_
#define CMDVEL2OMNI_H_

#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "rogi_link_msgs/RogiLink.h"
#include <math.h>

// params
const char RFMD=0x02;
const char LFMD=0x03;
const char LBMD=0x04;
const char RBMD=0x05;

const float WHEEL_DIAMETER = 0.127;

class VelConverter
{
public:
    VelConverter(ros::NodeHandle &nh, const float &body_height, const float &body_width, const int &lost_time_threshold, const int &loop_rate, const bool &gazebo_mode);
    ~VelConverter(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher pub_RF;
    ros::Publisher pub_LF;
    ros::Publisher pub_LB;
    ros::Publisher pub_RB;
    // ros::Publisher pub_Right;
    // ros::Publisher pub_Left;
    ros::Publisher control_pub;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber connection_status_sub_;

    //Configurations
    int loop_rate_;
    float BODY_HEIGHT;
    float BODY_WIDTH;
    int lost_time_threshold_;
    bool gazebo_mode_;

    //variables
    float vx;
    float vy;
    float omega;
    float target_speed[4];// 0:RF, 1:LF, 2:LB, 3:RB

    //flags
    bool emergency_stop_flag = false;
    bool connection_flag = false;

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time_;

    //Methods
    void init_variables();
    void init_drivers();
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void EmergencyStopFlagCallback(const std_msgs::Empty::ConstPtr &msg);
    void ConnectionFlagCallback(const std_msgs::Bool::ConstPtr &msg);
    bool isSubscribed();
    void publishMsg();
    void cmdvel2omni();
    void reset();
    void update();
};

#endif
