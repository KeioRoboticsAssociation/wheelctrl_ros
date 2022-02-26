#ifndef CMDVEL2OMNI_H_
#define CMDVEL2OMNI_H_

#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

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
    ros::Publisher pub_Right;
    ros::Publisher pub_Left;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber emergency_stop_sub_;

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
    bool emergency_stop_flag;

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time_;

    //Methods
    void init_variables();
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void EmergencyStopFlagCallback(const std_msgs::Empty::ConstPtr &msg);
    bool isSubscribed();
    void publishMsg();
    void cmdvel2omni();
    void reset();
    void update();
};

#endif
