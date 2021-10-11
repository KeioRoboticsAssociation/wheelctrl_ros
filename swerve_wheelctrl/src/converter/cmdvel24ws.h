#ifndef CMDVEL24WS_H_
#define CMDVEL24WS_H_

#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

class VelConverter
{
public:
    VelConverter(ros::NodeHandle &nh, const double &body_width, const int &lost_time_threshold, const int &loop_rate, const float &initial_table_angle, const bool& gazebo_mode);
    ~VelConverter(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher pub_RF;
    ros::Publisher pub_LF;
    ros::Publisher pub_LB;
    ros::Publisher pub_RB;
    ros::Publisher pub_RF_angle;
    ros::Publisher pub_LF_angle;
    ros::Publisher pub_LB_angle;
    ros::Publisher pub_RB_angle;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber init_angle_sub_;

    //Configurations
    int loop_rate_;
    double BODY_WIDTH;
    int lost_time_threshold_;
    float initial_table_angle_;
    bool gazebo_mode_;

    //variables
    double wheel_angle[4] = {0,0,0,0};
    double former_wheel_angle[4] = {0,0,0,0};
    
    double vx_;
    double vy_;
    double omega_;
    double target_speed[4];
    double target_theta[4];
    bool emergency_stop_flag;

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time_;

    //Methods
    void init_variables();
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void InitAngleFlagCallback(const std_msgs::Empty::ConstPtr &msg);
    void EmergencyStopFlagCallback(const std_msgs::Empty::ConstPtr &msg);
    bool isSubscribed();
    void publishMsg();
    void cmdvel24ws_per_step(const double &vx, const double &vy, const double &omega);
    void cmdvel24ws();
    void reset();
    void update();
};

#endif
