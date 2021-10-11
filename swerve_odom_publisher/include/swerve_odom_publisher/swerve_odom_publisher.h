#ifndef SWERVE_ODOM_PUBLISHER_H
#define SWERVE_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <string>

class Swerve_Odom_Publisher
{
public:
    Swerve_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const double &body_width, const std::string &base_farme_id, const float &initial_table_angle, const bool &gazebo_mode);
    ~Swerve_Odom_Publisher(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher odom_pub;
    ros::Subscriber bno_sub;
    ros::Subscriber sub_RF;
    ros::Subscriber sub_LF;
    ros::Subscriber sub_LB;
    ros::Subscriber sub_RB;
    ros::Subscriber sub_state_gazebo;

    tf::TransformBroadcaster odom_broadcaster;

    //Configurations
    int loop_rate_;
    double BODY_WIDTH;
    std::string base_frame_id_;
    float initial_table_angle_;
    bool gazebo_mode_;

    //variables
    double wheelpos[4][2]={}; // 0:RF, 1:LF, 2:LB, 3:RB
    double center_xy[2]={};
    double old_center_xy[2]={};
    double theta = 0;
    double old_theta = 0;

    double gazebo_RF_vel;
    double gazebo_RF_angle;
    double gazebo_LF_vel;
    double gazebo_LF_angle;
    double gazebo_RB_vel;
    double gazebo_RB_angle;
    double gazebo_LB_vel;
    double gazebo_LB_angle;

    //Methods
    void RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void state_gazebo_Callback(const sensor_msgs::JointState::ConstPtr &msg);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void processing_gazebo_data();
    void CalcRobotCenter();
    void CalcRobotAngle();
    void update();
};

#endif