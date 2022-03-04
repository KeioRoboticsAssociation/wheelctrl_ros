#ifndef OMNI_ODOM_PUBLISHER_H
#define OMNI_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <string>

class Omni_Odom_Publisher
{
public:
    Omni_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &body_height, const float &body_width, const std::string &base_farme_id);
    ~Omni_Odom_Publisher(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher odom_pub;
    // ros::Subscriber sub_RF;
    // ros::Subscriber sub_LF;
    // ros::Subscriber sub_LB;
    // ros::Subscriber sub_RB;
    ros::Subscriber sub_Right;
    ros::Subscriber sub_Left;

    tf::TransformBroadcaster odom_broadcaster;

    //Configurations
    int loop_rate_;
    float BODY_WIDTH;
    float BODY_HEIGHT;
    std::string base_frame_id_;

    //variables
    float x, y, theta;
    float vx, vy, omega;
    float old_vx, old_vy, old_omega;
    float wheel_speed[4]; // 0:RF, 1:LF, 2:LB, 3:RB

    //Methods
    void init_variables();
    // void RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    // void LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    // void RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    // void LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Right_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Left_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void update();
};

#endif