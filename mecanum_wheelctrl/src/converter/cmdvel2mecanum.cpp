#include "cmdvel2mecanum.h"

VelConverter::VelConverter(ros::NodeHandle &nh, const float &body_height, const float &body_width, const int &lost_time_threshold, const int &loop_rate, const bool &gazebo_mode)
    : nh_(nh), BODY_HEIGHT(body_height), BODY_WIDTH(body_width), lost_time_threshold_(lost_time_threshold), loop_rate_(loop_rate), gazebo_mode_(gazebo_mode)
{ //constructer, define pubsub
    ROS_INFO("Creating swerve_wheelctrl");
    ROS_INFO_STREAM("body_height [m]: " << BODY_HEIGHT);
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("lost_time_threshold [ms]: " << lost_time_threshold_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("gazebo_mode: " << gazebo_mode_);

    init_variables();

    if(gazebo_mode_){
        pub_RF = nh_.advertise<std_msgs::Float64>(
            "control_RF", 1);
        pub_LF = nh_.advertise<std_msgs::Float64>(
            "control_LF", 1);
        pub_LB = nh_.advertise<std_msgs::Float64>(
            "control_LB", 1);
        pub_RB = nh_.advertise<std_msgs::Float64>(
            "control_RB", 1);
    }
    else{
        pub_RF = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_RF", 1);
        pub_LF = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_LF", 1);
        pub_LB = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_LB", 1);
        pub_RB = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_RB", 1);
        // pub_Right = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_Right", 1);
        // pub_Left = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_Left", 1);
    }
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1,
                                 &VelConverter::cmdvelCallback, this);

    last_sub_vel_time_ = std::chrono::system_clock::now();

    update();
}

void VelConverter::init_variables(){
    vx = 0;
    vy = 0;
    omega = 0;
    for (int i = 0; i < 4; i++)
    {
        target_speed[i] = 0;
    }
}

void VelConverter::cmdvel2mecanum(){
    float a = BODY_WIDTH / 2.0;
    float b = BODY_HEIGHT / 2.0;
    target_speed[0] = vx + vy + (a + b) * omega;
    target_speed[1] = vx - vy - (a + b) * omega;
    target_speed[2] = vx + vy - (a + b) * omega;
    target_speed[3] = vx - vy + (a + b) * omega;

    // target_speed[1] *= -1.0;
    // target_speed[2] *= -1.0;
}

void VelConverter::reset(){
    ROS_ERROR("mecanum_wheelctrl: unable to subscribe topics. Reset velocity...");
    for (int i = 0; i < 4; i++) {
        target_speed[i] = 0;
        // reset velovity, sustain theta
    }
}

void VelConverter::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    ROS_DEBUG("Received cmd_vel");
    vx = cmd_vel->linear.x;
    vy = cmd_vel->linear.y;
    omega = cmd_vel->angular.z;

    last_sub_vel_time_ = std::chrono::system_clock::now();
}

bool VelConverter::isSubscribed() {
    auto current_time = std::chrono::system_clock::now();
    const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_sub_vel_time_).count();

    if (vel_elapsed < lost_time_threshold_) {
        return true;
    } else {
        return false;
    }
}

void VelConverter::publishMsg()
{
    if(gazebo_mode_){
        std_msgs::Float64 command;
        command.data = target_speed[0];
        pub_RF.publish(command);

        command.data = target_speed[1];
        pub_LF.publish(command);

        command.data = target_speed[2];
        pub_LB.publish(command);

        command.data = target_speed[3];
        pub_RB.publish(command);
    }
    else{
        std_msgs::Float32MultiArray floatarray;
        floatarray.data.resize(1);
        floatarray.data[0] = target_speed[0];
        pub_RF.publish(floatarray);

        floatarray.data[0] = target_speed[1];
        pub_LF.publish(floatarray);

        floatarray.data[0] = target_speed[2];
        pub_LB.publish(floatarray);

        floatarray.data[0] = target_speed[3];
        pub_RB.publish(floatarray);

        // floatarray.data.resize(2);
        // floatarray.data[0] = target_speed[0];
        // floatarray.data[1] = target_speed[3];
        // pub_Right.publish(floatarray);

        // floatarray.data[0] = target_speed[1];
        // floatarray.data[1] = target_speed[2];
        // pub_Left.publish(floatarray);
    }
}

void VelConverter::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        if(isSubscribed()){
            cmdvel2mecanum();
        }
        else{
            reset();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }
}
