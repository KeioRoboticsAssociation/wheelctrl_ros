#include "cmdvel2omni.h"

VelConverter::VelConverter(ros::NodeHandle &nh, const float &body_height, const float &body_width, const int &lost_time_threshold, const int &loop_rate, const bool &gazebo_mode)
    : nh_(nh), BODY_HEIGHT(body_height), BODY_WIDTH(body_width), lost_time_threshold_(lost_time_threshold), loop_rate_(loop_rate), gazebo_mode_(gazebo_mode)
{ //constructer, define pubsub
    ROS_INFO("Creating omni_wheelctrl");
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
        // pub_RF = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_RF", 1);
        // pub_LF = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_LF", 1);
        // pub_LB = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_LB", 1);
        // pub_RB = nh_.advertise<std_msgs::Float32MultiArray>(
        //     "control_RB", 1);
        // pub_Right = nh_.advertise<rogi_link_msgs::RogiLink>(
        //     "send_serial", 1);
        // pub_Left = nh_.advertise<rogi_link_msgs::RogiLink>(
        //     "send_serial", 1);
        control_pub = nh.advertise<rogi_link_msgs::RogiLink>(
            "send_serial", 100);
    }
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1,
                                &VelConverter::cmdvelCallback, this);
    emergency_stop_sub_ = nh_.subscribe("/emergency_stop_flag", 1,
                                &VelConverter::EmergencyStopFlagCallback, this);
    connection_status_sub_ = nh_.subscribe("/connection_status", 1,
                                &VelConverter::ConnectionFlagCallback, this);

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

void VelConverter::EmergencyStopFlagCallback(const std_msgs::Empty::ConstPtr &msg){
    for (int i = 0; i < 4; i++)
    {
        target_speed[i] = 0;
    }
    emergency_stop_flag = !emergency_stop_flag;
}

void VelConverter::ConnectionFlagCallback(const std_msgs::Bool::ConstPtr &msg){
    connection_flag= msg->data;
    // if(connection_flag==true) ROS_INFO("true");
    // else ROS_INFO("false");
}

void VelConverter::cmdvel2omni(){
    float a = BODY_WIDTH / 2.0;
    float b = BODY_HEIGHT / 2.0;
    float c = sqrt(pow(a,2)+pow(b,2));
    const float n = 1/sqrt(2);
    target_speed[0] = n*vx + n*vy + c * omega;
    target_speed[1] = -1*n*vx + n*vy + c * omega;
    target_speed[2] = -1*n*vx - n*vy + c * omega;
    target_speed[3] = n*vx - n*vy + c * omega;


    // エンコーダーの取り付けが通常と逆向きのため、-1倍
    for(int i = 0; i < 4; i++)
    {
        target_speed[i] *= -1;
    }
    // target_speed[1] *= -1.0;
    // target_speed[2] *= -1.0;
}

void VelConverter::reset(){
    ROS_ERROR_ONCE("omni_wheelctrl: unable to subscribe topics. Reset velocity...");
    for (int i = 0; i < 4; i++) {
        target_speed[i] = 0;
        // reset velovity, sustain theta
    }
}

void VelConverter::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    if(!emergency_stop_flag){
        ROS_DEBUG("Received cmd_vel");
        vx = cmd_vel->linear.x;
        vy = cmd_vel->linear.y;
        omega = cmd_vel->angular.z;

        last_sub_vel_time_ = std::chrono::system_clock::now();
    }

    else{
        vx = 0;
        vy = 0;
        omega = 0;
        last_sub_vel_time_ = std::chrono::system_clock::now();

        ROS_ERROR_ONCE("omni_wheelctrl: emergency stopping...");
    }
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
        // std_msgs::Float32MultiArray floatarray;
        // floatarray.data.resize(1);
        // floatarray.data[0] = target_speed[0];
        // pub_RF.publish(floatarray);

        // floatarray.data[0] = target_speed[1];
        // pub_LF.publish(floatarray);

        // floatarray.data[0] = target_speed[2];
        // pub_LB.publish(floatarray);

        // floatarray.data[0] = target_speed[3];
        // pub_RB.publish(floatarray);

        // floatarray.data.resize(2);
        // floatarray.data[0] = target_speed[0];
        // floatarray.data[1] = target_speed[3];
        // pub_Right.publish(floatarray);

        // floatarray.data[0] = target_speed[1];
        // floatarray.data[1] = target_speed[2];
        // pub_Left.publish(floatarray);

        //RF publish
        rogi_link_msgs::RogiLink control_msg;
        control_msg.id= RFMD << 6 | 0x04;
        *(float *)(&control_msg.data[0])=target_speed[0]/WHEEL_DIAMETER/2/M_PI;
        // *(float *)(&control_msg.data[4])=(float)0;
        control_pub.publish(control_msg);

        //LF publish
        control_msg.id= LFMD << 6 | 0x04;
        *(float *)(&control_msg.data[0])=target_speed[1]/WHEEL_DIAMETER/2/M_PI;
        // *(float *)(&control_msg.data[4])=0;
        control_pub.publish(control_msg);

        //LB publish
        control_msg.id= LBMD << 6 | 0x04;
        *(float *)(&control_msg.data[0])=target_speed[2]/WHEEL_DIAMETER/2/M_PI;
        // *(float *)(&control_msg.data[4])=0;
        control_pub.publish(control_msg);

        //RB publish
        control_msg.id= RBMD << 6 | 0x04;
        *(float *)(&control_msg.data[0])=target_speed[3]/WHEEL_DIAMETER/2/M_PI;
        // *(float *)(&control_msg.data[4])=0;
        control_pub.publish(control_msg);

    }
}

void VelConverter::update()
{
    ros::Rate r(loop_rate_);

    while(ros::ok() && connection_flag==false){
        ROS_WARN_ONCE("waiting for serial connection");
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("serial connected");

    while (ros::ok())
    {
        if(isSubscribed()){
            cmdvel2omni();
        }
        else{
            reset();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }
}
