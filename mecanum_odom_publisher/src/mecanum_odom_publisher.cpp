#include "mecanum_odom_publisher/mecanum_odom_publisher.h"

std::string node_name = "mecanum_odom_publisher";

Mecanum_Odom_Publisher::Mecanum_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &body_height, const float &body_width, const std::string &base_frame_id)
    : nh_(nh), loop_rate_(loop_rate), BODY_HEIGHT(body_height), BODY_WIDTH(body_width), base_frame_id_(base_frame_id)
{ //constructer, define pubsub
    ROS_INFO("Creating mecanum_odom_publisher");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("body_height [m]: " << BODY_HEIGHT);
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);

    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    sub_RF = nh_.subscribe("data_RF", 1, &Mecanum_Odom_Publisher::RF_Callback, this);
    sub_LF = nh_.subscribe("data_LF", 1, &Mecanum_Odom_Publisher::LF_Callback, this);
    sub_LB = nh_.subscribe("data_LB", 1, &Mecanum_Odom_Publisher::LB_Callback, this);
    sub_RB = nh_.subscribe("data_RB", 1, &Mecanum_Odom_Publisher::RB_Callback, this);
    // sub_Right = nh_.subscribe("data_Right", 1, &Mecanum_Odom_Publisher::Right_Callback, this);
    // sub_Left = nh_.subscribe("data_Left", 1, &Mecanum_Odom_Publisher::Left_Callback, this);
    // Float32MultiArray data[1]; data[0]=v

    init_variables();

    update();
}

void Mecanum_Odom_Publisher::init_variables()
{
    vx = 0;
    vy = 0;
    omega = 0;
    old_vx = 0;
    old_vy = 0;
    old_omega = 0;
    x = 0;
    y = 0;
    theta = 0;
    for (int i = 0; i < 4; i++)
    {
        wheel_speed[i] = 0;
    }
}

void Mecanum_Odom_Publisher::RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[0] = msg->data[0];
}

void Mecanum_Odom_Publisher::LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[1] = msg->data[0];
}

void Mecanum_Odom_Publisher::LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[2] = msg->data[0];
}

void Mecanum_Odom_Publisher::RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[3] = msg->data[0];
}

// void Mecanum_Odom_Publisher::Right_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//     wheel_speed[0] = msg->data[0];
//     wheel_speed[3] = msg->data[1];
// }

// void Mecanum_Odom_Publisher::Left_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//     wheel_speed[1] = msg->data[0];
//     wheel_speed[2] = msg->data[1];
// }

void Mecanum_Odom_Publisher::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        float a = BODY_WIDTH / 2.0;
        float b = BODY_HEIGHT / 2.0;
        vx = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / 4.0;
        vy = (wheel_speed[0] - wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) / 4.0;
        omega = (wheel_speed[0] - wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) / 4.0 / (a+b);

        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        float delta_t = (current_time - last_time).toSec();
        x = x + vx * delta_t;
        y = y + vy * delta_t;
        theta = theta + omega * delta_t;
        theta = atan2(sin(theta), cos(theta)); // inf to pi
        last_time = current_time;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = base_frame_id_;

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = base_frame_id_;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;

        //publish the message
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    float body_width = 0.440;
    float body_height = 0.440;
    std::string base_frame_id = "base_link";

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("body_width", body_width);
    arg_n.getParam("body_height", body_height);
    arg_n.getParam("base_frame_id", base_frame_id);

    Mecanum_Odom_Publisher publisher(nh, looprate, body_height, body_width, base_frame_id);
    return 0;
}
