#include "measure_wheel_odom_publisher/measure_wheel_odom_publisher.h"

std::string node_name = "Measure_Wheel_Odom_Publisher";

Measure_Wheel_Odom_Publisher::Measure_Wheel_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate,
                                                            const float &c_w_distance_a,
                                                            const float &c_w_distance_b,
                                                            const std::string &vertical_axis,
                                                            const std::string &base_frame_id,
                                                            const float &wheel_diameter)
    :nh_(nh),
    loop_rate_(loop_rate),
    CENTER_WHEEL_DISTANCE_A(c_w_distance_a),
    CENTER_WHEEL_DISTANCE_B(c_w_distance_b),
    VERTICAL_AXIS(vertical_axis),
    base_frame_id_(base_frame_id),
    wheel_diameter_(wheel_diameter)
{ //constructer, define pubsub
    ROS_INFO("Creating Measure_Wheel_Odom_Publisher");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("c_w_istance_a[m]"<<CENTER_WHEEL_DISTANCE_A);
    ROS_INFO_STREAM("c_w_istance_a[m]"<<CENTER_WHEEL_DISTANCE_B);
    // ROS_INFO_STREAM("body_height [m]: " << WHEEL_HEIGHT);
    // ROS_INFO_STREAM("body_width [m]: " << WHEEL_WIDTH);
    ROS_INFO_STREAM("vertical_axis: " << VERTICAL_AXIS);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);

    //publisher
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    //subscriber
    sub_wheel = nh_.subscribe("rcv_serial_01", 1, &Measure_Wheel_Odom_Publisher::Wheel_Callback, this);
    sub_IMU = nh_.subscribe("imu",1, &Measure_Wheel_Odom_Publisher::Imu_Callback, this);
    // Float32MultiArray data[1]; data[0]=v;

    init_variables();

    update();
}

void Measure_Wheel_Odom_Publisher::init_variables()
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
    for (int i = 0; i < 2; i++)
    {
        wheel_speed[i] = 0;
    }
    rotate_speed=0;
}

void Measure_Wheel_Odom_Publisher::Wheel_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[0] = msg->data[0] * wheel_diameter_ * 2 * PI;
    wheel_speed[1] = msg->data[1] * wheel_diameter_ * 2 * PI;
}

void Measure_Wheel_Odom_Publisher::Imu_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(VERTICAL_AXIS == "x")
    {
        rotate_speed = msg->angular_velocity.x;
    }else if(VERTICAL_AXIS == "y")
    {
        rotate_speed = msg->angular_velocity.y;
    }else if(VERTICAL_AXIS == "z")
    {
        rotate_speed = msg->angular_velocity.z;
    }
}

void Measure_Wheel_Odom_Publisher::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        omega = rotate_speed;
        vx = (wheel_speed[0] - omega * CENTER_WHEEL_DISTANCE_A)*cos(theta)
                - (wheel_speed[1] - omega * CENTER_WHEEL_DISTANCE_B) * sin(theta);
        vy = (wheel_speed[0] - omega * CENTER_WHEEL_DISTANCE_A)*sin(theta)
                + (wheel_speed[1] - omega * CENTER_WHEEL_DISTANCE_B) * cos(theta);

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
    // float body_width = 0.150;
    // float body_height = 0.150;
    float center_wheel_distance_a = 0.064;
    float center_wheel_distance_b = 0.074;

    std::string vertical_axis = "x";
    std::string base_frame_id = "base_link";
    float wheel_diameter = 0.048;

    arg_n.getParam("control_frequency", looprate);
    // arg_n.getParam("body_height", body_height);
    // arg_n.getParam("body_width", body_width);
    arg_n.getParam("c_w_distance_a",center_wheel_distance_a);
    arg_n.getParam("c_w_distance_b",center_wheel_distance_b);
    arg_n.getParam("vertical_axis",vertical_axis);
    arg_n.getParam("base_frame_id", base_frame_id);
    arg_n.getParam("wheel_diameter", wheel_diameter);

    Measure_Wheel_Odom_Publisher publisher(nh, looprate, center_wheel_distance_a, center_wheel_distance_b,vertical_axis,base_frame_id,wheel_diameter);
    return 0;
}
