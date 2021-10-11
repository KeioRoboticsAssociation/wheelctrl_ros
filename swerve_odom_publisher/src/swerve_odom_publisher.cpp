#include "swerve_odom_publisher/swerve_odom_publisher.h"

std::string node_name = "swerve_odom_publisher";

Swerve_Odom_Publisher::Swerve_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const double &body_width, const std::string &base_frame_id, const float &initial_table_angle, const bool &gazebo_mode)
    : nh_(nh), loop_rate_(loop_rate), BODY_WIDTH(body_width), base_frame_id_(base_frame_id), initial_table_angle_(initial_table_angle), gazebo_mode_(gazebo_mode)
{ //constructer, define pubsub
    ROS_INFO("Creating swerve_odom_publisher");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);
    ROS_INFO_STREAM("initial_table_angle [deg]: " << initial_table_angle_);
    ROS_INFO_STREAM("gazebo_mode: " << gazebo_mode_);

    initial_table_angle_ *= M_PI / 180.0; // rad

    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    bno_sub = nh_.subscribe("imu", 1, &Swerve_Odom_Publisher::imuCallback, this);
    if(gazebo_mode_){
        sub_state_gazebo = nh_.subscribe("/simple_swerve/joint_states", 1, &Swerve_Odom_Publisher::state_gazebo_Callback, this);
    }
    else{
        sub_RF = nh_.subscribe("data_RF", 1, &Swerve_Odom_Publisher::RF_Callback, this);
        sub_LF = nh_.subscribe("data_LF", 1, &Swerve_Odom_Publisher::LF_Callback, this);
        sub_LB = nh_.subscribe("data_LB", 1, &Swerve_Odom_Publisher::LB_Callback, this);
        sub_RB = nh_.subscribe("data_RB", 1, &Swerve_Odom_Publisher::RB_Callback, this);
        // Float32MultiArray data[2]; data[0]=v, data[1]=theta
    }

    for (int i = 0; i < 4; i++)
    {
        wheelpos[i][0] = BODY_WIDTH / sqrt(2) * cos(M_PI * (2 * i + 1) / 4);
        wheelpos[i][1] = BODY_WIDTH / sqrt(2) * sin(M_PI * (2 * i + 1) / 4);
    }

    update();
}

void Swerve_Odom_Publisher::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Passed by Reference
}

void Swerve_Odom_Publisher::imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, imu->orientation);
    theta = yaw;
    //ROS_INFO("%f", body_theta*180.0/M_PI);
}

void Swerve_Odom_Publisher::RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[0][0] += delta * cos(theta+msg->data[1]+initial_table_angle_);
    wheelpos[0][1] += delta * sin(theta+msg->data[1]+initial_table_angle_);
    last_time = current_time;
}

void Swerve_Odom_Publisher::LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[1][0] += delta * cos(theta+msg->data[1]+initial_table_angle_);
    wheelpos[1][1] += delta * sin(theta+msg->data[1]+initial_table_angle_);
    last_time = current_time;
}

void Swerve_Odom_Publisher::LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[2][0] += delta * cos(theta+msg->data[1]+initial_table_angle_);
    wheelpos[2][1] += delta * sin(theta+msg->data[1]+initial_table_angle_);
    last_time = current_time;
}

void Swerve_Odom_Publisher::RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[3][0] += delta * cos(theta+msg->data[1]+initial_table_angle_);
    wheelpos[3][1] += delta * sin(theta+msg->data[1]+initial_table_angle_);
    last_time = current_time;
}

void Swerve_Odom_Publisher::state_gazebo_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    gazebo_LB_vel = msg->velocity[1] * 0.035;
    gazebo_LF_vel = msg->velocity[3] * 0.035;
    gazebo_RB_vel = msg->velocity[5] * 0.035;
    gazebo_RF_vel = msg->velocity[7] * 0.035;

    gazebo_LB_angle = msg->position[0]+initial_table_angle_;
    gazebo_LF_angle = msg->position[2]+initial_table_angle_;
    gazebo_RB_angle = msg->position[4]+initial_table_angle_;
    gazebo_RF_angle = msg->position[6]+initial_table_angle_;
}

void Swerve_Odom_Publisher::processing_gazebo_data()
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double delta = gazebo_RF_vel * (current_time - last_time).toSec();
    wheelpos[0][0] += delta * cos(theta + gazebo_RF_angle);
    wheelpos[0][1] += delta * sin(theta + gazebo_RF_angle);
    delta = gazebo_LF_vel * (current_time - last_time).toSec();
    wheelpos[1][0] += delta * cos(theta + gazebo_LF_angle);
    wheelpos[1][1] += delta * sin(theta + gazebo_LF_angle);
    delta = gazebo_LB_vel * (current_time - last_time).toSec();
    wheelpos[2][0] += delta * cos(theta + gazebo_LB_angle);
    wheelpos[2][1] += delta * sin(theta + gazebo_LB_angle);
    delta = gazebo_RB_vel * (current_time - last_time).toSec();
    wheelpos[3][0] += delta * cos(theta + gazebo_RB_angle);
    wheelpos[3][1] += delta * sin(theta + gazebo_RB_angle);
    last_time = current_time;
}

void Swerve_Odom_Publisher::CalcRobotCenter()
{
    double center_x = 0;
    double center_y = 0;
    for (int i = 0; i < 4; i++)
    {
        center_x += wheelpos[i][0] / 4.0f;
        center_y += wheelpos[i][1] / 4.0f;
    }
    center_xy[0] = center_x;
    center_xy[1] = center_y;
}

void Swerve_Odom_Publisher::update()
{ // if use topic communication
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        if(gazebo_mode_){
            processing_gazebo_data();
        }

        CalcRobotCenter();
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        ros::Time current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = base_frame_id_;

        odom_trans.transform.translation.x = center_xy[0];
        odom_trans.transform.translation.y = center_xy[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = center_xy[0];
        odom.pose.pose.position.y = center_xy[1];
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        static ros::Time last_time = ros::Time::now();
        current_time = ros::Time::now();
        double delta_t = (current_time - last_time).toSec();
        odom.child_frame_id = base_frame_id_;
        double vx, vy, omega;
        vx = (center_xy[0] - old_center_xy[0]) / delta_t;
        vy = (center_xy[1] - old_center_xy[1]) / delta_t;
        omega = (theta - old_theta) / delta_t;

        odom.twist.twist.linear.x = cos(theta) * vx + sin(theta) * vy;
        odom.twist.twist.linear.y = -sin(theta) * vx + cos(theta) * vy;
        odom.twist.twist.angular.z = omega;
        last_time = current_time;

        //ROS_INFO("%f, %f, %f", odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z);

        odom.pose.covariance[0] = 0.001;
        odom.pose.covariance[7] = 0.001;
        odom.pose.covariance[14] = 1000000000000.0;
        odom.pose.covariance[21] = 1000000000000.0;
        odom.pose.covariance[28] = 1000000000000.0;

        if (std::fabs(odom.twist.twist.angular.z) < 0.0001)
        {
            odom.pose.covariance[35] = 0.01;
        }
        else
        {
            odom.pose.covariance[35] = 100.0;
        }

        odom.twist.covariance[0] = 0.001;
        odom.twist.covariance[7] = 0.001;
        odom.twist.covariance[14] = 0.001;
        odom.twist.covariance[21] = 1000000000000.0;
        odom.twist.covariance[28] = 1000000000000.0;

        if (std::fabs(odom.twist.twist.angular.z) < 0.0001)
        {
            odom.twist.covariance[35] = 0.01;
        }
        else
        {
            odom.twist.covariance[35] = 100.0;
        }

        //publish the message
        odom_pub.publish(odom);

        old_center_xy[0] = center_xy[0];
        old_center_xy[1] = center_xy[1];
        old_theta = theta;

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
    double body_width = 0.440;
    std::string base_frame_id = "base_link";
    float initial_table_angle = 0.0;
    bool gazebo_mode = false;

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("body_width", body_width);
    arg_n.getParam("base_frame_id", base_frame_id);
    arg_n.getParam("initial_table_angle", initial_table_angle);
    arg_n.getParam("gazebo_mode", gazebo_mode);

    Swerve_Odom_Publisher publisher(nh, looprate, body_width, base_frame_id, initial_table_angle, gazebo_mode);
    return 0;
}
