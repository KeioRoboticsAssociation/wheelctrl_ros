#ifndef PROCESSINGS_H_
#define PROCESSINGS_H_

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>


namespace processings {
    void PI2INF(double &, const double &);
    void AdjustDirection(double &angle, const double &former_angle, int &speed_flag);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
} // namespace processings

#endif
