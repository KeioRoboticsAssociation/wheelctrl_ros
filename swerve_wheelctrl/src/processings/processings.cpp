#include "processings.h"

namespace processings {
    void PI2INF(double &angle, const double &former_angle)
    {
        if (former_angle - angle > M_PI){
            while (abs(former_angle - angle) > M_PI)
            {
                angle += 2 * M_PI;
            }
        }
        else if (former_angle - angle < -1*M_PI)
        {
            while (abs(former_angle - angle) > M_PI)
            {
                angle -= 2 * M_PI;
            }
        }
    }

    void AdjustDirection(double &angle, const double &former_angle, int &speed_flag)
    {
        if (former_angle - angle > 1e-7f)
        {
            if (former_angle - angle < M_PI / 2.0f - 1e-7f)
            {
                speed_flag = 1;
            }
            else if (former_angle - angle > M_PI / 2.0f + 1e-7f)
            {
                angle += M_PI;
                speed_flag = -1;
            }
            else
            {
                speed_flag = 1;
            }
        }
        else if (former_angle - angle < -1.0 * 1e-7f)
        {
            if (former_angle - angle > -1.0 * M_PI / 2.0f + 1e-7f)
            {
                speed_flag = 1;
            }
            else if (former_angle - angle < -1.0 * M_PI / 2.0f - 1e-7f)
            {
                angle -= M_PI;
                speed_flag = -1;
            }
            else
            {
                speed_flag = 1;
            }
        }
        else
        {
            speed_flag = 1;
        }
    }

    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Passed by Reference
    }
}