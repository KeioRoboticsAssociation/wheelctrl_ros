#include <ros/ros.h>
#include "cmdvel2mecanum.h"

std::string node_name = "mecanum_wheelctrl";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /**
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    **/

    int loop_rate = 30;
    float body_height = 0.440;
    float body_width = 0.440;
    int lost_time_threshold = 500;
    bool gazebo_mode = false;

    pnh.getParam("control_frequency", loop_rate);
    pnh.getParam("lost_time_threshold", lost_time_threshold);
    pnh.getParam("body_height", body_height);
    pnh.getParam("body_width", body_width);
    pnh.getParam("gazebo_mode", gazebo_mode);

    VelConverter converter(nh, body_height, body_width, lost_time_threshold, loop_rate, gazebo_mode);
    return 0;
}
