#ifndef _WHEEL_
#define _WHEEL_
#include "rclcpp/rclcpp.hpp"
#include <math.hpp>
#include <string>
#include <vector>

typedef struct WHEEL_PARAMETER {
  std::string type_name;
  float radius;
  float quantity;
  float gear_ratio;
  float gear_ratio_horizonal;

  std::string coordinate;
  float distance;
  std::vector<double> arguments;

  float length_x;
  float length_y;
} W_PARAM, *P_W_PARAM;

typedef struct ODOMETRY{
  float x;
  float y;
  float theta;
} ODOM, *P_ODOM;

// encoder は回転数でくるよ
class Measuring {
  public:
   virtual ~Measuring();
   virtual void cal_disp();
};

class Moving {
  public:
   virtual ~Moving();
   virtual void cal_cmd();
};

#endif