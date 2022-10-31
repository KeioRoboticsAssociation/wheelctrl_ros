#ifndef _WHEEL_
#define _WHEEL_

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

  std::vector<std::string> topic_name;
} W_PARAM, *P_W_PARAM;

typedef struct ODOMETRY
{
  float x;
  float y;
  float theta;
} ODOM, *P_ODOM;

typedef struct COMMAND
{
  float x;
  float y;
  float theta;
} CMD, *P_CMD;

class Wheel {
 public:
  ~Wheel();
  ODOM cal_odom_pos(float encoder[], size_t size);
  ODOM cal_odom_vel(float encoder[], size_t size);


 private:
  W_PARAM wheel_param;
  
};

#endif