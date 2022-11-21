#ifndef _STEERING_
#define _STEERING_

#include "wheel.hpp"

class MeasureSteering : public Measuring {
 public:
  MeasureSteering(const W_PARAM &_w_param);
  void cal_disp(const float encoder[], size_t mysize, ODOM &dist_r);
  void cal_disp(const float encoder[], size_t mysize, const float &imu,
                ODOM &dist_r);

 private:
  W_PARAM w_param;
};

class MoveSteering : public Moving {
 public:
  MoveSteering(const W_PARAM &_w_param);
  void cal_cmd(float target_angle,const ODOM &cmd_r,float wheel_cmd[], size_t mysize);

 private:
  W_PARAM w_param;
};

#endif