#ifndef _MECHANAM_
#define _MECHANAM_

#include "wheel.hpp"

class MeasureMechanam : public Measuring {
 public:
  MeasureMechanam(const W_PARAM &_w_param);
  void cal_disp(const float encoder[], size_t mysize, ODOM &dist_r);
  void cal_disp(const float encoder[], size_t mysize, const float &imu,
                ODOM &dist_r);

 private:
  W_PARAM w_param;
};

class MoveMechanam : public Moving {
 public:
  MoveMechanam(const W_PARAM &_w_param);
  void cal_cmd(const ODOM &cmd_r, float wheel_cmd[], size_t mysize);

 private:
  W_PARAM w_param;
};

#endif