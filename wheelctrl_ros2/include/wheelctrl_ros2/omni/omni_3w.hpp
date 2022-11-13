#ifndef _OMNI_3W_
#define _OMNI_3W_

#include "wheel.hpp"

class MeasureOmni3W : public Measuring {
 public:
  MeasureOmni3W(const W_PARAM &_w_param);
  void cal_disp(const float encoder[], size_t mysize, ODOM &dist_r);
  void cal_disp(const float encoder[], size_t mysize, const float &imu,
                ODOM &dist_r);

 private:
  W_PARAM w_param;
};

class MoveOmni3W : public Moving {
 public:
  MoveOmni3W(const W_PARAM &_w_param);
  void cal_cmd(const ODOM &cmd_r, float wheel_cmd[], size_t mysize);

 private:
  W_PARAM w_param;
};

#endif