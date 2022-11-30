#ifndef _STEERING_
#define _STEERING_

#include "wheel.hpp"

namespace illias{
class MeasureSteering : public Measuring {
 public:
  MeasureSteering(const W_PARAM &_w_param,const POS &_past_pos);
  ~MeasureSteering(){}
  void cal_disp(const float encoder[], const int &length);
  void cal_disp(const float encoder[], const int &length, const float &imu) {
    for (int i = 0; i < length; i++) {
      printf("%f,", encoder[i]);
    }
    printf("%f\n", imu);
  }
};

class MoveSteering : public Moving {
 public:
  MoveSteering(const W_PARAM &_w_param);
  ~MoveSteering(){}
  void cal_cmd(const CMD &cmd) { printf("%f,%f,%f", cmd.x, cmd.y, cmd.theta); }
  void cal_cmd(const CMD &cmd, const float curvature) {
    printf("%f,%f,%f,%f", cmd.x, cmd.y, cmd.theta,curvature);
  }
  virtual void cal_cmd(const CMD &cmd, const float &table_angle);
};
}

#endif