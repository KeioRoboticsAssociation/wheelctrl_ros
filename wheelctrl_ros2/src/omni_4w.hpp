#ifndef _OMNI_4W_
#define _OMNI_4W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni4W : public Measuring {
 public:
  MeasureOmni4W(const W_PARAM &_w_param, const POS &_past_pos);
  ~MeasureOmni4W(){}
  void cal_disp(const float encoder[], const int &length);
  void cal_disp(const float encoder[], const int &length, const float &imu) {
    for (int i = 0; i < length; i++) {
      printf("%f,", encoder[i]);
    }
    printf("%f\n", imu);
  }
};

class MoveOmni4W : public Moving {
 public:
  MoveOmni4W(const W_PARAM &_w_param);
  ~MoveOmni4W(){}
  void cal_cmd(const CMD &cmd);
  void cal_cmd(const CMD &cmd, const float curvature) {
    printf("%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, curvature);
  }
  void cal_cmd(const CMD &cmd, const float &table_angle) {
    printf("%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, table_angle);
  }
};
}

#endif