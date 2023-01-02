#ifndef _OMNI_3W_
#define _OMNI_3W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni3W : public Measuring {
 public:
  MeasureOmni3W(const W_PARAM &_w_param, const POS &_past_pos);
  ~MeasureOmni3W(){}
  void cal_disp(std::shared_ptr<float[]> encoder, const int &length);
  void cal_disp(std::shared_ptr<float[]> encoder, const int &length,
                const float &imu) {
    for (int i = 0; i < length; i++) {
      printf("%f,", encoder[i]);
    }
    printf("%f\n", imu);
  }
};

class MoveOmni3W : public Moving {
 public:
  MoveOmni3W(const W_PARAM &_w_param);
  ~MoveOmni3W(){}
  void cal_cmd(const CMD &cmd);
  void cal_cmd(const CMD &cmd, const float &table_angle,const float curvature) {
    printf("%f,%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, table_angle, curvature);
  }
  void cal_cmd(const CMD &cmd, const float &table_angle) {
    printf("%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, table_angle);
  }
};
}
#endif