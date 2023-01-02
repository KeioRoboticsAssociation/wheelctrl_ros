#ifndef _OMNI_2W_
#define _OMNI_2W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni2W : public Measuring {
 public:
  MeasureOmni2W(const W_PARAM &_w_param, const POS &_past_pos);
  ~MeasureOmni2W(){}
  // convert encoder data into posture of robot (/odom)
  void cal_disp(std::shared_ptr<float[]> encoder, const int &length) {
    for (int i = 0; i < length; i++) {
      printf("%f,", encoder[i]);
    }
  }
  void cal_disp(std::shared_ptr<float[]> encoder, const int &length,
                const float &imu);
};
}

#endif