#ifndef _OMNI_2W_
#define _OMNI_2W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni2W : public Measuring {
 public:
  MeasureOmni2W(const W_PARAM &_w_param, const POS &_past_pos);

  // convert encoder data into posture of robot (/odom)
  void cal_disp(const float encoder[], const int &length, const float &imu);
};
}

#endif