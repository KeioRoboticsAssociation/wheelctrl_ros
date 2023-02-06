#ifndef _OMNI_2W_
#define _OMNI_2W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni2W : public Measuring {
 public:
  MeasureOmni2W(const U_PARAM &_u_param, const POS &_past_pos);
  ~MeasureOmni2W(){}
  // convert encoder data into posture of robot (/odom)
  void cal_disp(std::shared_ptr<float[]> encoder, const float imu = 0,
                bool is_transformed = false);
};
}

#endif