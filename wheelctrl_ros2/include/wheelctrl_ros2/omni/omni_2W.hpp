#ifndef _OMNI_2W_
#define _OMNI_2W_

#include "wheel.hpp"

class MeasureOmni2W : public Measuring {
 public:
  MeasureOmni2W(const W_PARAM &_w_param);

  void cal_disp(const float encoder[], size_t mysize, const float &imu,
                ODOM &dist_r);

 private:
  W_PARAM w_param;
};

#endif