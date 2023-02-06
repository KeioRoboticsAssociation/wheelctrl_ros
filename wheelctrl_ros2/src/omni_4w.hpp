#ifndef _OMNI_4W_
#define _OMNI_4W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni4W : public Measuring {
 public:
  MeasureOmni4W(const U_PARAM &_u_param, const POS &_past_pos);
  ~MeasureOmni4W(){}
  void cal_disp(std::shared_ptr<float[]> encoder, float imu = 0,
                bool is_transformed = false);
};

class MoveOmni4W : public Moving {
 public:
  MoveOmni4W(const U_PARAM &_u_param);
  ~MoveOmni4W(){}
  void cal_cmd(const CMD &cmd,bool is_transformed = false);
};
}

#endif