#ifndef _OMNI_3W_
#define _OMNI_3W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni3W : public Measuring {
 public:
  MeasureOmni3W(const U_PARAM &_u_param, const POS &_past_pos);
  ~MeasureOmni3W(){}
  void cal_disp(std::shared_ptr<float[]> encoder, const float imu = 0,
                bool is_transformed = false);
};

class MoveOmni3W : public Moving {
 public:
  MoveOmni3W(const U_PARAM &_u_param);
  ~MoveOmni3W(){}
  void cal_cmd(const CMD &cmd, bool is_transformed = false);
};
}
#endif