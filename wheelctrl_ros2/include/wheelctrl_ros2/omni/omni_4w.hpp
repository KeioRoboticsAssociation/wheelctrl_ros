#ifndef _OMNI_4W_
#define _OMNI_4W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni4W : public Measuring {
 public:
  MeasureOmni4W(const W_PARAM &_w_param, const POS &_past_pos);
  void cal_disp(const float encoder[], const int &length);
};

class MoveOmni4W : public Moving {
 public:
  MoveOmni4W(const W_PARAM &_w_param);
  void cal_cmd(const CMD &cmd);
};
}

#endif