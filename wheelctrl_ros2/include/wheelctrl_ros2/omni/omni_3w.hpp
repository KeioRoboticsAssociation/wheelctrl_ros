#ifndef _OMNI_3W_
#define _OMNI_3W_

#include "wheel.hpp"

namespace illias{
class MeasureOmni3W : public Measuring {
 public:
  MeasureOmni3W(const W_PARAM &_w_param, const POS &_past_pos);
  void cal_disp(const float encoder[], const int &length);

};

class MoveOmni3W : public Moving {
 public:
  MoveOmni3W(const W_PARAM &_w_param);
  void cal_cmd(const CMD &cmd);
};
}

#endif