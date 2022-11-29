#ifndef _STEERING_
#define _STEERING_

#include "wheel.hpp"

namespace illias{
class MeasureSteering : public Measuring {
 public:
  MeasureSteering(const W_PARAM &_w_param,const POS &_past_pos);
  void cal_disp(const float encoder[], const int &length);
};

class MoveSteering : public Moving {
 public:
  MoveSteering(const W_PARAM &_w_param);
  virtual void cal_cmd(const CMD &cmd, const float &table_angle);
};
}

#endif