#ifndef _STEERING_
#define _STEERING_

#include "wheel.hpp"

namespace illias{
class MeasureSteering : public Measuring {
 public:
  MeasureSteering(const U_PARAM &_u_param,const POS &_past_pos);
  ~MeasureSteering(){}
  void cal_disp(std::vector<float> encoder, float imu = 0 ,bool is_transformed = false);
  void set_initial_wheel_angle(float w0,float w1,float w2,float w3);

 private:
  float past_theta[4];
};

class MoveSteering : public Moving {
 public:
  MoveSteering(const U_PARAM &_u_param);
  ~MoveSteering() {}
  void cal_cmd(const CMD &cmd,bool is_transformed = false);

 private:
};
}

#endif