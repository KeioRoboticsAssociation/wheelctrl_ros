#ifndef _STEERING_
#define _STEERING_

#include <string.h>

#include "wheel.hpp"

namespace illias {
class MeasureSteering : public Measuring {
 public:
  MeasureSteering(const U_PARAM &_u_param, const POS &_past_pos);
  ~MeasureSteering() {}
  void cal_disp(std::vector<float> encoder, float imu = 0,
                bool is_transformed = false);
  void set_initial_wheel_angle(float w0, float w1, float w2, float w3);

 private:
  float past_theta[4];
};

class MoveSteering : public Moving {
 public:
  MoveSteering(const U_PARAM &_u_param);
  ~MoveSteering() {}
  void cal_cmd(const CMD &cmd, bool is_transformed = false);
  inline void set_current_wheel_angle(float w0, float w1, float w2, float w3) {
    current_wheel_angle[0] = w0;
    current_wheel_angle[1] = w1;
    current_wheel_angle[2] = w2;
    current_wheel_angle[3] = w3;
    // memset(vel_sign, 1, sizeof(vel_sign));
  }

 private:
  float current_wheel_angle[4];
  // float vel_sign[4] = {1.0, 1.0, 1.0, 1.0};
};
}  // namespace illias

#endif