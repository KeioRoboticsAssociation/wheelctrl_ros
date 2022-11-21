#include "omni_2W.hpp"

MeasureOmni2W::MeasureOmni2W(const W_PARAM &_w_param) : w_param(_w_param) {}

void MeasureOmni2W::cal_disp(const float encoder[], size_t mysize,
                             const float &imu, ODOM &dist_r) {
  if (mysize != 2) {
    std::printf("invalid argument size");
    dist_r.x = 0;
    dist_r.y = 0;
    dist_r.theta = 0;
  } else {
    dist_r.theta = imu;
    dist_r.x = encoder[0];
    dist_r.y = encoder[1];
  }
}