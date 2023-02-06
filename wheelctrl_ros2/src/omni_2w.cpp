#include "omni_2w.hpp"

illias::MeasureOmni2W::MeasureOmni2W(const U_PARAM &_u_param,
                                     const POS &_initial_pos)
    : illias::Measuring(_u_param,_initial_pos) {
  if (u_param.type_name == "omni" && u_param.quantity == 2) {
    printf("this is the subclass of two-wheel measuring module\n");
  } else {
    printf("you should use another subclass\n");
  }
}

void illias::MeasureOmni2W::cal_disp(std::shared_ptr<float[]> encoder,
                                     const float imu = 0,
                                     bool is_transformed = false) {
  POS delta;

  delta.x = this->rot_to_meter(encoder[0]) - imu * u_param.wheels[0].distance;
  delta.y = this->rot_to_meter(encoder[1]) - imu * u_param.wheels[1].distance;
  delta.w = imu;

  // ロボット座標系から現在の固定座標に変換
  this->current_pos.x = this->past_pos.x + cos(past_pos.w) * delta.x -
                        sin(past_pos.w) * delta.y;
  this->current_pos.y = this->past_pos.y + sin(past_pos.w) * delta.x +
                        cos(past_pos.w) * delta.y;
  this->current_pos.w = this->past_pos.w + delta.w;

  // past_posを更新
  this->past_pos = this->current_pos;

  // set current_vel
  this->current_vel = delta * this->u_param.loop_rate;
}