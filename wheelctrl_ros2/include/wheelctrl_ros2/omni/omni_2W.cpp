#include "omni_2W.hpp"

illias::MeasureOmni2W::MeasureOmni2W(const W_PARAM &_w_param,
                                     const POS &_past_pos)
    : illias::Measuring(_w_param,_past_pos) {
      if(w_param.type_name=="omni"&&w_param.quantity==2){
        printf("this is the subclass of two-wheel measuring module\n");
      }else{
        printf("you should use another subclass\n");
      }
    }

void illias::MeasureOmni2W::cal_disp(const float encoder[], const int &length,
                                     const float &imu) {
  POS delta;
  
  // エンコーダーの値を代入
  if (length != 2) {
    std::printf("invalid argument size");
    delta.x = 0;
    delta.y = 0;
    delta.theta = 0;
  } else {
    delta.x = this->rotate_to_meter(encoder[0])-imu*w_param.distance;
    delta.y = this->rotate_to_meter(encoder[1])-imu*w_param.distance;
    delta.theta = imu;
  }

  //ロボット座標系から現在の固定座標に変換
  this->current_pos.x = this->past_pos.x + cos(past_pos.theta) * delta.x -
                        sin(past_pos.theta) * delta.y;
  this->current_pos.y = this->past_pos.y + sin(past_pos.theta) * delta.x +
                        cos(past_pos.theta) * delta.y;
  this->current_pos.theta = change_range(this->past_pos.theta + delta.theta);

  //past_posを更新
  past_pos = current_pos;
}