#include "omni_4w.hpp"

illias::MeasureOmni4W::MeasureOmni4W(const W_PARAM &_w_param,const POS &_past_pos)
:Measuring(_w_param,_past_pos){
  if(w_param.type_name == "omni"&&w_param.quantity==4){
    printf("this is the subclass of the four-wheel measuring module\n");
  }else{
    printf("please use another subclass\n");
  }
}

illias::MoveOmni4W::MoveOmni4W(const W_PARAM &_w_param)
:Moving(_w_param){
  if (w_param.type_name == "omni" && w_param.quantity == 4) {
    printf("this is the subclass of the four-wheel moving module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureOmni4W::cal_disp(std::shared_ptr<float[]> encoder,
                                     const int &length) {
  POS delta;
  // エンコーダーの値を代入
  delta.x = 0;
  delta.y = 0;
  delta.theta = 0;
  if (length != 4) {
    std::printf("invalid argument size\n");
  } else {
    float v0 = rotate_to_meter(encoder[0]);
    float v1 = rotate_to_meter(encoder[1]);
    float v2 = rotate_to_meter(encoder[2]);
    float v3 = rotate_to_meter(encoder[3]);

    delta.x = (-v0 - v1 + v2 + v3) / (4*sqrt(2));
    delta.y = (v0 - v1 - v2 + v3) / (4 * sqrt(2));
    delta.theta = (v0 + v1 + v2 + v3) / 4;
  }

  //ロボット座標系から現在の固定座標に変換
  this->current_pos.x = this->past_pos.x + cos(past_pos.theta) * delta.x -
                        sin(past_pos.theta) * delta.y;
  this->current_pos.y = this->past_pos.y + sin(past_pos.theta) * delta.x +
                        cos(past_pos.theta) * delta.y;
  this->current_pos.theta = change_range(this->past_pos.theta + delta.theta);

  // past_posを更新
  past_pos = current_pos;

  // set current_vel
  this->current_vel.x = delta.x * w_param.loop_rate;
  this->current_vel.y = delta.y * w_param.loop_rate;
  this->current_vel.theta = delta.theta * w_param.loop_rate;
}

void illias::MoveOmni4W::cal_cmd(const CMD &cmd) {
  float arg = 1 / sqrt(2);
  wheel_cmd_meter[0] = arg * (-cmd.x + cmd.y) + w_param.distance * cmd.theta;
  wheel_cmd_meter[1] = arg * (-cmd.x - cmd.y) + w_param.distance * cmd.theta;
  wheel_cmd_meter[2] = arg * (cmd.x - cmd.y) + w_param.distance * cmd.theta;
  wheel_cmd_meter[3] = arg * (cmd.x + cmd.y) + w_param.distance * cmd.theta;

  for (int i = 0; i < 4;i++){
    wheel_cmd_rotate[i] = meter_to_rotate(wheel_cmd_meter[i]);
  }
}