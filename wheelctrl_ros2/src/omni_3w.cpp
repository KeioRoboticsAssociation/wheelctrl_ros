#include "omni_3w.hpp"

illias::MeasureOmni3W::MeasureOmni3W(const W_PARAM &_w_param,const POS &_past_pos)
: Measuring(_w_param,_past_pos) {
  if (w_param.type_name=="omni"&&w_param.quantity==3){
    printf("this is the subclass of three-wheel measuring module\n");
  }else{
    printf("please use another subclass\n");
  }
}

illias::MoveOmni3W::MoveOmni3W(const W_PARAM &_w_param):Moving(_w_param){
  if (w_param.type_name == "omni" && w_param.quantity == 3) {
    printf("this is the subclass of three-wheel moving module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureOmni3W::cal_disp(const float encoder[], const int &length) {
  POS delta;

  delta.x = 0;
  delta.y = 0;
  delta.theta = 0;

  if (length != 3) {
    std::printf("invalid argument size\n");
  } else {
    float a0 = (float)this->w_param.arguments[0];
    float a1 = (float)this->w_param.arguments[1];
    float a2 = (float)this->w_param.arguments[2];
    float v0 = this->rotate_to_meter(encoder[0]);
    float v1 = this->rotate_to_meter(encoder[1]);
    float v2 = this->rotate_to_meter(encoder[2]);

    delta.x = -sin(a0) * v0 - sin(a1) * v1 - sin(a2) * v2;
    delta.y = cos(a0) * v0 + cos(a1) * v1 + cos(a2) * v2;
    delta.theta = (v0 + v1 + v2) / (3 * w_param.distance);
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

void illias::MoveOmni3W::cal_cmd(const CMD &cmd) {
  float a = (float)this->w_param.arguments[0];
  float b = (float)this->w_param.arguments[1];
  float c = (float)this->w_param.arguments[2];

  wheel_cmd_meter[0] =
      0.5 * csc((a - b) / 2) * csc((a - c) / 2) * sin((b + c) / 2) * cmd.x -
      0.5 * csc((a - b) / 2) * csc((a - c) / 2) * cos((b + c) / 2) * cmd.y +
      1.5 * csc((a - b) / 2) * csc((a - c) / 2) * cos((b - c) / 2) * cmd.theta *
          w_param.distance;
  wheel_cmd_meter[1] =
      -0.5 * csc((a - b) / 2) * csc((b - c) / 2) * sin((a + c) / 2) * cmd.x +
      0.5 * csc((a - b) / 2) * csc((b - c) / 2) * cos((a + c) / 2) * cmd.y -
      1.5 * csc((a - b) / 2) * csc((b - c) / 2) * cos((a - c) / 2) * cmd.theta *
          w_param.distance;
  wheel_cmd_meter[2] =
      0.5 * csc((a - c) / 2) * csc((b - c) / 2) * sin((a + b) / 2) * cmd.x -
      0.5 * csc((a - c) / 2) * csc((b - c) / 2) * cos((a + b) / 2) * cmd.y +
      1.5 * csc((a - c) / 2) * csc((b - c) / 2) * cos((a - b) / 2) * cmd.theta *
          w_param.distance;

  for (int i = 0; i < 3;i++){
    wheel_cmd_rotate[i] = meter_to_rotate(wheel_cmd_meter[i]);
  }
}