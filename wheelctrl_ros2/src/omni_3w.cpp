#include "omni_3w.hpp"

illias::MeasureOmni3W::MeasureOmni3W(const U_PARAM &_u_param,const POS &_past_pos)
: Measuring(_u_param,_past_pos) {
  if (u_param.type_name=="omni"&&u_param.quantity==3){
    printf("this is the subclass of three-wheel measuring module\n");
  }else{
    printf("please use another subclass\n");
  }
}

illias::MoveOmni3W::MoveOmni3W(const U_PARAM &_u_param):Moving(_u_param){
  if (u_param.type_name == "omni" && u_param.quantity == 3) {
    printf("this is the subclass of three-wheel moving module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureOmni3W::cal_disp(std::shared_ptr<float[]> encoder,
                                     float imu = 0,
                                     bool is_transformed = false) {
  POS delta;

  delta.x = 0;
  delta.y = 0;
  delta.w = 0;

    float a0 = (float)this->u_param.wheels[0].argument;
    float a1 = (float)this->u_param.wheels[1].argument;
    float a2 = (float)this->u_param.wheels[2].argument;
    float v0 = this->rot_to_meter(encoder[0]);
    float v1 = this->rot_to_meter(encoder[1]);
    float v2 = this->rot_to_meter(encoder[2]);

    delta.x = -sin(a0) * v0 - sin(a1) * v1 - sin(a2) * v2;
    delta.y = cos(a0) * v0 + cos(a1) * v1 + cos(a2) * v2;
    delta.w = (v0 + v1 + v2) / (3 * u_param.wheels[0].distance);
  //ロボット座標系から現在の固定座標に変換
  this->current_pos.x = this->past_pos.x + cos(past_pos.w) * delta.x -
                        sin(past_pos.w) * delta.y;
  this->current_pos.y = this->past_pos.y + sin(past_pos.w) * delta.x +
                        cos(past_pos.w) * delta.y;
  this->current_pos.w = this->past_pos.w + delta.w;

  // past_posを更新
  past_pos = current_pos;

  // set current_vel
  this->current_vel.x = delta.x * u_param.loop_rate;
  this->current_vel.y = delta.y * u_param.loop_rate;
  this->current_vel.w = delta.w * u_param.loop_rate;
}

void illias::MoveOmni3W::cal_cmd(const CMD &cmd,bool is_transformed = false) {
  float a = (float)this->u_param.wheels[0].argument;
  float b = (float)this->u_param.wheels[1].argument;
  float c = (float)this->u_param.wheels[2].argument;

  wheel_cmd_meter[0] =
      0.5 * csc((a - b) / 2) * csc((a - c) / 2) * sin((b + c) / 2) * cmd.x -
      0.5 * csc((a - b) / 2) * csc((a - c) / 2) * cos((b + c) / 2) * cmd.y +
      1.5 * csc((a - b) / 2) * csc((a - c) / 2) * cos((b - c) / 2) * cmd.w *
          u_param.wheels[0].distance;
  wheel_cmd_meter[1] =
      -0.5 * csc((a - b) / 2) * csc((b - c) / 2) * sin((a + c) / 2) * cmd.x +
      0.5 * csc((a - b) / 2) * csc((b - c) / 2) * cos((a + c) / 2) * cmd.y -
      1.5 * csc((a - b) / 2) * csc((b - c) / 2) * cos((a - c) / 2) * cmd.w *
          u_param.wheels[1].distance;
  wheel_cmd_meter[2] =
      0.5 * csc((a - c) / 2) * csc((b - c) / 2) * sin((a + b) / 2) * cmd.x -
      0.5 * csc((a - c) / 2) * csc((b - c) / 2) * cos((a + b) / 2) * cmd.y +
      1.5 * csc((a - c) / 2) * csc((b - c) / 2) * cos((a - b) / 2) * cmd.w *
          u_param.wheels[2].distance;

  for (int i = 0; i < 3;i++){
    wheel_cmd_rot[i] = meter_to_rot(wheel_cmd_meter[i]);
  }
}