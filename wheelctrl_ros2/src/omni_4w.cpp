#include "omni_4w.hpp"

illias::MeasureOmni4W::MeasureOmni4W(const U_PARAM &_u_param,const POS &_past_pos)
:Measuring(_u_param,_past_pos){
  if(u_param.type_name == "omni"&&u_param.quantity==4){
    printf("this is the subclass of the four-wheel measuring module\n");
  }else{
    printf("please use another subclass\n");
  }
}

illias::MoveOmni4W::MoveOmni4W(const U_PARAM &_u_param)
:Moving(_u_param){
  if (u_param.type_name == "omni" && u_param.quantity == 4) {
    printf("this is the subclass of the four-wheel moving module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureOmni4W::cal_disp(std::vector<float> encoder,
                                     float imu, bool is_transformed)
{
  POS delta = {0};
  // エンコーダーの値を代入
    float v0 = rot_to_meter(encoder[0]);
    float v1 = rot_to_meter(encoder[1]);
    float v2 = rot_to_meter(encoder[2]);
    float v3 = rot_to_meter(encoder[3]);

    delta.x = (-v0 - v1 + v2 + v3) / (4 * sqrt(2));
    delta.y = (v0 - v1 - v2 + v3) / (4 * sqrt(2));
    delta.w = (v0 + v1 + v2 + v3) / (4 * u_param.wheels[0].distance);

    // ロボット座標系から現在の固定座標に変換
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

void illias::MoveOmni4W::cal_cmd(const CMD &cmd,bool is_transformed) {
  float arg = 1 / sqrt(2);
  wheel_cmd_meter[0] =
      arg * (-cmd.x + cmd.y) + u_param.wheels[0].distance * cmd.w;
  wheel_cmd_meter[1] =
      arg * (-cmd.x - cmd.y) + u_param.wheels[1].distance * cmd.w;
  wheel_cmd_meter[2] =
      arg * (cmd.x - cmd.y) + u_param.wheels[2].distance * cmd.w;
  wheel_cmd_meter[3] =
      arg * (cmd.x + cmd.y) + u_param.wheels[3].distance * cmd.w;

  for (int i = 0; i < 4;i++){
    wheel_cmd_rot[i] = meter_to_rot(wheel_cmd_meter[i]);
  }
}