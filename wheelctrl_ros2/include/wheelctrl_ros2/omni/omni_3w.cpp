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

  if (length != 3) {
    std::printf("invalid argument size\n");
    delta.x = 0;
    delta.y = 0;
    delta.theta = 0;
  } else {
    float a0 = this->w_param.arguments[0];
    float a1 = this->w_param.arguments[1];
    float a2 = this->w_param.arguments[2];
    float v0 = this->rotate_to_meter(encoder[0]);
    float v1 = this->rotate_to_meter(encoder[1]);
    float v2 = this->rotate_to_meter(encoder[2]);

    delta.x = -sin(a0) * v0 - sin(a1) * v1 - sin(a2) * v2;
    delta.y = cos(a0) * v0 + cos(a1) * v1 + cos(a2) * v2;
    delta.theta = (v0 + v1 + v2) / (3 * w_param.distance);
  }
}

void illias::MoveOmni3W::cal_cmd(const CMD &cmd) {
  float a = this->w_param.arguments[0];
  float b = this->w_param.arguments[1];
  float c = this->w_param.arguments[2];

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