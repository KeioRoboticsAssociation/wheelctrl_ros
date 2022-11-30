#include "steering.hpp"

illias::MeasureSteering::MeasureSteering(const W_PARAM &_w_param,
                                                        const POS &_past_pos)
    : Measuring(_w_param, _past_pos) {
      if(w_param.type_name=="steering"){
        printf("this is the subclass of the steering measure module\n");
      }else{
        printf("please use another subclass\n");
      }
    }

illias::MoveSteering::MoveSteering(const W_PARAM &_w_param) : Moving(_w_param) {
  if (w_param.type_name == "steering") {
    printf("this is the subclass of the steering measure module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureSteering::cal_disp(const float encoder[],const int &length) {
  POS delta;
  if (length != 8) {
    std::printf("invalid argument size");
    delta.x = 0;
    delta.y = 0;
    delta.theta = 0;
  } else {
    float a = (float)w_param.arguments[0];
    float b = (float)w_param.arguments[1];
    float c = (float)w_param.arguments[2];
    float n = 1 / (cos(c) * (sin(b) - sin(a)) + cos(b) * (sin(a) - sin(c)) +
                   cos(a) * (sin(c) - sin(b)));
    delta.x =
        n * (encoder[0] * (cos(c) - cos(b)) +
             encoder[1] * (cos(a) - cos(c) + encoder[2] * (cos(b) - cos(a))));
    delta.y =
        n * (encoder[0] * (sin(c) - sin(b)) +
             encoder[1] * (sin(a) - sin(c) + encoder[2] * (sin(b) - sin(a))));
    delta.theta = (-n * w_param.distance) *
                   (encoder[0] * (cos(b) * sin(c) - sin(b) * cos(c)) +
                    encoder[1] * (cos(c) * sin(a) - sin(c) * cos(a)) +
                    encoder[2] * (cos(a) * sin(b) - sin(a) * cos(b)));
  }
  //ロボット座標系から現在の固定座標に変換
  this->current_pos.x = this->past_pos.x + cos(past_pos.theta) * delta.x -
                        sin(past_pos.theta) * delta.y;
  this->current_pos.y = this->past_pos.y + sin(past_pos.theta) * delta.x +
                        cos(past_pos.theta) * delta.y;
  this->current_pos.theta = change_range(this->past_pos.theta + delta.theta);

  // past_posを更新
  past_pos = current_pos;
}

void illias::MoveSteering::cal_cmd(const CMD &cmd,const float &table_angle) {
  wheel_cmd_meter[0] =
      sqrt(pow(cmd.x - w_param.distance * cmd.theta * sin(table_angle), 2) +
           pow(cmd.y + w_param.distance * cmd.theta * sin(table_angle), 2));
  wheel_cmd_meter[1] =
      sqrt(pow(cmd.x - w_param.distance * cmd.theta * cos(table_angle), 2) +
           pow(cmd.y - w_param.distance * cmd.theta * sin(table_angle), 2));
  wheel_cmd_meter[2] =
      sqrt(pow(cmd.x + w_param.distance * cmd.theta * sin(table_angle), 2) +
           pow(cmd.y - w_param.distance * cmd.theta * cos(table_angle), 2));
  wheel_cmd_meter[3] =
      sqrt(pow(cmd.x + w_param.distance * cmd.theta * cos(table_angle), 2) +
           pow(cmd.y + w_param.distance * cmd.theta * sin(table_angle), 2));
  wheel_cmd_meter[4] =
      atan2(cmd.y + w_param.distance * cmd.theta * sin(table_angle),
            cmd.x - w_param.distance * cmd.theta * sin(table_angle));
  wheel_cmd_meter[5] =
      atan2(cmd.y - w_param.distance * cmd.theta * sin(table_angle),
            cmd.x - w_param.distance * cmd.theta * cos(table_angle));
  wheel_cmd_meter[6] =
      atan2(cmd.y - w_param.distance * cmd.theta * cos(table_angle),
            cmd.x + w_param.distance * cmd.theta * sin(table_angle));
  wheel_cmd_meter[7] =
      atan2(cmd.y + w_param.distance * cmd.theta * sin(table_angle),
            cmd.x + w_param.distance * cmd.theta * cos(table_angle));

  for(int i = 0; i < 8;i++){
    if(i<4){
      wheel_cmd_rotate[i] = meter_to_rotate(wheel_cmd_meter[i]);
    }else{
      wheel_cmd_meter[i] = wheel_cmd_meter[i] * w_param.gear_ratio_horizonal;
    }
  } 
}