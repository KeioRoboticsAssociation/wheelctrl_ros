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
    printf("this is the subclass of the steering moving module\n");
  } else {
    printf("please use another subclass\n");
  }
}

void illias::MeasureSteering::cal_disp(std::shared_ptr<float[]> encoder,
                                       const int &length) {
  POS delta;
  delta.x = 0;
  delta.y = 0;
  delta.theta = 0;
  if (length != 8) {
    std::printf("invalid argument size");
  } else {
    float table_arg = atan2(w_param.length_y, w_param.length_x);
    for (int i = 0; i < 4;i++)
    {
      delta.x += 0.25 * this->rotate_to_meter(encoder[i]) * cos(encoder[i + 4]);
      delta.y += 0.25 * this->rotate_to_meter(encoder[i]) * sin(encoder[i + 4]);
    }
    delta.theta = this->rotate_to_meter(encoder[0])*sin(encoder[4]-table_arg)-
                  this->rotate_to_meter(encoder[1])*sin(encoder[5]+table_arg)-
                  this->rotate_to_meter(encoder[2])*sin(encoder[6]-table_arg)+
                  this->rotate_to_meter(encoder[3])*sin(encoder[7]+table_arg);
    delta.theta /= (4*sqrt(pow(w_param.length_x/2,2)+pow(w_param.length_y/2,2)));
  }

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

void illias::MoveSteering::cal_cmd(const CMD &cmd,const float &table_angle) {
  float vx = 0;
  float vy = 0;
  float dist = sqrt(pow(w_param.length_x, 2) + pow(w_param.length_y, 2));
  float table_arg = atan2(w_param.length_y, w_param.length_x);

  vx = cmd.x + dist * cmd.theta * cos(M_PI / 2 + table_arg);
  vy = cmd.y + dist * cmd.theta * sin(M_PI / 2 + table_arg);
  wheel_cmd_meter[0] = cal_r(vx, vy);
  wheel_cmd_meter[4] = atan2(vy, vx);

  vx = cmd.x + dist * cmd.theta * cos(M_PI * 3 / 4 - table_arg);
  vy = cmd.y + dist * cmd.theta * sin(M_PI * 3 / 4 - table_arg);
  wheel_cmd_meter[1] = cal_r(vx, vy);
  wheel_cmd_meter[5] = atan2(vy, vx);

  vx = cmd.x + dist * cmd.theta * cos(M_PI * 3 / 4 + table_arg);
  vy = cmd.y + dist * cmd.theta * sin(M_PI * 3 / 4 + table_arg);
  wheel_cmd_meter[2] = cal_r(vx, vy);
  wheel_cmd_meter[6] = atan2(vy, vx);

  vx = cmd.x + dist * cmd.theta * cos(M_PI / 2 - table_arg);
  vy = cmd.y + dist * cmd.theta * sin(M_PI / 2 - table_arg);
  wheel_cmd_meter[3] = cal_r(vx, vy);
  wheel_cmd_meter[7] = atan2(vy, vx);

  for(int i = 0; i < 8;i++){
    if(i<4){
      wheel_cmd_rotate[i] = meter_to_rotate(wheel_cmd_meter[i]);
    }else{
      wheel_cmd_rotate[i] = rad_to_rotate(wheel_cmd_meter[i]*w_param.gear_ratio_horizonal-table_angle);
    }
  } 
}