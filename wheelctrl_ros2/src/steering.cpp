#include "steering.hpp"

illias::MeasureSteering::MeasureSteering(const U_PARAM &_u_param,
                                         const POS &_past_pos)
    : Measuring(_u_param, _past_pos) {
  if (u_param.type_name == "steering" && u_param.quantity == 4) {
    printf("this is a steering wheel package");
  } else {
    printf("you shouled use another subclass");
  }
}

illias::MoveSteering::MoveSteering(const U_PARAM &_u_param) : Moving(_u_param) {
  if (u_param.type_name == "steering" && u_param.quantity == 4) {
    printf("this is a steering wheel package");
  } else {
    printf("you shouled use another subclass");
  }
}

void illias::MeasureSteering::set_initial_wheel_angle(float w0, float w1,
                                                      float w2, float w3) {
  past_theta[0] = w0;
  past_theta[1] = w1;
  past_theta[2] = w2;
  past_theta[3] = w3;
}

void illias::MeasureSteering::cal_disp(std::shared_ptr<float[]> encoder,
                                       float imu, bool is_transformed) {
  POS delta = {0};

  float vel[4] = {0};
  float theta[4] = {0};
  float r_x[4] = {0};
  float r_y[4] = {0};

  // 各車輪の（理想的な）変位を計算
  for (int i = 0; i < 4; i++) {
    vel[i] = this->rot_to_meter(encoder[i]);
    theta[i] = this->rot_to_rad(encoder[i + 4]);
    float A = theta[i] - past_theta[i] == 0
                  ? 1
                  : sin(theta[i] - past_theta[i]) / (theta[i] - past_theta[i]);
    float B =
        theta[i] - past_theta[i] == 0
            ? 0
            : (1 - cos(theta[i] - past_theta[i])) / (theta[i] - past_theta[i]);
    r_x[i] = vel[i] * (cos(past_theta[i]) * A - sin(past_theta[i]) * B);
    r_y[i] = vel[i] * (sin(past_theta[i]) * A + cos(past_theta[i]) * B);
  }

  delta.x = (r_x[0] + r_x[1] + r_x[2] + r_x[3]) / 4;
  delta.y = (r_y[0] + r_y[1] + r_y[2] + r_y[3]) / 4;

  if (!is_transformed) {
    for (int i = 0; i < 4; i++) {
      delta.w = 0.25 * atan2(r_x[i], r_y[i]) / u_param.wheels[i].distance;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      delta.w = 0.25 * atan2(r_x[i], r_y[i]) / u_param.wheels[i + 4].distance;
    }
  }

  for (int i = 0; i < 4; i++) {
    past_theta[i] = theta[i];
  }

  // ロボット座標系から現在の固定座標に変換
  this->current_pos.x =
      this->past_pos.x + cos(past_pos.w) * delta.x - sin(past_pos.w) * delta.y;
  this->current_pos.y =
      this->past_pos.y + sin(past_pos.w) * delta.x + cos(past_pos.w) * delta.y;
  this->current_pos.w = this->past_pos.w + delta.w;

  // past_posを更新
  past_pos = current_pos;

  // set current_vel
  this->current_vel.x = delta.x * u_param.loop_rate;
  this->current_vel.y = delta.y * u_param.loop_rate;
  this->current_vel.w = delta.w * u_param.loop_rate;
}

void illias::MoveSteering::cal_cmd(const CMD &cmd, bool is_transformed) {
  float vx, vy = {0};
  if (!is_transformed) {
    for (int i = 0; i < 4; i++) {
      vx = cmd.x + cmd.w * u_param.wheels[i].distance *
                       cos(u_param.wheels[i].argument + M_PI / 2);
      vy = cmd.y + cmd.w * u_param.wheels[i].distance *
                       sin(u_param.wheels[i].argument + M_PI / 2);
      wheel_cmd_rot[i] = meter_to_rot(sqrt(vx * vx + vy * vy));
      wheel_cmd_rot[i + 4] = rad_to_rot(atan2(vx, vy));
    }
  } else {
    for (int i = 0; i < 4; i++) {
      vx = cmd.x + cmd.w * u_param.wheels[i + 4].distance *
                       cos(u_param.wheels[i + 4].argument + M_PI / 2);
      vy = cmd.y + cmd.w * u_param.wheels[i + 4].distance *
                       sin(u_param.wheels[i + 4].argument + M_PI / 2);
      wheel_cmd_rot[i] = meter_to_rot(sqrt(vx * vx + vy * vy));
      wheel_cmd_rot[i + 4] = rad_to_rot(atan2(vx, vy));
    }
  }
}