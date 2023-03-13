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
    printf("this is a steering wheel package\n");
  } else {
    printf("you shouled use another subclass\n");
  }
}

void illias::MeasureSteering::set_initial_wheel_angle(float w0, float w1,
                                                      float w2, float w3) {
  past_theta[0] = w0;
  past_theta[1] = w1;
  past_theta[2] = w2;
  past_theta[3] = w3;
}

void illias::MeasureSteering::cal_disp(std::vector<float> encoder, float imu,
                                       bool is_transformed) {
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
      int vel_sign = 1;
      vx = cmd.x + cmd.w * u_param.wheels[i].distance *
                       cos(u_param.wheels[i].argument + M_PI / 2);
      vy = cmd.y + cmd.w * u_param.wheels[i].distance *
                       sin(u_param.wheels[i].argument + M_PI / 2);
      vx = vx == -0.0 ? 0.0 : vx;
      vy = vy == -0.0 ? 0.0 : vy;
      float raw_angle = atan2(vy, vx);
      float cmd_angle = raw_angle;
      // printf("[cmd] %f\n", cmd_angle);
      bool flag = true;
      if (vx != 0 || vy != 0) {
        if (cmd_angle > 0) {
          if (-4 * M_PI < steer_angle[i] && steer_angle[i] < -5 * M_PI / 2) {
            cmd_angle -= 4 * M_PI;
          } else if (-5 * M_PI / 2 <= steer_angle[i] &&
                     steer_angle[i] < -M_PI / 2) {
            cmd_angle -= 2 * M_PI;
          } else if (-M_PI / 2 <= steer_angle[i] &&
                     steer_angle[i] < 3 * M_PI / 2) {
            // cmd_angle += 0;
          } else if (3 * M_PI / 2 <= steer_angle[i] &&
                     steer_angle[i] < 7 * M_PI / 2) {
            cmd_angle += 2 * M_PI;
          } else {
            // cmd_angle += 0;
            flag = false;
          }
        } else if (cmd_angle < 0) {
          if (-7 * M_PI / 2 < steer_angle[i] &&
              steer_angle[i] < -3 * M_PI / 2) {
            cmd_angle -= 2 * M_PI;
          } else if (steer_angle[i] < M_PI / 2) {
            // cmd_angle -= 0 * M_PI;
            if (steer_angle[i] < -2 * M_PI) {
              flag = false;
            }
          } else if (steer_angle[i] < 5 * M_PI / 2) {
            cmd_angle += 2 * M_PI;
          } else if (steer_angle[i] < 4 * M_PI) {
            cmd_angle += 4 * M_PI;
          } else {
            // cmd_angle = 0;
            flag = false;
          }
        } else {
          if (-3 * M_PI < steer_angle[i] && steer_angle[i] < -M_PI) {
            cmd_angle = -2 * M_PI;
          } else if (steer_angle[i] < M_PI) {
            cmd_angle = 0;
          } else if (steer_angle[i] < 3 * M_PI) {
            cmd_angle = 2 * M_PI;
          } else {
            cmd_angle = 0;
            flag = false;
          }
        }
        if (flag) {
          if (cmd_angle - steer_angle[i] > M_PI / 2) {
            cmd_angle -= M_PI;
            vel_sign *= -1;
          } else if (cmd_angle - steer_angle[i] < -M_PI / 2) {
            cmd_angle += M_PI;
            vel_sign *= -1;
          }
        }
        wheel_cmd_rot[i] =
            -1 * vel_sign * meter_to_rot(sqrt(vx * vx + vy * vy));
        wheel_cmd_rot[i + 4] = rad_to_rot(cmd_angle);
        steer_angle[i] = cmd_angle;
      }
      // printf("%f,%f\n", cmd_angle, steer_angle[i]);
      if (vx == 0 && vy == 0) {
        // if(steer_angle[i]>M_PI){
        //   steer_angle[i] -= 2 * M_PI;
        // }else if(steer_angle[i]<-M_PI){
        //   steer_angle[i] += 2 * M_PI;
        // }
        wheel_cmd_rot[i] = 0;
        wheel_cmd_rot[i + 4] = rad_to_rot(steer_angle[i]);
      }
    }
    printf("[ang] %f,%f,%f,%f\n", steer_angle[0], steer_angle[1],
           steer_angle[2], steer_angle[3]);
    // printf("[vel] %f,%f,%f,%f\n", wheel_cmd_rot[0], wheel_cmd_rot[1],
    //        wheel_cmd_rot[2], wheel_cmd_rot[3]);
  } else {
    for (int i = 0; i < 4; i++) {
      int vel_sign = 1;
      vx = cmd.x + cmd.w * u_param.wheels[i + 4].distance *
                       cos(u_param.wheels[i + 4].argument + M_PI / 2);
      vy = cmd.y + cmd.w * u_param.wheels[i + 4].distance *
                       sin(u_param.wheels[i + 4].argument + M_PI / 2);
      float raw_angle = atan2(vy, vx);
      float cmd_angle = raw_angle;
      if (steer_angle[i] > M_PI / 2 && cmd_angle < 0) {
        cmd_angle += 2 * M_PI;
      } else if (steer_angle[i] < -M_PI / 2 && cmd_angle > 0) {
        cmd_angle -= 2 * M_PI;
      }
      if (cmd_angle - steer_angle[i] > M_PI / 2) {
        cmd_angle -= M_PI;
        vel_sign *= -1;
      } else if (cmd_angle - steer_angle[i] < -M_PI / 2) {
        cmd_angle += M_PI;
        vel_sign *= -1;
      }
      // printf("%f,%f\n", cmd_angle, steer_angle[i]);
      wheel_cmd_rot[i] = -1 * vel_sign * meter_to_rot(sqrt(vx * vx + vy * vy));
      wheel_cmd_rot[i + 4] = rad_to_rot(cmd_angle);
      if (vx == 0 && vy == 0) {
        wheel_cmd_rot[i + 4] = rad_to_rot(steer_angle[i]);
      }
      steer_angle[i] = cmd_angle;
    }
  }
}