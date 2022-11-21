#include "steering.hpp"

MeasureSteering::MeasureSteering(const W_PARAM &_w_param) : w_param(_w_param) {}

MoveSteering::MoveSteering(const W_PARAM &_w_param) : w_param(_w_param) {}

void MeasureSteering::cal_disp(const float encoder[], size_t mysize,
                             ODOM &dist_r) {
  if (mysize != 3) {
    std::printf("invalid argument size");
    dist_r.x = 0;
    dist_r.y = 0;
    dist_r.theta = 0;
  } else {
    float a = w_param.arguments[0];
    float b = w_param.arguments[1];
    float c = w_param.arguments[2];
    float n = 1 / (cos(c) * (sin(b) - sin(a)) + cos(b) * (sin(a) - sin(c)) +
                   cos(a) * (sin(c) - sin(b)));
    dist_r.x =
        n * (encoder[0] * (cos(c) - cos(b)) +
             encoder[1] * (cos(a) - cos(c) + encoder[2] * (cos(b) - cos(a))));
    dist_r.y =
        n * (encoder[0] * (sin(c) - sin(b)) +
             encoder[1] * (sin(a) - sin(c) + encoder[2] * (sin(b) - sin(a))));
    dist_r.theta = (-n * w_param.distance) *
                   (encoder[0] * (cos(b) * sin(c) - sin(b) * cos(c)) +
                    encoder[1] * (cos(c) * sin(a) - sin(c) * cos(a)) +
                    encoder[2] * (cos(a) * sin(b) - sin(a) * cos(b)));
  }
}
void MeasureSteering::cal_disp(const float encoder[], size_t mysize,
                             const float &imu, ODOM &dist_r) {
  if (mysize != 3) {
    std::printf("invalid argument size");
    dist_r.x = 0;
    dist_r.y = 0;
    dist_r.theta = 0;
  } else {
    dist_r.theta = imu;
    dist_r.x =
        (-encoder[0] - encoder[1] + encoder[2] + encoder[3]) / (4 * sqrt(2));
    dist_r.y =
        (encoder[0] - encoder[1] - encoder[2] + encoder[3]) / (4 * sqrt(2));
  }
}

void MoveSteering::cal_cmd(float table_angle,const ODOM &cmd_r, float wheel_cmd[], size_t mysize) {
  if (mysize != 8) {
    std::printf("invalid argument size");
  } else {
    wheel_cmd[0] = sqrt(
        pow(cmd_r.x - w_param.distance * cmd_r.theta * sin(table_angle), 2) +
        pow(cmd_r.y + w_param.distance * cmd_r.theta * sin(table_angle), 2));
    wheel_cmd[1] = sqrt(
        pow(cmd_r.x - w_param.distance * cmd_r.theta * cos(table_angle), 2) +
        pow(cmd_r.y - w_param.distance * cmd_r.theta * sin(table_angle), 2));
    wheel_cmd[2] = sqrt(
        pow(cmd_r.x + w_param.distance * cmd_r.theta * sin(table_angle), 2) +
        pow(cmd_r.y - w_param.distance * cmd_r.theta * cos(table_angle), 2));
    wheel_cmd[3] = sqrt(
        pow(cmd_r.x + w_param.distance * cmd_r.theta * cos(table_angle), 2) +
        pow(cmd_r.y + w_param.distance * cmd_r.theta * sin(table_angle), 2));
    wheel_cmd[4] =
        atan2(cmd_r.y + w_param.distance * cmd_r.theta * sin(table_angle),
              cmd_r.x - w_param.distance * cmd_r.theta * sin(table_angle));
    wheel_cmd[5] =
        atan2(cmd_r.y - w_param.distance * cmd_r.theta * sin(table_angle),
              cmd_r.x - w_param.distance * cmd_r.theta * cos(table_angle));
    wheel_cmd[6] =
        atan2(cmd_r.y - w_param.distance * cmd_r.theta * cos(table_angle),
              cmd_r.x + w_param.distance * cmd_r.theta * sin(table_angle));
    wheel_cmd[7] =
        atan2(cmd_r.y + w_param.distance * cmd_r.theta * sin(table_angle),
              cmd_r.x + w_param.distance * cmd_r.theta * cos(table_angle));
  }
}