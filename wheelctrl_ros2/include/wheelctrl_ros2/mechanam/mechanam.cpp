#include "mechanam.hpp"

MeasureMechanam::MeasureMechanam(const W_PARAM &_w_param) : w_param(_w_param) {}

MoveMechanam::MoveMechanam(const W_PARAM &_w_param) : w_param(_w_param) {}

void MeasureMechanam::cal_disp(const float encoder[], size_t mysize,
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
void MeasureMechanam::cal_disp(const float encoder[], size_t mysize,
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

void MoveMechanam::cal_cmd(const ODOM &cmd_r, float wheel_cmd[], size_t mysize) {
  if (mysize != 3) {
    std::printf("invalid argument size");
  } else {
    wheel_cmd[0] = -cmd_r.x * sin(w_param.arguments[0]) +
                   cmd_r.y * cos(w_param.arguments[0]) +
                   cmd_r.theta * w_param.distance;
    wheel_cmd[1] = -cmd_r.x * sin(w_param.arguments[1]) +
                   cmd_r.y * cos(w_param.arguments[1]) +
                   cmd_r.theta * w_param.distance;
    wheel_cmd[2] = -cmd_r.x * sin(w_param.arguments[2]) +
                   cmd_r.y * cos(w_param.arguments[2]) +
                   cmd_r.theta * w_param.distance;
  }
}