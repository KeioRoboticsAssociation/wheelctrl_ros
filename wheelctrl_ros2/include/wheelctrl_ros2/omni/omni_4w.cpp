#include "omni_4w.hpp"

MeasureOmni4W::MeasureOmni4W(const W_PARAM &_w_param)
:w_param(_w_param){
}

MoveOmni4W::MoveOmni4W(const W_PARAM &_w_param)
:w_param(_w_param){

}

void MeasureOmni4W::cal_disp(const float encoder[],size_t mysize,ODOM &dist_r) {
    if(mysize!=4){
      std::printf("invalid argument size");
      dist_r.x = 0;
      dist_r.y = 0;
      dist_r.theta = 0;
    } else {
      dist_r.theta = (encoder[0] + encoder[1] + encoder[2] + encoder[3]) /
                   (4 * w_param.distance);
      dist_r.x =
          (-encoder[0] - encoder[1] + encoder[2] + encoder[3]) / (4 * sqrt(2));
      dist_r.y =
          (encoder[0] - encoder[1] - encoder[2] + encoder[3]) / (4 * sqrt(2));
    }
}
void MeasureOmni4W::cal_disp(const float encoder[], size_t mysize,const float &imu, ODOM &dist_r) {
  if (mysize != 4) {
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

void MoveOmni4W::cal_cmd(const ODOM &cmd_r, float wheel_cmd[], size_t mysize) {
  if(mysize !=4){
    std::printf("invalid argument size");
  }else{
    wheel_cmd[0] =
        sqrt(2) * (-cmd_r.x + cmd_r.y) + w_param.distance * cmd_r.theta;
    wheel_cmd[1] =
        sqrt(2) * (-cmd_r.x - cmd_r.y) + w_param.distance * cmd_r.theta;
    wheel_cmd[2] =
        sqrt(2) * (cmd_r.x - cmd_r.y) + w_param.distance * cmd_r.theta;
    wheel_cmd[3] =
        sqrt(2) * (cmd_r.x + cmd_r.y) + w_param.distance * cmd_r.theta;
  }
}