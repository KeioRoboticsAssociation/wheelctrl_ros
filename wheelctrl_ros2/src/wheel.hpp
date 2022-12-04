#pragma once
#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <string>
#include <vector>

using namespace std;

namespace illias{

typedef struct WHEEL_PARAMETER {
  std::string type_name;      //足回りの種類
  float radius;               //半径
  int quantity;             //タイヤの数
  float loop_rate;            //ループ周期
  float gear_ratio;  //タイヤ1回転あたりのモーター回転数
  float gear_ratio_horizonal; //タイヤ1回転あたりのモーター回転数

  std::string coordinate;     
  float distance;             //中心からタイヤまでの距離
  std::vector<double> arguments;//どの角度についているのか

  float length_x;//x軸方向の長さ
  float length_y;//y軸方向の長さ
} W_PARAM, *P_W_PARAM;

typedef struct POSTURE {
  float x;
  float y;
  float theta;
} POS, *P_POS;

typedef struct COMMAND {
  float x;
  float y;
  float theta;
} CMD, *P_CMD;

inline float change_range(float arg){
  if(arg>M_PI){
    return arg - 2*M_PI;
  }else if(arg < -M_PI){
    return arg + 2*M_PI;
  }else{
    return arg;
  }
}

inline float csc(float arg) {
  if (sin(arg) != 0){
    return 1 / sin(arg);
  }else{
    return 0;
  }
}

// base class of measuring wheels
class Measuring {
 public:
  Measuring(const W_PARAM &_w_param,const POS &_past_pos)
  :w_param(_w_param),past_pos(_past_pos){
    current_pos = past_pos;
    current_vel = {0,0,0};
  }
  virtual ~Measuring(){}

  //convert encoder data into robot posture(world coordinate)
  virtual void cal_disp(const float encoder[],const int &length){
    printf("please use subclass");
    for (int i = 0; i < length;i++)
    {
      printf("%f ", encoder[i]);
    }
    printf("\n");
  }

  // convert encoder data into robot posture(world coordinate)
  virtual void cal_disp(const float encoder[],const int &length,const float &imu){//エンコーダーの値から現在の座標を計算
    printf("please use subclass");
    for (int i = 0; i < length;i++){
      printf("%f ", encoder[i]);
    }
    printf("%f\n", imu);
  }

  // get current posture of robot (world coordinate)
  POS get_current_pos() { return this->current_pos; }

  float rotate_to_meter(const float &rotate){
    return (rotate/this->w_param.gear_ratio) * 2 * M_PI * this->w_param.radius;
  }
  float rotate_to_rad(const float &rotate){
    return (rotate / this->w_param.gear_ratio_horizonal) * 2 * M_PI;
  }
 protected:
  W_PARAM w_param;
  POS past_pos; //前の位置（固定座標系）
  POS current_pos;//現在の位置（固定座標系）
  POS current_vel;//現在の速度（固定座標系）
};

class Moving {
 public:
  Moving(const W_PARAM &_w_param) : w_param(_w_param){
    if (w_param.type_name == "steering"){
      wheel_cmd_meter = std::make_unique<float[]>(2 * w_param.quantity);
      wheel_cmd_rotate = std::make_unique<float[]>(2 * w_param.quantity);

      for (int i = 0; i < 2*w_param.quantity; i++) {
        wheel_cmd_meter[i] = 0;
        wheel_cmd_rotate[i] = 0;
      }
    }else{
      wheel_cmd_meter = std::make_unique<float[]>(w_param.quantity);
      wheel_cmd_rotate = std::make_unique<float[]>(w_param.quantity);
      for (int i = 0; i < w_param.quantity; i++) {
        wheel_cmd_meter[i] = 0;
        wheel_cmd_rotate[i] = 0;
      }
    }
  }
  virtual ~Moving(){}
  virtual void cal_cmd(const CMD &cmd) { 
    printf("please use subclass");
    printf("%f,%f,%f\n", cmd.x, cmd.y, cmd.theta);
  }
  virtual void cal_cmd(const CMD &cmd, const float curvature){
    printf("please use subclass");
    printf("%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, curvature);
  }
  virtual void cal_cmd(const CMD &cmd, const float &table_angle){
    printf("%f,%f,%f,%f\n", cmd.x, cmd.y, cmd.theta, table_angle);
  }

  float meter_to_rotate(const float &meter) {
    return meter * this->w_param.gear_ratio / (2 * M_PI * this->w_param.radius);
  }
  float rad_to_meter(const float &rad) {
    return rad * this->w_param.gear_ratio / (2 * M_PI);
  }

  std::unique_ptr<float[]> wheel_cmd_meter;
  std::unique_ptr<float[]> wheel_cmd_rotate;

 protected:
  W_PARAM w_param;
};
}
