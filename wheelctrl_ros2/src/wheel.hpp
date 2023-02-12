#pragma once
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <memory>

using namespace std;

namespace illias {
// 構造体の定義

// タイヤのパラメータ
typedef struct WHEEL_PARAMETER{
  float distance;  // ロボット中心からの距離
  float argument;//ロボット正面からの偏角
} W_PARAM, *P_W_PARAM;

// 足回りのパラメータ
typedef struct UNDERSIDE_PARAMETER {
  string type_name;       //足回りの種類
  float radius;           //タイヤ半径
  int quantity;         //タイヤの数
  float gear_ratio;       //ギア比（タイヤ１回転あたりモーター回転量）
  float gear_ratio_steer; //ギア比（ステア）
  float loop_rate;        //制御周期
  vector<W_PARAM> wheels; // 各タイヤのパラメタ
} U_PARAM, *P_U_PARAM;

// ロボットの姿勢
typedef struct POSTURE {
  float x;
  float y;
  float w;// オメガの代わり
} POS, *P_POS;

// 足回りの指令値
typedef struct COMMAND {
  float x;
  float y;
  float w;
} CMD, *P_CMD;
// ここまで構造体の定義

// ここから関数の定義

// POSTURE同士の加減とスカラ倍
inline POS operator+(const POS &a, const POS &b) { 
  POS c;
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.w = a.w + b.w;
  return c;
}
inline POS operator*(const POS &a, const float &f) { 
  POS c;
  c.x = a.x * f;
  c.y = a.y * f;
  c.w = a.w * f;
  return c;
}
inline POS operator/(const POS &a, const float &f) { 
  POS c;
  c.x = a.x / f;
  c.y = a.y / f;
  c.w = a.w / f;
  return c;
}

// 二次元ベクトルの大きさを計算
inline float cal_r(float x, float y) { return sqrt(pow(x, 2) + pow(y, 2)); }

// 余割を計算
inline float csc(float x){
  if (x == 0){
    return 0;
  } else {
    return 1 / sin(x);
  }
}

// 多めの回転を許容
inline float ex_rot(float current_rot,float command_rot){
  float ex_rot = 0;
  if(current_rot - command_rot > M_PI){
    ex_rot = command_rot + 1;
  } else if (current_rot - command_rot < -M_PI) {
    ex_rot = command_rot - 1;
  } else {
    ex_rot = command_rot;
  }
  if (ex_rot > 2){
    ex_rot = ex_rot - 2;
  } else if (ex_rot < -2){
    ex_rot = ex_rot + 2;
  }
}

// ここまで関数の定義


//測定用クラス
class Measuring {
  public:
  //コンストラクタ
   Measuring(const U_PARAM &_u_param,const POS &_initial_pos)
   :u_param(_u_param),past_pos(_initial_pos){
     current_pos = past_pos;
     current_vel = {0, 0, 0};
   }              
   //デストラクタ
   virtual ~Measuring(){}
   //変位を計算
   virtual void cal_disp(std::vector<float> encoder, const float imu = 0,
                         bool is_transformed = false)
   {
     cout << "ERROR : please use subclass" << endl;
     current_vel.x = 0;
     current_vel.y = 0;
     current_vel.w = 0;
     current_pos = current_pos + (current_vel / u_param.loop_rate);
   };
   //現在位置を取得
   inline POS get_current_pos() { return current_pos; }
   //現在速度を取得
   inline POS get_current_vel() { return current_vel; }

  protected:
   // 回転を距離に変換
   inline float rot_to_meter(float rot) {
     return rot * 2 * M_PI * u_param.radius / u_param.gear_ratio;
   }
   // 回転をラジアンに変換
   inline float rot_to_rad(float rot) {
     return rot * 2 * M_PI / u_param.gear_ratio_steer;
   }
   U_PARAM u_param;  // 足回りのパラメータ
   POS current_pos; // 現在位置
   POS current_vel; // 現在の速度
   POS past_pos;    // 過去の位置
};

//移動用クラス
class Moving {
  public:
   // コンストラクタ
   Moving(const U_PARAM &_u_param) : u_param(_u_param) {
     wheel_cmd_meter.resize(u_param.type_name == "steering"
                                ? u_param.quantity * 2
                                : u_param.quantity);
     wheel_cmd_rot.resize(u_param.type_name == "steering" ? u_param.quantity * 2
                                                          : u_param.quantity);
                                                          
     for (int i = 0; i < (u_param.type_name == "steering" ? u_param.quantity * 2
                                                          : u_param.quantity);
          i++) {
       wheel_cmd_meter[i] = 0;
       wheel_cmd_rot[i] = 0;
     }
    steer_angle[0] = 0;
    steer_angle[1] = 0;
    steer_angle[2] = 0;
    steer_angle[3] = 0;
   };
   // デストラクタ
   virtual ~Moving(){}
   // 指令値を各タイヤに計算
   virtual void cal_cmd(const CMD &cmd, bool is_transformed = false){
     cout << "ERROR : please use subclass" << endl;
   };

   std::vector<float> wheel_cmd_meter;
   std::vector<float> wheel_cmd_rot;

 protected:
  // 移動距離を回転数に換算
  inline float meter_to_rot(float meter) {
     return u_param.gear_ratio * meter / (2 * M_PI * u_param.radius);
  }
  // ラジアンを回転数に換算
  inline float rad_to_rot(float rad) {
     return u_param.gear_ratio_steer * rad / (2 * M_PI);
  }

  U_PARAM u_param;  // 足回りのパラメータ
  float steer_angle[4]; // ステアリングの現在角度
};
}