#pragma once
#include "wheel.hpp"

namespace illias_wheelctrl {
typedef struct MECHANAM_PARAM {
  std::string wheel_name;  // 車輪の名前
  float radius;            // 半径
  float lx;                // 機体中心からのx軸方向の距離
  float ly;                // 機体中心からのy軸方向の距離
  float gear_ratio;  // タイヤ１回転あたりのモーターの回転数
} M_PARAM, *P_M_PARAM;

class MechanamWheel {
 public:
  void set_params(M_PARAM m_param);  // メンバー変数の設定
  void set_pos(float _rot);          // 現在地を登録
  float cal_cmd(float _met);        // 指令値を送信
  operator float() { return disp; }  // 変位をインスタンスから取得

 private:
  float rot_to_meter(float _rot);  // 回転数をメートルに変換
  float meter_to_rot(float _met);  // 指令値を回転数に変換

  M_PARAM param;

  float disp;  // 単位時間あたりの変位　タイヤを正面から見て反時計回りが正
};
}  // namespace illias_wheelctrl
