# wheelctrl_ros
[![Build Status](https://app.travis-ci.com/KeioRoboticsAssociation/wheelctrl_ros.svg?branch=main)](https://app.travis-ci.com/KeioRoboticsAssociation/wheelctrl_ros) [![ROS: Humble](https://img.shields.io/badge/ROS-Noetic-brightgreen)](https://docs.ros.org/en/humble/index.html)  [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![KRA: ilias](https://img.shields.io/badge/KRA-ilias-blue.svg)](https://keiorogiken.wordpress.com/)

A ROS meta package for wheel control for RogiLink

# 構成
座標変換部
- [オムニ２輪（測定輪）](/wheelctrl_ros/wheelctrl_ros2/src/omni_2w.cpp)
- [オムニ３輪](/wheelctrl_ros/wheelctrl_ros2/src/omni_3w.cpp)
- [オムニ４輪](/wheelctrl_ros/wheelctrl_ros2/src/omni_4w.cpp)
- [独ステ](/wheelctrl_ros/wheelctrl_ros2/src/steering.cpp)
- メカナム（未制作）

トピック通信部
- [メイン部](/wheelctrl_ros/wheelctrl_ros2/src/general_wheelctrl.hpp)

## 使い方
1. ロボットのパラメータをyamlに書く
2. 起動する。
足回りの指令値は geometry_msgs::msg::Twist /cmd_vel をsubー＞座標変換ー＞Rogilink
現在異位置は    Rogilinkー＞座標変換ー＞nav_msgs::msg::Odom /odom をpub
という流れをとる。

## yamlの書き方
### 変形もしないし、測定輪もない場合（基本構成）
type_name : "omni"か"steering"か"mechanam"
radius    : タイヤ半径
quantity  : タイヤの個数
loop_rate : 制御周期 Hz
gear_ratio: ギア比（仮置き）
gear_ratio_horizonal: ギア比（ステア）
Wheel_name: Rogilinkで使うタイヤの名前
distance  : 中心-タイヤ距離
argument  : 正面からの偏角

※：変形機能との兼ね合いでdistanceはタイヤの数と同じ要素数の配列にしてください

### 変形する場合
distance,argumentを

``` yaml
（省略）
wheel_name: [FL,BL,BR,FR]
before:
    distance: [0.7, 0.7, 0.7, 0.7]
    argument: [45.0, 135.0, 225.0, 315.0]
after:
    distance: [0.7, 0.7, 0.8, 0.8]
    argument: [45.0, 135.0, 225.0, 315.0]
```
のように書いてください

### 測定輪を使う場合
測定輪について、
measuring_wheel: の下に書いてください
