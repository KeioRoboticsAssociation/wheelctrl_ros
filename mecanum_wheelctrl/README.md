# mecanum_wheelctrl

## 概要

base_linkの速度指令値を**4輪メカナム**のモデルに基づいて4つのモーターに送る指令値へ変換するROSパッケージ



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **lost_time_threshold** : timeout period (default : 500[ms])
- **body_width** : ロボットの横の長さ (default : 0.440[m])
- **body_height** : ロボットの縦の長さ (default : 0.440[m])
- **gazebo_mode** : gazeboでのシミュレーションをする際はtrueにする (default : false)



## Subscribed Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)



## Published Topics

if gazebo_mode == False:

- **/control_RF** (type : `std_msgs::Float32MultiArray`)
- **/control_LF** (type : `std_msgs::Float32MultiArray`)
- **/control_LB** (type : `std_msgs::Float32MultiArray`)
- **/control_RB** (type : `std_msgs::Float32MultiArray`)

if gazebo_mode == True:

- **/control_RF** (type : `std_msgs::Float64`)
- **/control_LF** (type : `std_msgs::Float64`)
- **/control_LB** (type : `std_msgs::Float64`)
- **/control_RB** (type : `std_msgs::Float64`)

<img src="https://i.imgur.com/3giWneE.png" style="zoom:50%;" />

(補足)

1. [こちら](https://github.com/moden3/serial_test)のROS nodeに接続して、マイコンに指令値を送ることを想定している

2. `Float32MultiArray`の具体的なコンテンツは以下の通り

```c++
std_msgs::Float32MultiArray floatarray;
floatarray.data.resize(1);
floatarray.data[0] = target_speed[0];
pub_RF.publish(floatarray);
```
