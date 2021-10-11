# swerve_odom_publisher

4輪ステアリングの各車輪の速度(m/s)を読んでオドメトリをpublishするパッケージ



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **body_width** : 4WSロボット(正方形)の一辺の長さ (default : 0.440[m])
- **base_frame_id** : ロボット座標系 (default : "base_link")
- **gazebo_mode** : gazeboでのシミュレーションをする際はtrueにする (default : false)



## Published Topics

**/odom** (type : `nav_msgs::Odometry`)



## Subscribed Topics

- **/imu** (type : `sensor_msgs::Imu`)

 if gazebo_mode == False:

- **/data_RF** (type : `std_msgs::Float64`)
- **/data_LF** (type : `std_msgs::Float32MultiArray`)
- **/data_LB** (type : `std_msgs::Float32MultiArray`)
- **/data_RB** (type : `std_msgs::Float32MultiArray`)

 if gazebo_mode == True:

- **/simple_swerve/joint_states** (type : `sensor_msgs::JointState`)

<img src="https://i.imgur.com/3giWneE.png" style="zoom:50%;" />

(補足)

1. [こちら](https://github.com/moden3/serial_test)のROS nodeに接続して、マイコンからステータスを受け取ることを想定している

2. `Float32MultiArray`には

   `Float32MultiArray data[2]; data[0]=v, data[1]=theta`

   のようにステータスが格納されている
