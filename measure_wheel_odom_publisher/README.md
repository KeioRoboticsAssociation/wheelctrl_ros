# measure_wheel_odom_publisher

測定輪の各車輪の速度(m/s)を読んでオドメトリをpublishするパッケージ



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **wheel_width** : ロボットの中心からのx車輪の距離 (default : 0.150[m])
- **wheel_height** : ロボットの中心からのy車輪の距離 (default : 0.150[m])
- **base_frame_id** : ロボット座標系 (default : "base_link")



## Published Topics

**/odom** (type : `nav_msgs::Odometry`)



## Subscribed Topics

- **/data_X_axis** (type : `std_msgs::Float32MultiArray`)
- **/data_Y_axis** (type : `std_msgs::Float32MultiArray`)

<img src="https://i.imgur.com/3giWneE.png" style="zoom:50%;" />

(補足)

1. [こちら](https://github.com/moden3/serial_test)のROS nodeに接続して、マイコンからステータスを受け取ることを想定している
2. `Float32MultiArray`の具体的なコンテンツは以下の通り。速度(m/s)が一つ入った配列を受け取る。

