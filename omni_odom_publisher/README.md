# mecanum_odom_publisher

4輪メカナムの各車輪の速度(m/s)を読んでオドメトリをpublishするパッケージ



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **body_width** : ロボットの横の長さ (default : 0.440[m])
- **body_height** : ロボットの縦の長さ (default : 0.440[m])
- **base_frame_id** : ロボット座標系 (default : "base_link")



## Published Topics

**/odom** (type : `nav_msgs::Odometry`)



## Subscribed Topics

- **/data_RF** (type : `std_msgs::Float32MultiArray`)
- **/data_LF** (type : `std_msgs::Float32MultiArray`)
- **/data_LB** (type : `std_msgs::Float32MultiArray`)
- **/data_RB** (type : `std_msgs::Float32MultiArray`)

<img src="https://i.imgur.com/3giWneE.png" style="zoom:50%;" />

(補足)

1. [こちら](https://github.com/moden3/serial_test)のROS nodeに接続して、マイコンからステータスを受け取ることを想定している
2. `Float32MultiArray`の具体的なコンテンツは以下の通り。速度(m/s)が一つ入った配列を受け取る。

```c++
void Mecanum_Odom_Publisher::RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_speed[0] = msg->data[0];
}
```


