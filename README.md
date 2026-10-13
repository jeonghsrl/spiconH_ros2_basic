# spiconH ROS2 Basic

ESP32を使った差動二輪ロボット(spiconH)を、**micro-ROS** でROS 2につなぐための基本プログラムです(Arduinoスケッチ)。

PCから `cmd_vel` で速度指令を送ってロボットを動かし、ロボットからはオドメトリ・TF・車輪の状態をROS 2トピックとして受け取れます。WiFiで**複数台を同時に**つなぎ、1台のPCからロボットを区別して操作できます。

## 主な機能

- **速度指令で走行**: `cmd_vel`(前進速度・旋回速度)を左右車輪の目標角速度に変換し、PID制御でモータを動かします(制御周期20ms)。
- **オドメトリとTFの配信**: 車輪エンコーダから位置・姿勢を計算し、`odom` と `/tf` を配信します。
- **車輪状態の配信**: 車輪の角度・角速度を `whValue` と `joint_states` で配信します。
- **WiFi / USB の切り替え**: `#define` 1つで、WiFi(UDP)接続とUSBシリアル接続を切り替えられます。
- **複数台の識別**: ESP32のMACアドレスから名前空間(例: `/robot_A1B2C3`)を自動で作ります。全台に同じプログラムを書き込めます。
- **自動再接続**: PC側のagentとの接続を監視し、切れたらロボットを停止して、自動でつながり直します。
- **WiFi省電力OFF**: ESP32のWiFi省電力モードを切り、通信の遅延を減らしています。

## 動作環境

| 項目 | 内容 |
|---|---|
| マイコン | ESP32系(spiconH基板) |
| 開発環境 | Arduino IDE + [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino) ライブラリ |
| PC側 | ROS 2 Humble 以降 + micro-ROS Agent |
| ロボット | 差動二輪(車輪半径 27.5mm、車体中心から車輪までの距離 55mm) |

## ファイル構成

```
Prog_spiconhRos2Basic/
├── Prog_spiconhRos2Basic.ino    メイン(setup/loop、WiFi接続、20ms周期の制御ループ)
├── ros2robocar.cpp / .h         micro-ROSの処理(トピック、接続監視・再接続、配信周期)
├── EducarSpiconLib_alpha.1.*    車輪・モータ制御ライブラリ(エンコーダ、PID、運動学)
├── wifi_config.example.h        WiFi設定の見本(コピーして wifi_config.h を作る)
└── MULTI_ROBOT_WIFI.md          複数台接続の詳しい説明とトラブル対処
specification.md                 仕様書
worklog.md                       重要な変更履歴
```

## ROS 2トピック

`XXXXXX` はロボットごとのID(MACアドレスの末尾3バイト)です。ノード名は `/robot_XXXXXX/mycar_node` になります。

| トピック | 型 | 向き | 周期 | 内容 |
|---|---|---|---|---|
| `/robot_XXXXXX/cmd_vel` | geometry_msgs/Twist | PC → ロボット | - | `linear.x`: 前進速度 [m/s]、`angular.z`: 旋回速度 [rad/s] |
| `/robot_XXXXXX/whCmd` | std_msgs/Int32MultiArray | PC → ロボット | - | 左右車輪の指令 `[左, 右]`(受信のみ。現在の制御には使っていません) |
| `/robot_XXXXXX/odom` | nav_msgs/Odometry | ロボット → PC | 20Hz | 位置・姿勢・速度 |
| `/tf` | tf2_msgs/TFMessage | ロボット → PC | 20Hz | `robot_XXXXXX/odom` → `robot_XXXXXX/base_link`(全台共通の `/tf`) |
| `/robot_XXXXXX/whValue` | std_msgs/Int32MultiArray | ロボット → PC | 5Hz | `[左角度, 右角度, 左角速度, 右角速度]`(deg, deg/s) |
| `/robot_XXXXXX/joint_states` | sensor_msgs/JointState | ロボット → PC | 5Hz | `left_wheel_joint` / `right_wheel_joint` の角度・角速度(rad, rad/s) |

配信周期は `ros2robocar.h` の `ODOM_PUB_PERIOD_MS` と `STATE_PUB_PERIOD_MS` で変えられます。

## 使い方

### 1. ロボット側の準備

1. このリポジトリをダウンロードします。
   ```bash
   git clone https://github.com/jeonghsrl/spiconH_ros2_basic.git
   ```
2. `Prog_spiconhRos2Basic/ros2robocar.h` で接続方法を選びます。
   ```cpp
   #define USE_WIFI_TRANSPORT  1   // 1: WiFi(UDP)接続  0: USBシリアル接続
   ```
3. **WiFi接続の場合**: `wifi_config.example.h` をコピーして `wifi_config.h` という名前で保存し、環境に合わせて書き換えます。
   ```cpp
   #define WIFI_SSID   "your_ssid"        // WiFiのSSID(2.4GHz帯)
   #define WIFI_PASS   "your_password"    // WiFiのパスワード
   #define AGENT_IP    "192.168.1.100"    // micro-ROS Agentを動かすPCのIPアドレス
   #define AGENT_PORT  8888               // agentのUDPポート
   ```
   `wifi_config.h` はパスワードを含むため、Gitの管理外にしてあります(GitHubには上がりません)。このファイルがないとコンパイルエラーになります。
4. Arduino IDEで `Prog_spiconhRos2Basic.ino` を開き、ESP32に書き込みます。複数台の場合も、全台に同じプログラムを書き込みます。

### 2. PC側でagentを起動

**WiFi接続の場合**(1つのagentで全台を受けられます)
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# Dockerの場合
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```
ファイアウォールで UDP 8888番の受信を許可してください。

**USB接続の場合**(ロボットごとに1つ起動します)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### 3. ロボットを動かす

```bash
# つながっているロボットを確認
ros2 node list
# /robot_A1B2C3/mycar_node

# 前進させる
ros2 topic pub /robot_A1B2C3/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# キーボードで操作する
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_A1B2C3/cmd_vel

# オドメトリを確認する
ros2 topic echo /robot_A1B2C3/odom
ros2 topic hz /robot_A1B2C3/odom    # 約20Hz
```

### 4. 動作確認(WiFi接続時のシリアルモニタ、115200bps)

```
===== spiconH micro-ROS (WiFi) =====
WiFi connecting to "your_ssid" .....
WiFi connected
  ...
  PowerSave: OFF
ROS namespace     : /robot_A1B2C3
waiting agent ...
agent connected
```

`waiting agent ...` のまま進まない場合は、agentの起動、`AGENT_IP` / `AGENT_PORT`、ファイアウォールを確認してください。表示の意味やトラブル対処、RVizでの表示方法は [MULTI_ROBOT_WIFI.md](Prog_spiconhRos2Basic/MULTI_ROBOT_WIFI.md) にまとめてあります。

## 注意点

- **agentとの接続が切れるとロボットは停止します**が、agentが動いたままPC側の操縦ノードだけが落ちた場合は、最後に受け取った `cmd_vel` で走り続けます。
- USB接続では、シリアルポートをmicro-ROSの通信に使うため、シリアルモニタは使えません。
- PCのIPアドレスが変わると全台を書き込み直す必要があるので、DHCP予約などでPCのIPを固定しておくことをおすすめします。

## 変更履歴

主な変更は [worklog.md](worklog.md) と、Gitのタグ(`v1.0`〜)に記録しています。
