# SpiconH ROS2 Basic 仕様書

## 1. 概要
本システムは、差動二輪ロボット向けに micro-ROS を用いて ROS2 と通信し、速度指令の受信、車輪状態・オドメトリ・TF・Joint State の配信を行う基本実装である。WiFi(UDP)接続により、複数台のロボットを1つの ROS2 ネットワークで区別して運用できる。

対象実装は以下のファイルを基準とする。

- `Prog_spiconhRos2Basic/Prog_spiconhRos2Basic.ino`
- `Prog_spiconhRos2Basic/ros2robocar.cpp`
- `Prog_spiconhRos2Basic/ros2robocar.h`
- `Prog_spiconhRos2Basic/EducarSpiconLib_alpha.1.h`
- `Prog_spiconhRos2Basic/EducarSpiconLib_alpha.1.cpp`
- `Prog_spiconhRos2Basic/wifi_config.h`(Git管理外。見本は `wifi_config.example.h`)

## 2. 目的
- ROS2 ノードとしてロボット制御を実行する
- 車輪の角度・角速度を計測し、状態を公開する
- `cmd_vel` から速度指令を受け取り、車輪目標角速度へ変換する
- 車輪制御入力をモータに出力し、ロボットを運動させる
- オドメトリ、TF、Joint State を ROS2 トピックとして配信する
- 複数台のロボットを名前空間で識別し、通信断から自動復帰する

## 3. 対象環境
- MCU: ESP32 系
- 開発環境: Arduino IDE + micro_ros_arduino
- 通信: micro-ROS(WiFi/UDP または USBシリアル)
- ROS2 互換: ROS2 Humble 以降を想定した micro-ROS 環境
- 制御周期: 20 ms(`SAMP_TIMEms`、Ticker によるタイマ割り込み)
- ROS2 処理: `loop()` で約10 ms ごとに `ros2Spin()` を実行
- 配信周期: `odom`・`/tf` 50 ms(20 Hz)、`whValue`・`joint_states` 200 ms(5 Hz)

## 4. ハードウェア構成
本実装は差動二輪ロボットを前提とする。

- 左右車輪の駆動用 PWM 出力
- 左右車輪のエンコーダ入力
- モータドライバ制御用 GPIO
- 位置・姿勢計算用の車輪速度・回転角計算
- 追加の IMU 関係処理の準備があるが、現行実装では主に車輪情報を利用

### 4.1 車輪パラメータ
- 車輪半径 `whR = 0.0275 [m]`
- 車体中心から車輪接地点までの距離 `bwh = 0.055 [m]`
- 最大 duty `dutyMax = 50`
- PWM 周波数: `LEDC_SERVO_FREQ = 2500 [Hz]`

## 5. 制御ループ
制御は 20 ms 周期のタイマ割り込み(`timerloop()`)で行い、ROS2 の通信処理は `loop()` 側で独立して行う。

- `timerloop()` は `SAMP_TIMEms`(20 ms)ごとに呼ばれる
- 車輪角度、角速度、速度、姿勢の更新を実施
- 目標車輪角速度を計算
- PID による車輪制御入力を計算
- 左右モータへ PWM 出力

### 5.1 制御処理順序
1. 左右車輪角度計算
2. 左右車輪角速度計算
3. 前進速度計算
4. 旋回速度計算
5. 前進距離計算
6. 姿勢角計算
7. X/Y 速度、位置計算
8. 目標車輪角速度計算(`cmd_vel` の `vf_d`, `dphi_d` から)
9. 目標角度軌道生成
10. PID 制御により duty 計算(ゲイン Kp=5, Ki=0.1, Kd=0)
11. モータ出力

## 6. ROS2 インターフェース

### 6.1 ノードと名前空間
- ノード名: `mycar_node`
- 名前空間: ESP32 の MAC アドレス下位3バイトから生成する `robot_XXXXXX`(例: `robot_A1B2C3`)
- 完全なノード名: `/robot_XXXXXX/mycar_node`
- `/tf` 以外のトピックは相対名で作成し、名前空間が自動で付く

### 6.2 Subscriber
#### `whCmd`(`/robot_XXXXXX/whCmd`)
- 型: `std_msgs/msg/Int32MultiArray`
- 内容: 左右車輪の制御指令
- 配列要素:
  - `data[0]`: 左車輪指令(`whCmdL`)
  - `data[1]`: 右車輪指令(`whCmdR`)
- コールバック: `sub_whCmd_callback`
- 備考: 受信して変数に保存するのみで、現行の制御ループでは使用していない

#### `cmd_vel`(`/robot_XXXXXX/cmd_vel`)
- 型: `geometry_msgs/msg/Twist`
- 内容: ロボットの速度指令
- フィールド:
  - `linear.x`: 目標前進速度 `vf_d` [m/s]
  - `angular.z`: 目標旋回速度 `dphi_d` [rad/s]
- コールバック: `sub_cmdVel_callback`

### 6.3 Publisher
配信周期は `ros2robocar.h` の `ODOM_PUB_PERIOD_MS` / `STATE_PUB_PERIOD_MS` で設定する。

#### `odom`(`/robot_XXXXXX/odom`、20 Hz)
- 型: `nav_msgs/msg/Odometry`
- `header.frame_id`: `robot_XXXXXX/odom`、`child_frame_id`: `robot_XXXXXX/base_link`
- `pose.pose.position.x/y`: 位置 `px`, `py` [m]
- `pose.pose.orientation`: 姿勢角 `phi` から生成(7.3 の注意を参照)
- `twist.twist.linear.x`: 前進速度 [m/s]、`twist.twist.angular.z`: 旋回速度 [rad/s]
- タイマコールバック: `odom_timer_callback`

#### `/tf`(20 Hz)
- 型: `tf2_msgs/msg/TFMessage`
- 内容: `robot_XXXXXX/odom` → `robot_XXXXXX/base_link` の変換
- 全台共通の絶対名 `/tf` に配信し、フレーム名で区別する
- タイマコールバック: `odom_timer_callback`

#### `whValue`(`/robot_XXXXXX/whValue`、5 Hz)
- 型: `std_msgs/msg/Int32MultiArray`
- 配列要素(整数に切り捨て):
  - `data[0]`: 左車輪角度 [deg]
  - `data[1]`: 右車輪角度 [deg]
  - `data[2]`: 左車輪角速度 [deg/s]
  - `data[3]`: 右車輪角速度 [deg/s]
- タイマコールバック: `state_timer_callback`

#### `joint_states`(`/robot_XXXXXX/joint_states`、5 Hz)
- 型: `sensor_msgs/msg/JointState`
- 関節名: `left_wheel_joint`, `right_wheel_joint`(URDF の joint 名と一致させる)
- `position`: 車輪角度 [rad]、`velocity`: 車輪角速度 [rad/s]、`effort`: 0
- タイマコールバック: `state_timer_callback`

### 6.4 タイムスタンプ
- agent 接続時に `rmw_uros_sync_session()` で1回だけ時刻同期する
- 各メッセージの `header.stamp` には同期済みの時刻(`rmw_uros_epoch_millis/nanos`)を用いる

## 7. 運動計算仕様
### 7.1 差動二輪運動学
目標速度 `vf_d`, `dphi_d` から各車輪の目標角速度を計算する。

- 左車輪目標角速度:
  - `lwh.angVel_d = (vf_d - bwh * dphi_d) / whR`
- 右車輪目標角速度:
  - `rwh.angVel_d = (vf_d + bwh * dphi_d) / whR`

### 7.2 速度・位置の計算
- 前進速度: `dwr.vf = whR * (lwh.newAngVel + rwh.newAngVel) / 2 * deg2rad`
- 旋回速度: `dwr.dphi = whR * (rwh.newAngVel - lwh.newAngVel) / (2 * bwh) * deg2rad`
- X 方向速度: `dwr.vx = dwr.vf * cos(dwr.phi)`
- Y 方向速度: `dwr.vy = dwr.vf * sin(dwr.phi)`
- X 位置: `px += vx * dt`
- Y 位置: `py += vy * dt`
- 姿勢角: `phi += dphi * dt`

### 7.3 姿勢(クォータニオン)の生成
- 現行実装: `orientation.z = sin(phi)`, `orientation.w = cos(phi)`(`odom`, `/tf` とも)
- 注意: ヨー角 `phi` の正しいクォータニオンは `z = sin(phi/2)`, `w = cos(phi/2)` である。現行実装では ROS2 側で姿勢角が2倍(`2*phi`)として解釈される

## 8. 車輪制御仕様
### 8.1 角度制御
- `wh_pidAngleL()` / `wh_pidAngleR()` により、左右車輪が目標角度へ追従するよう PID 制御を行う
- 目標角度は `wh_calConstVelTrajectoryL/R()` により生成する
- 制御入力は duty 値としてモータへ出力される

### 8.2 モータ出力
- `wh_motorOutputL()`: 左モータ・PWM出力
- `wh_motorOutputR()`: 右モータ・PWM出力
- 実際の出力は `duty` を用いてドライバへ送られる

## 9. 通信接続仕様

### 9.1 接続方法の選択
`ros2robocar.h` の `USE_WIFI_TRANSPORT` で選択する。

| 設定 | 接続方法 | agent | シリアル表示 |
|---|---|---|---|
| `1` | WiFi(UDP) | 全台で1つ(`udp4`) | あり(`LOG_PRINT` / `LOG_PRINTLN`、115200 bps) |
| `0` | USBシリアル | ロボットごとに1つ(`serial`) | なし(シリアルを micro-ROS が使用) |

### 9.2 WiFi 接続
- 接続設定(`WIFI_SSID`, `WIFI_PASS`, `AGENT_IP`, `AGENT_PORT`)は `wifi_config.h` に記述する。ファイルがない場合はコンパイルエラーとする
- `connectWifiTransport()` で接続状態をシリアル表示しながら接続し、10 秒接続できない場合は再試行する
- WiFi 省電力モードを無効化する(`WiFi.setSleep(false)` と `esp_wifi_set_ps(WIFI_PS_NONE)`)。接続前・接続後・再接続時に設定する
- `monitorWifi()` で動作中の切断・再接続をシリアル表示する

### 9.3 ノードのライフサイクル
| 関数 | 呼ぶタイミング | 内容 |
|---|---|---|
| `ros2Setup()` | `setup()` で1回 | 名前空間の生成、メッセージ用メモリの確保(agent には接続しない) |
| `ros2CreateEntities()` | agent 接続ごと | support / node / publisher / timer / subscriber / executor の生成、時刻同期 |
| `ros2DestroyEntities()` | 切断ごと | 上記の解放 |
| `ros2Spin()` | `loop()` で毎回 | 接続状態の管理、executor の spin |

### 9.4 接続監視と自動再接続
`ros2Spin()` は次の状態を遷移する。

| 状態 | 動作 |
|---|---|
| `WAITING_AGENT` | 500 ms ごとに `rmw_uros_ping_agent(100, 1)` で agent を探す |
| `AGENT_AVAILABLE` | `ros2CreateEntities()` を実行。失敗したら解放して `WAITING_AGENT` へ |
| `AGENT_CONNECTED` | 500 ms ごとに `rmw_uros_ping_agent(100, 3)` で生存確認し、executor を spin する |
| `AGENT_DISCONNECTED` | `vf_d`, `dphi_d`, `whCmdL`, `whCmdR` を 0 にして停止し、解放して `WAITING_AGENT` へ |

## 10. エラー処理
- `RCRETURN()`: `ros2CreateEntities()` 内で使用。失敗時は `false` を返し、エンティティを解放して agent 待ちに戻る
- `RCSOFTCHECK()`: publish や spin など、失敗しても動作を継続する処理で使用
- `RCCHECK()`: 失敗時に `error_loop()` で停止する。現行の接続処理では使用していない
- agent が起動していない、または通信が切れた場合も停止せず、自動で再接続を試みる

## 11. 制約・非機能要件
- micro-ROS の制限により、メッセージのメモリは起動時に静的に確保する
- 連続的な制御は 20 ms 周期で処理する前提で設計されている
- `cmd_vel` と `whCmd` の両方を受信するが、制御には `cmd_vel` のみを使用する
- 停止するのは agent との通信が切れた場合のみ。agent が動作したまま操縦ノードだけが停止した場合は、最後の `cmd_vel` で走行を続ける
- 時刻同期は接続時の1回のみで、長時間の運用では時刻がずれる
- 小型ロボット向けに、車輪速度・姿勢の簡易計算を前提とする

## 12. 変更履歴
- 2023/04/20: Odom, TF, Time Sync を追加
- 2023/04/22: Joint State を追加
- 2023/04/20 以降: `whPwm` から `whCmd` への命名変更を反映
- 2026/10/08: 複数台対応(MAC アドレスによる名前空間、フレーム名)、WiFi/USB 切り替え、agent 自動再接続を追加
- 2026/10/12: WiFi 省電力モードを無効化(v1.1)、配信周期を odom・tf 20 Hz / whValue・joint_states 5 Hz に変更(v1.2)
- 2026/10/13: WiFi 接続設定を `wifi_config.h` に分離(v1.3)

## 13. 補足
本仕様は現行の実装コードに基づく最小構成の仕様であり、今後は IMU 連携、速度制御の高度化、`cmd_vel` 受信タイムアウトによる停止、ロボット状態管理の追加が可能である。複数台運用の詳細は `Prog_spiconhRos2Basic/MULTI_ROBOT_WIFI.md` を参照。
