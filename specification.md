# SpiconH ROS2 Basic 仕様書

## 1. 概要
本システムは、差動二輪ロボット向けに micro-ROS を用いて ROS2 と通信し、車輪制御指令、速度指令、車輪状態、オドメトリ、Joint State を配信するための基本実装である。

対象実装は以下のファイルを基準とする。

- `Prog_spiconhRos2Basic/Prog_spiconhRos2Basic.ino`
- `Prog_spiconhRos2Basic/ros2robocar.cpp`
- `Prog_spiconhRos2Basic/ros2robocar.h`
- `Prog_spiconhRos2Basic/EducarSpiconLib_alpha.1.h`
- `Prog_spiconhRos2Basic/EducarSpiconLib_alpha.1.cpp`

## 2. 目的
- ROS2 ノードとしてロボット制御を実行する
- 車輪の角度・角速度を計測し、状態を公開する
- `cmd_vel` から速度指令を受け取り、車輪目標角速度へ変換する
- 車輪制御入力をモータに出力し、ロボットを運動させる
- オドメトリ、TF、Joint State を ROS2 トピックとして配信する

## 3. 対象環境
- MCU: ESP32 系
- 通信: micro-ROS
- ROS2 互換: ROS2 humble 以降を想定した micro-ROS 環境
- 制御周期: 20 ms
- 監視周期: 10 ms で spin_some 実行

## 4. ハードウェア構成
本実装は差動二輪ロボットを前提とする。

- 左右車輪の駆動用 PWM 出力
- 左右車輪のエンコーダ入力
- モータドライバ制御用 GPIO
- 位置・姿勢計算用の車輪速度・回転角計算
- 追加の IMU 関係処理の準備があるが、現行実装では主に車輪情報を利用

### 4.1 車輪パラメータ
- 車輪半径 `whR = 0.0275 [m]`
- 車輪間距離 `bwh = 0.055 [m]`
- 最大 duty `dutyMax = 50`
- PWM 周波数: `LEDC_SERVO_FREQ = 2500 [Hz]`

## 5. 制御ループ
制御は 20 ms 単位のタイマ割り込みと、その後の ROS2 executor によって動作する。

- `timerloop()` は `SAMP_TIMEms`（20 ms）ごとに呼ばれる
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
8. 目標車輪角速度計算
9. 目標角度軌道生成
10. PID 制御により duty 計算
11. モータ出力

## 6. ROS2 インターフェース
### 6.1 Subscriber
#### `whCmd`
- 型: `std_msgs/msg/Int32MultiArray`
- 内容: 左右車輪の目標制御指令
- 配列要素:
  - `data[0]`: 左車輪指令
  - `data[1]`: 右車輪指令
- コールバック: `sub_whCmd_callback`

#### `cmd_vel`
- 型: `geometry_msgs/msg/Twist`
- 内容: ロボットの速度指令
- フィールド:
  - `linear.x`: 目標前進速度 `vf_d` [m/s]
  - `angular.z`: 目標旋回速度 `dphi_d` [rad/s]
- コールバック: `sub_cmdVel_callback`

### 6.2 Publisher
#### `whValue`
- 型: `std_msgs/msg/Int32MultiArray`
- 内容: 左右車輪の状態情報
- 配列要素: 既定の 4 要素を保持する構造として扱う
- 使用目的: 車輪状態の可視化・外部制御へのフィードバック

#### `odom`
- 型: `nav_msgs/msg/Odometry`
- 内容: オドメトリ情報
- 位置・速度と姿勢の更新に基づくデータを発行

#### `tf`
- 型: `tf2_msgs/msg/TFMessage`
- 内容: `odom -> base_link` の変換情報
- 初期化時に `odom`, `base_link` を定義

#### `joint_states`
- 型: `sensor_msgs/msg/JointState`
- 内容: 左右車輪の Joint State
- 要素数: 2
- 用途: ROS2 可視化・制御ツール連携

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

## 8. 車輪制御仕様
### 8.1 角度制御
- `wh_pidAngleL()` / `wh_pidAngleR()` により、左右車輪が目標角度へ追従するよう PID 制御を行う
- 目標角度は `wh_calConstVelTrajectoryL/R()` により生成する
- 制御入力は duty 値としてモータへ出力される

### 8.2 モータ出力
- `wh_motorOutputL()`: 左モータ・PWM出力
- `wh_motorOutputR()`: 右モータ・PWM出力
- 実際の出力は `duty` を用いてドライバへ送られる

## 9. ノード動作
`ros2Setup()` で以下を初期化する。

- `rclc_support_init()`
- `rclc_node_init_default()`
- Publisher 初期化
- Timer 初期化
- Subscriber 初期化
- Executor 初期化
- 変数初期化（車輪状態、TF、Joint State など）

## 10. エラー処理
- `RCCHECK()` は `rcl_ret_t` が `RCL_RET_OK` でない場合、`error_loop()` を呼び出す
- `RCSOFTCHECK()` は軽微なエラーを握りつぶす
- 重大な ROS2 初期化失敗時はエラー状態へ遷移する

## 11. 制約・非機能要件
- ROS2 micro-ROS の制限により、低レベルなメモリ管理とタイマ制御が必要である
- 連続的な制御は 20 ms 周期で処理する前提で設計されている
- `cmd_vel` と `whCmd` の両方式が存在するが、実際の制御は `cmd_vel` を使って目標軌道を生成し、車輪指令へ変換する設計である
- 小型ロボット向けに、車輪速度・姿勢の簡易計算を前提とする

## 12. 変更履歴
- 2023/04/20: Odom, TF, Time Sync を追加
- 2023/04/22: Joint State を追加
- 2023/04/20 以降: `whPwm` から `whCmd` への命名変更を反映

## 13. 補足
本仕様は現行の実装コードに基づく最小構成の仕様であり、今後は IMU 連携、速度制御の高度化、障害検知、ロボット状態管理の追加が可能である。
