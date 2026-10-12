# 複数台ロボットのWiFi接続・識別（micro-ROS）

複数台のロボット（ESP32 + micro-ROS）を1つのWiFiネットワークにつなぎ、1台のPCからロボットを区別して制御するための変更です。

- **識別**：MACアドレスからロボットごとの名前空間（例：`/robot_A1B2C3`）を自動で作ります。同じプログラムを全台に書き込めます。
- **接続**：WiFi（UDP）とUSBシリアルを `#define` 1つで切り替えられます。WiFiなら、PC側のmicro-ROS Agentは1つで全台を受けられます。
- **自動再接続**：`rmw_uros_ping_agent()` でエージェントとの接続を監視します。接続が切れたらロボットを停止し、自動でつながり直します。

---

## 1. 変更点

### 対象ファイル

| ファイル | 内容 |
|---|---|
| `ros2robocar.cpp` | 名前空間の生成、TFのフレーム名、接続監視・再接続 |
| `ros2robocar.h` | 接続方法の選択（`USE_WIFI_TRANSPORT`）、デバッグ表示用マクロ、追加した関数の宣言 |
| `Prog_spiconhRos2Basic.ino` | WiFi接続の設定、接続方法による処理の切り替え、`loop()` から `ros2Spin()` を呼ぶ形に変更 |

### 1.1 名前空間による識別

- `makeRobotNs()` で、ESP32のMACアドレス（`ESP.getEfuseMac()`）の末尾3バイトから名前空間を作ります。
  - 例：MACアドレスが `xx:xx:xx:A1:B2:C3` なら `robot_A1B2C3`
- ノードは `/robot_A1B2C3/mycar_node` になります。
- トピックは相対名なので、自動で名前空間が付きます。

| 変更前 | 変更後 | 型 | 向き |
|---|---|---|---|
| `/cmd_vel` | `/robot_XXXXXX/cmd_vel` | geometry_msgs/Twist | PC → ロボット |
| `/whCmd` | `/robot_XXXXXX/whCmd` | std_msgs/Int32MultiArray | PC → ロボット |
| `/odom` | `/robot_XXXXXX/odom` | nav_msgs/Odometry | ロボット → PC |
| `/whValue` | `/robot_XXXXXX/whValue` | std_msgs/Int32MultiArray | ロボット → PC |
| `/joint_states` | `/robot_XXXXXX/joint_states` | sensor_msgs/JointState | ロボット → PC |
| `/tf` | `/tf`（変更なし・全台共通） | tf2_msgs/TFMessage | ロボット → PC |

### 1.2 TFのフレーム名

全台が同じ `odom → base_link` を出すとTFが衝突するので、フレーム名にも名前空間を付けました。

- `odom` → `robot_XXXXXX/odom`
- `base_link` → `robot_XXXXXX/base_link`

`tf` トピックは絶対名の `/tf` にしてあります。全ロボットのTFが1つのツリーにまとまるので、RVizで同時に表示できます。

関節名（`left_wheel_joint`、`right_wheel_joint`）は変えていません。`robot_state_publisher` の `frame_prefix` はリンク名にだけ付くので、関節名を変える必要はありません。

### 1.3 接続方法の選択（WiFi / USB）

`ros2robocar.h` の `USE_WIFI_TRANSPORT` で、エージェントとの接続方法を選びます。

```cpp
#define USE_WIFI_TRANSPORT  1   // 1: WiFi(UDP)接続  0: USBシリアル接続
```

| | WiFi（`1`） | USB（`0`） |
|---|---|---|
| 接続関数 | `connectWifiTransport()`（状態をシリアルに表示しながら接続） | `set_microros_transports()` |
| PC側のエージェント | 全台で1つ（`udp4`） | ロボットごとに1つ（`serial`） |
| シリアルモニタ表示 | あり（`LOG_PRINT` / `LOG_PRINTLN`） | なし（シリアルはmicro-ROSの通信に使うため） |
| 向いている用途 | 複数台の運用 | 1台でのデバッグ、WiFiが使えない環境 |

- WiFi接続では `WiFi.setSleep(false)` でWiFiの省電力機能を切り、通信の遅延を減らしています。
- シリアル表示には `Serial.print` を直接使わず、`LOG_PRINT` / `LOG_PRINTLN` を使います。USB接続ではこのマクロが何もしなくなるので、表示がmicro-ROSの通信を壊すことはありません。
- 名前空間による識別と自動再接続は、どちらの接続方法でも同じように動きます。

### 1.4 接続監視と自動再接続

ROS 2の処理を次のように分けました。

| 関数 | 呼ぶタイミング | 内容 |
|---|---|---|
| `ros2Setup()` | `setup()` で1回 | 名前空間の生成、メッセージ用メモリの確保（エージェントには接続しない） |
| `ros2CreateEntities()` | 接続するたび | node / publisher / subscriber / timer / executorの作成、時刻同期 |
| `ros2DestroyEntities()` | 切断するたび | 上で作ったものの解放 |
| `ros2Spin()` | `loop()` で毎回 | 接続状態の管理、executorのspin |

`ros2Spin()` は次の4つの状態を切り替えます。

```
 WAITING_AGENT ──(pingに応答あり)──▶ AGENT_AVAILABLE ──(作成に成功)──▶ AGENT_CONNECTED
       ▲                                  │ 作成に失敗                     │ 100ms × 3回応答なし
       │                                  ▼                                ▼
       └──────────────────────────── (解放) ◀──────────────────── AGENT_DISCONNECTED
                                                                  （ロボットを停止）
```

| 状態 | 動作 |
|---|---|
| `WAITING_AGENT` | 500msごとに `rmw_uros_ping_agent(100, 1)` でエージェントを探す |
| `AGENT_AVAILABLE` | `ros2CreateEntities()` を呼ぶ。失敗したら解放して待ちに戻る |
| `AGENT_CONNECTED` | 500msごとに `rmw_uros_ping_agent(100, 3)` で生存確認し、executorをspinする |
| `AGENT_DISCONNECTED` | `vf_d`・`dphi_d`・`whCmdL`・`whCmdR` を0にして停止し、解放して待ちに戻る |

### 1.5 そのほかの変更

- **起動時に止まらない**：エージェントがまだ起動していなくても `error_loop()` で止まらず、エージェントを待ち続けます。
- **spinのエラー処理**：executorのspinを `RCCHECK` から `RCSOFTCHECK` に変えました。通信エラーで止まらなくなります。
- **時刻同期**：これまでは `timer_callback` の中で10msごとに `rmw_uros_sync_session(1000)` を呼んでいました。これを接続時に1回だけ同期する形に変えました。同期のたびに処理が止まることがなくなり、WiFiの通信量も減ります。
- **odomの文字列長**：odomメッセージの `frame_id.size` と `child_frame_id.size` が設定されていなかったので、設定するようにしました。

---

## 2. 使い方

### 2.1 ロボット側（書き込み前の設定）

まず `ros2robocar.h` で接続方法を選びます。

```cpp
#define USE_WIFI_TRANSPORT  1   // 1: WiFi(UDP)接続  0: USBシリアル接続
```

WiFi接続（`1`）の場合は、`Prog_spiconhRos2Basic.ino` の先頭にあるWiFi設定を、実際の環境に合わせて書き換えます。USB接続（`0`）の場合、この設定は使いません。

```cpp
char wifi_ssid[]  = "your_ssid";         // WiFiのSSID
char wifi_pass[]  = "your_password";     // WiFiのパスワード
char agent_ip[]   = "192.168.1.100";     // micro_ros_agentを動かすPCのIPアドレス
const uint agent_port = 8888;            // agentのUDPポート（全台共通）
```

- 全台に**同じプログラム**を書き込みます。ロボットごとに書き換える箇所はありません。
- PCのIPアドレスが変わると全台を書き込み直すことになるので、ルーターのDHCP予約などでPCのIPを固定しておくことをおすすめします。
### 2.2 PC側（エージェントの起動）

**WiFi接続の場合**

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

- エージェントは1つで全台を受けられます。
- Dockerで起動する場合：
  ```bash
  docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
  ```
- ファイアウォールで **UDPの8888番** の受信を許可してください。
- WSL2でエージェントを動かす場合は、ミラーモード（`networkingMode=mirrored`）にするか、ポート転送の設定が必要です。

**USB接続の場合**

ロボットごとに、つながっているシリアルポートを指定してエージェントを起動します。

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200   # 1台目
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200   # 2台目
```

- WSL2では、`usbipd` でESP32のUSBをWSLに接続してから起動します。
- USB接続ではシリアルモニタを開かないでください。シリアルポートをエージェントが使っているためです。

### 2.3 ロボットの操作

まず、つながっているロボットのIDを確認します。

```bash
ros2 node list
# /robot_A1B2C3/mycar_node
# /robot_D4E5F6/mycar_node
```

ロボットを指定して動かします。

```bash
# 速度指令（robot_A1B2C3を前進させる）
ros2 topic pub /robot_A1B2C3/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# キーボードで操作する
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_A1B2C3/cmd_vel

# odomを確認する
ros2 topic echo /robot_A1B2C3/odom
```

### 2.4 RVizで表示する（URDFを使う場合）

ロボットごとに `robot_state_publisher` を起動します。名前空間とフレーム名のプレフィックスは、ロボットのIDに合わせます。

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -r __ns:=/robot_A1B2C3 \
  -p frame_prefix:=robot_A1B2C3/ \
  -p robot_description:="$(xacro mycar.urdf)"
```

RVizでは、Fixed Frameに `robot_A1B2C3/odom` などを指定します。全台を同じ座標系で表示したい場合は、共通の `map` フレームから各ロボットの `robot_XXXXXX/odom` へのstatic transformを追加してください。

---

## 3. 確認方法

### 3.1 シリアルモニタ（115200bps、WiFi接続の場合のみ）

WiFi接続でロボットを起動すると、シリアルモニタに次のように表示されます。USB接続では何も表示されないので、エージェントのログと `ros2 node list` で確認してください。

```
===== spiconH micro-ROS (WiFi) =====
WiFi connecting to "your_ssid" .....
WiFi connected
  SSID    : your_ssid
  IP      : 192.168.1.23
  Gateway : 192.168.1.1
  MAC     : 24:6F:28:A1:B2:C3
  RSSI    : -52 dBm
  Agent   : 192.168.1.100:8888
ROS namespace     : /robot_A1B2C3
waiting agent ...
agent connected
```

| 表示 | 意味 |
|---|---|
| `WiFi connecting to ...` のあと `.` が続く | WiFiに接続中 |
| `WiFi connect timeout  status: NO_SSID_AVAIL` | SSIDが見つからない（SSIDの綴り、2.4GHz帯かどうか、電波の届く範囲かを確認） |
| `WiFi connect timeout  status: CONNECT_FAILED` | 接続に失敗した（パスワードを確認） |
| `WiFi retry` | 10秒つながらなかったので再試行している |
| `WiFi connected` と接続情報 | WiFiに接続成功。`RSSI` が -75dBm より低い場合は電波が弱いので、通信が不安定になりやすい |
| `WiFi lost  status: ...` | 動作中にWiFiが切れた（ESP32が自動で再接続を試みる） |
| `WiFi reconnected` | WiFiに再接続できた |
| `waiting agent ...` のまま | WiFiにはつながったが、エージェントが見つからない（`Agent` に表示されたIPとポート、エージェントの起動、ファイアウォールを確認） |
| `agent connected` | 接続成功 |
| `agent disconnected, waiting ...` | エージェントとの接続が切れた（ロボットは停止し、自動で再接続を待つ） |

### 3.2 チェックリスト

| # | 手順 | 期待する結果 |
|---|---|---|
| 1 | エージェントを止めた状態でロボットを起動する | `waiting agent ...` のまま待つ（フリーズしない） |
| 2 | エージェントを起動する | 約1秒以内に `agent connected` と表示される |
| 3 | `ros2 node list` を実行する | `/robot_XXXXXX/mycar_node` がロボットの台数分表示される |
| 4 | `ros2 topic list` を実行する | ロボットごとに `/robot_XXXXXX/odom` などが表示される |
| 5 | 1台の `cmd_vel` に指令を送る | そのロボットだけが動く |
| 6 | `ros2 run tf2_tools view_frames` を実行する | `robot_XXXXXX/odom → robot_XXXXXX/base_link` がロボットの台数分出る |
| 7 | 走行中にエージェントを止める（Ctrl+C） | 約1秒でロボットが止まり、`agent disconnected` と表示される |
| 8 | エージェントを再起動する | ロボットをリセットしなくても `agent connected` に戻り、操作できる |
| 9 | ロボットのWiFiを一時的に遮断する（ルーターから離すなど） | 7・8と同じように停止し、自動で再接続する |

---

## 4. 注意点・制限

- **操縦ノードだけが落ちた場合は止まらない**：切断時に停止するのは、エージェントとの通信が切れた場合だけです。エージェントは動いていて、PC側の操縦ノードだけが落ちた場合は、最後に受け取った `cmd_vel` で走り続けます。
  - これも防ぎたい場合は、`cmd_vel` を一定時間（たとえば500ms）受信しなかったら停止する処理を追加してください。
- **時刻のずれ**：時刻同期は接続時の1回だけです。長時間動かすと時刻が少しずつずれていくので、必要であれば接続中に定期的に同期し直す処理を追加してください。
- **IDの重複**：IDはMACアドレスの末尾3バイトから作るので、実際に重複することはまずありません。重複した場合は `makeRobotNs()` で使うバイト数を増やしてください。
- **台数を増やす場合**：1台あたり `odom`・`/tf` を20Hz（`ODOM_PUB_PERIOD_MS = 50`ms）、`whValue`・`joint_states` を5Hz（`STATE_PUB_PERIOD_MS = 200`ms）で送っています。台数が多くてWiFiの帯域が足りない場合は、`ros2robocar.h` のこれらの値を大きくして送信周期を下げてください。
