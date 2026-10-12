#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <Ticker.h>
#include "ros2robocar.h"              
#include "EducarSpiconLib_alpha.1.h" 

void timerloop();                   //タイマー割込み timerloop関数
Ticker timerLoop; 

//--左右車輪角度と角速度のpubliser用変数
extern std_msgs__msg__Int32MultiArray msg_whValue;
extern rclc_executor_t exec_timer;


//--左右車輪pwmのsubscribe用変数
extern std_msgs__msg__Int32MultiArray msg_whCmd;  //subscribe topic用message
extern rclc_executor_t exec_whCmd;
extern rclc_executor_t exec_cmdVel;

extern int32_t whCmdL,whCmdR;            //車輪コマンド用変数
extern float vf_d, dphi_d;

extern char robot_ns[];                  //MACアドレスから生成した名前空間

//--接続方法(USB/WiFi)の選択は ros2robocar.h の USE_WIFI_TRANSPORT で行う
#if USE_WIFI_TRANSPORT
//--WiFi接続設定 (環境に合わせて変更)
char wifi_ssid[]  = "YOUR_SSID";         //WiFiのSSID
char wifi_pass[]  = "YOUR_PASSWORD";     //WiFiのパスワード
char agent_ip[]   = "192.168.11.2";     //micro_ros_agentを動かすPCのIPアドレス
const uint agent_port = 8888;            //agentのUDPポート (全台共通でよい)

//--WiFi状態を文字列にする (シリアル表示用)
const char* wifiStatusStr(wl_status_t st){
  switch(st){
    case WL_CONNECTED:       return "CONNECTED";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (SSIDが見つからない)";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (パスワード違いなど)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    case WL_IDLE_STATUS:     return "IDLE";
    default:                 return "UNKNOWN";
  }
}

//--WiFi接続情報の表示
void printWifiInfo(){
  LOG_PRINT("  SSID    : "); LOG_PRINTLN(WiFi.SSID());
  LOG_PRINT("  IP      : "); LOG_PRINTLN(WiFi.localIP());
  LOG_PRINT("  Gateway : "); LOG_PRINTLN(WiFi.gatewayIP());
  LOG_PRINT("  MAC     : "); LOG_PRINTLN(WiFi.macAddress());
  LOG_PRINT("  RSSI    : "); LOG_PRINT(WiFi.RSSI()); LOG_PRINTLN(" dBm");
  LOG_PRINT("  Agent   : "); LOG_PRINT(agent_ip); LOG_PRINT(":"); LOG_PRINTLN(agent_port);
}

//--状態を表示しながらWiFiに接続し、micro-ROSのWiFi(UDP)通信を設定する
//  (set_microros_wifi_transports()は接続完了まで無表示で待つため、同じ処理を表示付きで行う)
void connectWifiTransport(){
  LOG_PRINT("WiFi connecting to \""); LOG_PRINT(wifi_ssid); LOG_PRINT("\" ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);

  unsigned long t0 = millis();
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    LOG_PRINT(".");
    if(millis() - t0 > 10000){                     //10秒つながらなければ状態を表示して再試行
      LOG_PRINTLN("");
      LOG_PRINT("WiFi connect timeout  status: "); LOG_PRINTLN(wifiStatusStr(WiFi.status()));
      LOG_PRINT("WiFi retry ");
      WiFi.disconnect();
      WiFi.begin(wifi_ssid, wifi_pass);
      t0 = millis();
    }
  }
  WiFi.setSleep(false);                            //WiFi省電力を切って通信遅延を減らす
  LOG_PRINTLN("");
  LOG_PRINTLN("WiFi connected");
  printWifiInfo();

  //--micro-ROSのWiFi(UDP)通信設定 (set_microros_wifi_transports()と同じ内容)
  static struct micro_ros_agent_locator locator;
  locator.address.fromString(agent_ip);
  locator.port = agent_port;
  rmw_uros_set_custom_transport(false, (void *) &locator,
    arduino_wifi_transport_open, arduino_wifi_transport_close,
    arduino_wifi_transport_write, arduino_wifi_transport_read);
}

//--動作中のWiFi状態変化を表示 (loop()から呼ぶ)
void monitorWifi(){
  static wl_status_t last_st = WL_CONNECTED;
  wl_status_t st = WiFi.status();
  if(st != last_st){
    if(st == WL_CONNECTED){
      LOG_PRINTLN("WiFi reconnected");
      printWifiInfo();
    }else{
      LOG_PRINT("WiFi lost  status: "); LOG_PRINTLN(wifiStatusStr(st));   //ESP32が自動で再接続を試みる
    }
    last_st = st;
  }
}
#endif

//-------------------------------------------------------------

void setup() {

#if USE_WIFI_TRANSPORT
  Serial.begin(115200);                  //WiFi接続時はSerialをデバッグ表示に使える
  delay(500);                            //シリアルモニタの準備待ち
  LOG_PRINTLN("");
  LOG_PRINTLN("===== spiconH micro-ROS (WiFi) =====");
  connectWifiTransport();                //WiFi接続(状態をシリアル表示) + micro-ROSのWiFi(UDP)設定
  delay(2000);
#else
  set_microros_transports();             //usbでagentに接続する場合 (Serialはmicro-ROSが使用)
  delay(2000);
#endif

  //ロボット制御用セットアップ関数
  robotSetup();
  ros2Setup();                           //agentへの接続はloop()内のros2Spin()で行う(自動再接続)

  LOG_PRINT("ROS namespace     : /"); LOG_PRINTLN(robot_ns);
  LOG_PRINTLN("waiting agent ...");  //"agent connected"が出ない場合はagent_ip/ポート/agent起動を確認

  timerLoop.attach_ms(SAMP_TIMEms,timerloop);                   //timerloop関数をSAMP_TIMEmsで設定
  
}

void loop() {
   
  delay(10);
#if USE_WIFI_TRANSPORT
  monitorWifi();                         //WiFi切断・再接続をシリアル表示
#endif
  ros2Spin();                            //agent接続監視・再接続・executorのspin
 }

void timerloop(){
  
// double time_ms = millis();               //プログラムの実行時間を計測
// double time_us = micros();

 //-------------------
  wh_calAngleL();
  wh_calAngleR();
  wh_calAngVelL();
  wh_calAngVelR();
  rb_calFowardVelocity();
  rb_calSteeringVelocity();
  rb_calFowardDistance(); 
  rb_calSteeringRadian(); 
  rb_calVx(); 
  rb_calVy(); 
  rb_calPx();
  rb_calPy();
//---------------------

 rb_calDiffWheelGoalAngularVelocity(vf_d, dphi_d); //vf_dとdphi_dで目標車輪角速度を計算

 float wh_angvelL_d = wh_getGoalAngVelL()*Rad2Deg;
 float wh_angvelR_d = wh_getGoalAngVelR()*Rad2Deg;

 float wh_angL_d = wh_calConstVelTrajectoryL(wh_angvelL_d);    //f_angvelで定速走行する目標角度d_ang計算
 float wh_angR_d = wh_calConstVelTrajectoryR(wh_angvelR_d);

 //------------------------------
 int wh_dutyL=wh_pidAngleL(wh_angL_d, 5, 0.1, 0);                  //d_angで走行する制御入力計算
 int wh_dutyR=wh_pidAngleR(wh_angR_d, 5, 0.1, 0);
   
  wh_motorOutputL(wh_dutyL);                //左車輪モータへpwm出力
  wh_motorOutputR(wh_dutyR);                //右車輪モータへpwm出力
//-------------------------------- 
 
}

//////////////////////////////
