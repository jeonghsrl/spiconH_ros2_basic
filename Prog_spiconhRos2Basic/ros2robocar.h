
#ifndef Ros2robocar_h
#define Ros2robocar_h

//============ agentとの接続方法の選択 ============
//  1: WiFi(UDP)接続   (WiFi設定は Prog_spiconhRos2Basic.ino で変更)
//  0: USBシリアル接続
#define USE_WIFI_TRANSPORT  1

//--デバッグ表示 (USB接続時はSerialをmicro-ROS通信に使うため表示しない)
#if USE_WIFI_TRANSPORT
  #define LOG_PRINT(x)    Serial.print(x)
  #define LOG_PRINTLN(x)  Serial.println(x)
#else
  #define LOG_PRINT(x)
  #define LOG_PRINTLN(x)
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void ros2Setup();             //起動時に1回 (メッセージ初期化)
void ros2Spin();              //loop()から毎回呼ぶ (接続監視・自動再接続・spin)
bool ros2CreateEntities();    //agent接続時の生成
void ros2DestroyEntities();   //切断時の解放
void error_loop();

//-- publisherの配信周期(ms)
#define ODOM_PUB_PERIOD_MS   50     //odom, /tf           : 20Hz
#define STATE_PUB_PERIOD_MS  200    //whValue, joint_states: 5Hz

//-- publisherのtimer callback
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time);   //odom, /tf
void state_timer_callback(rcl_timer_t * timer, int64_t last_call_time);  //whValue, joint_states

//-- subscriberのcallback
void sub_whCmd_callback(const void * msgin);
void sub_cmdVel_callback(const void * msgin);

#endif
