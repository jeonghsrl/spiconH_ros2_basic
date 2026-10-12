//------  whPwm --> whCmdに変更
//-----  20230420 odom tf, time_syncを追加 
//-----  20230422 jointstate追加
//-----  20261008 複数台対応: MACアドレスから名前空間(robot_XXXXXX)を自動生成
//-----  20261008 rmw_uros_ping_agent()で接続監視、切断時に停止・自動再接続

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>              //timesync用

#include <std_msgs/msg/int32_multi_array.h>
#include <nav_msgs/msg/odometry.h>                  //added odometry
#include <geometry_msgs/msg/twist.h>                //added twist
#include <geometry_msgs/msg/transform_stamped.h>    //added odom_tf
#include <tf2_msgs/msg/tf_message.h>                //added odom_tf
#include <sensor_msgs/msg/joint_state.h>            //added jointstate 

#include "ros2robocar.h"
#include "EducarSpiconLib_alpha.1.h" 

int32_t whCmdL,whCmdR;           //左右車輪制御指令用変数          
float vf_d, dphi_d;              //ロボット速度目標指令変数             

//--左右車輪角度と角速度のpub用変数
rcl_publisher_t pub_whValue;                 //pub用構造体
std_msgs__msg__Int32MultiArray msg_whValue;  //pub用message変数 
rclc_executor_t exec_timer;                //実行体

//--odom topic pub用変数                      //added odometry
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
//rclc_executor_t exec_odom;

//--左右車輪pwmのsub用変数
rcl_subscription_t sub_whCmd;              //subscribe構造体  
std_msgs__msg__Int32MultiArray msg_whCmd;  //subscribe topic用message
rclc_executor_t exec_whCmd;

//--cmdvel topicのsub用変数  ----- added 
rcl_subscription_t sub_cmdVel;            //subscribe構造体  
geometry_msgs__msg__Twist msg_cmdVel;     //subscribe topic用message
rclc_executor_t exec_cmdVel;

//micro-ros サポート用
rclc_support_t support;    //サポート構造体(１個でいい）
rcl_allocator_t allocator; //動的メモリ管理用（１個でいい）
rcl_node_t node;           //node構造体  
rcl_timer_t timer;         //timer構造体

//--odom_tf用変数                        //added odom_tf
rcl_publisher_t pub_odom_tf;
tf2_msgs__msg__TFMessage * odom_tf;

//--joint_state pub用変数
rcl_publisher_t pub_jointState;
sensor_msgs__msg__JointState msg_jointState;

int64_t time_ns;           //Agentとのtime sync用時間変数
int64_t time_ms;

//--複数台識別用 (MACアドレス下位3byteから生成)
char robot_ns[16];         //名前空間  例: "robot_A1B2C3"
char frame_odom[32];       //odomフレーム名 例: "robot_A1B2C3/odom"
char frame_base[32];       //base_linkフレーム名 例: "robot_A1B2C3/base_link"

//--MACアドレスから名前空間とフレーム名を作る
void makeRobotNs(){
  uint64_t mac = ESP.getEfuseMac();          //MACアドレス(先頭byteが最下位に格納)
  snprintf(robot_ns, sizeof(robot_ns), "robot_%02X%02X%02X",
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));  //個体固有の下位3byte
  snprintf(frame_odom, sizeof(frame_odom), "%s/odom", robot_ns);
  snprintf(frame_base, sizeof(frame_base), "%s/base_link", robot_ns);
}

//---------------------- subscriberのcallback
void sub_whCmd_callback(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

   whCmdL = msg->data.data[0];
   whCmdR = msg->data.data[1];
}

void sub_cmdVel_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

   vf_d = msg->linear.x;
   dphi_d = msg->angular.z;
}
//--------------------------------------------

/////=======================SETUP================================
//--接続状態 (agentとの接続を監視して自動再接続する)
enum Ros2State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
Ros2State ros2_state = WAITING_AGENT;

#define RCRETURN(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

//--起動時に1回だけ呼ぶ (メッセージのメモリ確保など。agentには接続しない)
void ros2Setup(){
  allocator = rcl_get_default_allocator();                              //動的メモリ管理
  makeRobotNs();                                                        //名前空間の生成

  //------------------------------変数の初期化-------------------
  //--init msg_whValue (multiArrayは、これをしておかないとagentとうまく通信できない)
  msg_whValue.data.capacity =10;
  msg_whValue.data.size=4;
  msg_whValue.data.data=(int32_t*) malloc(msg_whValue.data.capacity * sizeof(int32_t));
  msg_whValue.data.data[0]=0;   msg_whValue.data.data[1]=0;   msg_whValue.data.data[2]=0;   msg_whValue.data.data[3]=0;

  //--init msg_whCam
  msg_whCmd.data.capacity =10;
  msg_whCmd.data.size=2;
  msg_whCmd.data.data=(int32_t*) malloc(msg_whCmd.data.capacity * sizeof(int32_t)); 
  msg_whCmd.data.data[0]=0;   msg_whCmd.data.data[1]=0;

  //--odom_tfの初期設定
  odom_tf = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&odom_tf->transforms, 1);
  odom_tf->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  memcpy(odom_tf->transforms.data[0].header.frame_id.data, frame_odom, strlen(frame_odom) + 1);
  odom_tf->transforms.data[0].header.frame_id.size = strlen(odom_tf->transforms.data[0].header.frame_id.data);
  odom_tf->transforms.data[0].header.frame_id.capacity = 100;
  odom_tf->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(odom_tf->transforms.data[0].child_frame_id.data, frame_base, strlen(frame_base) + 1);
  odom_tf->transforms.data[0].child_frame_id.size = strlen(odom_tf->transforms.data[0].child_frame_id.data);
  odom_tf->transforms.data[0].child_frame_id.capacity = 100;
  odom_tf->transforms.data[0].transform.translation.x = rb_getPx(); 
  odom_tf->transforms.data[0].transform.translation.y = rb_getPy();
  odom_tf->transforms.data[0].transform.translation.z = 0;     
  odom_tf->transforms.data[0].transform.rotation.x = 0;
  odom_tf->transforms.data[0].transform.rotation.y = 0;
  odom_tf->transforms.data[0].transform.rotation.z = sin(rb_getSteeringRadian()); 
  odom_tf->transforms.data[0].transform.rotation.w = cos(rb_getSteeringRadian()); 
  odom_tf->transforms.data[0].header.stamp.nanosec = micros();
  odom_tf->transforms.data[0].header.stamp.sec = millis()/1000;

  //--jointStateの初期設定 (これがないとうまくいかない) 
  msg_jointState.name.capacity= 10;      //memory size
  msg_jointState.name.data = (rosidl_runtime_c__String*) malloc(msg_jointState.name.capacity * sizeof(rosidl_runtime_c__String));  
  msg_jointState.name.size= 2;          //要素数(joint数) 
  msg_jointState.position.capacity= 10;
  msg_jointState.position.data = (double*) malloc(msg_jointState.position.capacity * sizeof(double));   
  msg_jointState.position.size= 2;
  msg_jointState.velocity.capacity= 10;
  msg_jointState.velocity.data = (double*) malloc(msg_jointState.velocity.capacity * sizeof(double));   
  msg_jointState.velocity.size= 2; 
  msg_jointState.effort.capacity= 10;
  msg_jointState.effort.data = (double*) malloc(msg_jointState.effort.capacity * sizeof(double));  
  msg_jointState.effort.size= 2; 
   

} //End of Setup

//--agentとの接続ごとに呼ぶ (node/publisher/subscriber/executorの生成)
bool ros2CreateEntities(){

 //--Nodeの初期設定
  RCRETURN(rclc_support_init(&support, 0, NULL, &allocator));            //create init_options 初期オプション設定 
  RCRETURN(rclc_node_init_default(&node, "mycar_node", robot_ns, &support)); //create node (/robot_XXXXXX/mycar_node)
 
 //--create publisher
 RCRETURN(rclc_publisher_init_default(&pub_whValue,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32MultiArray),"whValue")); //wheelState 
 RCRETURN(rclc_publisher_init_default(&pub_odom,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"odom"));              //odometry
 RCRETURN(rclc_publisher_init_default(&pub_odom_tf,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs,msg,TFMessage),"/tf"));           //odom_tf 全台共通の/tfに出す(フレーム名で区別)
 RCRETURN(rclc_publisher_init_default(&pub_jointState,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,JointState),"joint_states"));  //jointstate 
   
  //--create timer_callback for publisher
  const unsigned int timer_timeout = 10;    //publisher topicの配信時間(ms)
  RCRETURN(rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback));

  //--create subscriber 
  RCRETURN(rclc_subscription_init_default(&sub_whCmd,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),"whCmd"));
  RCRETURN(rclc_subscription_init_default(&sub_cmdVel,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"cmd_vel")); //added cmdvel
   
  //--reate executor
  RCRETURN(rclc_executor_init(&exec_timer, &support.context, 1, &allocator)); //for publiser timer callback
  RCRETURN(rclc_executor_add_timer(&exec_timer, &timer));                     //
  RCRETURN(rclc_executor_init(&exec_whCmd, &support.context, 1, &allocator));   //for exec_whCmd
  RCRETURN(rclc_executor_add_subscription(&exec_whCmd, &sub_whCmd, &msg_whCmd, &sub_whCmd_callback, ON_NEW_DATA)); //
  RCRETURN(rclc_executor_init(&exec_cmdVel, &support.context, 1, &allocator));  //for exec_cmdVel
  RCRETURN(rclc_executor_add_subscription(&exec_cmdVel, &sub_cmdVel, &msg_cmdVel, &sub_cmdVel_callback, ON_NEW_DATA));//

  //--agentと時刻同期 (接続時に1回)
  rmw_uros_sync_session(1000);

  RCSOFTCHECK(rcl_publish(&pub_odom_tf, odom_tf, NULL));
  return true;
}

//--切断時に呼ぶ (生成したものを解放)
void ros2DestroyEntities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);  //agentがいないので応答を待たない

  RCSOFTCHECK(rcl_publisher_fini(&pub_whValue, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_odom, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_odom_tf, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_jointState, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sub_whCmd, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sub_cmdVel, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&exec_timer));
  RCSOFTCHECK(rclc_executor_fini(&exec_whCmd));
  RCSOFTCHECK(rclc_executor_fini(&exec_cmdVel));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

//--loop()から呼ぶ。接続監視・再接続・executorのspin
void ros2Spin(){
  static unsigned long last_ping = 0;

  switch(ros2_state){
    case WAITING_AGENT:                                   //agentを探す (500ms毎にping)
      if(millis() - last_ping > 500){
        last_ping = millis();
        if(rmw_uros_ping_agent(100, 1) == RMW_RET_OK){
          ros2_state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:                                 //agentが見つかったら接続
      if(ros2CreateEntities()){
        ros2_state = AGENT_CONNECTED;
        LOG_PRINTLN("agent connected");
      }else{
        ros2DestroyEntities();
        ros2_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:                                 //接続中: 500ms毎に生存確認
      if(millis() - last_ping > 500){
        last_ping = millis();
        if(rmw_uros_ping_agent(100, 3) != RMW_RET_OK){    //100ms x 3回応答なしで切断と判断
          ros2_state = AGENT_DISCONNECTED;
          break;
        }
      }
      RCSOFTCHECK(rclc_executor_spin_some(&exec_timer, RCL_MS_TO_NS(10)));  //publisherのためのtimer
      RCSOFTCHECK(rclc_executor_spin_some(&exec_whCmd, RCL_MS_TO_NS(100)));
      RCSOFTCHECK(rclc_executor_spin_some(&exec_cmdVel, RCL_MS_TO_NS(100)));
      break;

    case AGENT_DISCONNECTED:                              //切断: ロボットを止めて解放し、再接続待ちへ
      vf_d = 0;  dphi_d = 0;                              //安全のため停止指令
      whCmdL = 0;  whCmdR = 0;
      ros2DestroyEntities();
      ros2_state = WAITING_AGENT;
      LOG_PRINTLN("agent disconnected, waiting ...");
      break;
  }
}


//---------
void error_loop(){
  while(1){    
    delay(100);
  }
}

//-- publisherのtimer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  //-- agentとのtime sync (同期は接続時にros2CreateEntities()で実施)
  if(rmw_uros_epoch_synchronized()){
    time_ms = rmw_uros_epoch_millis();
    time_ns = rmw_uros_epoch_nanos();  
   }

  //--whValue update
  msg_whValue.data.data[0] = wh_getAngleL();                   //角度計算関数の実行
  msg_whValue.data.data[2] = wh_getAngVelL();               //角速度計算関数の実行
  msg_whValue.data.data[1] = wh_getAngleR();                   //角度計算関数の実行
  msg_whValue.data.data[3] = wh_getAngVelR();               //角速度計算関数の実行

  //--odom updatevoi
  msg_odom.header.frame_id.data = frame_odom;
  msg_odom.header.frame_id.size = strlen(frame_odom);
  msg_odom.child_frame_id.data = frame_base;
  msg_odom.child_frame_id.size = strlen(frame_base);
  msg_odom.pose.pose.position.x = rb_getPx();
  msg_odom.pose.pose.position.y = rb_getPy();
  msg_odom.pose.pose.position.z = 0;      
  msg_odom.pose.pose.orientation.x=0;  
  msg_odom.pose.pose.orientation.y=0; 
  msg_odom.pose.pose.orientation.z=sin(rb_getSteeringRadian()); 
  msg_odom.pose.pose.orientation.w=cos(rb_getSteeringRadian()); 
  msg_odom.twist.twist.linear.x = rb_getFowardVelocity(); 
  msg_odom.twist.twist.angular.z = rb_getSteeringVelocity(); 
  msg_odom.header.stamp.nanosec = time_ns;
  msg_odom.header.stamp.sec = time_ms/1000.; 
   
  //--odom_tf update
  odom_tf->transforms.data[0].transform.translation.x = rb_getPx(); 
  odom_tf->transforms.data[0].transform.translation.y = rb_getPy();
  odom_tf->transforms.data[0].transform.translation.z = 0;     
  odom_tf->transforms.data[0].transform.rotation.x = 0;
  odom_tf->transforms.data[0].transform.rotation.y = 0;
  odom_tf->transforms.data[0].transform.rotation.z = sin(rb_getSteeringRadian()); 
  odom_tf->transforms.data[0].transform.rotation.w = cos(rb_getSteeringRadian()); 
  odom_tf->transforms.data[0].header.stamp.nanosec = time_ns; //micros();
  odom_tf->transforms.data[0].header.stamp.sec = time_ms/1000.;   //millis()/1000;
   
  //--jointState update
  msg_jointState.header.stamp.nanosec = time_ns;   //micros();
  msg_jointState.header.stamp.sec = time_ms/1000.; //milis()/1000;
  msg_jointState.name.data[0].data = "left_wheel_joint";      //joint name, URDFのjoint名と一致させる  
  msg_jointState.name.data[1].data = "right_wheel_joint";     
  msg_jointState.position.data[0]= wh_getAngleL()*Deg2Rad; //角度
  msg_jointState.position.data[1]= wh_getAngleR()*Deg2Rad;
  msg_jointState.velocity.data[0]= wh_getAngVelL()*Deg2Rad; //速度
  msg_jointState.velocity.data[1]= wh_getAngVelR()*Deg2Rad;
  msg_jointState.effort.data[0]= 0.0;
  msg_jointState.effort.data[1]= 0.0;
  
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&pub_whValue, &msg_whValue, NULL));
    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));   //added odom
    RCSOFTCHECK(rcl_publish(&pub_odom_tf, odom_tf, NULL));   //added odom_tf
    RCSOFTCHECK(rcl_publish(&pub_jointState, &msg_jointState, NULL));   //jointState
  }

}
