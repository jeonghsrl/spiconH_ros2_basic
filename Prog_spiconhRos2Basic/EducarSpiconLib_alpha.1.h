/*
EducarSpiconLib.h               
                              2024.04.13   jeong
- alpha.1: mobilerobot_beta.0.4からコピー  20240413 
      
*/

#ifndef EducarSpiconLib_h 
#define EducarSpiconLib_h  

//-------------グルーバル定義(プログラム内で値が変わらないものを定義しておく）

#define SAMP_TIMEms 20                          //タイマー割込み時間
#define Disp_TIMEms 100                         //状態displayタイマー割り込み
#define whR   0.0275             //0.055/2.          //車輪半径[m]
#define bwh   0.055              //0.110/2.          //車体中心から車輪接地点までの距離[m] -------- (14から修正)
#define moLdir   1               //motor  L direction
#define encLdir  -1              //encoder L direction 
#define moRdir   -1              //motor  R direction   
#define encRdir  -1              //encoder R direction
#define dutyMax 50     //車輪用dutyの最大値


//pwm発生用のタイマーチャンネル用定義
#define LEDC_CHANNEL_0 0       //左車輪用pwmA チャンネル0
#define LEDC_CHANNEL_1 1       //右車輪用pwmB チャンネル1
#define LEDC_TIMER_BIT 13      //LEDC PWMタイマー 13bit
#define LEDC_SERVO_FREQ 2500   //pwm 周波数2.5khz

//モータドライバ制御用PIN番号の定義
#define PWMA_PIN 25    //左車輪用PWM出力用IO pin番号    
#define AIN2_PIN 26    //左車輪用方向制御用IO pin番号
#define AIN1_PIN 27    //左車輪用方向制御用IO pin番号
#define BIN2_PIN 12    //右車輪用方向制御用IO pin番号
#define BIN1_PIN 14    //右車輪用方向制御用IO pin番号
#define PWMB_PIN 13    //右車輪用PWM出力用IO pin番号


//エンコーダ読み込み用PIN番号の定義
#define EncL_A_PIN 19   // 左車輪用Encoder A相入力ピン
#define EncL_B_PIN 23   // 左車輪用Encoder B相入力ピン
#define EncR_A_PIN 5    // 右車輪用Encoder A相入力ピン
#define EncR_B_PIN 18   // 右車輪用Encoder B相入力ピン　　 
#define Cnt2Ang    0.375  //カウンター値を角度に変換 角度= CNT*(360度/1回転時のカウンター値） 
#define Deg2Rad  0.0174          //度をradianに変換
#define Rad2Deg  57.32           //radianを度に変換  

// ＊＊＊MPU-6050(IMUセンサ)のアドレス、レジスタ設定値＊＊＊(230712_OG)
#define MPU6050_ADDRESS      0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b


//---- Wheel Struct
typedef struct __wheel_type{  
  float newAng;              //車輪現在角度(deg)         
  float newAngVel;           //車輪現在角速度(deg/s)         
  float ang_d;               //車輪目標角度(deg)
  float angVel_d;            //車輪目標角速度(deg/s)  
  float duty;                //車輪駆動用duty比
  float ang_g;               //車輪最終目標角度(deg)
  float angVel_g;            //車輪最終目標角速度(deg/s)                                                          
}Wheel;

//---- Robot Struct
typedef struct __diffwheelrobot_type{
  float pf;                 //ロボット前方移動距離(m) 
  float vf;                 //ロボット前方移動速度(m/s)
  float px;                 //ロボットX位置(m)
  float vx;                 //ロボットX方向速度(m/s)
  float py;                 //ロボットy位置(m)
  float vy;                 //ロボットy方向速度(m/s)
  float phi;                //ロボット旋回角度(rad)
  float dphi;               //ロボット旋回角速度(rad/s)
  float vf_d;               //ロボット目標前方移動速度(m/s)
  float dphi_d;             //ロボット目標旋回速度(rad/s)                                             
}DiffWheelRobot;               

//---- IMU Struct(230712_OG)
typedef struct __IMU_type{
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, Temperature;  //加速度、角速度（生データ）
  float ax, ay, az, gx, gy, gz;                                         //加速度,角速度
  float ax_m_ave, ay_m_ave, az_m_ave, gx_m_ave, gy_m_ave, gz_m_ave;     //加速度、角速度（移動平均処理）
  float offset_x, offset_y, offset_z;                                   //gyroのX,Y,Z軸のオフセット値
  float angle_pitch, angle_roll, angle_yaw;                             //姿勢角                                               
  float interval, preInterval;
  float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
  double acc_angle_x, acc_angle_y;
}IMU;


/* 
//---- IMU Struct(230712_OG)
typedef struct __IMU_type{
  int16_t acc_raw[3];     //加速度x,y,z
  int16_t gyro_raw[3];    //角速度x,y,z  (rpy)
  float acc_ave[3];       //
  float gyro_ave[3];      //加速度、角速度（移動平均処理）
  int16_t Temperature;   
  float acc[3];
  float gyro[3];
  float gyro_offset[3];                                   //gyroのX,Y,Z軸のオフセット値
  float roll;
  float pitch;
  float yaw;                                            
  float interval, preInterval;
  float gyro_angle[3];
  double acc_angle[3];
}IMU;
 */
 
//--------------------------関数定義

void robotSetup();                  //ロボットのセットアップ関数
//車輪用
void IRAM_ATTR ISR();               //外部割込みcallback関数 
void wh_motorOutputL(int mDuty);    //左モータドライバにduty比を与える関数
void wh_motorOutputR(int mDuty);    //右モータドライバにduty値を与える関数
void wh_calAngleL();                //左車輪角度計算
void wh_calAngleR();                //右車輪角度計算
void wh_calAngVelL();               //左車輪角速度計算
void wh_calAngVelR();               //右車輪角速度計算
//インタフェース
float wh_getAngleL();               //左車輪角度取得
float wh_getAngleR();               //右車輪角度取得
float wh_getAngVelL();              //左車輪角速度取得
float wh_getAngVelR();              //右車輪角速度取得
float wh_getGoalAngVelL();          //左車輪目標角速度取得
float wh_getGoalAngVelR();          //右車輪目標角速度取得
//制御用
int wh_pidAngleL(float angle_d, float p, float d, float i);   //左車輪角度制御用pid関数
int wh_pidAngleR(float angle_d, float p, float d, float i);   //右車輪角度制御用pid関数
int wh_piAngVelL(float angVel_d, float p, float i);           //左車輪角速度制御用pi関数
int wh_piAngVelR(float angVel_d, float p, float i);           //右車輪角速度制御用pi関数
//軌道用
float wh_calConstVelTrajectoryL(float vel_f);   //車輪定速制御用目標角度生成関数
float wh_calConstVelTrajectoryR(float vel_f);   //車輪定速制御用目標角度生成関数

//移動ロボット用
void rb_calFowardVelocity();                    //左右車輪角速度から直進速度を計算
void rb_calSteeringVelocity();                  //左右車輪角速度から旋回速度を計算
float rb_getFowardVelocity();                   //左右車輪角速度から直進速度を取得
float rb_getSteeringVelocity();                 //左右車輪角速度から旋回速度を取得
void rb_calFowardDistance();                    //直進距離を計算
void rb_calSteeringRadian();                    //旋回角度を計算
float rb_getFowardDistance();                   //直進距離を取得
float rb_getSteeringRadian();                   //旋回角度を取得
void rb_calVx();                               //前方速度と旋回角速度から、x方向の速度を計算
void rb_calVy();                               //前方速度と旋回角速度から、y方向の速度を計算
float rb_getVy();                              //x方向の速度を取得
float rb_getVx();                              //y方向の速度を取得
void rb_calPx();                               //x位置計算
void rb_calPy();                               //y位置計算
float rb_getPx();                              //x位置取得 
float rb_getPy();                              //y位置取得

//移動ロボット軌道用
void rb_calDiffWheelGoalAngularVelocity(float vf_d_val, float omega_d_val); //目標前方速度及び旋回角速度から各車輪の目標角速度を計算


//センサ用関数     
void sen_writeMPU6050(byte reg, byte data);   //IMUセンサ関連(230712_OG)
byte sen_readMPU6050(byte reg);
void sen_cal_IMU();
void sen_calibration_gyro();
float sen_get_ax();
float sen_get_ay();
float sen_get_az();
float sen_get_gx();
float sen_get_gy();
float sen_get_gz();
void sen_cal_angle();
float sen_get_angle_pitch();
float sen_get_angle_roll();
float sen_get_angle_yaw();

#endif
