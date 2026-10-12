/*
EducarSpiconLib.cpp
- alpha.0.1: mobilerobot_beta.0.4からコピー 20240413                         
*/

#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード
#include <Wire.h>                  //(230712_OG)
#include "EducarSpiconLib_alpha.1.h"  
   
RotaryEncoder encoderL(EncL_A_PIN, EncL_B_PIN);   //IO19とIO23を左車輪カウンター入力用として設定
RotaryEncoder encoderR(EncR_B_PIN, EncR_A_PIN);   //IO5とIO18を右車輪　　”　（左車輪と逆方向）

//--added Struct 宣言
Wheel lwh,rwh;                 //車輪構造体
DiffWheelRobot dwr;            //差動車輪ロボット構造体
IMU imu;                       //IMUセンサ構造体(230712_OG)

//----------robotのGIO等の設定
void robotSetup(){
  
  //--【PIN mode（機能）の設定】　
  pinMode(PWMA_PIN,OUTPUT);    //PWMA_PINをpwm用のOUTPUTに設定
  pinMode(AIN1_PIN,OUTPUT);    //AIN1_PINを回転方向用のOUTPUTに設定
  pinMode(AIN2_PIN,OUTPUT);    //AIN1_PINを回転方向用のOUTPUTに設定 
  pinMode(PWMB_PIN,OUTPUT);    //PWMB_PINをpwm用のOUTPUTに設定
  pinMode(BIN1_PIN,OUTPUT);    //BIN1_PINを回転方向用のOUTPUTに設定
  pinMode(BIN2_PIN,OUTPUT);    //BIN2_PINを回転方向用のOUTPUTに設定
 

  //--【LEDCの設定】
  ledcSetup(LEDC_CHANNEL_0, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //左車輪用LEDC_CHANNEL_0の設定
  ledcSetup(LEDC_CHANNEL_1, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //右車輪用LEDC_CHANNEL_1の設定

  //--【LEDCとpwm用pinの接続】
  ledcAttachPin(PWMA_PIN, LEDC_CHANNEL_0);      //LEDC_CHANNEL_0をPWMA_PINへ出力 
  ledcAttachPin(PWMB_PIN, LEDC_CHANNEL_1);      //LEDC_CHANNEL_1をPWMB_PINへ出力

   //--【Encoder用pinを外部interruptに設定】
  attachInterrupt(EncL_A_PIN,ISR,CHANGE);   //IO19番ピンを外部割込み用として設定
  attachInterrupt(EncL_B_PIN,ISR,CHANGE);   //IO23番ピンを外部割込み用として設定
  attachInterrupt(EncR_A_PIN,ISR,CHANGE);   //IO5番ピンを外部割込み用として設定
  attachInterrupt(EncR_B_PIN,ISR,CHANGE);   //IO18番ピンを外部割込み用として設定  
/*
  //--【MPU6050に設定を書き込む】(230712_OG)
  Wire.begin();
  sen_writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  sen_writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  sen_writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: ±250dps, LSB: 131 deg/s
  sen_writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g, LSB: 16384 LSB/g
  sen_writeMPU6050(MPU6050_PWR_MGMT_1, 0x00);   // disable sleep mode, Internal 8MHz os  
  //正常に接続されているかの確認
  if (sen_readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
    
//  sen_calibration_gyro();                       //キャリブレーションを実行
*/
}

///////////////////////////////////////////////

/*--------------------------
// vx: フォワード速度 (m/s)
// omega: 旋回速度    (rad/s)
// 説明：対向２輪ロボットの逆運動学。目標vf,dphiを用いて各車輪の目標角速度を求める。
--------------------------------*/
void rb_calDiffWheelGoalAngularVelocity(float vf_d_val, float omega_d_val){

   lwh.angVel_d = (vf_d_val - bwh * omega_d_val) / whR; 
   rwh.angVel_d = (vf_d_val + bwh * omega_d_val) / whR; 
 
}

//interface
float wh_getGoalAngVelL(){return lwh.angVel_d;}
float wh_getGoalAngVelR(){return rwh.angVel_d;}

/*------------------------------------------
 * void calPx(), calPy()
 * 返り値：なし
 * 説明：ロボットのx,y速度を積分して位置を求める
-------------------------------------------- */
void rb_calPx(){ dwr.px += dwr.vx * (SAMP_TIMEms*0.001); } 
void rb_calPy(){ dwr.py += dwr.vy * (SAMP_TIMEms*0.001); } 
//interface 返り値: float
float rb_getPx(){return dwr.px;}
float rb_getPy(){return dwr.py;}

/*------------------------------------------
 * void calVx(), calVy()
 * 返り値：なし
 * 説明：直進速度及び旋回速度からロボットのx,y速度を求める
-------------------------------------------- */
void rb_calVx(){ dwr.vx = dwr.vf*cos(dwr.phi); } 
void rb_calVy(){ dwr.vy = dwr.vf*sin(dwr.phi); } 
//interface 返り値: float
float rb_getVx(){return dwr.vx;}
float rb_getVy(){return dwr.vy;}


/*------------------------------------------
 * void calFowardDistance(), calSteeringRadian()
 * 返り値：なし
 * 説明：直進速度及び旋回速度から直進距離と旋回角度を求める
-------------------------------------------- */
void rb_calFowardDistance(){ dwr.pf += dwr.vf * (SAMP_TIMEms*0.001); } 
void rb_calSteeringRadian(){ dwr.phi += dwr.dphi * (SAMP_TIMEms*0.001);} 
//interface 返り値: float
float rb_getFowardDistance(){return dwr.pf;}
float rb_getSteeringRadian(){return dwr.phi;}


/*------------------------------------------
 * void calFowardVelocity, calSteeringVelocity
 * 返り値：なし
 * 説明：左右車輪角速度から直進速度と旋回速度を求める。
-------------------------------------------- */
void rb_calFowardVelocity(){ dwr.vf = whR * ( lwh.newAngVel*Deg2Rad + rwh.newAngVel*Deg2Rad)/2.; } 
void rb_calSteeringVelocity(){ dwr.dphi = whR * (rwh.newAngVel*Deg2Rad - lwh.newAngVel*Deg2Rad)/(2*bwh);}  //--14から修正
//interface 返り値: float
float rb_getFowardVelocity(){return dwr.vf;}
float rb_getSteeringVelocity(){return dwr.dphi;}


/*------------------------------------------
 * float calConstVelTrajectoryL
 * 引数：double vel_f       目標角速度(deg/s)
 * 返り値：float angle_d     目標角度(deg)
 * 説明：左右車輪を定速で回転させるための目標角度を計算
-------------------------------------------- */
float wh_calConstVelTrajectoryL(float vel_f){  
   static float angle_d=0;
   angle_d += vel_f*(SAMP_TIMEms*0.001);   //目標角速度を達成するための、目標角度
   return angle_d;
}

float wh_calConstVelTrajectoryR(float vel_f){
   static float angle_d=0;
   angle_d += vel_f*(SAMP_TIMEms*0.001);   //目標角速度を達成するための、目標角度
   return angle_d;
}



/*------------------------------------------
 * float PIDangleL() PIDangleR()
 * 引数：float angle_d  目標角度(deg)
 * 返り値：int duty      制御入力(%)
 * 説明：左右車輪の角度のをPID制御し、制御入力を返す
-------------------------------------------- */
int wh_pidAngleL(float angle_d, float p, float d, float i){
  
  float K_p = p;       //Pゲイン(目標角度と現在角度との差分にかけるゲイン）
  float K_d = d;       //Dゲイン(現在の速度にかけるゲイン）
  float K_i = i;       //Iゲイン（目標角度と現在角度の差分の積分にかけるゲイン）
  static float err = 0; 
 
  float duty =  K_p*(angle_d-lwh.newAng) - K_d*lwh.newAngVel + K_i*err;

  err = err + (angle_d-lwh.newAng);         //I項の誤差積分

   return (int)duty;
}

int wh_pidAngleR(float angle_d, float p,float d, float i){
  
  float K_p = p;       //Pゲイン(目標角度と現在角度との差分にかけるゲイン）
  float K_d = d;       //Dゲイン(現在の速度にかけるゲイン）
  float K_i = i;       //Iゲイン（目標角度と現在角度の差分の積分にかけるゲイン）
  static float err = 0; 
  
  float duty =  K_p*(angle_d-rwh.newAng)-K_d*rwh.newAngVel+K_i*err;

  err = err +(angle_d-rwh.newAng);         //I項の誤差積分

   return (int)duty;
}


/*-------------beta0.4で追加--------------------------
 * float wh_pidAngVelL() wh_pidAngVelL()
 * 引数：float angVel_d  目標角速度(deg/s)
 * 返り値：int duty      制御入力(%)
 * 説明：左右車輪の角速度をPI制御し、制御入力を返す
-------------------------------------------- */
int wh_piAngVelL(float angVel_d, float p, float i){
  
  float K_p = p;       //Pゲイン(目標角度と現在角度との差分にかけるゲイン）
  float K_i = i;       //Iゲイン（目標角度と現在角度の差分の積分にかけるゲイン）
  static float err = 0; 
 
  float duty =  K_p*(angVel_d-lwh.newAngVel) + K_i*err;

  err = err + (angVel_d-lwh.newAngVel);         //I項の誤差積分

   return (int)duty;
}

int wh_piAngVelR(float angVel_d, float p, float i){
  
  float K_p = p;       //Pゲイン(目標角度と現在角度との差分にかけるゲイン）
  float K_i = i;       //Iゲイン（目標角度と現在角度の差分の積分にかけるゲイン）
  static float err = 0; 
 
  float duty =  K_p*(angVel_d-rwh.newAngVel) + K_i*err;

  err = err + (angVel_d-rwh.newAngVel);         //I項の誤差積分

   return (int)duty;
}



/*------------------------------------------
 * float calAngleL() 
 * 引数：なし
 * 返り値：angle
 * 左車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
void wh_calAngleL(){
  
  //--【counter値の読み込み】  
  int newCnt = encLdir*encoderL.getPosition();  //左車輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  float angle = newCnt*Cnt2Ang;       //左車輪の角度を計算
    
  lwh.newAng = angle;
}

float wh_getAngleL(){ return lwh.newAng ; }    //interface関数

/*------------------------------------------
 * float getAngleR()
 * 引数：なし
 * 返り値：angle
 * 右車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
void wh_calAngleR(){
  
  //--【counter値の読み込み】  
  int newCnt = encRdir*encoderR.getPosition();  //右輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  float angle = newCnt*Cnt2Ang;         //右車輪の角度を計算

  rwh.newAng = angle;
    
}

float wh_getAngleR(){ return rwh.newAng ; }   //interface関数


/*------------------------------------------
 * float getAngVelL()
 * 引数：なし
 * 返り値：angleVel
 * 左車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
void wh_calAngVelL(){

  static int oldCnt=0;                              //１サイクルタイム前のカウント格納用変数
  int newCnt = encLdir*encoderL.getPosition();                //現在のカウント読み込み
  double diffAngle = (newCnt-oldCnt)*Cnt2Ang;       //角度差分値計算
  double angleVel = diffAngle/(SAMP_TIMEms*0.001);  //角速度の計算 

  oldCnt = newCnt;                                   //現在のカウントを次回の計算のために保存
  
  lwh.newAngVel = angleVel;
}

float wh_getAngVelL(){return lwh.newAngVel; }


/*------------------------------------------
 * float getAngVelR()
 * 引数：なし
 * 返り値：angleVel
 * 右車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
void wh_calAngVelR(){

  static int oldCnt=0;                              //１サイクルタイム前のカウント格納用変数
  int newCnt = encRdir*encoderR.getPosition();                //現在のカウント読み込み
  double diffAngle = (newCnt-oldCnt)*Cnt2Ang;       //角度差分値計算
  double angleVel = diffAngle/(SAMP_TIMEms*0.001);  //角速度の計算 

  oldCnt = newCnt;                                   //現在のカウントを次回の計算のために保存
  
  rwh.newAngVel = angleVel;
}

float wh_getAngVelR(){return rwh.newAngVel; }


/*--------------------------------------------
 * void motorOutputL(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
void wh_motorOutputL(int mDuty){

   uint32_t duty = 0;
   mDuty = moLdir*mDuty;
    
  //--【引数のduty比を、正と負にわけて処理】
   if(mDuty < 0) { 
      digitalWrite(AIN1_PIN,LOW);            //時計方向(-)回転 
      digitalWrite(AIN2_PIN,HIGH);
      mDuty = -1*mDuty;                      //duty比を正の数に変換
   }
   else {
      digitalWrite(AIN1_PIN,HIGH);           //反時計方向(+)回転 
      digitalWrite(AIN2_PIN,LOW);
      mDuty =  1*mDuty;
   }
   if(mDuty > dutyMax) mDuty = dutyMax;             //duty比を100%に制限
   duty = (mDuty/100.)*8192;                //正の整数に変換(duty 100の場合8192)　
   ledcWrite(LEDC_CHANNEL_0,duty);           //LEDC_CHANNELへpwmを出力
}

/*--------------------------------------------
 * void motorOutputR(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 右車輪の回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
 void wh_motorOutputR(int mDuty){
   
   uint32_t duty = 0;
   mDuty = moRdir*mDuty;
      
  //--【引数のduty比を、正と負にわけて処理】
   if(mDuty < 0) { 
      digitalWrite(BIN1_PIN,LOW);            //時計方向(-)回転 
      digitalWrite(BIN2_PIN,HIGH);
      mDuty = -1*mDuty;                      //duty比を正の数に変換
   }
   else {
      digitalWrite(BIN1_PIN,HIGH);           //反時計方向(+)回転 
      digitalWrite(BIN2_PIN,LOW);
      mDuty =  1*mDuty;
   }

   if(mDuty > dutyMax) mDuty = dutyMax;           //duty比を100%に制限
   duty = (mDuty/100.)*8192;             //正の整数に変換(duty 100の場合8192)　　
   ledcWrite(LEDC_CHANNEL_1,duty);        //LEDC_CHANNELへpwmを出力

}


/*--------------------------------------------
 * void IRAM_ATTR ISR()
 * 引数：なし　 返り値：なし
 * Encoder外部割込みサービスの呼び出し 
 ----------------------------------------------*/
void IRAM_ATTR ISR() {            
  encoderL.tick();               //左車輪用エンコーダの状態を確認
  encoderR.tick();               //右車輪用エンコーダの状態を確認
}

/*--------------------------------------------(230712_OG)
 * void sen_writeMPU6050()
 * 引数：byte reg(バイトアドレス), byte data(設定データ)
 * 返り値：なし
 * IMUセンサへの設定データのI2C書き込み関数
 ----------------------------------------------*/
void sen_writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

/*--------------------------------------------(230712_OG)
 * byte sen_readMPU6050()
 * 引数：byte reg(バイトアドレス)
 * 返り値：data(センサ1バイト分データ)
 * IMUセンサからのデータの読み出し関数
 ----------------------------------------------*/
byte sen_readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDRESS, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

/*--------------------------------------------(230712_OG)
 * void sen_calc_IMU()
 * 引数：なし, * 返り値：なし
 * IMUセンサからの加速度、角速度のデータ取得と移動平均処理関数
 ----------------------------------------------*/
 //＊＊＊レジスタアドレス0x3Bから、計14バイト分のデータを出力する関数＊＊＊
void sen_cal_IMU(){
  int num = 1;  //移動平均区間数
  imu.ax = 0; 
  imu.ay = 0; 
  imu.az = 0; 

  imu.gx = 0;
  imu.gy = 0; 
  imu.gz = 0;  

    for(int k = 0; k < num; k++){
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true);
  
    imu.ax_raw = Wire.read() << 8 | Wire.read();
    imu.ay_raw = Wire.read() << 8 | Wire.read();
    imu.az_raw = Wire.read() << 8 | Wire.read();
    imu.Temperature = Wire.read() << 8 | Wire.read();
    imu.gx_raw = Wire.read() << 8 | Wire.read();
    imu.gy_raw = Wire.read() << 8 | Wire.read();
    imu.gz_raw = Wire.read() << 8 | Wire.read();

    //＊＊＊加速度値を分解能で割って加速度(G)に,角速度値を分解能で割って角速度(degrees per sec)に変換＊＊＊
    imu.ax += -(imu.ax_raw / 16384.0)*9.8;  //FS_SEL_0 16,384 LSB / g
    imu.ay += -(imu.ay_raw / 16384.0)*9.8;
    imu.az += (imu.az_raw / 16384.0)*9.8;

    imu.gx += -imu.gx_raw / 131;
    imu.gy += -imu.gy_raw / 131;
    imu.gz += imu.gz_raw / 131;  
    
    }

    //移動平均処理
    imu.ax_m_ave = imu.ax / num;
    imu.ay_m_ave = imu.ay / num;
    imu.az_m_ave = imu.az / num;

    imu.gx_m_ave = imu.gx / num;
    imu.gy_m_ave = imu.gy / num;
    imu.gz_m_ave = imu.gz / num;    

}

/*--------------------------------------------(230712_OG)
 * void sen_calibration_gyro()
 * 引数：なし, * 返り値：なし
 * IMUセンサからの角速度のキャリブレーション関数
 ----------------------------------------------*/
//ジャイロセンサ　キャリブレーション
void sen_calibration_gyro(){
  Serial.print("Calculate Calibration");

  int16_t dpsX, dpsY, dpsZ ;
  int cal_num = 300;  //キャリブレーション計測回数

  //500回分の初期値の平均値を計算
  for(int i = 0; i < cal_num; i++){
    
    sen_cal_IMU();
    
    imu.offset_x += imu.gx_m_ave;
    imu.offset_y += imu.gy_m_ave;
    imu.offset_z += imu.gz_m_ave;
    
    if(i % 100 == 0){
      Serial.print(".");
    }
  }
  
  Serial.println();

  imu.offset_x /= cal_num;
  imu.offset_y /= cal_num;
  imu.offset_z /= cal_num;

  Serial.print("offsetX : ");
  Serial.println(imu.offset_x);
  Serial.print("offsetY : ");
  Serial.println(imu.offset_y);
  Serial.print("offsetZ : ");
  Serial.println(imu.offset_z);
}

/*--------------------------------------------(230712_OG)
 * fliat sen_get_ax(), float sen_get_ay(), float sen_get_az()
 * float sen_get_gx(), float sen_get_gy(), float sen_get_gz()
 * 引数：なし, * 返り値：加速度、角速度の値
 * IMUセンサの加速度・角速度の計測値取得関数
 ----------------------------------------------*/
//＊＊＊X軸加速度を取得＊＊＊
float sen_get_ax(){return imu.ax_m_ave;}

//＊＊＊Y軸加速度を取得＊＊＊
float sen_get_ay(){return imu.ay_m_ave;}

//＊＊＊Z軸加速度を取得＊＊＊
float sen_get_az(){return imu.az_m_ave;}

//＊＊＊X軸角速度(degrees per sec)を取得＊＊＊
float sen_get_gx(){
  imu.gx_m_ave -= imu.offset_x;
  return imu.gx_m_ave;}

//＊＊＊Y軸角速度(degrees per sec)を取得＊＊＊
float sen_get_gy(){
  imu.gy_m_ave -= imu.offset_y;
  return imu.gy_m_ave;}

//＊＊＊Z軸角速度(degrees per sec)を取得＊＊＊
float sen_get_gz(){
  imu.gz_m_ave -= imu.offset_z;
  return imu.gz_m_ave;}

/*--------------------------------------------(230712_OG)
 * fliat sen_get_ax(), float sen_get_ay(), float sen_get_az()
 * float sen_get_gx(), float sen_get_gy(), float sen_get_gz()
 * 引数：なし, * 返り値：加速度、角速度の値
 * IMUセンサの加速度・角速度の計測値取得関数
 ----------------------------------------------*/
//＊＊＊roll，pitch，yawの姿勢角計算＊＊＊
void sen_cal_angle(){
  
  //＊＊＊roll，pitchの姿勢角計算＊＊＊
  imu.acc_angle_y = -atan2(imu.ax_m_ave, imu.az_m_ave) * 360 / 2.0 / PI;
  imu.acc_angle_x = atan2(imu.ay_m_ave, sqrt(imu.ax_m_ave*imu.ax_m_ave + imu.az_m_ave*imu.az_m_ave)) * 360 / 2.0 / PI;

  //＊＊＊pitch、roll角を相補フィルタ、yaw角を積分計算＊＊＊
  //前回計算した時から今までの経過時間を算出
  imu.interval = millis() - imu.preInterval;
  imu.preInterval = millis();

  //数値積分
  imu.gyro_angle_x += (imu.ax_m_ave - imu.offset_x) * (imu.interval * 0.001);
  imu.gyro_angle_y += (imu.ay_m_ave - imu.offset_y) * (imu.interval * 0.001);
  imu.gyro_angle_z += (imu.az_m_ave - imu.offset_z) * (imu.interval * 0.001);
  
  //相補フィルタ計算
  imu.angle_pitch = (0.75* imu.gyro_angle_x) + (0.25 * imu.acc_angle_x);
  imu.angle_roll = (0.75* imu.gyro_angle_y) + (0.25 * imu.acc_angle_y);
  imu.angle_yaw = imu.gyro_angle_z;
  imu.gyro_angle_x = imu.angle_pitch;
  imu.gyro_angle_y = imu.angle_roll;
  imu.gyro_angle_z = imu.angle_yaw;
  
}

/*--------------------------------------------(230712_OG)
 * fliat sen_get_angle_pitch(), float sen_get__angle_roll(), float sen_get_angle_yaw()
 * 引数：なし, * 返り値：姿勢角度の値
 * IMUセンサからの姿勢角度値取得関数
 ----------------------------------------------*/
//＊＊＊pitch角を取得＊＊＊
float sen_get_angle_pitch(){return imu.angle_pitch;}

//＊＊＊roll角を取得＊＊＊
float sen_get_angle_roll(){return imu.angle_roll;}

//＊＊＊Yaw角を取得＊＊＊
float sen_get_angle_yaw(){return imu.angle_yaw;}
