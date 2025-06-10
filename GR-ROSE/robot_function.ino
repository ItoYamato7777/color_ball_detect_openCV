#include <Arduino.h>
#include <ICS.h>

#define KRS_MIN 3500
#define KRS_MAX 11500
#define KRS_ORG 7500
#define KRS_FREE 0
#define SERVO_MIN -135
#define SERVO_MAX 135

IcsController ICS1(Serial1);
IcsController ICS3(Serial3);
IcsController ICS4(Serial4);
IcsServo C_servo[5]; //回収装置．ID変え忘れていたのでこうなっているが，使うのはID4,5のみ
IcsServo F_servo[2]; //前輪
IcsServo R_servo[3]; //後輪＆かご

//元の関数をラジアンから角度指定にするよう変えた．-135~135度で指定．
void krs_setposition(IcsServo* servo, float angle){
  int pos = map(angle, SERVO_MIN, SERVO_MAX, KRS_MIN, KRS_MAX);
  if(pos >= KRS_MIN && pos <= KRS_MAX){
    servo->setPosition(pos);
    delay(1);
  }
}

void setup() {
  Serial.begin(9600);
  ICS1.begin(1250000);
  ICS3.begin(1250000);
  ICS4.begin(1250000);
  //回収装置セットアップ
  for(int i = 0; i < 5; i++){
    C_servo[i].attach(ICS1, i+1); 
    C_servo[i].setStretch(30);
    delay(1);
    C_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&C_servo[i], 45);//回収装置の初期角度は45度．
    delay(1);
  }
  //前輪セットアップ
  for(int i = 0; i < 2; i++){
    F_servo[i].attach(ICS3, i+1); 
    F_servo[i].setStretch(30);
    delay(1);
    F_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&F_servo[i], 0);
    delay(1);
  }
  //後輪セットアップ
  for(int i = 0; i < 2; i++){
    R_servo[i].attach(ICS4, i+1); 
    R_servo[i].setStretch(30);
    delay(1);
    R_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&R_servo[i], 0);
    delay(1);
  }
  //かごセットアップ
  R_servo[2].attach(ICS4, 2+1); 
  R_servo[2].setStretch(30);
  delay(1);
  R_servo[2].setSpeed(30);
  delay(1);
  krs_setposition(&R_servo[2], -2);//かごの初期角度は-2度程度
  delay(1);

  delay(1000);
}

void loop() {
  //回収動作
  krs_setposition(&C_servo[4], 45);
  krs_setposition(&C_servo[3], 45); //セットアップでこの角度になっているはず
  delay(500);
  krs_setposition(&C_servo[4], 0);
  delay(500);
  krs_setposition(&C_servo[3], 0);
  delay(500);
  krs_setposition(&C_servo[4], 90);
  delay(2000);
  krs_setposition(&C_servo[4], 45);
  delay(500);
  krs_setposition(&C_servo[3], 45);
  delay(500);

  //前進
  krs_setposition(&F_servo[0], 0);
  krs_setposition(&R_servo[0], 0);
  delay(2000);
  krs_setposition(&F_servo[1], 100); //0で停止．135まで．マイナス付けると逆回転
  krs_setposition(&R_servo[1], 100); //セットスピードとセットポジションの値で速度がきまるらしいが，純粋な比例かどうか不明
  delay(3000); //回転時間
  krs_setposition(&F_servo[1], 0);
  krs_setposition(&R_servo[1], 0);
  delay(3000);

  //後退
  krs_setposition(&F_servo[0], 0);
  krs_setposition(&R_servo[0], 0);
  delay(2000);
  krs_setposition(&F_servo[1], -100); //0で停止．135まで．マイナス付けると逆回転
  krs_setposition(&R_servo[1], -100); //セットスピードとセットポジションの値で速度がきまるらしいが，純粋な比例かどうか不明
  delay(3000); //回転時間
  krs_setposition(&F_servo[1], 0);
  krs_setposition(&R_servo[1], 0);
  delay(3000);


//右移動
  krs_setposition(&F_servo[0], 10);
  krs_setposition(&R_servo[0], 10);
  delay(2000);
  krs_setposition(&F_servo[1], 100); //0で停止．135まで．マイナス付けると逆回転
  krs_setposition(&R_servo[1], 100); //セットスピードとセットポジションの値で速度がきまるらしいが，純粋な比例かどうか不明
  delay(3000); //回転時間
  krs_setposition(&F_servo[1], 0);
  krs_setposition(&R_servo[1], 0);
  delay(3000);

//左移動
  krs_setposition(&F_servo[0], -10);
  krs_setposition(&R_servo[0], -10);
  delay(2000);
  krs_setposition(&F_servo[1], 100); //0で停止．135まで．マイナス付けると逆回転
  krs_setposition(&R_servo[1], 100); //セットスピードとセットポジションの値で速度がきまるらしいが，純粋な比例かどうか不明
  delay(3000); //回転時間
  krs_setposition(&F_servo[1], 0);
  krs_setposition(&R_servo[1], 0);
  delay(3000);

  // //斜め移動
  // krs_setposition(&F_servo[0], 25); //-90~90度まで．それ以上はコードが断線する危険があるのでダメ．
  // krs_setposition(&R_servo[0], 25);
  // delay(2000);
  // krs_setposition(&F_servo[1], 30);
  // krs_setposition(&R_servo[1], 30);
  // delay(3000);
  // krs_setposition(&F_servo[1], 0);
  // krs_setposition(&R_servo[1], 0);
  // delay(2000);

  // //旋回的な何か．サーボIDで楽しようとしたらこうなった，ID変える作業めんどくさい．
  // krs_setposition(&F_servo[0], 90);
  // krs_setposition(&R_servo[0], 90);
  // delay(2000);
  // krs_setposition(&F_servo[1], 20);
  // krs_setposition(&R_servo[1], -20);
  // delay(3000);
  // krs_setposition(&F_servo[1], 0);
  // krs_setposition(&R_servo[1], 0);
  // delay(3000);

  //かごを傾け、ボールを落とす動作
  krs_setposition(&R_servo[2], 90);
  delay(4000);
  krs_setposition(&R_servo[2], -2);
  delay(1000);
}