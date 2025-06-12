// robot_action.ino

#include <Arduino.h>
#include <ICS.h>
#include <WiFiEsp.h>
#include <WiFiEspServer.h> // <--- UDPから変更

// --- 定数定義 ---
#define TCP_PORT 12345
#define ESP_BAUD 115200
#define ESP_SERIAL Serial6
char ssid[] = "KXR-wifi_2.4G";
char pass[] = "zutt0issy0";

#define KRS_MIN 3500
#define KRS_MAX 11500
#define SERVO_MIN -135
#define SERVO_MAX 135
#define MOVE_SPEED 100

// --- グローバル変数 ---
WiFiEspServer server(TCP_PORT); // <--- TCPサーバーオブジェクト
IcsController ICS1(Serial1);
IcsController ICS3(Serial3);
IcsController ICS4(Serial4);
IcsServo C_servo[5];
IcsServo F_servo[2];
IcsServo R_servo[3];


// --- プロトタイプ宣言 ---
void krs_setposition(IcsServo* servo, float angle);
void move_forward(float value);
void move_backward(float value);
void move_right(float value);
void move_left(float value);
void pick();
void drop();

// --- セットアップ ---
void setup() {
  Serial.begin(9600);
  ESP_SERIAL.begin(ESP_BAUD);
  WiFi.init(&ESP_SERIAL);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  
  server.begin(); // <--- TCPサーバーを開始
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("TCP server started on port ");
  Serial.println(TCP_PORT);

  // サーボ初期化処理
  ICS1.begin(1250000);
  ICS3.begin(1250000);
  ICS4.begin(1250000);

  // 回収装置セットアップ
  for(int i = 0; i < 5; i++){
    C_servo[i].attach(ICS1, i+1);
    C_servo[i].setStretch(30);
    delay(1);
    C_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&C_servo[i], 45);
    delay(1);
  }
  // 前輪セットアップ
  for(int i = 0; i < 2; i++){
    F_servo[i].attach(ICS3, i+1);
    F_servo[i].setStretch(30);
    delay(1);
    F_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&F_servo[i], 0);
  }
  // 後輪セットアップ
  for(int i = 0; i < 2; i++){
    R_servo[i].attach(ICS4, i+1);
    R_servo[i].setStretch(30);
    delay(1);
    R_servo[i].setSpeed(60);
    delay(1);
    krs_setposition(&R_servo[i], 0);
    delay(1);
  }
  // かごセットアップ
  R_servo[2].attach(ICS4, 2+1);
  R_servo[2].setStretch(30);
  delay(1);
  R_servo[2].setSpeed(30);
  delay(1);
  krs_setposition(&R_servo[2], -2);

  delay(1000);
  Serial.println("Robot setup complete. Waiting for commands...");
}

// --- メインループ ---
void loop() {
  WiFiEspClient client = server.available(); // PCからの接続を待つ

  // クライアントが接続してきた場合
  if (client) {
    Serial.println("Client connected.");
    
    // <--- 1コマンドのみを処理し、接続を維持しない ---
    bool received_command = false;
    while (client.connected() && !received_command) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(line);

        char buffer[32];
        line.toCharArray(buffer, sizeof(buffer));

        char* command = strtok(buffer, ",");
        char* value_str = strtok(NULL, ",");

        if (command != NULL && value_str != NULL) {
          float value = atof(value_str);

          if (strcmp(command, "up") == 0) move_forward(value);
          else if (strcmp(command, "down") == 0) move_backward(value);
          else if (strcmp(command, "right") == 0) move_right(value);
          else if (strcmp(command, "left") == 0) move_left(value);
          else if (strcmp(command, "pick") == 0) {
            Serial.println("Action: pick");
            pick();
          }
          else if (strcmp(command, "drop") == 0) drop();
          else Serial.println("Unknown command");
          
          Serial.println("Action complete. Sending 'done'.");
          client.println("done"); 
        } else {
          Serial.println("Invalid packet format");
        }
        
        received_command = true; // 1コマンド処理したのでフラグを立てる
      }
    }
    
    // 処理が終わったらクライアントを切断
    client.stop();
    Serial.println("Client disconnected.");
  }
}

// --- 各動作の関数 ---

void krs_setposition(IcsServo* servo, float angle){ //
  int pos = map(angle, SERVO_MIN, SERVO_MAX, KRS_MIN, KRS_MAX);
  if(pos >= KRS_MIN && pos <= KRS_MAX){
    servo->setPosition(pos);
    delay(1);
  }
}

// 前進
void move_forward(float value) {
  int duration_ms = value * 100; // PCからの数値(cm)を駆動時間(ms)に変換
  Serial.print("Action: move_forward for ");
  Serial.print(duration_ms);
  Serial.println(" ms");
  
  krs_setposition(&F_servo[0], 0); //
  krs_setposition(&R_servo[0], 0); //
  delay(100);
  krs_setposition(&F_servo[1], MOVE_SPEED); //
  krs_setposition(&R_servo[1], MOVE_SPEED); //
  delay(duration_ms); // 指示された時間だけ回転
  krs_setposition(&F_servo[1], 0); //
  krs_setposition(&R_servo[1], 0); //
}

// 後退
void move_backward(float value) {
  int duration_ms = value * 100;
  Serial.print("Action: move_backward for ");
  Serial.print(duration_ms);
  Serial.println(" ms");

  krs_setposition(&F_servo[0], 0); //
  krs_setposition(&R_servo[0], 0); //
  delay(100);
  krs_setposition(&F_servo[1], -MOVE_SPEED); //
  krs_setposition(&R_servo[1], -MOVE_SPEED); //
  delay(duration_ms);
  krs_setposition(&F_servo[1], 0); //
  krs_setposition(&R_servo[1], 0); //
}

// 右移動
void move_right(float value) {
  int duration_ms = value * 100;
  Serial.print("Action: move_right for ");
  Serial.print(duration_ms);
  Serial.println(" ms");
  
  krs_setposition(&F_servo[0], -90); //
  krs_setposition(&R_servo[0], -90); //
  delay(200); // 向きが変わるのを待つ
  krs_setposition(&F_servo[1], MOVE_SPEED); //
  krs_setposition(&R_servo[1], MOVE_SPEED); //
  delay(duration_ms);
  krs_setposition(&F_servo[1], 0); //
  krs_setposition(&R_servo[1], 0); //
}

// 左移動
void move_left(float value) {
  int duration_ms = value * 100;
  Serial.print("Action: move_left for ");
  Serial.print(duration_ms);
  Serial.println(" ms");

  krs_setposition(&F_servo[0], 90); //
  krs_setposition(&R_servo[0], 90); //
  delay(200); // 向きが変わるのを待つ
  krs_setposition(&F_servo[1], MOVE_SPEED); //
  krs_setposition(&R_servo[1], MOVE_SPEED); //
  delay(duration_ms);
  krs_setposition(&F_servo[1], 0); //
  krs_setposition(&R_servo[1], 0); //
}

// ボールを拾う
void pick() { //
  Serial.println("Action: pick");
  krs_setposition(&C_servo[4], 45);
  krs_setposition(&C_servo[3], 45);
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
}

// ボールを落とす
void drop() { //
  Serial.println("Action: drop");
  krs_setposition(&R_servo[2], 90);
  delay(2000);
  krs_setposition(&R_servo[2], -2);
  delay(1000);
}