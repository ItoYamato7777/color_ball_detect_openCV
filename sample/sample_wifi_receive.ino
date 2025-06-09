#include <Arduino.h>
#include <ICS.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>

#define KRS_MIN 3500
#define KRS_MAX 11500
#define KRS_ORG 7500
#define KRS_FREE 0
#define SERVO_MIN -135
#define SERVO_MAX 135

#define ESP_BAUD 115200
#define ESP_SERIAL Serial6
#define UDP_PORT 12345

char ssid[] = "KXR-wifi_2.4G";       // ←wifi
char pass[] = "zutt0issy0";   // ←passward

WiFiEspUDP Udp;
IcsController ICS(Serial1);
IcsServo servo[3];

void krs_setposition(IcsServo* servo, float radian) {
  int angle = radian * 180 / PI;
  int pos = map(angle, SERVO_MIN, SERVO_MAX, KRS_MIN, KRS_MAX);
  if (pos >= KRS_MIN && pos <= KRS_MAX) {
    servo->setPosition(pos);
    delay(1);
  }
}

void setup() {
  Serial.begin(9600);
  ESP_SERIAL.begin(ESP_BAUD);
  WiFi.init(&ESP_SERIAL);
  // WiFi接続
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    delay(10);
  }

  Serial.println("WiFi connected");
  Udp.begin(UDP_PORT);

  ICS.begin();
  for (int i = 0; i < 3; i++) {
    servo[i].attach(ICS, i + 1);  // ID 1〜3
    servo[i].setStretch(30);
    delay(1);
    servo[i].setSpeed(30);
    delay(1);
    krs_setposition(&servo[i], 0);
    delay(1);
  }
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    char buffer[32];
    int len = Udp.read(buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';

    int servo_id = 0;
    float rad = 0.0;
    char* token = strtok(buffer, ",");
    if (token != NULL) {
      servo_id = atoi(token);
      token = strtok(NULL, ",");
      if (token != NULL) {
        rad = atof(token);
      } 
      else {
        Serial.println("角度データなし");
        return;
      }
    } 
    else {
      Serial.println("受信形式が不正");
      return;
    }

    if (servo_id >= 1 && servo_id <= 3) {
      krs_setposition(&servo[servo_id - 1], rad);
    } 
    else {
      Serial.println("サーボ番号が範囲外");
    }
  }

  delay(1);
}

