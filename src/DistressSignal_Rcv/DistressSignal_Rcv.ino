#include <esp_now.h>
#include <WiFi.h>
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// use HardwareSerial UART1TX
HardwareSerial myHardwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;
//const int busyPin = 23;
void printDetail(uint8_t type, int value);

esp_now_peer_info_t slave;
static int LED = 13;
static esp_err_t result;
static bool state = false;
static bool rcvDataFlag = false;

// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  char msg[1];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: %s\n", macStr);
  Serial.printf("Last Packet Recv Data(%d): ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    // dataから1文字ずつ取り出して出力する
    msg[1] = data[i];
    Serial.print(msg[1]);
  }
  Serial.println("");
  rcvDataFlag = true;
  state = !state;
  if (state == true) {
    Serial.println("LED 点灯");
    digitalWrite(LED, HIGH);
    // myDFPlayer.play(2);  //Play the first mp3
  } else {
    Serial.println("LED 消灯");
    digitalWrite(LED, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }

  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //  myHardwareSerial.begin(9600, SERIAL_8N1, 14, 12); // RX, TX
  //  pinMode(busyPin, INPUT);
  //  Serial.println();
  //  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  //  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  //
  //  if (!myDFPlayer.begin(myHardwareSerial)) {
  //    Serial.println(F("Unable to begin:"));
  //    Serial.println(F("1.Please recheck the connection!"));
  //    Serial.println(F("2.Please insert the SD card!"));
  //    DFplaerの初期化が完了するまで待つ
  //    while (true) {
  //      delay(0); // Code to compatible with ESP8266 watch dog.
  //    }
  //  }
  //  Serial.println(F("DFPlayer Mini online."));
  //
  //  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
}

void loop() {
  if (rcvDataFlag == true) {
    Serial.println("応答しました。");
    uint8_t data[13] = {72, 101, 108, 108, 111, 32, 69, 83, 80, 45, 78, 79, 87};
    result = esp_now_send(slave.peer_addr, data, sizeof(data));
    rcvDataFlag = false;
  }

  // 1ms間隔
  delay(1);
}
