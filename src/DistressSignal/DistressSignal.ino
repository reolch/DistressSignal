#include <esp_now.h>
#include <WiFi.h>
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// use HardwareSerial UART1TX
HardwareSerial myHardwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;
const int busyPin = 23;
void printDetail(uint8_t type, int value);

// 送信ボタン
const int sendButton = 2;
esp_now_peer_info_t slave;

volatile esp_err_t result;
// 排他用処理中フラグ
volatile bool flag_sendButton_pushed = false;
// 操作時刻保存用
volatile ulong time_sendButton_pushed = 0;
// LED
static int LED = 13;
// LED状態保持用
static bool ledState = false;

// ボタン押下時割り込み処理
void IRAM_ATTR onPushed() {
  // ボタンが押されていない場合にしか処理しない
  if (!flag_sendButton_pushed)
  {
    // 押されたフラグを保存
    flag_sendButton_pushed = true;
    // 押された時刻を保存
    time_sendButton_pushed = millis();
  }
}

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
  Serial.println("LED 点灯");
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

void setup() {
  Serial.begin(115200);
  // ピン情報設定
  pinMode(LED, OUTPUT);
  pinMode(sendButton, INPUT_PULLUP);
  // 割り込み処理設定
  attachInterrupt(sendButton, onPushed, FALLING);

  // 押されたフラグを初期化
  flag_sendButton_pushed = false;
  // 押された時刻を初期化
  time_sendButton_pushed = millis();

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
}

void loop() {
  // 状態取得
  if (flag_sendButton_pushed)
  {
    if ((digitalRead(sendButton) == LOW))
    {
      Serial.println("ボタンが押された");
      uint8_t data[13] = {72, 101, 108, 108, 111, 32, 69, 83, 80, 45, 78, 79, 87};
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.print("Send Status: ");
      if (result == ESP_OK) {
        Serial.println("Success");
      } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW not Init.");
      } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
      } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
      } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
      } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
      } else {
        Serial.println("Not sure what happened");
      }
    }

    // 押しっぱなしの時は無限ループで待つ
    while (digitalRead(sendButton) == LOW)
    {
      delay(10);
    }
    // フラグリセット
    flag_sendButton_pushed = false;
  }

  // 一定時間以上経過していたらフラグをリセット
  //  フラグ状態がおかしくなった時用のトラップ
  if ((millis() - time_sendButton_pushed) > 100000) {
    // フラグリセット
    flag_sendButton_pushed = false;
    // 押された時刻をリセット
    time_sendButton_pushed = millis();
  }

  // 1ms間隔
  delay(1);
}
