#ifndef __MERIDIAN_MESSAGE_H__
#define __MERIDIAN_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "keys.h"

// ライブラリ導入
#include <WiFi.h>

//------------------------------------------------------------------------------------
//  起動時メッセージ
//------------------------------------------------------------------------------------

/// @brief 指定された秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
/// @param a_mill 充電プロセスの期間を秒単位で指定.
void mrd_msg_charging(int a_mill) {
  Serial.print("Charging the capacitor.");
  for (int i = 0; i < a_mill; i++) {
    if (i % 100 == 0) { // 100msごとにピリオドを表示
      Serial.print(".");
    }
    delay(1);
  }
  Serial.println();
}

/// ★ TWIN用新変数
/// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する.
void mrd_msg_twin_esp_hello(String a_version) {
  Serial.println();
  Serial.print("Hi, This is ");
  Serial.println(a_version);
  Serial.print("Set PC-USB ");
  Serial.print(SERIAL_PC_BPS);
  Serial.println(" bps");
  Serial.print("Set SPI0   ");
  Serial.print(SPI0_SPEED);
  Serial.println(" bps");
}

/// @brief システムに接続されているIMU/AHRSセンサーのタイプを表示する.
/// @param a_imuahrs_type 接続されているセンサーのタイプを示す列挙型.
void mrd_msg_print_imuahrs(ImuAhrsType a_imuahrs_type) {
  Serial.print("IMU/AHRS Sensor mounted: ");

  switch (a_imuahrs_type) {
  case NO_IMU:
    Serial.println("None. ");
    break;
  case MPU6050_IMU:
    Serial.println("MPU6050(GY-521) ");
    break;
  case MPU9250_IMU:
    Serial.println("MPU9250(GY-6050/GY-9250) ");
    break;
  case BNO055_AHRS:
    Serial.println("BNO055 ");
    break;
  default:
    break;
  }
}

/// @brief wifiの接続開始メッセージを出力する.
void mrd_msg_esp_wifi() {
  Serial.println("WiFi connecting to => " + String(WIFI_AP_SSID)); // WiFi接続完了通知
}

/// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
void mrd_msg_esp_ip(bool mode_fixed_ip) {
  Serial.println("WiFi successfully connected.");                      // WiFi接続完了通知
  Serial.println("PC's IP address target => " + String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

  if (mode_fixed_ip) {
    Serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) +
                   " (*Fixed)"); // ESP32自身のIPアドレスの表示
  } else {
    Serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
    Serial.println(WiFi.localIP().toString());
  }
}

/// @brief システムに接続設定したジョイパッドのタイプを表示する.
void mrd_msg_mounted_pad() {
  Serial.print("Pad Receiver mounted : ");

  if (MOUNT_PAD == MERIMOTE) {
    Serial.println("Merimote.");
  } else if (MOUNT_PAD == BLUERETRO) {
    Serial.println("BlueRetro.");
  } else if (MOUNT_PAD == SBDBT) {
    Serial.println("SBDBT.");
  } else if (MOUNT_PAD == KRR5FH) {
    Serial.println("KRC-5FH.");
  } else {
    Serial.println("None (PC).");
  }
}

/// @brief システムの動作開始を示すメッセージを出力する.
void mrd_msg_lite_flow_start() {
  Serial.println();
  Serial.println("-) Meridian -LITE- system on ESP32 now flows. (-");
}

//------------------------------------------------------------------------------------
//  イベントメッセージ
//------------------------------------------------------------------------------------

/// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示する.
/// @param a_line サーボモーターが接続されているUARTライン（L, R, C）.
/// @param a_num エラーが発生しているサーボの番号.
/// @param a_disp_error エラーメッセージを表示するかどうかのブール値.
/// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
bool mrd_msg_servo_err(UartLine a_line, int a_num, bool a_disp_error) {
  if (a_disp_error) {
    Serial.print("Servo err ");
    if (a_line == L) {
      Serial.print("L_");
      Serial.println(a_num);
      return true;
    } else if (a_line == R) {
      Serial.print("R_");
      Serial.println(a_num);
      return true;
    } else if (a_line == C) {
      Serial.print("C_");
      Serial.println(a_num);
      return true;
    }
  }
  return false;
}

/// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
/// @param a_err エラーデータの入った構造体.
/// @param mrd_disp_all_error モニタリング表示のオンオフ.
/// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
bool mrd_msg_all_err(MrdErr a_err, bool mrd_disp_all_error) {
  if (mrd_disp_all_error) {
    Serial.print("[ERR] es>pc:");
    Serial.print(a_err.esp_pc);
    Serial.print(" pc>es:");
    Serial.print(a_err.pc_esp);
    Serial.print(" es>ts:");
    Serial.print(a_err.esp_tsy);
    Serial.print(" ts>es:");
    Serial.print(a_err.esp_tsy);
    Serial.print(" tsSkp:");
    Serial.print(a_err.tsy_skip);
    Serial.print(" esSkp:");
    Serial.print(a_err.esp_skip);
    Serial.print(" pcSkp:");
    Serial.print(a_err.pc_skip);
    Serial.println();
    return true;
  }
  return false;
}

/// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
/// @param a_seq_expect 期待するシーケンス番号.
/// @param a_seq_rcvd 実際に受信したシーケンス番号.
/// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
bool mrd_msg_seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool mrd_disp_seq_num) {
  if (mrd_disp_seq_num) {
    Serial.print("Seq ep/rv ");
    Serial.print(a_seq_expect);
    Serial.print("/");
    Serial.println(a_seq_rcvd);
    return true;
  }
  return false;
}

#endif // __MERIDIAN_MESSAGE_H__
