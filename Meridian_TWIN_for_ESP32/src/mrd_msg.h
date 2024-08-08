#ifndef __MERIDIAN_MESSAGE_H__
#define __MERIDIAN_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入
#include <WiFi.h>

//------------------------------------------------------------------------------------
//  起動時メッセージ
//------------------------------------------------------------------------------------

/// @brief 指定された秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
/// @param a_mill 充電プロセスの期間を秒単位で指定.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_charging(int a_mill, HardwareSerial &a_serial) {
  a_serial.print("Charging the capacitor.");
  for (int i = 0; i < a_mill; i++) {
    if (i % 100 == 0) { // 100msごとにピリオドを表示
      a_serial.print(".");
    }
    delay(1);
  }
  a_serial.println();
}

/// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する.
/// @param a_version バージョン情報.
/// @param a_pc_speed PCとのUSBシリアル通信速度.
/// @param a_spi_speed SPIの通信速度.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_twin_esp_hello(String a_version, int a_pc_speed, int a_spi_speed,
                            HardwareSerial &a_serial) {
  a_serial.println();
  a_serial.print("Hi, This is ");
  a_serial.println(a_version);
  a_serial.print("Set PC-USB ");
  a_serial.print(a_pc_speed);
  a_serial.println(" bps");
  a_serial.print("Set SPI0   ");
  a_serial.print(a_spi_speed);
  a_serial.println(" bps");
}

/// @brief wifiの接続開始メッセージを出力する.
/// @param a_ssid 接続先のSSID.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_esp_wifi(const char *a_ssid, HardwareSerial &a_serial) {
  a_serial.println("WiFi connecting to => " + String(a_ssid)); // WiFi接続完了通知
}

/// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
/// @param mode_fixed_ip 固定IPかどうか. true:固定IP, false:動的IP.
/// @param a_ssid 接続先のSSID.
/// @param a_fixedip 固定IPの場合の値.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_esp_ip(bool mode_fixed_ip, const char *a_ssid, const char *a_fixedip,
                    HardwareSerial &a_serial) {
  a_serial.println("WiFi successfully connected."); // WiFi接続完了通知
  a_serial.println("PC's IP address target => " +
                   String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

  if (mode_fixed_ip) {
    a_serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) +
                     " (*Fixed)"); // ESP32自身のIPアドレスの表示
  } else {
    a_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
    a_serial.println(WiFi.localIP().toString());
  }
}

/// @brief システムに接続設定したジョイパッドのタイプを表示する.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_mounted_pad(HardwareSerial &a_serial) {
  a_serial.print("Pad Receiver mounted : ");

  if (MOUNT_PAD == MERIMOTE) {
    a_serial.println("Merimote.");
  } else if (MOUNT_PAD == BLUERETRO) {
    a_serial.println("BlueRetro.");
  } else if (MOUNT_PAD == SBDBT) {
    a_serial.println("SBDBT.");
  } else if (MOUNT_PAD == KRR5FH) {
    a_serial.println("KRC-5FH.");
  } else {
    a_serial.println("None (PC).");
  }
}

/// @brief システムの動作開始を示すメッセージを出力する.
/// @param a_serial 出力先シリアルの指定.
void mrd_msg_TWIN_ESP_flow_start(HardwareSerial &a_serial) {
  a_serial.println();
  a_serial.println("-) Meridian TWIN system on side ESP32 now flows. (-");
}

//------------------------------------------------------------------------------------
//  イベントメッセージ
//------------------------------------------------------------------------------------

/// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
/// @param mrd_disp_all_err モニタリング表示のオンオフ.
/// @param a_err エラーデータの入った構造体.
/// @param a_serial 出力先シリアルの指定.
/// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
bool mrd_msg_all_err(bool mrd_disp_all_err, MrdErr a_err, HardwareSerial &a_serial) {
  if (mrd_disp_all_err) {
    a_serial.print("[ERR] es>pc:");
    a_serial.print(a_err.esp_pc);
    a_serial.print(" pc>es:");
    a_serial.print(a_err.pc_esp);
    a_serial.print(" es>ts:");
    a_serial.print(a_err.esp_tsy);
    a_serial.print(" ts>es:");
    a_serial.print(a_err.esp_tsy);
    a_serial.print(" tsSkp:");
    a_serial.print(a_err.tsy_skip);
    a_serial.print(" esSkp:");
    a_serial.print(a_err.esp_skip);
    a_serial.print(" pcSkp:");
    a_serial.print(a_err.pc_skip);
    a_serial.println();
    return true;
  }
  return false;
}

/// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
/// @param a_seq_expect 期待するシーケンス番号.
/// @param a_seq_rcvd 実際に受信したシーケンス番号.
/// @param a_disp_seq_num 表示するかどうかのブール値.
/// @param a_serial 出力先シリアルの指定.
/// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
bool mrd_msg_seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool a_disp, HardwareSerial &a_serial) {
  if (a_disp) {
    a_serial.print("Seq ep/rv ");
    a_serial.print(a_seq_expect);
    a_serial.print("/");
    a_serial.println(a_seq_rcvd);
    return true;
  }
  return false;
}

#endif // __MERIDIAN_MESSAGE_H__