#ifndef __MERIDIAN_DISPLEY_MESSAGE_H__
#define __MERIDIAN_DISPLEY_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入
#include <WiFi.h>

//================================================================================================================
//  シリアルモニタリング出力関連の処理
//================================================================================================================

class MrdMsgHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdMsgHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  起動時メッセージ
  //------------------------------------------------------------------------------------

  /// @brief 指定された秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
  /// @param a_mill 充電プロセスの期間を秒単位で指定.
  /// @param m_serial 出力先シリアルの指定.
  void charging(int a_mill) {
    m_serial.print("Charging the capacitor.");
    for (int i = 0; i < a_mill; i++) {
      if (i % 100 == 0) { // 100msごとにピリオドを表示
        m_serial.print(".");
      }
      delay(1);
    }
    m_serial.println();
  }

  /// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する.
  /// @param a_version バージョン情報.
  /// @param a_pc_speed PCとのUSBシリアル通信速度.
  /// @param a_spi_speed SPIの通信速度.
  /// @param m_serial 出力先シリアルの指定.
  void hello_twin_esp(String a_version, int a_pc_speed, int a_spi_speed) {
    m_serial.println();
    m_serial.print("Hi, This is ");
    m_serial.println(a_version);
    m_serial.print("Set PC-USB ");
    m_serial.print(a_pc_speed);
    m_serial.println(" bps");
    m_serial.print("Set SPI0   ");
    m_serial.print(a_spi_speed);
    m_serial.println(" bps");
  }

  /// @brief wifiの接続開始メッセージを出力する.
  /// @param a_ssid 接続先のSSID.
  /// @param m_serial 出力先シリアルの指定.
  void esp_wifi(const char *a_ssid) {
    m_serial.println("WiFi connecting to => " + String(a_ssid)); // WiFi接続完了通知
  }

  /// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
  /// @param a_flg_fixed_ip 固定IPかどうか. true:固定IP, false:動的IP.
  /// @param a_ssid 接続先のSSID.
  /// @param a_fixedip 固定IPの場合の値.
  /// @param m_serial 出力先シリアルの指定.
  void esp_ip(bool a_flg_fixed_ip, const char *a_ssid, const char *a_fixedip) {
    m_serial.println("WiFi successfully connected."); // WiFi接続完了通知
    m_serial.println("PC's IP address target => " +
                     String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

    if (a_flg_fixed_ip) {
      m_serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) +
                       " (*Fixed)"); // ESP32自身のIPアドレスの表示
    } else {
      m_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
      m_serial.println(WiFi.localIP().toString());
    }
  }

  /// @brief システムに接続設定したジョイパッドのタイプを表示する.
  /// @param m_serial 出力先シリアルの指定.
  void mounted_pad() {
    m_serial.print("Pad Receiver mounted : ");

    if (MOUNT_PAD == MERIMOTE) {
      m_serial.println("Merimote.");
    } else if (MOUNT_PAD == BLUERETRO) {
      m_serial.println("BlueRetro.");
    } else if (MOUNT_PAD == SBDBT) {
      m_serial.println("SBDBT.");
    } else if (MOUNT_PAD == KRR5FH) {
      m_serial.println("KRC-5FH.");
    } else {
      m_serial.println("None (PC).");
    }
  }

  /// @brief システムの動作開始を示すメッセージを出力する.
  /// @param m_serial 出力先シリアルの指定.
  void flow_start_twin_esp() {
    m_serial.println();
    m_serial.println("-) Meridian TWIN system on side ESP32 now flows. (-");
  }

  //------------------------------------------------------------------------------------
  //  イベントメッセージ
  //------------------------------------------------------------------------------------

  /// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
  /// @param mrd_disp_all_err モニタリング表示のオンオフ.
  /// @param a_err エラーデータの入った構造体.
  /// @param m_serial 出力先シリアルの指定.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool all_err(bool mrd_disp_all_err, MrdErr a_err) {
    if (mrd_disp_all_err) {
      m_serial.print("[ERR] es>pc:");
      m_serial.print(a_err.esp_pc);
      m_serial.print(" pc>es:");
      m_serial.print(a_err.pc_esp);
      m_serial.print(" es>ts:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" ts>es:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" tsSkp:");
      m_serial.print(a_err.tsy_skip);
      m_serial.print(" esSkp:");
      m_serial.print(a_err.esp_skip);
      m_serial.print(" pcSkp:");
      m_serial.print(a_err.pc_skip);
      m_serial.println();
      return true;
    }
    return false;
  }

  /// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
  /// @param a_seq_expect 期待するシーケンス番号.
  /// @param a_seq_rcvd 実際に受信したシーケンス番号.
  /// @param a_disp_seq_num 表示するかどうかのブール値.
  /// @param m_serial 出力先シリアルの指定.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool a_disp) {
    if (a_disp) {
      m_serial.print("Seq ep/rv ");
      m_serial.print(a_seq_expect);
      m_serial.print("/");
      m_serial.println(a_seq_rcvd);
      return true;
    }
    return false;
  }
};

#endif // __MERIDIAN_DISPLEY_MESSAGE_H__