// Meridian_TWIN_for_ESP32 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Meridan TWIN ESP32用スクリプト
// 20240804 第三次リファクタリング. モジュール化と表記法の共通化.
// 20240804 mrd_writedim90の概念を導入.
// 20240809 wiiリモコン, ヌンチャクに対応. Homeボタンでヌンチャクスティックのキャリブレーション.

#define VERSION "Meridian_TWIN_for_ESP32_v1.1.1_2024.08.09" // バージョン表示

//================================================================================================================
//  初期設定
//================================================================================================================

// ヘッダファイルの読み込み
#include "main.h"
#include "config.h"
#include "keys.h"
#include "mrd_bt_pad.h"
#include "mrd_msg.h"
#include "mrd_wifi.h"

// ライブラリ導入
#include <Arduino.h>

#include <ESP32DMASPISlave.h> // DMAでSPI通信を高速化するめのライブラリ
ESP32DMASPI::Slave slave;

//================================================================================================================
//  SETUP
//================================================================================================================
void setup() {

  // BT接続確認用LED設定
  pinMode(PIN_LED_BT, OUTPUT);
  digitalWrite(PIN_LED_BT, HIGH);

  // シリアルモニタ表示
  Serial.begin(SERIAL_PC_BPS);
  while (!Serial) {
    delay(1); // シリアルポートが開くのを待つ
  }

  // 起動メッセージ表示
  mrd_msg_twin_esp_hello(VERSION, SERIAL_PC_BPS, SPI0_SPEED, Serial);

  // WiFiの初期化と開始
  mrd_msg_esp_wifi(WIFI_AP_SSID, Serial);
  mrd_wifi_init(udp, WIFI_AP_SSID, WIFI_AP_PASS, Serial);

  // wifiIPの表示
  mrd_msg_esp_ip(MODE_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR, Serial);

  // SPI送受信用DMAの設定(バッファサイズは4の倍数, 末尾に4バイト分0が入る不具合対策で+4)
  s_spi_meridim_dma = slave.allocDMABuffer(MRDM_BYTE + 4); // DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MRDM_BYTE + 4); // DMAバッファ設定

  // SPI送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MRDM_BYTE + 4); // ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MRDM_BYTE + 4); // ※+4は不具合対策

  // SPI通信の初回送信データをセット
  memset(s_spi_meridim.bval, 0, MRDM_BYTE + 4); // ※+4は不具合対策
  s_spi_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_spi_meridim.sval, MRDM_LEN); // チェックサムを格納
  memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MRDM_BYTE + 4); // 送信データをDMAバッファに転記

  // SPI通信の設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MRDM_BYTE + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1);  // キューサイズ とりあえず1
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）
                 // ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12

  // Bluetoothの開始と表示(WIIMOTE)
  mrd_bt_settings(MOUNT_PAD, PAD_INIT_TIMEOUT, wiimote, PIN_LED_BT, Serial);

  // BT用スレッドの開始
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);

  // 開始メッセージ
  mrd_msg_TWIN_ESP_flow_start(Serial);
}

//================================================================================================================
// MAIN LOOP
//================================================================================================================
void loop() {

  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP受信待受ループ (PC → ESP32)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", monitor.flow); // 動作チェック用シリアル表示

  // @[1-1] UDPの受信待ち受けループ
  if (flg.udp_receive_mode) // UDPの受信実施フラグの確認（モード確認）
  {
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをアゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
    while (!flg.udp_rcvd) {
      // UDP受信処理
      if (mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp)) // 受信確認
      {
        flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
      }

      // タイムアウト抜け処理
      unsigned long current_tmp = millis();
      if (current_tmp - start_tmp >= UDP_TIMEOUT) {
        if (millis() > MONITOR_SUPPRESS_DURATION) { // 起動直後はエラー表示を抑制
          Serial.println("UDP timeout");
        }
        flg.udp_rcvd = false;
        break;
      }
      delay(1);
    }
  }
  flg.udp_busy = false; // UDP使用中フラグをサゲる

  // @[1-end] この時点で r_udp_meridim にupdから届いた最新データが格納されている.

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信品質チェック (PC → ESP32)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // 動作チェック用シリアル表示

  // @[2-1] UDP受信データ r_udp_meridim のチェックサムを確認.
  if (mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN)) {
    // UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MRDM_BYTE + 4);

    // エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    s_spi_meridim.bval[MRD_ERR_u] &= 0b10111111;

  } else { // チェックサムがNGならバッファから転記せず前回のデータを使用する
    // エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    s_spi_meridim.bval[MRD_ERR_u] |= 0b01000000;
    err.pc_esp++;
  }
  // @[2-2] この時点で最新データは s_spi_meridim に転記済み.

  // @[2-3] シーケンス番号チェック
  mrdsq.r_expect = mrd_seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成

  // シーケンス番号のシリアルモニタ表示
  mrd_msg_seq_number(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ], monitor.seq_num, Serial);

  // @[2-4] シーケンス番号予想値と受信値が合致しているかのチェック
  if (mrd.seq_compare_nums(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ])) { // 予想通りなら
    s_spi_meridim.bval[MRD_ERR_u] &= 0b11111011; // 10番bit[ESP受信のスキップ検出]をサゲる
    flg.meridim_rcvd = true;                     // Meridim受信成功フラグをアゲる
  } else { // 受信シーケンス番号の値が予想と違ったら,
    mrdsq.r_expect = r_udp_meridim.usval[MRD_SEQ]; // 現在の受信値を予想結果としてキープ
    s_spi_meridim.bval[MRD_ERR_u] |= 0b00000100; // 10番ビット[ESP受信のスキップ検出]をアゲる
    if (mrdsq.r_past != r_udp_meridim.usval[MRD_SEQ]) {
      err.esp_skip++;
      flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる
    }
  }
  mrdsq.r_past = r_udp_meridim.usval[MRD_SEQ];

  // @[2-5] エラーリポートの表示
  mrd_msg_all_err(MONITOR_ALL_ERR, err, Serial);

  // @[2-end] この時点での最新データは s_spi_meridim.
  //          チェックサムとシーケンス番号チェックに適合しなかった場合は前回のデータ.
  //          エラーフラグを操作済みのため、s_spi_meridimのチェックサムは合わない.

  //------------------------------------------------------------------------------------
  //  [ 3 ] 受信値による処理 (ESP32)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", monitor.flow); // 動作チェック用シリアル表示

  // @[3-1] マスターコマンドによる処理
  execute_master_command_from_PC(s_spi_meridim, flg.meridim_rcvd);

  // @[3-end] この時点でPCから受信したデータ(s_spi_meridim)に基づく処理が完了している.

  //------------------------------------------------------------------------------------
  //  [ 4 ] SPI送信データ作成 (ESP32 → Teensy)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", monitor.flow); // 動作チェック用シリアル表示

  // @[4-1] ユーザー定義の送信データの書き込み
  // ・SPIに送るデータをここで作成, s_spi_meridim.svalに書き込み

  // @[4-2] リモコンデータの書き込み
  if (MOUNT_PAD > 0) { // リモコンがマウントされていれば
    mrd_writedim90_pad(s_spi_meridim, PAD_BUTTON_MARGE, pad_array);
  }

  // @[4-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）
  // → PCから受け取った値がs_spi_meridim.sval[MRD_SEQ]に入っているのでここでは何もしない

  // @[4-4] チェックサムの更新
  mrd_writedim90_cksm(s_spi_meridim);

  // @[4-end] ここで SPIに送信するs_spi_meridim のデータが完成.

  //------------------------------------------------------------------------------------
  //  [ 5 ] SPI送受信の実行(送信後に受信の待ち受け) (ESP32 → Teensy, Teensy → ESP32)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // 動作チェック用シリアル表示

  flg.spi_rcvd = false; // SPI受信完了フラグをサゲる

  // SPIを送信し, ダミーではない実データを待ち受けるループ
  while (!flg.spi_rcvd) {

    // @[5-1] SPIのトランザクションが空ならデータをキューに装填
    if (slave.remained() == 0) {
      memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MRDM_BYTE + 4);
      slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MRDM_BYTE + 4);
    }

    // @[5-2] SPIの送受信
    if (slave.available()) { // 送受信の完了待ち
      memcpy(tmp_meridim.bval, r_spi_meridim_dma,
             MRDM_BYTE + 4); // DMAの受信データをtmp配列に転記
      slave.pop();           // DMAのデータ配列の先頭を削除

      // 受信データが実データなら処理後に抜ける, ダミーデータなら次のデータを待つ
      if (tmp_meridim.sval[0] != MCMD_DUMMY_DATA) {

        // @[5-3] 受信したSPI実データのチェックサム
        if (mrd.cksm_rslt(tmp_meridim.sval, MRDM_LEN)) {
          // チェックサムがOKなら, DMAからUDP送信配列に転記
          memcpy(s_udp_meridim.bval, tmp_meridim.bval, MRDM_BYTE);
          flg.meridim_rcvd = true; // Meridim受信成功フラグをアゲる
          flg.spi_rcvd = true;     // SPI受信完了フラグをアゲてループを抜ける
        } else {
          // チェックサムがNGなら, 前回の受信値を使用する
          s_udp_meridim.bval[MRD_ERR_u] |= 0b00010000; // 12番ビット[ESP32のSPI受信エラー]をアゲる
          mrd_writedim90_cksm(s_udp_meridim); // チェックサムの更新
          flg.meridim_rcvd = false;           // Meridim受信成功フラグをサゲる
          flg.spi_rcvd = true; // SPI受信完了フラグをアゲてループを抜ける
        }
      }
    }
    delay(1);
  }

  // @[5-end] s_spi_meridimのSPIに送信が完了し,s_udp_meridimにチェック済みの受信データが格納される.
  //          SPI受信失敗なら,前回のSPI受信データにエラーフラグとチェックサムを上書きしたものを格納.

  //------------------------------------------------------------------------------------
  //  [ 6 ] UDP送信データ作成 (ESP32 → PC)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", monitor.flow); // 動作チェック用シリアル表示

  // @[6-1] 必要に応じてs_udp_meridimのシーケンス番号の確認をしてもよい
  // (略)

  // @[6-2] マスターコマンドによる処理
  execute_master_command_from_Tsy(s_udp_meridim, flg.meridim_rcvd);

  // @[6-3] このESP32内で計算処理したデータをs_udp_meridim.svalに格納する
  // ・Teensy→ESP32→PCという経路
  // ・Teensyからのリモコン値等はここでのみ補足可能

  // @[6-4] チェックサムの更新
  // mrd_writedim90_cksm(s_udp_meridim);

  // @[6-end] s_udp_meridimにPCに送信用の最新データが格納された状態.

  //------------------------------------------------------------------------------------
  //  [ 7 ] UDP送信実行 (ESP32 → PC)
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]\n", monitor.flow); // デバグ用フロー表示

  // @[7-1] UDP送信の実行
  while (flg.bt_busy) { // BT使用中フラグの確認
    delay(1);
  }
  if (flg.udp_send_mode) // UDPの送信実施フラグの確認（モード確認）
  {
    flg.udp_busy = true; // UDP使用中フラグをアゲる
    mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp);
    flg.udp_busy = false; // UDP使用中フラグをサゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
  }

  // @[7-end] s_udp_meridimをPCに送信完了. [ 1 ]のudpの受信待ち受けへ.
}

//================================================================================================================
//   [ 関 数 各 種
//================================================================================================================

/// @brief PCから受けたMaster Commandを実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_from_PC(Meridim90Union a_meridim, bool a_flg_exe) {
  switch (a_meridim.sval[MRD_MASTER]) {
  case MCMD_TEST_VALUE:    // ダミーコード
    flg.test_value = true; // ダミーコード
    return true;

  default:
    return true;
  }
  return false;
}

/// @brief Teensyから受けたMaster Commandを実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_from_Tsy(Meridim90Union a_meridim, bool a_flg_exe) {
  switch (a_meridim.sval[MRD_MASTER]) {
  case MCMD_TEST_VALUE:    // ダミーコード
    flg.test_value = true; // ダミーコード
    return true;

  default:
    return true;
  }
  return false;
}
