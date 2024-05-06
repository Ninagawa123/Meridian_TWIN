#define VERSION "Meridian_TWIN_for_ESP32_2024.05.06" // バージョン表示

// Meridian_TWIN_for_ESP32 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Meridan TWIN ESP32用スクリプト
// 20240413 PS4リモコンのESP32受待ちサポートを一旦廃止.
// 20240414 第二次リファクタリング.
// 20240414 SPI通信を1フレームあたり2回, 往復実行するように修正.
// 20240422 システム用の変数を構造化,変数名を修正.
// 20240506 PS4リモコンはMerimoteに負荷分散 https://github.com/Ninagawa123/Merimote

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

// コンフィグファイルの読み込み
#include "config.h"
#include "keys.h"

// ヘッダファイルの読み込み
#include "main.h"

// ライブラリ導入
#include <Meridian.h>
MERIDIANFLOW::Meridian mrd; // ライブラリのクラスを mrdという名前でインスタンス化
#include <Arduino.h>
#include <IPAddress.h>
#include <WiFi.h>             // UDPの設定
#include <WiFiUdp.h>          // UDPの設定
WiFiUDP udp;                  // wifi設定
#include <ESP32DMASPISlave.h> // DMAでSPI通信を高速化するめのライブラリ
ESP32DMASPI::Slave slave;
#include <ESP32Wiimote.h> // Wiiコントローラー
ESP32Wiimote wiimote;
#include <SPI.h> // SPI自体は使用しないがBNO055に必要

// システム用
const int MSG_BUFF = MSG_SIZE * 2;     // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
uint8_t *s_spi_meridim_dma;            // DMA用
uint8_t *r_spi_meridim_dma;            // DMA用
TaskHandle_t thp[4];                   // マルチスレッドのタスクハンドル格納用
MrdFlags flg;                          // フラグ用
MrdCheck mrdck;                        // システム管理用
UnionPad pad_array = {0};              // リモコン値格納用の配列
PadValue pad;                          // リモコンの入力結果

// Meridim配列用の共用体の設定
UnionData s_spi_meridim; // SPI受信用共用体
UnionData r_spi_meridim; // SPI受信用共用体
UnionData s_udp_meridim; // UDP送信用共用体
UnionData r_udp_meridim; // UDP受信用共用体
UnionData tmp_meridim;   // チェック用配列

//================================================================================================================
//---- SET UP ----------------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  // シリアルモニタ表示
  Serial.begin(SERIAL_PC_BPS);
  delay(100); // シリアルの開始を待ち安定化させるためのディレイ（ほどよい）

  // 起動メッセージ1
  mrd.print_esp_hello_start(VERSION, String(SERIAL_PC_BPS), WIFI_AP_SSID);

  // WiFiの初期化と開始
  WiFi.disconnect(true, true); // WiFi接続をリセット
  if (MODE_FIXED_IP)           // 固定IP設定の場合には以下を実行
  {
    if (!WiFi.config(makeIPAddress(FIXED_IP_ADDR), makeIPAddress(FIXED_IP_GATEWAY), makeIPAddress(FIXED_IP_SUBNET)))
    {
      Serial.println("Wifi Fixed IP failed to configure!");
    }
  }
  WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS); // WiFiに接続
  while (WiFi.status() != WL_CONNECTED)
  {           // https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(1); // 接続が完了するまでループで待つ
  }

  // 起動メッセージ2
  mrd.print_esp_hello_ip(WIFI_SEND_IP, WiFi.localIP().toString(), FIXED_IP_ADDR, MODE_FIXED_IP);

  // UDP通信の開始
  udp.begin(UDP_RESV_PORT);
  delay(100);

  // Bluetoothの開始
  bt_settings();

  // SPI送受信用DMAの設定
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  // 注:バッファサイズは4で割り切れる必要があり, また末尾に4バイト分0が入る不具合があるので+4で対策

  // SPI送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4); // ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4); // ※+4は不具合対策

  // SPI通信の初回送信データをセット
  memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);                                   // ※+4は不具合対策
  s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE); // データ末尾にチェックサムを入れる
  memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);                   // 送信データをDMAバッファに転記

  // SPI通信の設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1);  // キューサイズ とりあえず1
  slave.begin();          // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12

  // スレッドの開始
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);
  // xTaskCreatePinnedToCore(Core1_SPI_r, "Core1_SPI_r", 4096, NULL, 5, &thp[3], 0);
  //  注：無線系はすべてCORE0で動く.メインループはCORE1

  Serial.println("-) Meridian TWIN system on side ESP32 now flows. (-"); //
}

//================================================================================================================
//---- MAIN LOOP -------------------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{
  //------------------------------------------------------------------------------------
  //   [ 1 ] UDP受信待受ループ
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[1-1] UDP受信の待受
  flg.udp_rcvd = false;
  int udp_time_count = 0;
  while (!flg.udp_rcvd) // データの受信バッファ確認
  {
    delay(1);
    short udp_packet = udp.parsePacket();
    if (udp_packet >= MSG_BUFF)
    {
      udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
      flg.udp_rcvd = true;
      mrd.monitor_check_flow("UdpRcvd", MONITOR_FLOW); // 動作チェック用シリアル表示
    }
    if (flg.udp_board_passive)
    {                                                         // パッシブモードならUDP待ちでタイムアウトしない
      mrd.monitor_check_flow("*Passive mode.", MONITOR_FLOW); // 動作チェック用シリアル表示
    }
    else if (udp_time_count > UDP_TIMEOUT) // UDPの受信待ちのタイムアウト
    {
      mrd.monitor_check_flow("*UdpResvTimeOut", MONITOR_FLOW); // 動作チェック用シリアル表示
      break;
    }
    udp_time_count++;
  }
  flg.udp_busy = false; // UDP使用中のフラグをサゲる

  //------------------------------------------------------------------------------------
  //   [ 2 ] UDP受信品質チェック
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[2-1] UDP受信データ r_udp_meridim のチェックサムを確認.
  if (mrd.cksm_rslt(r_udp_meridim.sval, MSG_SIZE))
  {
    // @[2-1a] 受信成功ならUDP受信データをSPI送信データに上書き更新する.
    memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
    s_spi_meridim.bval[MSG_ERR_u] &= 0b10111111; // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをサゲる.
  }
  else
  {
    // @[2-1b] 受信失敗なら今回の受信データを使わず, 前回のSPI送信データにエラーフラグだけ上乗せする.
    s_spi_meridim.bval[MSG_ERR_u] |= 0b01000000; // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをアゲる.
  }

  // @[2-2] 連番スキップ検出
  // @[2-2-1] シーケンス番号の予想値の生成
  mrdck.seq_r_expect = mrd.seq_predict_num(mrdck.seq_r_expect);
  if (MONITOR_SEQ)
  {
    Serial.print("SeqNum exp: ");
    Serial.print(mrdck.seq_r_expect);
    Serial.print(" / udp_rcvd: ");
    Serial.print(int(r_udp_meridim.usval[1]));
  }

  // @[2-2-2] シーケンス番号予想値と受信値が合致しているかのチェック
  if (mrd.seq_compare_nums(mrdck.seq_r_expect, int(r_udp_meridim.usval[1]))) // 受信シーケンス番号の値が予想通りなら,
  {
    s_spi_meridim.bval[MSG_ERR_u] &= 0b11111011; // エラーフラグ10番(ESP受信のスキップ検出)をサゲる.
    if (MONITOR_SEQ)
    {
      Serial.println("  ok.");
    }
  }
  else // 受信シーケンス番号の値が予想と違ったら,
  {
    mrdck.seq_r_expect = int(r_udp_meridim.usval[1]); // 現在の受信値を予想結果としてキープ
    s_spi_meridim.bval[MSG_ERR_u] |= 0b00000100;      // エラーフラグ10番(ESP受信のスキップ検出)をアゲる.
    if (mrdck.seq_r_past == int(r_udp_meridim.usval[1]))
    {
      if (MONITOR_SEQ)
      {
        Serial.println(" same.");
      }
    }
    else
    {
      if (MONITOR_SEQ)
      {
        Serial.println(" *NG*");
      }
    }
  }
  mrdck.seq_r_past = int(r_udp_meridim.usval[1]);

  // [check!] ここで s_spi_meridim にはチェック済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.

  //------------------------------------------------------------------------------------
  //   [ 3 ] 受信値による処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[3-1] マスターコマンドによる処理
  if (s_spi_meridim.sval[0] == MCMD_BOARD_TRANSMIT_PASSIVE) // ボードが受信を待ち返信するモード（PC側が定刻送信）
  {
    flg.udp_board_passive = 1;
  }

  if (s_spi_meridim.sval[0] == MCMD_BOARD_TRANSMIT_ACTIVE) // ボードが受信を待たず返信するモード（PC側が定刻送信）
  {
    flg.udp_board_passive = 0;
  }

  //------------------------------------------------------------------------------------
  //   [ 4 ] SPI送信データ作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[4-1] ユーザー定義の送信データの書き込み
  // ・Teensyへ送るデータをここで作成, s_spi_meridim.svalに書き込み.

  // @[4-2] リモコンデータの書き込み
  for (int i = 0; i < 4; i++)
  { // Meridim配列のリモコン該当箇所に, ESPで受信したリモコン値を加算する
    s_spi_meridim.sval[i + 15] = r_udp_meridim.sval[i + 15] | pad_array.usval[i];
  }

  // @[4-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）
  // → すでにPCから受け取った値がs_spi_meridim.sval[1]に入っているのでここでは何もしない.

  // @[4-4] チェックサムの追記
  s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);
  // [check!] ここでSPI送信データ"s_spi_meridim"はチェックサムが入り完成している状態.

  //------------------------------------------------------------------------------------
  //   [ 5 ] SPI送受信の実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", MONITOR_FLOW); // 動作チェック用シリアル表示

  flg.spi_rcvd = false;
  while (!flg.spi_rcvd)
  {
    // @[5-1] SPI通信のトランザクションがなければデータをキューに補充
    if (slave.remained() == 0)
    {
      memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
      slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    }
    delay(1);
    // @[5-2] SPI通信で受信したデータについての処理
    if (slave.available())
    {
      memcpy(tmp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4); // DMAのSPI受信データをtmp配列に転記
      slave.pop();                                               // DMAのデータ配列の先頭を削除
      if (tmp_meridim.sval[0] != MCMD_DUMMY_DATA)
      {
        if (mrd.cksm_rslt(tmp_meridim.sval, MSG_SIZE)) // 受信データのチェックサム
        {
          memcpy(s_udp_meridim.bval, tmp_meridim.bval, MSG_BUFF); // DMAのSPI受信データをUDP送信配列に転記
          flg.spi_rcvd = true;
        }
      }
    }
  }

  //------------------------------------------------------------------------------------
  //   [ 6 ] UDP送信データ作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[6-1] このESP32内で計算処理したデータをs_udp_meridim.svalに格納する
  // ・Teensy→ESP32→PCという経路.
  // ・Teensyからのリモコン値等はここでのみ補足可能.
  // ・ここでチェックサムを行ってもよい.
  // ・今は特になにもなし.

  //------------------------------------------------------------------------------------
  //   [ 7 ] UDP送信実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]\n", MONITOR_FLOW); // 動作チェック用シリアル表示

  // @[7-1] UDP送受信の実行
  while (flg.bt_busy)
  {
    delay(1);
  }
  flg.udp_busy = true; // UDP使用中のフラグをアゲる
  udp_send();          // UDP送受信の実行
}

//================================================================================================================
//   [ 関 数 各 種
//================================================================================================================

/// @brief 文字列形式のIPv4アドレスをIPAddressオブジェクトに変換します.
/// @param ip_str ドット10進記法で記述されたIPv4アドレスを含む文字列.
/// @return 提供された文字列から構築されたIPAddressオブジェクト.
IPAddress makeIPAddress(const char *ip_str)
{
  int _a, _b, _c, _d;
  sscanf(ip_str, "%d.%d.%d.%d", &_a, &_b, &_c, &_d);
  return IPAddress(_a, _b, _c, _d);
}

/// @brief Bluetoothデバイスアドレスを文字列形式に変換します.
/// @param bda Bluetoothデバイスアドレスを表す6バイトの配列.
/// @param str 変換されたアドレスを格納するための文字列バッファ.
/// @param size 文字列バッファのサイズ.
/// @return 正常に変換できた場合は文字列バッファへのポインタを, エラーが発生した場合はNULLを返します.
char *bda2str(const uint8_t *bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18)
  {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

/// @brief Wiiリモコンからの入力データを受信し, 処理します.
void pad_wiimote_receive()
{
  wiimote.task();
  while (flg.udp_busy)
  {
    delay(1);
  }
  flg.bt_busy = true;
  if (wiimote.available() > 0)
  {
    static bool isFirstCall = true; // 初回の呼び出しフラグ
    if (isFirstCall)
    {
      Serial.println("Wiimote successfully connected. ");
      isFirstCall = false; // 初回の呼び出しフラグをオフにする
    }
    uint16_t button = wiimote.getButtonState();
    pad.btn = 0;
    for (int i = 0; i < 16; i++)
    {
      uint16_t mask = 1 << i;
      if ((JOYPAD_GENERALIZE && (PAD_WIIMOTE_SOLO[i] & button)) || (!JOYPAD_GENERALIZE && (PAD_WIIMOTE_ORIG[i] & button)))
      {
        pad.btn |= mask;
      }
    }
    pad_array.usval[0] = pad.btn; // short型で4個の配列データを持つ
  }
  if (MOUNT_JOYPAD == 6)
  {
    // NunchukState nunchuk = wiimote.getNunchukState();
    //  int calib_l1x = 5;  // キャリブレーション値
    //   int calib_l1y = -6; // キャリブレーション値
    //    pad.stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
    //     if (nunchuk.cBtn == 1)
    //       pad.btn |= (0b00000100 * 256) + 0b00000000;
    //      if (nunchuk.zBtn == 1)
    //        pad.btn |= (0x00000001 * 256) + 0b00000000;
    //       pad_array.sval[1] = pad.stick_L_x * 256 + pad.stick_L_y;
    //       pad_array.sval[2] = pad.stick_R_x * 256 + pad.stick_R_y;
    //       pad_array.sval[3] = pad.L2_val * 256 + pad.R2_val;
  }
  flg.bt_busy = false;
}

/// @brief UDPを使用してメリディムデータを送信します.
void udp_send()
{
  udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // Start UDP packet.
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // End UDP packet.
}

/// @brief UDPを介して受信されたメリディムデータを読み込みます.
void udp_receive()
{
  if (udp.parsePacket() >= MSG_BUFF) // Check the receive buffer for data.
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); // Receive data.
  }
}

/// @brief Bluetoothの設定を行い, 条件に応じてWiiコントローラの接続を開始します.
void bt_settings()
{
  // Wiiコントローラの接続開始
  if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
  {
    Serial.println("Try to connect Wiimote...");
    wiimote.init();
  }
}

//================================================================================================================
//   Bluetooth用スレッド
//================================================================================================================

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチンです.
/// @param args この関数に渡される引数ですが, 現在は使用されていません.
void Core0_BT_r(void *args)
{ // サブCPU(Core0)で実行するプログラム
  int interval = JOYPAD_POLLING - 1;
  if (interval < 0)
  {
    interval = 0;
  }

  while (true)
  {                       // Bluetooth待受用の無限ループ
    while (!flg.udp_busy) // UDP使用中を避ける
    {
      // Wiimote
      if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
      {
        pad_wiimote_receive();
      }
      delay(interval); // interval ms
    }
    delay(1); // 1ms
  }
}

/// @brief サブCPU (Core1) で実行されるSPI通信用のルーチンです.
/// @param args この関数に渡される引数ですが, 現在は使用されていません.
void Core1_SPI_r(void *args)
{ // サブCPU(Core1)で実行するプログラム
  while (true)
  {
    delay(100); // 1ms
  }
}
