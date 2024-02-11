// 2023.01.14 AM

// Meridian_TWIN_for_ESP32_20240107 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Meridan TWIN ESP32用スケッチ 20240107版
// スレッド制を見直し, 連続する工程をひとまとまりに.
// UDPの受信待受とSPI待受でループするようにした.
// UDP受信についてタイムアウトを導入.
// 動作が大幅に安定し通信エラーはほぼゼロ。
// PCからのリモコン受信をTeensyにも反映できるように調整 ここまで2022年までの改訂.
// 2023.05.03 変数を調整し,関数とライブラリを使うように変更
// 2023.05.03 各種設定はconfig.hからまとめて行えるようにした.
// 2023.05.03 シーケンス番号を0-60000に設定, udp受信値についてチェック.
// 2023.05.03 固定IPアドレスが設定可能.
// 2023.06.17 起動メッセージをwifi検出前から出るように変更.
// 2023.06.17 フローモニタ（MONITOR_FLOWでオンオフ）, 起動メッセージをライブラリ関数化.
// 2023.06.30 ライブラリ、変数の整合
// 2023.07.01 処理順についての日本語コメントの整理
// 2023.07.01 関数コメントの整理
// 2023.07.01 リモコン関連の調整
// 2023.12.31 key.hの分離
// 2023.12.31 stepモードに対応
// 2024.01.07 SPIの送信、受信分離に対応

// 【課題】2022.07.30
// PS4リモコンを接続するとTeensyの受信スキップが5~10%発生する.
// Wiiリモコンを接続すると受信スキップは1%以下だが、最初のコネクション確立がしにくい.
// よって現状では通信に影響のないKRC-5FHをTeensy側に接続するのがよい.
// I2C経由の無線でさまざまなコントローラーに接続できるものを開発中。

#define VERSION "Meridian_TWIN_for_ESP32_20240107." // バージョン表示

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* コンフィグファイルの読み込み */
#include "config.h"
#include "keys.h"

/* ヘッダファイルの読み込み */
#include "main.h"

/* ライブラリ導入 */
#include <Meridian.h>
MERIDIANFLOW::Meridian mrd; // ライブラリのクラスを mrdという名前でインスタンス化
#include <Arduino.h>
#include <IPAddress.h>
#include <WiFi.h>             // UDPの設定
#include <WiFiUdp.h>          // UDPの設定
WiFiUDP udp;                  // wifi設定
#include <ESP32DMASPISlave.h> // DMAでSPI通信を高速化するめのライブラリ
ESP32DMASPI::Slave slave;
#include <PS4Controller.h> // PS4コントローラー
#include <ESP32Wiimote.h>  // Wiiコントローラー
ESP32Wiimote wiimote;
#include <SPI.h> // SPI自体は使用しないがBNO055に必要

/* PS4リモコンのためのペアリング情報設定用ライブラリ */
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
uint8_t pairedDeviceBtAddr[BT_PAIR_MAX_DEVICES][6];
char bda_str[18];

/* Meridim配列設定 */
const int MSG_BUFF = MSG_SIZE * 2;     // Meridim配列のバイト長
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）

/* システム用変数 */
uint8_t *s_spi_meridim_dma;            // DMA用
uint8_t *r_spi_meridim_dma;            // DMA用
TaskHandle_t thp[4];                   // マルチスレッドのタスクハンドル格納用
volatile bool flag_spi_rising = false; // SPIスレーブのCS立ち上げ監視用
int udp_time_count = 0;                // UDPタイムアウトをリセット

/* フラグ関連変数 */
bool flag_spi_ready = true;      // SPI送信の順番制御用
bool flag_spi_send_ok = false;   // SPI送信の順番制御用. ダミーデータ受信完了フラグ
bool flag_spi_rcvd_ok = false;   // SPI送信の順番制御用. 受信完了フラグ
int flag_udp_rcvd = 0;           // UDPスレッドでの受信完了フラグ 0:未完, 1:完了 -1:タイムアウト
bool flag_udp_busy = false;      // UDPスレッドでの受信中フラグ（送信抑制）
int mrd_seq_r_expect = 0;        // フレーム毎に前回受信値に+１として受信値と比較（-30000~29999)
int spi_test_counter = 0;        // フレーム毎に前回受信値に+１として受信値と比較（-30000~29999)
bool flag_send_spi_qued = false; // フレーム内で１回目のみUDPに正常値を送るためのフラグ（他の回はダミーデータ）
bool flag_board_passive = false; // ボードがパッシブモードか

/* Meridim配列用の共用体の設定 */
typedef union
{
  short sval[MSG_SIZE + 4];
  unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
  uint8_t bval[MSG_BUFF + 4];
} UnionData;
UnionData s_spi_meridim; // SPI受信用共用体
UnionData r_spi_meridim; // SPI受信用共用体
UnionData s_udp_meridim; // UDP送信用共用体
UnionData r_udp_meridim; // UDP受信用共用体
UnionData s_spi_dummy;   // SPI受信用のダミーデータ
UnionData test; // UDP受信用共用体

// UnionData pad_bt_meridim; // リモコンのBT受信用共用体のインスタンスを宣言

/* リモコン用変数 */
// int joypad_search = 3;
typedef union
{
  short sval[4];       // short型で4個の配列データを持つ
  uint16_t usval[4];   // 上記のunsigned short型
  int8_t bval[8];      // 上記のbyte型
  uint8_t ubval[8];    // 上記のunsigned byte型
  uint64_t ui64val[1]; // 上記のunsigned int16型
                       // button, pad_stick_L_x:pad_stick_L_y,
                       // pad_stick_R_x:pad_stick_R_y, pad_L2_val:pad_R2_val
} UnionPad;
UnionPad pad_array = {0}; // リモコン値格納用の配列

unsigned short pad_btn = 0;
int pad_stick_R = 0;
int pad_stick_R_x = 0;
int pad_stick_R_y = 0;
int pad_stick_L = 0;
int pad_stick_L_x = 0;
int pad_stick_L_y = 0;
int pad_stick_V = 0;
int pad_R2_val = 0;
int pad_L2_val = 0;

//================================================================================================================
//---- セ ッ ト ア ッ プ -------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  //-------------------------------------------------------------------------
  //---- 起 動 時 設 定  -----------------------------------------------------
  //-------------------------------------------------------------------------

  /* シリアルモニタ表示 */
  Serial.begin(SERIAL_PC_BPS);
  delay(120); // シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  mrd.print_esp_hello_start(String(VERSION), String(SERIAL_PC_BPS), String(WIFI_AP_SSID));

  /* WiFiの初期化と開始 */
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
  {           // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    delay(1); // 接続が完了するまでループで待つ
  }
  mrd.print_esp_hello_ip(WIFI_SEND_IP, WiFi.localIP().toString(), FIXED_IP_ADDR, MODE_FIXED_IP);

  /* UDP通信の開始 */
  udp.begin(UDP_RESV_PORT);
  delay(100);

  /* Bluetoothリモコン関連の処理 */
  bt_settings();

  /* SPI送受信用DMAの設定 */
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  // DMAは高速な上, 一度に送受信できるデータ量を増やせる
  // バッファサイズは4で割り切れる必要があり, また末尾に4バイト分0が入る不具合があるので+4で対策

  /* SPI送受信バッファをリセット */
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4); // ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4); // ※+4は不具合対策
  memset(s_spi_dummy.sval, 0, MSG_SIZE + 2);  // 配列要素を0でリセット

  /* SPI用ダミーデータを作成*/
  s_spi_dummy.sval[0] = MCMD_DUMMY_DATA; // ダミーデータは10000
  s_spi_dummy.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_dummy.sval, MSG_SIZE);
  Serial.print("dummy_cksm: ");
  Serial.println(s_spi_dummy.sval[MSG_SIZE - 1]);

  /* SPI通信の初回送信データをセット */
  memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);                                   // ※+4は不具合対策
  s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE); // データ末尾にチェックサムを入れる
  memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);                   // 送信データをDMAバッファに転記

  /* SPI通信の設定 */
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(1); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1);  // キューサイズ　とりあえず1
  slave.begin();          // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  delay(100);             // この行は削除できそう

  attachInterrupt(digitalPinToInterrupt(SPI_CS), onReceived, RISING);

  /* スレッドの開始 */
  // マルチスレッドの宣言（無線系はすべてCORE0で動く.メインループはCORE1）
  //  xTaskCreatePinnedToCore(Core1_SPI, "Core1_SPI", 4096, NULL, 15, &thp[0], 1);
  //  xTaskCreatePinnedToCore(Core0_UDP, "Core0_UDP", 4096, NULL, 10, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);

  Serial.println("-) Meridian TWIN system on side ESP32 now flows. (-"); //
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

  //------------------------------------------------------------------------------------
  //---- [ 1 ] UDP受信待受ループ ---------------------------------------------------------
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", MONITOR_FLOW); // 動作チェック用シリアル表示

  /* @[1-1] UDP受信の待受 */
  udp_time_count = 0;        // UDPタイムアウトをリセット
  flag_udp_rcvd = 0;         // UDP受信フラグを下げる
  flag_udp_busy = true;      // UDP使用中のフラグを挙げる
  while (flag_udp_rcvd == 0) // データの受信バッファ確認
  {
    short udp_packet = udp.parsePacket();
    if (udp_packet >= MSG_BUFF)
    {
      udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
      flag_udp_rcvd = 1;
      mrd.monitor_check_flow("UdpRcvd", MONITOR_FLOW); // 動作チェック用シリアル表示
      //for (int i = 0; i < MSG_SIZE; i++)
      //{
      //  Serial.print(r_udp_meridim.sval[i]);
      //  Serial.print(" ");
      //}
    }
    if ((udp_time_count > UDP_TIMEOUT)) // and (!flag_board_passive)) // UDPの受信待ちのタイムアウト
    {
      mrd.monitor_check_flow("*UdpRcvTmOut", MONITOR_FLOW); // 動作チェック用シリアル表示
      flag_udp_rcvd = -1;                                   // 受信タイムアウトで抜ける
    }
    udp_time_count++;
    delay(1);
  }
  flag_udp_busy = false; // UDP使用中のフラグをサゲる

  //------------------------------------------------------------------------------------
  //---- [ 2 ]  UDP受信品質チェック ---------------------------------------------------------
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", MONITOR_FLOW); // 動作チェック用シリアル表示

  /* @[2-1] UDP受信データ r_udp_meridim のチェックサムを確認. */
  if (mrd.cksm_rslt(r_udp_meridim.sval, MSG_SIZE))
  {
    /* @[2-1a] 受信成功ならUDP受信データをSPI送信データに上書き更新する.*/
    memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
    s_spi_meridim.bval[MSG_ERR_u] &= 0b10111111; // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをサゲる.
  }
  else
  {
    /* @[2-1b] 受信失敗なら今回の受信データを使わず、前回のSPI送信データにエラーフラグだけ上乗せする.*/
    s_spi_meridim.bval[MSG_ERR_u] |= 0b01000000;        // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをアゲる.
    mrd.monitor_check_flow("*UdpRcvErr", MONITOR_FLOW); // 動作チェック用シリアル表示
  }

  /* @[2-2] 連番スキップ検出 */
  /* @[2-2-1] シーケンス番号の予想値の生成 */
  mrd_seq_r_expect = mrd.seq_predict_num(mrd_seq_r_expect);
  if (MONITOR_SEQ)
  {
    Serial.print("SeqNum exp: ");
    Serial.print(mrd_seq_r_expect);
    Serial.print(" / udp_rcvd: ");
    Serial.print(int(r_udp_meridim.usval[1]));
  }

  /* @[2-2-2] シーケンス番号予想値と受信値が合致しているかのチェック */
  if (mrd.seq_compare_nums(mrd_seq_r_expect, int(r_udp_meridim.usval[1]))) // 受信シーケンス番号の値が予想通りなら,
  {
    s_spi_meridim.bval[MSG_ERR_u] &= 0b11111011; // エラーフラグ10番(ESP受信のスキップ検出)をサゲる.
    if (MONITOR_SEQ)
    {
      Serial.println("  ok.");
    }
  }
  else // 受信シーケンシャルカウンタの値が予想と違ったら,
  {
    mrd_seq_r_expect = int(r_udp_meridim.usval[1]); // 現在の受信値を予想結果としてキープ
    s_spi_meridim.bval[MSG_ERR_u] |= 0b00000100;    // エラーフラグ10番(ESP受信のスキップ検出)をアゲる.
    if (MONITOR_SEQ)
    {
      Serial.println(" *NG*");
    }
  }

  // [check!] ここで s_spi_meridim にはチェック済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.

  //------------------------------------------------------------------------------------
  //---- [ 3 ] SPI送信データ作成 ---------------------------------------------------------
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", MONITOR_FLOW); // 動作チェック用シリアル表示

  /* @[3-2] リモコンデータの書き込み */
  for (int i = 0; i < 4; i++)
  { // Meridim配列のリモコン該当箇所に、ESPで受信したリモコン値を加算する
    s_spi_meridim.sval[i + 15] = r_udp_meridim.sval[i + 15] | pad_array.usval[i];
  }

  /* @[3-4] チェックサムの追記 */
  s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);
  // [check!] ここでSPI送信データ"s_spi_meridim"はチェックサムが入り完成している状態.
  Serial.print("SndCksm:");
  Serial.print(s_spi_meridim.sval[MSG_SIZE - 1]);

  /* @[3-5] 完成したSPI送信データをDMAに転記*/
  memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
  flag_spi_ready = true; // SPI送信の順番制御用

  //for (int i = 0; i < MSG_SIZE; i++)
  //{
  //  Serial.print(s_spi_meridim.sval[i]);
  //  Serial.print(" ");
  //}

  //------------------------------------------------------------------------------------
  //---- [ 4 ] SPI送信データのキューセット -------------------------------------------------
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", MONITOR_FLOW); // 動作チェック用シリアル表示

  flag_send_spi_qued = false;
  flag_spi_rcvd_ok = false;
  while (!flag_spi_rcvd_ok)
  {
    /* @[1-1] SPI通信のトランザクションがなければデータをキューに補充 */
    if (slave.remained() == 0) // トランザクションの終了確認にもなっている？
    {
      // if (flag_spi_ready) // SPI送信データの作成が完了しているか
      //{
      if (flag_send_spi_qued) // 一度SPI実データを入れていたら次からはダミーを入れる
      {
        //Serial.print(flag_send_spi_qued);
        mrd.monitor_check_flow("4b", MONITOR_FLOW);                     // 動作チェック用シリアル表示
        slave.queue(r_spi_meridim_dma, s_spi_dummy.bval, MSG_BUFF + 4); // ダミデーターをキューに入れる
        //for (int i = 0; i < MSG_SIZE; i++)
        //{
        //  Serial.print(s_spi_dummy.sval[i]);
        //  Serial.print(" ");
        //}
        //Serial.println();
        //flag_send_spi_qued = true;
      }
      else
      {
        //Serial.print(flag_send_spi_qued);
        mrd.monitor_check_flow("4a", MONITOR_FLOW);                      // 動作チェック用シリアル表示
        slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4); // 送信SPIデータをキューに入れる
        memcpy(test.bval, s_spi_meridim_dma, MSG_BUFF + 4);
        for (int i = 0; i < MSG_SIZE; i++)
        {
          Serial.print(test.sval[i]);
          Serial.print(" ");
        }
        //Serial.println();
        flag_send_spi_qued = true;
      }
    }
    //Serial.println("----------");
    //delay(5);
    //------------------------------------------------------------------------------------
    //---- [ 5 ] SPI送受信完了の待ち受け ----------------------------------------------------
    //------------------------------------------------------------------------------------
    flag_spi_rcvd_ok = false;

    while (!flag_spi_rcvd_ok) // データが届いていれば転記する
    {
      while (slave.available()) // データが届いていれば転記する
      {
        mrd.monitor_check_flow("[5]", MONITOR_FLOW); // 動作チェック用シリアル表示

        memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4); // DMAのSPI受信データをUDP送信配列に転記
        slave.pop();                                                 // DMAのデータ配列の先頭を削除

        //------------------------------------------------------------------------------------
        //---- [ 6 ] SPI送受データのチェック ----------------------------------------------------
        //------------------------------------------------------------------------------------
        mrd.monitor_check_flow("[6]", MONITOR_FLOW); // 動作チェック用シリアル表示
        if (mrd.cksm_rslt(s_udp_meridim.sval, MSG_SIZE))
        {
          if (s_udp_meridim.sval[0] != MCMD_DUMMY_DATA)
          {
            mrd.monitor_check_flow("SpiOk", MONITOR_FLOW); // 動作チェック用シリアル表示
            flag_spi_rcvd_ok = true;
          }
          else
          {
            mrd.monitor_check_flow("SpiDummy", MONITOR_FLOW); // 動作チェック用シリアル表示
          }
        }
        else
        {
          Serial.print("SpiRcvError");
          flag_spi_rcvd_ok = true;
        }
      }
      delay(1);
    }

    //delay(1);
  }

  //------------------------------------------------------------------------------------
  //---- [ 7 ] UDP送信データ作成 ---------------------------------------------------------
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]", MONITOR_FLOW); // 動作チェック用シリアル表示

  //------------------------------------------------------------------------------------
  //---- [ 8 ] UDP送信実行 --------------------------------------------------------------
  //------------------------------------------------------------------------------------

  if (flag_spi_rcvd_ok)
  {
    delay(200);
    mrd.monitor_check_flow("[8]", MONITOR_FLOW); // 動作チェック用シリアル表示
    /* @[3-1] UDP送受信の実行 */
    flag_udp_busy = true; // UDP使用中のフラグをアゲる
    udp_send();           // UDP送受信の実行
    Serial.print("UdpSend");
    flag_udp_busy = false; // UDP使用中のフラグをサゲる
    Serial.println();
    Serial.println();
  }
}

//
//

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Convert a period-separated IP address string to an IPAddress object.
 *
 * @param ip_str const char*, period-separated IP address string.
 * @return IPAddress
 */
IPAddress makeIPAddress(const char *ip_str)
{
  int _a, _b, _c, _d;
  sscanf(ip_str, "%d.%d.%d.%d", &_a, &_b, &_c, &_d);
  return IPAddress(_a, _b, _c, _d);
}

/**
 * @brief Pairing Bluetooth.
 *
 */
bool initBluetooth()
{
  if (!btStart())
  {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK)
  {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK)
  {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

/**
 * @brief Get a Bluetooth pairing address.
 *
 * @param bda A pointer that points to the Bluetooth Device Address (BDA).
 *            The BDA is represented as a 6-byte array of bytes.
 * @param str A pointer to store the converted result string.
 * @param size The size of the str buffer.
 * @return char*
 */
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

/**
 * @brief Receive input values from the PS4 remote control
 *        and store them in the following variables:
 *        pad_btn, pad_L2_val, pad_R2_val, pad_stick_L, pad_stick_R, pad_stick_V
 */
void pad_ps4_receive()
{
  // Below has all accessible outputs from the controller
  if (PS4.isConnected())
  {
    static bool isFirstCall = true; // 初回の呼び出しフラグ
    if (isFirstCall)
    {
      Serial.println("PS4 controller successfully connected. ");
      isFirstCall = false; // 初回の呼び出しフラグをオフにする
    }
    pad_btn = 0;
    if (JOYPAD_GENERALIZE)
    { // 一般化,ROS準拠
      if (PS4.Up())
        pad_btn |= (0b00000000 * 256) + 0b00010000; // 16
      if (PS4.Right())
        pad_btn |= (0b00000000 * 256) + 0b00100000; // 32
      if (PS4.Down())
        pad_btn |= (0b00000000 * 256) + 0b01000000; // 64
      if (PS4.Left())
        pad_btn |= (0b00000000 * 256) + 0b10000000; // 128
      if (PS4.Triangle())
        pad_btn |= (0b00010000 * 256) + 0b00000000; // 4096
      if (PS4.Circle())
        pad_btn |= (0b00100000 * 256) + 0b00000000; // 8192
      if (PS4.Cross())
        pad_btn |= (0b01000000 * 256) + 0b00000000; // 16384
      if (PS4.Square())
        pad_btn |= (0b10000000 * 256) + 0b00000000; // 32768
      if (PS4.UpRight())
        pad_btn |= (0b00000000 * 256) + 0b00110000; // 48
      if (PS4.DownRight())
        pad_btn |= (0b00000000 * 256) + 0b01100000; // 96
      if (PS4.UpLeft())
        pad_btn |= (0b00000000 * 256) + 0b10010000; // 144
      if (PS4.DownLeft())
        pad_btn |= (0b00000000 * 256) + 0b11000000; // 192
      if (PS4.Share())
        pad_btn |= (0b00000000 * 256) + 0b00000001; // 1
      if (PS4.L3())
        pad_btn |= (0b00000000 * 256) + 0b00000010; // 2
      if (PS4.R3())
        pad_btn |= (0b00000000 * 256) + 0b00000100; // 4
      if (PS4.Options())
        pad_btn |= (0b00000000 * 256) + 0b00001000; // none
      if (PS4.L1())
        pad_btn |= (0b00000100 * 256) + 0b00000000; // 1024
      if (PS4.R1())
        pad_btn |= (0b00001000 * 256) + 0b00000000; // 2048

      if (PS4.L2())
      {
        pad_btn |= (0b00000001 * 256) + 0b00000000;
        pad_L2_val = constrain(PS4.L2Value(), 0, 255); // 256
      }
      if (PS4.R2())
      {
        pad_btn |= 512;                                //(0x00000010 * 256) + 0b00000000;
        pad_R2_val = constrain(PS4.R2Value(), 0, 255); // 512
      }

      if (PS4.PSButton())
        pad_btn |= (0b00000000 * 256) + 0b01010000; // same as up & down
      if (PS4.Touchpad())
        pad_btn |= (0b00000000 * 256) + 0b10100000; // same as left & right

      if (PS4.LStickX())
      {
        pad_stick_L_x = constrain(PS4.LStickX(), -127, 127);
      }
      if (PS4.LStickY())
      {
        pad_stick_L_y = constrain(PS4.LStickY(), -127, 127);
      }
      if (PS4.RStickX())
      {
        pad_stick_R_x = constrain(PS4.RStickX(), -127, 127);
      }
      if (PS4.RStickY())
      {
        pad_stick_R_y = constrain(PS4.RStickY(), -127, 127);
      }
    }
    else
    { // ストレート

      if (PS4.Up())
        pad_btn |= (0b00000000 * 256) + 0b00010000; // 16
      if (PS4.Right())
        pad_btn |= (0b00000000 * 256) + 0b00100000; // 32
      if (PS4.Down())
        pad_btn |= (0b00000000 * 256) + 0b01000000; // 64
      if (PS4.Left())
        pad_btn |= (0b00000000 * 256) + 0b10000000; // 128
      if (PS4.Triangle())
        pad_btn |= (0b00010000 * 256) + 0b00000000; // 4096
      if (PS4.Circle())
        pad_btn |= (0b00100000 * 256) + 0b00000000; // 8192
      if (PS4.Cross())
        pad_btn |= (0b01000000 * 256) + 0b00000000; // 16384
      if (PS4.Square())
        pad_btn |= (0b10000000 * 256) + 0b00000000; // 32768
      if (PS4.UpRight())
        pad_btn |= (0b00000000 * 256) + 0b00110000; // 48
      if (PS4.DownRight())
        pad_btn |= (0b00000000 * 256) + 0b01100000; // 96
      if (PS4.UpLeft())
        pad_btn |= (0b00000000 * 256) + 0b10010000; // 144
      if (PS4.DownLeft())
        pad_btn |= (0b00000000 * 256) + 0b11000000; // 192
      if (PS4.L1())
        pad_btn |= (0b00000100 * 256) + 0b00000000; // 1024
      if (PS4.R1())
        pad_btn |= (0b00001000 * 256) + 0b00000000; // 2048
      if (PS4.Share())
        pad_btn |= (0b00000000 * 256) + 0b00000001; // 1
      if (PS4.R3())
        pad_btn |= (0b00000000 * 256) + 0b00000010; // 2
      if (PS4.L3())
        pad_btn |= (0b00000000 * 256) + 0b00000100; // 4
      if (PS4.Options())
        pad_btn |= (0b00000000 * 256) + 0b00001000; // 8
      // if (PS4.PSButton())
      //   pad_btn |= (0b00000000 * 256) + 0b01010000; // same as up & down
      // if (PS4.Touchpad())
      //   pad_btn |= (0b00000000 * 256) + 0b10100000; // same as left & right
      if (PS4.L2())
      {
        pad_btn |= (0b00000001 * 256) + 0b00000000;
        pad_L2_val = constrain(PS4.L2Value(), 0, 255);
      }
      if (PS4.R2())
      {
        pad_btn |= 512; //(0x00000010 * 256) + 0b00000000;
        pad_R2_val = constrain(PS4.R2Value(), 0, 255);
      }
      if (PS4.LStickX())
      {
        pad_stick_L_x = constrain(PS4.LStickX(), -127, 127);
      }
      if (PS4.LStickY())
      {
        pad_stick_L_y = constrain(PS4.LStickY(), -127, 127);
      }
      if (PS4.RStickX())
      {
        pad_stick_R_x = constrain(PS4.RStickX(), -127, 127);
      }
      if (PS4.RStickY())
      {
        pad_stick_R_y = constrain(PS4.RStickY(), -127, 127);
      };
    }
    pad_array.usval[0] = pad_btn;                            // short型で4個の配列データを持つ
    pad_array.sval[1] = pad_stick_L_x * 256 + pad_stick_L_y; // short型で4個の配列データを持つ
    pad_array.sval[2] = pad_stick_R_x * 256 + pad_stick_R_y; // short型で4個の配列データを持つ
    pad_array.sval[3] = pad_L2_val * 256 + pad_R2_val;       // short型で4個の配列データを持つ
  }
}

/**
 * @brief Receive input values from the wiimote
 *        and store them in pad_btn.
 */
void pad_wiimote_receive()
{
  wiimote.task();
  if (wiimote.available() > 0)
  {
    static bool isFirstCall = true; // 初回の呼び出しフラグ
    if (isFirstCall)
    {
      Serial.println("Wiimote successfully connected. ");
      isFirstCall = false; // 初回の呼び出しフラグをオフにする
    }
    uint16_t button = wiimote.getButtonState();
    pad_btn = 0;
    for (int i = 0; i < 16; i++)
    {
      uint16_t mask = 1 << i;
      if ((JOYPAD_GENERALIZE && (PAD_WIIMOTE_SOLO[i] & button)) || (!JOYPAD_GENERALIZE && (PAD_WIIMOTE_ORIG[i] & button)))
      {
        pad_btn |= mask;
      }
    }
    pad_array.usval[0] = pad_btn; // short型で4個の配列データを持つ
  }
  if (MOUNT_JOYPAD == 6)
  {
    // NunchukState nunchuk = wiimote.getNunchukState();
    //  int calib_l1x = 5;  // キャリブレーション値
    //   int calib_l1y = -6; // キャリブレーション値
    //    pad_stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
    //     if (nunchuk.cBtn == 1)
    //       pad_btn |= (0b00000100 * 256) + 0b00000000;
    //      if (nunchuk.zBtn == 1)
    //        pad_btn |= (0x00000001 * 256) + 0b00000000;
    //       pad_array.sval[1] = pad_stick_L_x * 256 + pad_stick_L_y;
    //       pad_array.sval[2] = pad_stick_R_x * 256 + pad_stick_R_y;
    //       pad_array.sval[3] = pad_L2_val * 256 + pad_R2_val;
  }
}

/**
 * @brief Send s_udp_meridim to UDP.
 *
 */
void udp_send()
{
  udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // Start UDP packet.
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // End UDP packet.
}

/**
 * @brief Check Check received UDP packets.
 *  　　　　When a reception is complete, store the values in the union r_udp_meridim
 *  　　　　and set the flag_udp_rcvd flag to true.
 */
void udp_receive()
{
  if (udp.parsePacket() >= MSG_BUFF) // Check the receive buffer for data.
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); // Receive data.
  }
}

/**
 * @brief Setting for PS4 Bluetooth.
 *
 */
void bt_settings()
{
  /* Bluetoothの初期化 */
  if (true) // ※常にBluetoothの初期化とアドレス表示を実行する設定
  {

    initBluetooth();
    Serial.print("ESP32's Bluetooth Mac Address is => "); // ESP32自身のBluetoothMacアドレスを表示
    Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
    delay(100);

    /* Bluetoothのペアリング情報 */
    int bt_count = esp_bt_gap_get_bond_device_num();
    if (!bt_count)
    {
      Serial.println("No bonded BT device found.");
    }
    else
    {
      Serial.print("Bonded BT device count: ");
      Serial.println(bt_count);
      if (BT_PAIR_MAX_DEVICES < bt_count)
      {
        bt_count = BT_PAIR_MAX_DEVICES;
        Serial.print("Reset bonded device count: ");
        Serial.println(bt_count);
      }
      esp_err_t tError = esp_bt_gap_get_bond_device_list(&bt_count, pairedDeviceBtAddr);
      if (ESP_OK == tError)
      {
        for (int i = 0; i < bt_count; i++)
        {
          Serial.print("Found bonded BT device # ");
          Serial.print(i);
          Serial.print(" -> ");
          Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
          if (BT_REMOVE_BONDED_DEVICES)
          {
            esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
            if (ESP_OK == tError)
            {
              Serial.print("Removed bonded BT device # ");
            }
            else
            {
              Serial.print("Failed to remove bonded BT device # ");
            }
            Serial.println(i);
          }
        }
      }
    }
  }

  /* PS4コントローラの接続開始 */
  if (MOUNT_JOYPAD == 4)
  {
    PS4.begin(BT_MAC_ADDR); // ESP32のMACが入ります.PS4にも設定します.
    Serial.println("Try to connect PS4 controller...");
  }

  /* Wiiコントローラの接続開始 */
  if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
  {
    Serial.println("Try to connect Wiimote...");
    wiimote.init();
  }
}

//================================================================================================================
//---- Bluetooth 用 ス レ ッ ド -----------------------------------------------------------------------------------
//================================================================================================================
void Core0_BT_r(void *args)
{ // サブCPU(Core0)で実行するプログラム
  int _wait = JOYPAD_POLLING - 1;
  if (_wait < 0)
  {
    _wait = 0;
  }

  while (true)
  {                        // Bluetooth待受用の無限ループ
    while (!flag_udp_busy) // UDP使用中を避ける
    {
      // PS4 controller
      if (MOUNT_JOYPAD == 4)
      {
        pad_ps4_receive();
      }
      // Wiimote
      if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
      {
        pad_wiimote_receive();
      }
      delay(_wait); // _wait ms
    }
    delay(1); // 1ms
  }
}

void IRAM_ATTR onReceived()
{
  flag_spi_rising = true;
  //  spi_test_counter++;
  //  Serial.print("SPI RISING:");
  //  Serial.println(spi_test_counter);
}
