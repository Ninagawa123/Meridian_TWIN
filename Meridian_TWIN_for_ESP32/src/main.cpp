// Meridian_TWIN_for_ESP32_20230507 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Meridan TWIN ESP32用スケッチ　202507版
// スレッド制を見直し, 連続する工程をひとまとまりに.
// UDPの受信待受とSPI待受でループするようにした.
// UDP受信についてタイムアウトを導入.
// 動作が大幅に安定し通信エラーはほぼゼロ。
// PCからのリモコン受信をTeensyにも反映できるように調整　ここまで2022年までの改訂.
// 2023.05.03 変数を調整し,関数とライブラリを使うように変更
// 2023.05.03 各種設定はconfig.hからまとめて行えるようにした.
// 2023.05.03 シーケンス番号を0-60000に設定, udp受信値についてチェック.
// 2023.05.03 固定IPアドレスが設定可能.
// 2023.05.07 起動メッセージ時,Wifiが接続できていないときはSearching WIFIで止まるようにした.

// 【課題】2022.07.30
// PS4リモコンを接続するとTeensyの受信スキップが5~10%発生する.
// Wiiリモコンを接続すると受信スキップは1%以下だが、最初のコネクション確立がしにくい.
// よって現状では通信に影響のないKRC-5FHをTeensy側に接続するのがよい.
// I2C経由の無線でさまざまなコントローラーに接続できるものを開発中。

#define VERSION "Meridian_TWIN_for_ESP32_20230507." // バージョン表示

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* コンフィグファイルの読み込み */
#include "config.h"

/* ヘッダファイルの読み込み */
#include "main.h"

/* ライブラリ導入 */
#include "Meridian.h"
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
uint8_t *s_spi_meridim_dma; // DMA用
uint8_t *r_spi_meridim_dma; // DMA用
TaskHandle_t thp[4];        // マルチスレッドのタスクハンドル格納用

/* フラグ関連変数 */
bool spi_ready_flag = true;  // SPI送信の順番制御用
bool udp_rsvd_flag = true;   // UDPスレッドでの受信完了フラグ
bool udp_busy_flag = false;  // UDPスレッドでの受信中フラグ（送信抑制）
int frame_sync_r_expect = 0; // フレーム毎に前回受信値に+１として受信値と比較（-30000~29999)

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
// UnionData pad_bt_meridim; // リモコンのBT受信用共用体のインスタンスを宣言

/* リモコン用変数 */
int joypad_search = 3;
unsigned short pad_btn = 0;
short pad_stick_R = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_stick_V = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;

//================================================================================================================
//---- セ ッ ト ア ッ プ -------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  //-------------------------------------------------------------------------
  //---- 起 動 時 設 定  -----------------------------------------------------
  //-------------------------------------------------------------------------
  /* WiFiの初期化と開始 */
  WiFi.disconnect(true, true); // WiFi接続をリセット
  if (MODE_FIXED_IP)           // 固定IP設定の場合には以下を実行
  {
    if (!WiFi.config(mrd.makeIPAddress(FIXED_IP_ADDR), mrd.makeIPAddress(FIXED_IP_GATEWAY), mrd.makeIPAddress(FIXED_IP_SUBNET)))
    {
      Serial.println("Wifi Fixed IP failed to configure!");
    }
  }
  WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS); // WiFiに接続
  while (WiFi.status() != WL_CONNECTED)
  {           // https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(1); // 接続が完了するまでループで待つ
  }

  /* シリアルモニタ表示1 */
  Serial.begin(SERIAL_PC_BPS);
  delay(120); // シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  print_hello_esp1();
  // mrd.print_hello_esp(VERSION, MODE_FIXED_IP, String(FIXED_IP_ADDR), String(WiFi.localIP()), String(WIFI_AP_SSID), String(WIFI_SEND_IP));

  /* UDP通信の開始 */
  udp.begin(UDP_RESV_PORT);
  delay(100);

  /* シリアルモニタ表示2 */
  print_hello_esp2();

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

  /* SPI通信の初回送信データをセット */
  memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);                                   // ※+4は不具合対策
  s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE); // データ末尾にチェックサムを入れる
  memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);                   // 送信データをDMAバッファに転記

  /* SPI通信の設定 */
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1);  // キューサイズ　とりあえず1
  slave.begin();          // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  delay(100);             // この行は削除できそう

  /* スレッドの開始 */
  // マルチスレッドの宣言（無線系はすべてCORE0で動く.メインループはCORE1）
  //  xTaskCreatePinnedToCore(Core1_SPI, "Core1_SPI", 4096, NULL, 15, &thp[0], 1);
  //  xTaskCreatePinnedToCore(Core0_UDP, "Core0_UDP", 4096, NULL, 10, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);
}

//================================================================================================================
//---- メ　イ　ン　ル　ー　プ-----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

  //------------------------------------------------------------------------
  // [ 1 ]  S P I 送 受 信 の 実 行
  //------------------------------------------------------------------------

  /* @[1-1] SPI通信のトランザクションがなければデータをキューに補充 */
  if (slave.remained() == 0)
  {
    if (spi_ready_flag) // SPI送信データの作成が完了しているか
    {
      monitor_check_flow("[1]"); // 動作チェック用シリアル表示
      slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
      spi_ready_flag = false;
    }
  }

  // [check!] ここがTeensyをマスターとして実施されるSPI通信の完了を待つ待機ループになる

  /* @[1-2] SPI通信で受信したデータについての処理（工程[6] まで一気に行う） */
  while (slave.available()) // SPI受信を完了したデータについての作業はこのwhileの中で行う
  {
    memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4); // DMAのSPI受信データをUDP送信配列に転記
    slave.pop();                                                 // DMAのデータ配列の先頭を削除

    //------------------------------------------------------------------------
    // [ 2 ]  U D P 送 信 デ ー タ 作 成
    //------------------------------------------------------------------------
    monitor_check_flow("[2]"); // 動作チェック用シリアル表示

    // → ここでチェックサムを行ってもよい.
    // → Teensyからのリモコン値はここでのみ補足可能.
    // → 今回は特になにもなし.

    //------------------------------------------------------------------------
    // [ 3 ]  U D P 送 信 実 行
    //------------------------------------------------------------------------
    monitor_check_flow("[3]"); // 動作チェック用シリアル表示
    /* @[3-1] UDP送受信の実行 */
    udp_busy_flag = true; // UDP使用中のフラグをアゲる
    sendUDP();            // UDP送受信の実行

    //------------------------------------------------------------------------
    // [ 4 ]  U D P 受 信 待 受 ル ー プ
    //------------------------------------------------------------------------
    monitor_check_flow("[4]"); // 動作チェック用シリアル表示

    /* @[4-1] UDP受信の待受 */
    udp_rsvd_flag = false;
    int udp_time_count = 0;
    while (!udp_rsvd_flag) // データの受信バッファ確認
    {
      short udp_packet = udp.parsePacket();
      if (udp_packet >= MSG_BUFF)
      {
        udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
        udp_rsvd_flag = true;
        monitor_check_flow("UdpRsvd"); // 動作チェック用シリアル表示
      }
      if (udp_time_count > UDP_TIMEOUT) // UDPの受信待ちのタイムアウト
      {
        monitor_check_flow("*UdpResvTimeOut"); // 動作チェック用シリアル表示
        break;
      }
      udp_time_count++;
      delay(1);
    }
    udp_busy_flag = false; // UDP使用中のフラグをサゲる

    //------------------------------------------------------------------------
    // [ 5 ]  U D P 受 信 品 質 チ ェ ッ ク
    //------------------------------------------------------------------------
    monitor_check_flow("[5]"); // 動作チェック用シリアル表示

    /* @[5-1] UDP受信データ r_udp_meridim のチェックサムを確認. */
    if (mrd.cksm_rslt(r_udp_meridim.sval, MSG_SIZE))
    {
      /* @[5-2a] 受信成功ならUDP受信データをSPI送信データに上書き更新する.*/
      memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
      s_spi_meridim.bval[MSG_ERR_u] &= 0b10111111; // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをサゲる.
    }
    else
    {
      /* @[5-2b] 受信失敗なら今回の受信データを使わず、前回のSPI送信データにエラーフラグだけ上乗せする.*/
      s_spi_meridim.bval[MSG_ERR_u] |= 0b01000000; // meridimの[MSG_ERR]番の14ビット目(ESPのUDP受信成否)のフラグをアゲる.
    }

    /* @[5-3] 連番スキップ検出 */
    /* @[5-3-1] シーケンス番号の予想値の生成 */
    frame_sync_r_expect = mrd.predict_seq_num(frame_sync_r_expect);

    if (MONITOR_SEQ)
    {
      Serial.print("SeqNum exp: ");
      Serial.print(frame_sync_r_expect);
      Serial.print(" / udp_rsvd: ");
      Serial.print(int(r_udp_meridim.usval[1]));
    }

    /* @[5-3-2] シーケンス番号予想値と受信値が合致しているかのチェック */
    if (mrd.predict_seq_nums(frame_sync_r_expect, int(r_udp_meridim.usval[1]))) // 受信シーケンス番号の値が予想通りなら,
    {
      s_spi_meridim.bval[MSG_ERR_u] &= 0b11111011; // エラーフラグ10番(ESP受信のスキップ検出)をサゲる.
      if (MONITOR_SEQ)
      {
        Serial.println("  ok.");
      }
    }
    else // 受信シーケンシャルカウンタの値が予想と違ったら,
    {
      frame_sync_r_expect = int(r_udp_meridim.usval[1]); // 現在の受信値を予想結果としてキープ
      s_spi_meridim.bval[MSG_ERR_u] |= 0b00000100;       // エラーフラグ10番(ESP受信のスキップ検出)をアゲる.
      if (MONITOR_SEQ)
      {
        Serial.println(" *NG*");
      }
    }

    // [check!] ここで s_spi_meridim にはチェック済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.

    //------------------------------------------------------------------------
    // [ 6 ]  S P I 送 信 デ ー タ 作 成
    //------------------------------------------------------------------------
    monitor_check_flow("[6]\n"); // 動作チェック用シリアル表示

    /* @[6-1] ユーザー定義の送信データの書き込み */
    // Teensyへ送るデータをこのパートで作成, 書き込み  → 今回はとくに何もしない.

    /* @[6-2] リモコンデータの書き込み */
    s_spi_meridim.sval[15] = r_udp_meridim.sval[15] | pad_btn;
    s_spi_meridim.sval[16] = r_udp_meridim.sval[16] | pad_stick_L;
    s_spi_meridim.sval[17] = r_udp_meridim.sval[17] | pad_stick_R;
    s_spi_meridim.sval[18] = r_udp_meridim.sval[18] | pad_stick_V;

    /* @[6-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）*/
    // → すでにPCから受け取った値がs_spi_meridim.sval[1]に入っているのでここでは何もしない.

    /* @[6-4] チェックサムの追記 */
    s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);
    // [check!] ここでSPI送信データ"s_spi_meridim"はチェックサムが入り完成している状態.

    /* @[6-5] 完成したSPI送信データをDMAに転記*/
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
    spi_ready_flag = true; // SPI送信の順番制御用
    // [check!] この時点で 次回のSPI送受信に備え、DMAにデータが格納された状態.
  }
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// +----------------------------------------------------------------------
// | 関数名　　:  initBluetooth()
// +----------------------------------------------------------------------
// | 機能     :  Bluetoothペアリングを設定する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
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

// +----------------------------------------------------------------------
// | 関数名　　:  *bda2str(const uint8_t* bda, char *str, size_t size)
// +----------------------------------------------------------------------
// | 機能     :  Bluetoothペアリングアドレスを取得する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
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

// +----------------------------------------------------------------------
// | 関数名　　:  PS4pad_receive()
// +----------------------------------------------------------------------
// | 機能     :  PS4リモコンの入力値を受信し, 以下に値を格納する
// | 　　        pad_btn, pad_L2_val, pad_R2_val, pad_stick_L , pad_stick_R, pad_stick_V
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void PS4pad_receive()
{
  // Below has all accessible outputs from the controller
  if (PS4.isConnected())
  {
    pad_btn = 0;

    if (PS4.Right())
      pad_btn |= (0b00000000 * 256) + 0b00100000;
    if (PS4.Down())
      pad_btn |= (0b00000000 * 256) + 0b01000000;
    if (PS4.Up())
      pad_btn |= (0b00000000 * 256) + 0b00010000;
    if (PS4.Left())
      pad_btn |= (0b00000000 * 256) + 0b10000000;

    if (PS4.Square())
      pad_btn |= (0b10000000 * 256) + 0b00000000;
    if (PS4.Cross())
      pad_btn |= (0b01000000 * 256) + 0b00000000;
    if (PS4.Circle())
      pad_btn |= (0b00100000 * 256) + 0b00000000;
    if (PS4.Triangle())
      pad_btn |= (0b00010000 * 256) + 0b00000000;

    if (PS4.UpRight())
      pad_btn |= (0b00000000 * 256) + 0b00110000;
    if (PS4.DownRight())
      pad_btn |= (0b00000000 * 256) + 0b01100000;
    if (PS4.UpLeft())
      pad_btn |= (0b00000000 * 256) + 0b10010000;
    if (PS4.DownLeft())
      pad_btn |= (0b00000000 * 256) + 0b11000000;

    if (PS4.L1())
      pad_btn |= (0b00000100 * 256) + 0b00000000;
    if (PS4.R1())
      pad_btn |= (0b00001000 * 256) + 0b00000000;

    if (PS4.Share())
      pad_btn |= (0b00000000 * 256) + 0b00000001;
    if (PS4.Options())
      pad_btn |= (0b00000000 * 256) + 0b00001000;
    if (PS4.L3())
      pad_btn |= (0b00000000 * 256) + 0b00000100;
    if (PS4.R3())
      pad_btn |= (0b00000000 * 256) + 0b00000010;

    if (PS4.PSButton())
      pad_btn |= (0b00000000 * 256) + 0b01010000; // same as up & down
    if (PS4.Touchpad())
      pad_btn |= (0b00000000 * 256) + 0b00101000; // same as left & right

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
    }

    pad_stick_L = pad_stick_L_x * 256 + pad_stick_L_y;
    pad_stick_R = pad_stick_R_x * 256 + pad_stick_R_y;
    pad_stick_V = pad_L2_val * 256 + pad_R2_val;
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  Wiipad_receive_h()
// +----------------------------------------------------------------------
// | 機能     :  Wiiコントローラ(横持ち・横持ち+ヌンチャク)の入力値を受信し, 以下に値を格納する
// | 　　        pad_btn, pad_stick_L
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void Wiipad_receive_h()
{
  wiimote.task();
  if (wiimote.available() > 0)
  {
    uint16_t button = wiimote.getButtonState();
    pad_btn = 0;

    if (button & 0x0400)
      pad_btn |= (0b00000000 * 256) + 0b00100000; // right
    if (button & 0x0100)
      pad_btn |= (0b00000000 * 256) + 0b01000000; // down
    if (button & 0x0200)
      pad_btn |= (0b00000000 * 256) + 0b00010000; // up
    if (button & 0x0800)
      pad_btn |= (0b00000000 * 256) + 0b10000000; // left

    if (button & 0x0008)
      pad_btn |= (0b10000000 * 256) + 0b00000000; // A
    if (button & 0x0002)
      pad_btn |= (0b01000000 * 256) + 0b00000000; // 1
    if (button & 0x0001)
      pad_btn |= (0b00100000 * 256) + 0b00000000; // 2
    if (button & 0x0004)
      pad_btn |= (0b00010000 * 256) + 0b00000000; // B

    if (button & 0x0010)
      pad_btn |= (0b00000000 * 256) + 0b00000001; //+
    if (button & 0x1000)
      pad_btn |= (0b00000000 * 256) + 0b00001000; //-

    if (button & 0x0080)
      pad_btn |= (0b00000000 * 256) + 0b01010000; // same as up & down//home

    if (MOUNT_JOYPAD == 6)
    {
      NunchukState nunchuk = wiimote.getNunchukState();
      int calib_l1x = 5;  // LスティックX軸のセンターのキャリブレーション値
      int calib_l1y = -6; // LスティックY軸のセンターのキャリブレーション値
      pad_stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
      // if (nunchuk.cBtn == 1)
      //    pad_btn |= (0b00000100 * 256) + 0b00000000;
      //  if (nunchuk.zBtn == 1)
      //    pad_btn |= (0x00000001 * 256) + 0b00000000;
    }
    delay(1); // ここの数値でCPU負荷を軽減できるかも?
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  sendUDP()
// +----------------------------------------------------------------------
// | 機能     :  共用体s_udp_meridimをUDP通信で送信する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void sendUDP()
{
  udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケットの開始
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // UDPパケットの終了
}

// +----------------------------------------------------------------------
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  UDP通信の受信パケットを確認する.
// | 　　        受信完了を検出したら共用体 r_udp_meridim に値を格納し,
// | 　　        udp_rsvd_flag のフラグをtrueにする
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void receiveUDP()
{
  if (udp.parsePacket() >= MSG_BUFF) // データの受信バッファ確認
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  bt_settings()
// +----------------------------------------------------------------------
// | 機能     :  Bluttooth接続リモコン (PS4,wiimote)関連の設定
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void bt_settings()
{
  /* Bluetoothの初期化 */
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

  /* PS4コントローラの接続開始 */
  if (MOUNT_JOYPAD == 4)
  {
    PS4.begin(BT_MAC_ADDR); // ESP32のMACが入ります.PS4にも設定します.
    Serial.println("PS4 controller connecting...");
  }

  /* Wiiコントローラの接続開始 */
  if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
  {
    wiimote.init();
    // wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
    Serial.println("Wiimote connecting...");
    joypad_search = 3;
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  monitor_check_flow(String str)
// +----------------------------------------------------------------------
// | 機能     :  MONITOR_FLOWがtrueの時, strの文字列を表示する. デバグ用.
// | 引数　　　:  String str
// +----------------------------------------------------------------------
void monitor_check_flow(const String &text)
// void monitor_check_flow(String text)
{
  if (MONITOR_FLOW)
  {
    String tmp = text;
    Serial.print(String(text));
  }
}

// +----------------------------------------------------------------------
// | func name : print_hello_esp1()
// +----------------------------------------------------------------------
// | function  : 起動時にステータスをシリアルモニタに出力する.
// | return    : none.
// +----------------------------------------------------------------------
void print_hello_esp1()
{
  Serial.println();
  Serial.println("Hello, This is " + String(VERSION)); // バージョン表示
  delay(100);
  Serial.println("PC Serial Speed : " + String(SERIAL_PC_BPS) + " bps"); // PCシリアル速度
  Serial.println("Searching WiFi ... ");                                 // WiFi接続完了通知
}

// +----------------------------------------------------------------------
// | func name : print_hello_esp2()
// +----------------------------------------------------------------------
// | function  : 起動時にステータスをシリアルモニタに出力する.
// | return    : none.
// +----------------------------------------------------------------------
void print_hello_esp2()
{
  Serial.println("WiFi connected to => " + String(WIFI_AP_SSID));   // WiFi接続完了通知
  Serial.println("PC's IP address is  => " + String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示
  if (MODE_FIXED_IP)
  {
    Serial.println("ESP32's IP address is  => " + String(FIXED_IP_ADDR) + " (*Fixed)"); // ESP32自身のIPアドレスの表示
  }
  else
  {
    Serial.print("ESP32's IP address is  => "); // ESP32自身のIPアドレスの表示
    Serial.println(WiFi.localIP());
  }
}

//================================================================================================================
//---- Bluetooth 用 ス レ ッ ド -----------------------------------------------------------------------------------
//================================================================================================================
void Core0_BT_r(void *args)
{ // サブCPU(Core0)で実行するプログラム
  while (true)
  {                        // ここで無限ループを作っておく
    while (!udp_busy_flag) // UDP使用中を避ける
    {
      // PS4コントローラの受信
      if (MOUNT_JOYPAD == 4)
      {
        PS4pad_receive();
      }
      // Wiiコントローラの受信
      if ((MOUNT_JOYPAD == 5) or (MOUNT_JOYPAD == 6))
      {
        Wiipad_receive_h();
      }
      delay(JOYPAD_POLLING - 1); // JOYPAD_POLLING ms秒待つ
    }
    delay(1); // JOYPAD_POLLING ms秒待つ
  }
}
