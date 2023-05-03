//================================================================================================================
//---- E S P 3 2 の 配 線  ----------------------------------------------------------------------------------------
//================================================================================================================
/*
  ESP32devkitC  -  Teensy4.0 or PIN
  [3.3v]        -> x
  [EN]          -> x
  [SENSOR_VP]   -> x
  [SENSOR_VN]   -> x
  [34]          -> x
  [35]          -> x
  [32]          -> x
  [33]          -> x
  [25]          -> x
  [26]          -> x
  [27]          -> x
  [14]          -> SPI/SD_SCK (Teensy[13]) & SD_CLK (SD[5])
  [12]          -> MISO (Teensy[12])
  [GND1]        -> x
  [13]          -> MOSI (Teensy[11])
  [D2]          -> x
  [D3]          -> x
  [CMD]         -> x
  [5V]          -> 5V

  [GND3]        -> GND
  [23]          -> x
  [22]          -> x
  [TXD0]        -> x
  [RXD0]        -> x
  [21]          -> x
  [GND2]        -> x
  [19]          -> x
  [18]          -> x
  [05]          -> x
  [17]          -> x
  [16]          -> x
  [04]          -> x
  [00]          -> x
  [02]          -> x
  [15]          -> SPI_CS (Teensy[10])
  [D1]          -> x
  [D0]          -> x
  [CLK]         -> x
*/

/* Wifiアクセスポイントの設定 */
#define WIFI_AP_SSID "xxxxxxxx"    // アクセスポイントのWIFI_AP_SSID
#define WIFI_AP_PASS "xxxxxxxx"     // アクセスポイントのパスワード
#define WIFI_SEND_IP "192.168.1.16" // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define UDP_SEND_PORT 22222         // 送り先のポート番号
#define UDP_RESV_PORT 22224         // このESP32のポート番号
#define UDP_TIMEOUT 4               // UDPの待受タイムアウト（単位ms,推奨値0）
/* ESP32のIPアドレスを固定する場合は下記の5項目を設定 */
#define MODE_FIXED_IP 0                    // IPアドレスを固定するか（0:NO, 1:YES）
#define FIXED_IP_ADDR "192. 168. 1. 86"    // ESP32のIPアドレスを固定する場合のESPのIPアドレス
#define FIXED_IP_GATEWAY "192. 168. 1. 1"  // ESP32のIPアドレスを固定する場合のルーターのゲートウェイ
#define FIXED_IP_SUBNET "255. 255. 255. 0" // ESP32のIPアドレスを固定する場合のサブネット

/* リモコンの設定 */
#define MOUNT_JOYPAD 0                  // ジョイパッドの搭載 (※KRC-5FHはTeensy側に接続)
                                        // 0:なし, 1:SBDBT(未), 2:KRC-5FH, 3:PS3(未), 4:PS4 ,5:Wii_yoko, 6:Wii+Nun(未), 7:WiiPRO(未), 8:Xbox(未)
#define JOYPAD_POLLING 10               // ジョイパッドの問い合わせフレーム間隔(PSは10)
#define BT_MAC_ADDR "xx:xx:xx:xx:xx:xx" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
                                        // PS4リモコン(Bluetooth)を使わない場合は不要.
#define BT_PAIR_MAX_DEVICES 20          // BT接続デバイスの記憶可能数
#define BT_REMOVE_BONDED_DEVICES 0      // 0でバインドデバイス情報表示, 1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)

/* Meridimの基本設定 */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define FRAME_DURATION 10 // 1フレームあたりの単位時間（単位ms）

/* シリアルモニタリング */
#define MONITOR_ALL_ERROR 0 // Teensyでのシリアル表示:全経路の受信エラー率
#define MONITOR_JOYPAD 0    // Teensyでのシリアル表示:リモコンのデータ
#define MONITOR_FLOW 0      // シリアルモニタでフローを表示するかの設定（0:OFF, 1:ON）
#define MONITOR_SEQ 0       // シリアルモニタでシーケンス番号チェックを表示するかの設定（0:OFF, 1:ON）

/* UDP通信のオンオフ */
#define UDP_SEND 1    // PCへのデータ送信を行うか（0:OFF, 1:ON, 通常は1）
#define UDP_RESEIVE 1 // PCからのデータ受信を行うか（0:OFF, 1:ON, 通常は1）

// PC接続関連設定
#define SERIAL_PC_BPS 200000 // ESP-PC間のシリアル速度（モニタリング表示用）