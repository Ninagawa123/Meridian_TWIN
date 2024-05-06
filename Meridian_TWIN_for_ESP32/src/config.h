//================================================================================================================
//---- E S P 3 2 の 配 線  ----------------------------------------------------------------------------------------
//================================================================================================================
//
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

// Meridimの基本設定
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define FRAME_DURATION 10 // 1フレームあたりの単位時間（単位ms）

// シリアルモニタリング
#define MONITOR_FLOW 0   // シリアルモニタでフローを表示（0:OFF, 1:ON）
#define MONITOR_SEQ 0    // シリアルモニタでシーケンス番号チェックを表示（0:OFF, 1:ON）
#define MONITOR_JOYPAD 0 // シリアルモニタでリモコンのデータを表示（0:OFF, 1:ON）

// リモコンの設定
#define MOUNT_JOYPAD 0             // ジョイパッドの搭載 (※KRC-5FHはTeensy側に接続)
                                   // 0:なし, 1:SBDBT(未), 2:KRC-5FH, 3:PS3(未), 4:PS4 ,5:Wii_yoko, 6:Wii+Nun(未), 7:WiiPRO(未), 8:Xbox(未)
                                   // 2:KRC-5FH : Teensy側に接続のため0を選ぶ. 通信エラーなし. 利用可. 
                                   // 4:PS4 : 通信エラーが 10%程度発生. 要修正. 
                                   // 5:Wii_yoko : wiiリモコン単体. 通信エラーが 1%以下発生. 利用可. 
#define JOYPAD_POLLING 10          // ジョイパッドの問い合わせフレーム間隔(PSは10)
#define JOYPAD_GENERALIZE 1        // ジョイパッドの入力値をPS系に一般化する
                                   // PS4リモコン(Bluetooth)を使わない場合は不要.
#define BT_PAIR_MAX_DEVICES 20     // BT接続デバイスの記憶可能数
#define BT_REMOVE_BONDED_DEVICES 0 // 0でバインドデバイス情報表示, 1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_WIIMOTE_SOLO[16] = {0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_WIIMOTE_ORIG[16] = {0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

// PC接続関連設定
#define SERIAL_PC_BPS 2000000 // ESP-PC間のシリアル速度（モニタリング表示用）

// UDP通信のオンオフ
#define UDP_SEND 1    // PCへのデータ送信を行うか（0:OFF, 1:ON, 通常は1）
#define UDP_RESEIVE 1 // PCからのデータ受信を行うか（0:OFF, 1:ON, 通常は1）
#define UDP_TIMEOUT 3 // UDPの待受タイムアウト（単位ms,推奨値0）

// マスターコマンド定義
#define MCMD_DUMMY_DATA -32768            // SPI送受信用のダミーデータ判定用
#define MCMD_BOARD_TRANSMIT_ACTIVE 10005  // ボードが定刻で送信を行うモード（デフォルト設定.PC側が受信待ち）
#define MCMD_BOARD_TRANSMIT_PASSIVE 10006 // ボードが受信を待ち返信するモード（PC側が定刻送信）
#define MCMD_RESET_MRD_TIMER 10007        // フレーム管理時計mrd_t_milを現在時刻にリセット
