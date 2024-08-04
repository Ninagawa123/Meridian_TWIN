#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include <Meridian.h> // Meridianのライブラリ導入
MERIDIANFLOW::Meridian mrd;

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum UartLine { // サーボ系統の列挙型(L,R,C)
  L,            // Left
  R,            // Right
  C             // Center
};

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
};

enum PadReceiverType { // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  NONE = 0,            // リモコンなし
  PC = 0,              // PCからのPD入力情報を使用
  MERIMOTE = 1,        // MERIMOTE(未導入)
  BLUERETRO = 2,       // BLUERETRO(未導入)
  SBDBT = 3,           // SBDBT(未導入)
  KRR5FH = 4,          // KRR5FH
  WIIMOTE = 5,         // WIIMOTE 単体横持ち
  WIIMOTE2 = 6         // WIIMOTE+Analog
};

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
const int MRDM_BYTE = MRDM_LEN * 2; // Meridim配列のバイト型の長さ
const int MRD_ERR = MRDM_LEN - 2; // エラーフラグの格納場所（配列の末尾から2つめ）
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
const int MRD_CKSM = MRDM_LEN - 1;     // チェックサムの格納場所（配列の末尾）
const int PAD_LEN = 5;                 // リモコン用配列の長さ
TaskHandle_t thp[4];                   // マルチスレッドのタスクハンドル格納用

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体の設定
typedef union {
  short sval[MRDM_LEN + 4];           // short型で90個の配列データを持つ
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型
  uint8_t bval[+4];                   // byte型で180個の配列データを持つ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;
Meridim90Union s_udp_meridim; // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;       // Meridim配列データ受信用
Meridim90Union s_spi_meridim;       // Meridim配列データ送信用
Meridim90Union r_spi_meridim;       // Meridim配列データ受信用
Meridim90Union s_udp_meridim_dummy; // SPI送信ダミー用
Meridim90Union tmp_meridim;         // チェック用配列
uint8_t *s_spi_meridim_dma;         // DMA用
uint8_t *r_spi_meridim_dma;         // DMA用

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true; // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false; // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool frame_timer_reset = false; // フレーム管理時計をリセットする
  bool stop_board_during = false; // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;       // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;        // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT; // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;         // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;       // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;        // SDCARDからの読み込みモード.
  bool wire0_init = false;              // I2C 0系統の初期化合否
  bool wire1_init = false;              // I2C 1系統の初期化合否
  bool udp_rcvd = false;                // UDPが受信できたか.
  bool udp_busy = false;                // UDPスレッドでの受信中フラグ（送信抑制）
  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施（0:OFF, 1:ON, 通常は1）
  bool udp_send_mode = MODE_UDP_SEND; // PCへのデータ送信実施（0:OFF, 1:ON, 通常は1）
  bool spi_rcvd = true;               // SPIのデータ受信判定
  bool bt_busy = false;      // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool meridim_rcvd = false; // Meridimが正しく受信できたか.
};
MrdFlags flg;

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdSq mrdsq;

// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  unsigned long mrd_mil = 0;      // フレーム管理時計の時刻 Meridian Clock.
  unsigned long now_mil = 0;      // 現在時刻を取得
  unsigned long now_mic = 0;      // 現在時刻を取得
  int loop_count = 0;             // サイン計算用の循環カウンタ
  int loop_count_dlt = 2; // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int loop_count_max = 359999; // 循環カウンタの最大値
};
MrdTimer tmr;

// エラーカウント用
struct MrdErr {
  int esp_pc = 0;   // PCの受信エラー（ESP32からのUDP）
  int pc_esp = 0;   // ESP32の受信エラー（PCからのUDP）
  int esp_tsy = 0;  // Teensyの受信エラー（ESP32からのSPI）
  int tsy_esp = 0;  // ESP32の受信エラー（TeensyからのSPI）
  int esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
  int tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
  int pc_skip = 0;  // PC受信のカウントの連番スキップ回数
};
MrdErr err;

typedef union // リモコン値格納用
{
  short sval[PAD_LEN];        // short型で4個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val[1];        // 上記のunsigned int16型
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0}; // pad値の格納用配列
PadUnion pad_i2c = {0};   // pad値のi2c送受信用配列

struct PadValue // リモコンのアナログ入力データ
{
  unsigned short stick_R = 0;
  int stick_R_x = 0;
  int stick_R_y = 0;
  unsigned short stick_L = 0;
  int stick_L_x = 0;
  int stick_L_y = 0;
  unsigned short stick_L2R2V = 0;
  int R2_val = 0;
  int L2_val = 0;
};
PadValue pad_analog;

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ALL_ERROR;   // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_SERVO_ERR; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ_NUMBER;  // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
MrdMonitor monitor;

//================================================================================================================
//  関数各種
//================================================================================================================

// 予約用
void Core0_BT_r(void *args);
// IPAddress makeIPAddress(const char *ip_str);
void Core0_BT_r(void *args);

/// @brief meridim配列にチェックサムを算出して書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_writedim90_cksm(Meridim90Union &a_meridim) {
  a_meridim.sval[89] = mrd.cksm_val(a_meridim.sval, 90);
  return true;
}

#endif //__MERIDIAN_MAIN_FUNC__