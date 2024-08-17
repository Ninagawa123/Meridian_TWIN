#ifndef __MERIDIAN_MAIN_H__
#define __MERIDIAN_MAIN_H__

// ヘッダファイルの読み込み
#include "config.h"

#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入
#include <SPI.h>                        // SDカードやSPI通信用
#include <TsyDMASPI.h>                  // SPI通信Master用
#include <arduino.h>
#include <cstdint>
MERIDIANFLOW::Meridian mrd; // ライブラリのクラスを mrdという名前でインスタンス化

#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
IcsHardSerialClass ics_L(&Serial2, PIN_EN_L, SERVO_BAUDRATE_L, SERVO_TIMEOUT_L);
IcsHardSerialClass ics_R(&Serial3, PIN_EN_R, SERVO_BAUDRATE_R, SERVO_TIMEOUT_R);
IcsHardSerialClass ics_C(&Serial1, PIN_EN_C, SERVO_BAUDRATE_C, SERVO_TIMEOUT_C);

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum UartLine { // サーボ系統の列挙型(L,R,C)
  L,            // Left
  R,            // Right
  C             // Center
};

enum ServoType { // サーボプロトコルのタイプ
  NOSERVO = 0,   // サーボなし
  PWM_S = 1,     // Single PWM (WIP)
  PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
  DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (WIP)
  JRXBUS = 51,   // JRPROPO_XBUS (WIP)
  FTCSTS = 61,   // FEETECH_STS (WIP)
  FTCSCS = 62    // FEETECH_SCS (WIP)
};

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
};

enum PadType {   // リモコン種の列挙型(PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  PC = 0,        // PCからのPD入力情報を使用
  MERIMOTE = 1,  // MERIMOTE(未導入)
  BLUERETRO = 2, // BLUERETRO(未導入)
  SBDBT = 3,     // SBDBT(未導入)
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5    // WIIMOTE(未導入)
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

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体(SPI送信時に末尾の4バイトが0になる不具合の対策として+4バイト)
typedef union {
  short sval[MRDM_LEN + 4];           // short型で90個の配列データ
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型
  uint8_t bval[MRDM_BYTE + 4];        // byte型で180個の配列データ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;
Meridim90Union s_spi_meridim;       // SPI送信用配列(short型, センサや角度は100倍値)
Meridim90Union r_spi_meridim;       // SPI受信用配列
Meridim90Union s_spi_meridim_dma;   // SPI送信DMA用配列
Meridim90Union r_spi_meridim_dma;   // SPI受信DMA用配列
Meridim90Union s_spi_meridim_dummy; // SPI送信ダミーデータ用配列

// フラグ管理用の構造体
struct MrdFlags {
  bool imuahrs_available = true; // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false; // UDP通信の周期制御がボード主導(false) か, PC主導(true)か
  bool count_frame_reset = false; // フレーム管理時計をリセットする
  bool stop_board_during = false; // ボードの末端処理をmeridim[MRD_STOP_FRAMES]ms止める
  bool eeprom_write_mode = false; // EEPROMへの書き込みモード
  bool eeprom_read_mode = false;  // EEPROMからの読み込みモード
  bool eeprom_protect = EEPROM_PROTECT; // EEPROMの書き込みプロテクト
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;         // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;       // SDCARDへの書き込みモード
  bool sdcard_read_mode = false;        // SDCARDからの読み込みモード
  bool wire0_init = false;              // I2C 0系統の初期化合否
  bool wire1_init = false;              // I2C 1系統の初期化合否
  bool bt_busy = false;      // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool spi_trans = true;     // ESP32とのSPI通信の実施
  bool spi_rcvd = true;      // SPIのデータ受信判定
  bool udp_rcvd = false;     // UDPのデータ受信判定
  bool udp_busy = false;     // UDPスレッドでの受信中フラグ（送信抑制）
  bool meridim_rcvd = false; // Meridimが正しく受信できたか
  bool servoL_drive = false; // L系統のサーボの送受信
  bool servoR_drive = false; // R系統のサーボの送受信
  bool servoC_drive = false; // C系統のサーボの送受信
};
MrdFlags flg;

// シーケンス番号管理用の構造体
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdSq mrdsq;

// タイマー管理用の構造体
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  int count_loop = 0;             // サイン計算用の循環カウンタ
  int count_loop_dlt = 2; // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999; // 循環カウンタの最大値
  int count_pad_interval = 0; // JOYPADのデータを読みに行くためのフレームカウント★
  unsigned long count_frame = 0; // メインフレームのカウント
};
MrdTimer tmr;

// エラーカウント管理用の構造体
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

// リモコン値格納用の共用体
typedef union {
  short sval[PAD_LEN];        // short型で4個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val[1];        // 上記のunsigned int16型
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0}; // PAD値の格納用配列(一次転記)
PadUnion pad_new = {0};   // PAD値の格納用配列(二次転記)
PadUnion pad_i2c = {0};   // PAD値のi2c送受信用配列

// リモコンのアナログ入力データ用の構造体
struct PadValue {
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

// 6軸or9軸センサー用の構造体
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
  MPU6050 mpu6050;                                        // MPU6050のインスタンス
  uint8_t mpuIntStatus;                // holds actual interrupt status byte from MPU
  uint8_t devStatus;                   // return status after each device operation (0 = success,
                                       // !0 = error)
  uint16_t packetSize;                 // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64];              // FIFO storage buffer
  Quaternion q;                        // [w, x, y, z]         quaternion container
  VectorFloat gravity;                 // [x, y, z]            gravity vector
  float ypr[3];                        // [roll, pitch, yaw]   roll/pitch/yaw container and gravity
                                       // vector
  float yaw_origin = 0;                // ヨー軸の補正センター値
  float yaw_source = 0;                // ヨー軸のソースデータ保持用
  float read[16];                      // mpuから読み込んだ一次データ
                                       // acc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ（二次データ）
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0; // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;   // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro; // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;  // [x, y, z]            磁力センサの測定値
  long temperature; // センサの温度測定値
};
AhrsValue ahrs;

// サーボパラメータ用の構造体
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数）
  int num_max;

  // 各サーボのマウントありなし(config.hで設定)
  int ixl_mount[IXL_MAX]; // L系統サーボのマウント有無
  int ixr_mount[IXR_MAX]; // R系統サーボのマウント有無
  int ixc_mount[IXC_MAX]; // C系統サーボのマウント有無

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号
  int ixc_id[IXC_MAX]; // C系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.hで設定)
  int ixl_cw[IXL_MAX]; // L系統サーボの正逆
  int ixr_cw[IXR_MAX]; // R系統サーボの正逆
  int ixc_cw[IXC_MAX]; // C系統サーボの正逆

  // 各サーボの直立ポーズトリム値(config.hで設定)
  float ixl_trim[IXL_MAX]; // L系統サーボのトリム値
  float ixr_trim[IXR_MAX]; // R系統サーボのトリム値
  float ixc_trim[IXC_MAX]; // C系統サーボのトリム値

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統サーボの目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統サーボの目標値
  float ixc_tgt[IXC_MAX] = {0};      // C系統サーボの目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統サーボの前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統サーボの前回の値
  float ixc_tgt_past[IXC_MAX] = {0}; // C系統サーボの前回の値

  // 各サーボの実行コマンド値(degree)
  float ixl_cmd[IXL_MAX] = {0}; // L系統サーボのコマンド
  float ixr_cmd[IXR_MAX] = {0}; // R系統サーボのコマンド
  float ixc_cmd[IXC_MAX] = {0}; // C系統サーボのコマンド

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統サーボのエラーカウンタ配列
  int ixr_err[IXR_MAX] = {0}; // R系統サーボのエラーカウンタ配列
  int ixc_err[IXC_MAX] = {0}; // C系統サーボのエラーカウンタ配列

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
  uint16_t ixc_stat[IXC_MAX] = {0}; // C系統サーボのコンディションステータス配列
};
ServoParam sv;

// モニタリング設定用の構造体
struct MrdMonitor {
  bool frame_delay = MONITOR_FRAME_DELAY; // フレーム遅延時間を表示
  bool flow = MONITOR_FLOW;               // フローを表示
  bool all_err = MONITOR_ERR_ALL;         // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO;     // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;             // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;                 // リモコンのデータを表示
};

MrdMonitor monitor;

#include "mrd_disp.h"
MrdMsgHandler mrd_disp(Serial);

#include "mrd_sd.h"
MrdSdHandler mrd_sd(Serial);

//================================================================================================================
//  関 数 各 種
//================================================================================================================

// 予約関数

bool execute_MasterCommand_1(Meridim90Union a_meridim, bool a_flg_exe);
bool execute_MasterCommand_2(Meridim90Union a_meridim, bool a_flg_exe);
void mrd_countup_errs();

#endif //__MERIDIAN_MAIN_H__
