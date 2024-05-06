#ifndef __MERIDIAN_MAIN_H__
#define __MERIDIAN_MAIN_H__

#include <arduino.h>
#include <cstdint>
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <SPI.h>                        // SDカードやSPI通信用
#include <TsyDMASPI.h>                  // SPI通信Master用
#include <Meridian.h>                   // Meridianのライブラリ導入
MERIDIANFLOW::Meridian mrd;             // ライブラリのクラスを mrdという名前でインスタンス化

#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
IcsHardSerialClass ics_L(&Serial2, PIN_EN_L, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass ics_R(&Serial3, PIN_EN_R, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass ics_C(&Serial1, PIN_EN_C, ICS_BAUDRATE, ICS_TIMEOUT); // 3系もICSの場合

enum Uart_line
{
    L, // Left
    R, // Right
    C  // Center
};

const char *getLineName(Uart_line line)
{
    switch (line)
    {
    case L:
        return "L";
    case R:
        return "R";
    case C:
        return "C";
    default:
        return "Unknown";
    }
}

enum ImuAhrsType
{
    NO_IMU = 0,
    MPU6050_IMU = 1,
    MPU9250_IMU = 2,
    BNO055_AHRS = 3
};

enum PadReceiverType
{
    PC = 0,
    MERIMOTE = 1,
    BLUERETRO = 2,
    SBDBT = 3,
    KRR5FH = 4
};

// システム用
const int MSG_BUFF = MSG_SIZE * 2;     // Meridim配列のバイト型の長さ
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
short pad_btn_past = 0;
short pad_cksm_past = 0;

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体の設定
typedef union
{
    short sval[MSG_SIZE + 4];           // short型で90個の配列データを持つ
    unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
    uint8_t bval[MSG_BUFF + 4];         // byte型で180個の配列データを持つ
    uint8_t ubval[MSG_BUFF + 4];        // 上記のunsigned byte型

} Meridim90Union;
Meridim90Union s_spi_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_spi_meridim;       // Meridim配列データ受信用
Meridim90Union s_spi_meridim_dma;   // SPI送信DMA用
Meridim90Union r_spi_meridim_dma;   // SPI受信DMA用
Meridim90Union s_spi_meridim_dummy; // SPI送信ダミー用

// フラグ用変数
struct MrdFlags
{
    bool imuahrs_available = true;    // メインが結果値を読み取る瞬間, サブスレッドによる書き込みをウェイト
    bool udp_board_passive = false;   // UDP受信のパッシブモード（0:active/デフォルト, 1:passive/PC側が通信周期制御)
    bool reset_meridian_time = false; // フレーム管理時計をリセットするかどうか(パッシブモードの終了時にリセットする設定)
    bool stop_board_during = false;   // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
    bool eeprom_write_mode = false;   // EEPROMへの書き込みモード.
    bool eeprom_read_mode = false;    // EEPROMからの読み込みモード.
    bool sdcard_write_mode = false;   // SDCARDへの書き込みモード.
    bool sdcard_read_mode = false;    // SDCARDからの読み込みモード.
    bool wire0_init = false;          // I2C 0系統の初期化合否
    bool wire1_init = false;          // I2C 0系統の初期化合否
    bool mrd_spi_rcvd = true;         // SPIが正しく受信できたか.
    bool servoL_drive = false;        // L系統のサーボの送受信
    bool servoR_drive = false;        // R系統のサーボの送受信
    bool servoC_drive = false;        // C系統のサーボの送受信
    bool eeprom_load = EEPROM_LOAD;   // 起動時にEEPROMの内容を読み込む
    bool eeprom_set = EEPROM_SET;     // 起動時にEEPROMに規定値をセット
};
MrdFlags flg;

// シーケンス番号理用の変数
struct MrdCheck
{
    int seq_s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
    int seq_r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdCheck mrdck;

// タイマー管理用の変数
struct MrdTimer
{
    long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
    long mrd_mil = 0;               // フレーム管理時計の時刻 Meridian Clock.
    long now_mil = 0;               // 現在時刻を取得
    long now_mic = 0;               // 現在時刻を取得
    int loop_count = 0;             // サイン計算用の循環カウンタ
    int loop_count_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
    int loop_count_max = 359999;    // 循環カウンタの最大値
    int JOYPAD_INTERVAL_count = 0;  // JOYPADのデータを読みに行くためのフレームカウント
};
MrdTimer tmr;

// エラーカウント用
struct MrdErr
{
    int esp_pc = 0;   // PCの受信エラー（ESP32からのUDP）
    int pc_esp = 0;   // ESP32の受信エラー（PCからのUDP）
    int esp_tsy = 0;  // Teensyの受信エラー（ESP32からのSPI）
    int tsy_esp = 0;  // ESP32の受信エラー（TeensyからのSPI）
    int esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
    int tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
    int pc_skip = 0;  // PC受信のカウントの連番スキップ回数
};
MrdErr err;

// リモコン用変数
#define JOYPAD_LEN 4
#define JOYPAD_I2C_LEN 5

typedef union // リモコン値格納用
{
    short sval[JOYPAD_LEN];        // short型で4個の配列データを持つ
    uint16_t usval[JOYPAD_LEN];    // 上記のunsigned short型
    int8_t bval[JOYPAD_LEN * 2];   // 上記のbyte型
    uint8_t ubval[JOYPAD_LEN * 2]; // 上記のunsigned byte型
    uint64_t ui64val[1];           // 上記のunsigned int16型
                                   // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                                   // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0};

typedef union // Merimoto_I2C受信リモコン値格納用
{
    short sval[JOYPAD_I2C_LEN];        // short型で4個の配列データを持つ
    uint16_t usval[JOYPAD_I2C_LEN];    // 上記のunsigned short型
    int8_t bval[JOYPAD_I2C_LEN * 2];   // 上記のbyte型
    uint8_t ubval[JOYPAD_I2C_LEN * 2]; // 上記のunsigned byte型
} PadUnionWire;
PadUnionWire pad_i2c = {0};

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
PadValue pad;

// 6軸or9軸センサーの値
struct AhrsValue
{
    MPU6050 mpu6050;
    uint8_t mpuIntStatus;                                               // holds actual interrupt status byte from MPU
    uint8_t devStatus;                                                  // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;                                                // expected DMP packet size (default is 42 bytes)
    uint8_t fifoBuffer[64];                                             // FIFO storage buffer
    Quaternion q;                                                       // [w, x, y, z]         quaternion container
    VectorFloat gravity;                                                // [x, y, z]            gravity vector
    float ypr[3];                                                       // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
    float yaw_origin = 0;                                               // ヨー軸の補正センター値
    float read[16];                                                     // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
    float zeros[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // リセット用
    float ave_data[16];                                                 // 上記の移動平均値を入れる
    float result[16];                                                   // 加工後の最新のmpuデータ（二次データ）
    float stock_data[IMUAHRS_STOCK][16];                                // 上記の移動平均値計算用のデータストック
    int stock_count = 0;                                                // 上記の移動平均値計算用のデータストックを輪番させる時の変数
    VectorInt16 aa;                                                     // [x, y, z]            加速度センサの測定値
    VectorInt16 gyro;                                                   // [x, y, z]            角速度センサの測定値
    VectorInt16 mag;                                                    // [x, y, z]            磁力センサの測定値
    long temperature;                                                   // センサの温度測定値
};
AhrsValue ahrs;                                         // 6軸or9軸センサーの値
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055用変数

// サーボ用変数
struct ServoId
{
    // サーボの最大接続 (サーボ送受信のループ処理数）
    int num_max = max(max(MOUNT_L_SERVO_NUM, MOUNT_R_SERVO_NUM), MOUNT_C_SERVO_NUM);

    // 各サーボのマウントありなし(config.hで設定)
    int idl_mount[IDL_MAX] = {IDL_MT0, IDL_MT1, IDL_MT2, IDL_MT3, IDL_MT4, IDL_MT5, IDL_MT6, IDL_MT7, IDL_MT8, IDL_MT9, IDL_MT10, IDL_MT11, IDL_MT12, IDL_MT13, IDL_MT14}; // L系統
    int idr_mount[IDR_MAX] = {IDR_MT0, IDR_MT1, IDR_MT2, IDR_MT3, IDR_MT4, IDR_MT5, IDR_MT6, IDR_MT7, IDR_MT8, IDR_MT9, IDR_MT10, IDR_MT11, IDR_MT12, IDR_MT13, IDR_MT14}; // R系統
    int IDC_mount[IDC_MAX] = {IDC_MT0, IDC_MT1, IDC_MT2, IDC_MT3, IDC_MT4, IDC_MT5, IDC_MT6, IDC_MT7, IDC_MT8, IDC_MT9, IDC_MT10, IDC_MT11, IDC_MT12, IDC_MT13, IDC_MT14}; // C系統

    // 各サーボの正逆方向補正用配列(config.hで設定)
    int idl_cw[IDL_MAX] = {IDL_CW0, IDL_CW1, IDL_CW2, IDL_CW3, IDL_CW4, IDL_CW5, IDL_CW6, IDL_CW7, IDL_CW8, IDL_CW9, IDL_CW10, IDL_CW11, IDL_CW12, IDL_CW13, IDL_CW14}; // L系統
    int idr_cw[IDR_MAX] = {IDR_CW0, IDR_CW1, IDR_CW2, IDR_CW3, IDR_CW4, IDR_CW5, IDR_CW6, IDR_CW7, IDR_CW8, IDR_CW9, IDR_CW10, IDR_CW11, IDR_CW12, IDR_CW13, IDR_CW14}; // R系統
    int IDC_cw[IDC_MAX] = {IDC_CW0, IDC_CW1, IDC_CW2, IDC_CW3, IDC_CW4, IDC_CW5, IDC_CW6, IDC_CW7, IDC_CW8, IDC_CW9, IDC_CW10, IDC_CW11, IDC_CW12, IDC_CW13, IDC_CW14}; // R系統

    // 各サーボの直立ポーズトリム値(config.hで設定)
    float idl_trim[IDL_MAX] = {IDL_TRIM0, IDL_TRIM1, IDL_TRIM2, IDL_TRIM3, IDL_TRIM4, IDL_TRIM5, IDL_TRIM6, IDL_TRIM7, IDL_TRIM8, IDL_TRIM9, IDL_TRIM10, IDL_TRIM11, IDL_TRIM12, IDL_TRIM13, IDL_TRIM14}; // L系統
    float idr_trim[IDR_MAX] = {IDR_TRIM0, IDR_TRIM1, IDR_TRIM2, IDR_TRIM3, IDR_TRIM4, IDR_TRIM5, IDR_TRIM6, IDR_TRIM7, IDR_TRIM8, IDR_TRIM9, IDR_TRIM10, IDR_TRIM11, IDR_TRIM12, IDR_TRIM13, IDR_TRIM14}; // R系統
    float IDC_trim[IDC_MAX] = {IDC_TRIM0, IDC_TRIM1, IDC_TRIM2, IDC_TRIM3, IDC_TRIM4, IDC_TRIM5, IDC_TRIM6, IDC_TRIM7, IDC_TRIM8, IDC_TRIM9, IDC_TRIM10, IDC_TRIM11, IDC_TRIM12, IDC_TRIM13, IDC_TRIM14}; // R系統

    // 各サーボのポジション値(degree)
    float idl_tgt[IDL_MAX] = {0};      // L系統の目標値
    float idr_tgt[IDR_MAX] = {0};      // R系統の目標値
    float IDC_tgt[IDC_MAX] = {0};      // R系統の目標値
    float idl_tgt_past[IDL_MAX] = {0}; // L系統の前回の値
    float idr_tgt_past[IDR_MAX] = {0}; // R系統の前回の値
    float IDC_tgt_past[IDC_MAX] = {0}; // R系統の前回の値

    // サーボのエラーカウンタ配列
    int idl_err[IDL_MAX] = {0}; // 15要素
    int idr_err[IDR_MAX] = {0}; // 15要素
    int IDC_err[IDC_MAX] = {0}; // 15要素
};
ServoId sv;

// モニタリング設定
struct MrdMonitor
{
    bool all_error = MONITOR_ALL_ERROR; // Teensyでのシリアル表示:全経路の受信エラー率
};
MrdMonitor monitor;

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// 予約用
void mrd_msg_print_error_monitor();
void countup_errors();
bool execute_MasterCommand_1(bool flg_mrd_spi_rcvd);
bool execute_MasterCommand_2(bool flg_mrd_spi_rcvd);

// ビット操作マクロ
void setBit16(uint16_t &byte, uint16_t bitPosition); // ビットをセットする関数（16ビット用）
void clearBit16(uint16_t &byte, uint16_t bitPosition); // ビットをクリアする関数（16ビット用）
void setBit8(uint8_t &value, uint8_t bitPosition); // ビットをセットする関数（8ビット用）
void clearBit8(uint8_t &value, uint8_t bitPosition); // ビットをクリアする関数（8ビット用）

#endif
