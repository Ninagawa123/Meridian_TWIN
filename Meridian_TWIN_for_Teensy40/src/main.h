#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <cstdint>
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用

//================================================================================================================
//---- クラス・構造体・共用体  --------------------------------------------------------------------------------------
//================================================================================================================

/* Meridim配列用の共用体の設定 */
typedef union
{
    short sval[MSG_SIZE + 4];           // short型で90個の配列データを持つ
    unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
    uint8_t bval[MSG_BUFF + 4];         // byte型で180個の配列データを持つ
} UnionData;

/* フラグ用変数 */
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
    bool mrd_spi_rcvd = true;         // SPIが正しく受信できたか.
    bool wire0_activate = false;      // wire1の起動.
    bool wire1_activate = false;      // wire2の起動.
};

/* システム管理用の変数 */
struct MrdCheck
{
    int seq_s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
    int seq_r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};

/* タイマー管理用の変数 */
struct MrdTimer
{
    long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
    long mrd_mil = 0;               // フレーム管理時計の時刻 Meridian Clock.
    long now_mil = 0;               // 現在時刻を取得
    long now_mic = 0;               // 現在時刻を取得
    int loop_count = 0;             // サイン計算用の循環カウンタ
    int loop_count_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
    int loop_count_max = 359999;    // 循環カウンタの最大値
    int joypad_polling_count = 0;   // JOYPADのデータを読みに行くためのフレームカウント
};

/* エラーカウント用 */
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

/* リモコン用変数 */
typedef union
{
    short sval[JOYPAD_LEN];        // short型で4個の配列データを持つ
    uint16_t usval[JOYPAD_LEN];    // 上記のunsigned short型
    int8_t bval[JOYPAD_LEN * 2];   // 上記のbyte型
    uint8_t ubval[JOYPAD_LEN * 2]; // 上記のunsigned byte型
    uint64_t ui64val[1];           // 上記のunsigned int16型
                                   // button, pad.stick_L_x:pad.stick_L_y,
                                   // pad.stick_R_x:pad.stick_R_y, pad.L2_val:pad.R2_val
} UnionPad;

typedef union
{
    short sval[JOYPAD_I2C_LEN];        // short型で4個の配列データを持つ
    uint16_t usval[JOYPAD_I2C_LEN];    // 上記のunsigned short型
    int8_t bval[JOYPAD_I2C_LEN * 2];   // 上記のbyte型
    uint8_t ubval[JOYPAD_I2C_LEN * 2]; // 上記のunsigned byte型
} UnionPadWire;

struct PadValue // リモコンの値
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

/* 6軸or9軸センサーの値 */
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

/* サーボ用変数 */
struct ServoId
{
    /* サーボの最大接続*/
    int num_max = max(max(MOUNT_SERVO_NUM_L, MOUNT_SERVO_NUM_R), MOUNT_SERVO_NUM_3); // サーボ送受信のループ処理数（L系R系で多い方）

    /* 各サーボのマウントありなし(config.hで設定)  */
    int idl_mount[IDL_MAX] = {IDL_MT0, IDL_MT1, IDL_MT2, IDL_MT3, IDL_MT4, IDL_MT5, IDL_MT6, IDL_MT7, IDL_MT8, IDL_MT9, IDL_MT10, IDL_MT11, IDL_MT12, IDL_MT13, IDL_MT14}; // L系統
    int idr_mount[IDR_MAX] = {IDR_MT0, IDR_MT1, IDR_MT2, IDR_MT3, IDR_MT4, IDR_MT5, IDR_MT6, IDR_MT7, IDR_MT8, IDR_MT9, IDR_MT10, IDR_MT11, IDR_MT12, IDR_MT13, IDR_MT14}; // R系統
    int id3_mount[ID3_MAX] = {ID3_MT0, ID3_MT1, ID3_MT2, ID3_MT3, ID3_MT4, ID3_MT5, ID3_MT6, ID3_MT7, ID3_MT8, ID3_MT9, ID3_MT10, ID3_MT11, ID3_MT12, ID3_MT13, ID3_MT14}; // 3系統

    /* 各サーボの正逆方向補正用配列(config.hで設定)  */
    int idl_cw[IDL_MAX] = {IDL_CW0, IDL_CW1, IDL_CW2, IDL_CW3, IDL_CW4, IDL_CW5, IDL_CW6, IDL_CW7, IDL_CW8, IDL_CW9, IDL_CW10, IDL_CW11, IDL_CW12, IDL_CW13, IDL_CW14}; // L系統
    int idr_cw[IDR_MAX] = {IDR_CW0, IDR_CW1, IDR_CW2, IDR_CW3, IDR_CW4, IDR_CW5, IDR_CW6, IDR_CW7, IDR_CW8, IDR_CW9, IDR_CW10, IDR_CW11, IDR_CW12, IDR_CW13, IDR_CW14}; // R系統
    int id3_cw[ID3_MAX] = {ID3_CW0, ID3_CW1, ID3_CW2, ID3_CW3, ID3_CW4, ID3_CW5, ID3_CW6, ID3_CW7, ID3_CW8, ID3_CW9, ID3_CW10, ID3_CW11, ID3_CW12, ID3_CW13, ID3_CW14}; // R系統

    /* 各サーボの直立ポーズトリム値(config.hで設定)  */
    float idl_trim[IDL_MAX] = {IDL_TRIM0, IDL_TRIM1, IDL_TRIM2, IDL_TRIM3, IDL_TRIM4, IDL_TRIM5, IDL_TRIM6, IDL_TRIM7, IDL_TRIM8, IDL_TRIM9, IDL_TRIM10, IDL_TRIM11, IDL_TRIM12, IDL_TRIM13, IDL_TRIM14}; // L系統
    float idr_trim[IDR_MAX] = {IDR_TRIM0, IDR_TRIM1, IDR_TRIM2, IDR_TRIM3, IDR_TRIM4, IDR_TRIM5, IDR_TRIM6, IDR_TRIM7, IDR_TRIM8, IDR_TRIM9, IDR_TRIM10, IDR_TRIM11, IDR_TRIM12, IDR_TRIM13, IDR_TRIM14}; // R系統
    float id3_trim[ID3_MAX] = {ID3_TRIM0, ID3_TRIM1, ID3_TRIM2, ID3_TRIM3, ID3_TRIM4, ID3_TRIM5, ID3_TRIM6, ID3_TRIM7, ID3_TRIM8, ID3_TRIM9, ID3_TRIM10, ID3_TRIM11, ID3_TRIM12, ID3_TRIM13, ID3_TRIM14}; // R系統

    /* 各サーボのポジション値(degree) */
    float idl_tgt[IDL_MAX] = {0};      // L系統の目標値
    float idr_tgt[IDR_MAX] = {0};      // R系統の目標値
    float id3_tgt[ID3_MAX] = {0};      // R系統の目標値
    float idl_tgt_past[IDL_MAX] = {0}; // L系統の前回の値
    float idr_tgt_past[IDR_MAX] = {0}; // R系統の前回の値
    float id3_tgt_past[ID3_MAX] = {0}; // R系統の前回の値

    /* サーボのエラーカウンタ配列.*/
    int idl_err[IDL_MAX] = {0}; // 15要素
    int idr_err[IDR_MAX] = {0}; // 15要素
    int id3_err[ID3_MAX] = {0}; // 15要素
};

/* 各種モード設定 */
struct MrdMonitor
{
    bool all_error = MONITOR_ALL_ERROR; // Teensyでのシリアル表示:全経路の受信エラー率
};

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Initialize sensors like MPU6050, BNO055, and others.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setupIMUAHRS();

/**
 * @brief Store values for MPU6050, BNO055, and other sensors.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void IMUAHRS_getYawPitchRoll();

/**
 * @brief Receive input data from the gamepad and return it in PS2/3 gamepad array format.
 *
 * @param mount_joypad  Gamepad type (currently only 2: KRC-5FH).
 * @param pre_val Previous received value (8 bytes, assuming union data).
 * @param polling Frame count for inquiry frequency.
 * @param joypad_reflesh 1:To reset the JOYPAD's received button data to 0 with this device
 *                       0:perform logical addition without resetting .(usually 1)
 * @return uint64_t
 */
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh);

/**
 * @brief Initializing and performing read/write tests for an SD card.
 *
 */
void check_sd();

/**
 * @brief Displaying the error detection count on Teensy's serial interface.
 *
 */
void print_error_monitor();

/**
 * @brief Counting up the communication error detection count.
 *
 */
void countup_errors();

/**
 * @brief Starting the IMU sensor or AHRS sensor.
 *
 */
void imuahrs_start();

/**
 * @brief Storing the values of the IMU sensor and AHRS sensor in an array.
 *
 */
void imuahrs_store(short sensorValues[], float storageArray[], int type);

/**
 * @brief Execute mastercommands group1.
 *
 */
void execute_MasterCommand_1();

/**
 * @brief Execute mastercommands group2.
 *
 */
void execute_MasterCommand_2();

/**
 * @brief Resetting the origin of the yaw axis.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setyaw();

/**
 * @brief Powering off all servos.

 *
 */
void servo_all_off();

void wire_manager0();
void wire_manager1();

void setupWire();

void get_merimote_i2c();

#endif
