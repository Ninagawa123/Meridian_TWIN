
// Meridian_TWIN_for_Teensy_20221028 By Izumi Ninagawa
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト　20221028版
// 220723 内部計算時に degree*100 を単位として使用するように変更
// 220723 センサーの関数を集約
// 220723 サーボオン時にリモコン左十字キー入力で首を左右に振る動作サンプル入り
// 220730 PCからのリモコン受信が有効となるように調整
// 220828 サーボからの受信が-1(タイムアウト)の時、前に取得した情報を使用する（データ飛びや表示のブレを防止）
// 220828 SERVO_NUM_L, SERVO_NUM_R に左右の接続サーボ数の登録を設定
// 220828 左右の接続サーボ数の多い方をservo_numとし、サーボ送信命令も左右交互に実行するよう改定

//【更新情報】
// 220906 センサの初期化整理, BNO055対応, 関数名変更[旧]setyaw()→[新]setyawcenter()
// 220906 変数名変更[旧] YAW_ZERO→[新]yaw_center
// 220908 SDcardの初期化＆テストを追加. SD_TEST 1 で番号の書き込み読み取りを実行 (LITE版と同じ動作)
// 220908 EEPROMのテスト
// 221022 SPI送信のタイミングを外部に依存する「待受モード」を追加 meridim[0]が5で待ち受けモードフラグ下げ、6で上げ
// 221025 BNO055に対応（調整の余地あり）
// 221025 共用体のunsigned対応, 要素数の設定ミスを修正
// 221025 シーケンシャルカウンタを0スタート、unsigned shortで60,000で0リセットに
// 221025 ※よって、Teensy,ESP,Meridian Consoleをすべて最新版にアップデートする必要あり
// 221028 KRC-5FHの外付けアナログパッドに対応するコードを追加 (+Hori)

//================================================================================================================
//---- Teensy4.0 の 配 線 / ピンアサイン ----------------------------------------------------------------------------
//================================================================================================================
/*
  [GND]               -> GND
  [00] RX1, CRX2      -> ICS_3rd_TX
  [01] TX1, CTX2      -> ICS_3rd_RX
  [02]                -> LED（lights up when the processing time is not within the specified time.）
  [03]                -> (NeoPixel?)
  [04]                -> (NeoPixel?)
  [05]                -> ICS_Right_EN
  [06]                -> ICS_Left_EN
  [07] RX2            -> ICS_Left_TX
  [08] TX2            -> ICS_Left_RX
  [09]                -> SD_DAT3/CD (SD[2])
  [10] CS             -> SPI_CS (ESP32[15])
  [11] MOSI           -> SPI/SD_MOSI (ESP32[13]) & SD_CMD (SD[3])
  [12] MISO           -> SPI/SD_MISO (ESP32[12]) & SD_DAT0 (SD[7])
  [Vin]               -> 5V
  [AGND]              ->
  [3.3v]              -> IMU/AHRS 3.3Vin & SD_VDD (SD[4])
  [23] CRX1           -> ICS_3rd_EN
  [22] CTX1           ->
  [21] RX5            ->
  [20] TX5            ->
  [19] I2C-SCL0       -> IMU/AHRS SCL
  [18] I2C-SDA0       -> IMU/AHRS SDA
  [17] TX4, I2C-SDA1  -> (PC/Raspi etc.)
  [16] RX4, I2C-SCL1  -> (PC/Raspi etc.)
  [15] RX3            -> ICS_Right_TX
  [14] TX3            -> ICS_Right_RX
  [13] SCK(CRX1)      -> SPI/SD_SCK (ESP32[14]) & SD_CLK (SD[5])
*/

//================================================================================================================
//---- サーボIDとロボット部位、軸との対応表 ----------------------------------------------------------------------------
//================================================================================================================
/*
  ID    Parts-Axis　＜ICS_Left_Upper SIO1,SIO2＞
  [L00] 頭ヨー
  [L01] 左肩ピッチ
  [L02] 左肩ロール
  [L03] 左肘ヨー
  [L04] 左肘ピッチ
  [L05] -
  ID    Parts-Axis　＜ICS_Left_Lower SIO3,SIO4＞
  [L06] 左股ロール
  [L07] 左股ピッチ
  [L08] 左膝ピッチ
  [L09] 左足首ピッチ
  [L10] 左足首ロール
  ID    Parts-Axis  ＜ICS_Right_Upper SIO5,SIO6＞
  [R00] 腰ヨー
  [R01] 右肩ピッチ
  [R02] 右肩ロール
  [R03] 右肘ヨー
  [R04] 右肘ピッチ
  [R05] -
  ID    Parts-Axis  ＜ICS_Right_Lower SIO7,SIO8＞
  [R06] 右股ロール
  [R07] 右股ピッチ
  [R08] 右膝ピッチ
  [R09] 右足首ピッチ
  [R10] 右足首ロール
*/

//================================================================================================================
//---- Meridim配列 一覧表 ------------------------------------------------------------------------------------------
//================================================================================================================
/*
  [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
  [01]      シーケンシャルカウンタ
  [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
  [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
  [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
  [11]      IMU/AHRS:temp                   温度
  [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
  [15]      ボタンデータ1
  [16]      ボタンアナログ1（Stick Left）
  [17]      ボタンアナログ2（Stick Right）
  [18]      ボタンアナログ3（L2,R2 アナログ）
  [19]      モーション設定（フレーム数/イージング）
  [20]      サーボID LO  コマンド
  [21]      サーボID LO  データ値
  ...
  [48]      サーボID L14 コマンド
  [49]      サーボID L14 データ値
  [50]      サーボID RO  コマンド
  [51]      サーボID RO  データ値
  ...
  [78]      サーボID R14 コマンド
  [79]      サーボID R14 データ値
  [80]-[MSG_SIZE-3] free (Meridim90では[87]まで)
  [MSG_SIZE-2] ERROR CODE
  [MSG_SIZE-1] チェックサム
*/

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* 頻繁に変更するであろう#DEFINE */
#define VERSION "Meridian_TWIN_for_Teensy_2022.10.28" // バージョン表示
#define FRAME_DURATION 10                             // 1フレームあたりの単位時間（単位ms）
#define SET_EEPROM 0                                  // EEPROMの内容をコードで指定するか(0:EEPOROMの保存内容優先, 1:コード指定値を書き込む)

/* シリアルモニタリング切り替え */
#define MONITOR_ALL_ERROR 0 // Teensyでのシリアル表示:全経路の受信エラー率
#define MONITOR_JOYPAD 0    // Teensyでのシリアル表示:リモコンのデータ

/* マウント有無とピンアサイン */
#define ESP32_MOUNT 1   // ESPの搭載 0:なし(SPI通信およびUDP通信を実施しない), 1:あり
#define IMUAHRS_MOUNT 0 // IMU/AHRSの搭載状況 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
#define IMUAHRS_FREQ 10 // IMU/AHRSのセンサの読み取り間隔(ms)
#define JOYPAD_MOUNT 2  // ジョイパッドの搭載 0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH (※2のみ実装済,MeridianBoardではICS_R系に接続)
#define JOYPAD_FRAME 4  // 上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define ICS3_MOUNT 0    // 半二重サーボ信号の3系のありなし
#define SD_MOUNT 1      // SDカードリーダーのありなし. MeridianBoard Type.Kは有り
#define SD_TEST 0       // SDカードリーダーの起動時読み書きチェック.
#define CHIPSELECT_SD 9 // SDカードSPI通信用のChipSelectのピン番号
#define SERVO_NUM_L 11  // L系統につないだサーボの数
#define SERVO_NUM_R 11  // R系統につないだサーボの数

/* マスターコマンド定義 */
#define TRIM_ADJUST_MODE 0      // トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
#define UPDATE_YAW_CENTER 10002 // センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 10004   // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

/* 各種固定値 */
#define MSG_SIZE 90                    // Meridim配列の長さ設定（デフォルトは90）
#define ERR_LED 2                      // LED用 処理が時間内に収まっていない場合に点灯
#define EN_L_PIN 6                     // ICSサーボ信号の左系のENピン番号（固定）
#define EN_R_PIN 5                     // ICSサーボ信号の右系のENピン番号（固定）
#define EN_3_PIN 23                    // 半二重サーボ信号の3系のENピン番号（固定）
#define SERIAL_PC 6000000              // PCとのシリアル速度（モニタリング表示用）
#define SPI_CLOCK 6000000              // SPI通信の速度（6000000kHz推奨）
#define BAUDRATE 1250000               // ICSサーボの通信速度1.25M
#define TIMEOUT 3                      // ICS返信待ちのタイムアウト時間。通信できてないか確認する場合には1000ぐらいに設定するとよい
const int MSG_BUFF = MSG_SIZE * 2;     // Meridim配列の長さ（byte換算）
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）

/* ライブラリ導入 */
#include <Wire.h>                       // I2Cのライブラリ導入
#include <SPI.h>                        // SPIのライブラリ導入
#include <SD.h>                         // SDカード用のライブラリ導入
#include <TsyDMASPI.h>                  // SPI Master用のライブラリを導入
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050のライブラリ導入
#include <IcsHardSerialClass.h>         // ICSサーボのライブラリ導入
#include <MsTimer2.h>                   // タイマーのライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ導入
#include <TeensyThreads.h>              // マルチスレッド用のライブラリ導入
#include <EEPROM.h>                     // EEPROMのライブラリ導入

/* 変数一般 */
int spi_ok = 0;                                // 通信のエラーカウント
int spi_trial = 0;                             // 通信のエラーカウント
int k;                                         // 各サーボの計算用変数
int servo_num = max(SERVO_NUM_L, SERVO_NUM_R); // サーボ送受信のループ処理数（L系R系で多い方）

/* フラグ用変数 */
bool flag_sensor_IMUAHRS_writable = true; // メインが結果値を読み取る瞬間、サブスレッドによる書き込みをウェイト
bool flag_spi_initiative_outer = false;   // SPI送受信タイミングの主導権、false=Teensy, true=外部
short r_spi_meridim_last_cksm = 0;        // 前回の受信値のチェックサムキープ用

/* Meridim配列用の共用体の設定 */
typedef union // 共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
    short sval[MSG_SIZE + 2];           // short型で90個の配列データを持つ
    unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
    uint8_t bval[MSG_BUFF + 4];         // 1バイト単位で180個の配列データを持つ
} UnionData;
UnionData s_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData s_spi_meridim_dma; // SPI送信用の共用体のインスタンスを作成
UnionData r_spi_meridim_dma; // SPI受信用の共用体のインスタンスを作成

/* EEPROM読み書き用 */
typedef union // 共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
    short sval[270];   // short型で270個の配列データを持つ
    uint8_t bval[540]; // 1バイト単位で540個の配列データを持つ
} UnionEEPROM;
UnionEEPROM eeprom_write; // EEPROM書き込み用
UnionEEPROM eeprom_load;  // EEPROM読み込み用

/* タイマー管理用の変数 */
long frame_ms = FRAME_DURATION;         // 1フレームあたりの単位時間(ms)
long merc = (long)millis();             // フレーム管理時計の時刻 Meridian Clock.
long curr = (long)millis();             // 現在時刻を取得
long curr_micro = (long)micros();       // 現在時刻を取得
int frame_count = 0;                    // サイン計算用の変数
int frame_count_diff = 2;               // サインカーブ動作などのフレームカウントをいくつずつ進めるか
int frame_count_max = 360000;           // フレームカウントの最大値
int joypad_frame_count = 0;             // JOYPADのデータを読みに行くためのフレームカウント
unsigned short frame_sync_s = 0;        // フレーム毎に0-59,000をカウントし、送信用Meridm[1]に格納
unsigned short frame_sync_r_expect = 0; // フレーム毎に0-59,000をカウントし、受信値と比較
unsigned short frame_sync_r_resv = 0;   // 今フレームに受信したframe_sync_rを格納
unsigned short frame_sync_r_last = 0;   // 前回フレームに受信したframe_sync_resvをキープ ★★★

/* エラーカウント用 */
int err_esp_pc = 0;   // PCの受信エラー（ESP32からのUDP）
int err_pc_esp = 0;   // ESP32の受信エラー（PCからのUDP）
int err_esp_tsy = 0;  // Teensyの受信エラー（ESP32からのSPI）
int err_tsy_esp = 0;  // ESP32の受信エラー（TeensyからのSPI）
int err_esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
int err_tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
int err_pc_skip = 0;  // PC受信のカウントの連番スキップ回数

/* 各種モード設定 */
bool trim_adjust = TRIM_ADJUST_MODE;        // トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
bool monitor_all_error = MONITOR_ALL_ERROR; // Teensyでのシリアル表示:全経路の受信エラー率

/* リモコン用変数 */
unsigned short button_1 = 0; // 受信ボタンデータ1群
unsigned short button_2 = 0; // 受信ボタンデータ2群
short stick_Lx = 0;          // 受信ジョイスティックデータLx
short stick_Ly = 0;          // 受信ジョイスティックデータLy
short stick_Rx = 0;          // 受信ジョイスティックデータRx
short stick_Ry = 0;          // 受信ジョイスティックデータRy
unsigned short pad_btn = 0;  // ボタン変数一般化変換
short pad_analog[4];         // アナログパッド用変数一般化変換

// SDカードテスト用
File sdFile;

/* MPU6050のアドレス、レジスタ設定値 */
#define I2C_CLOCK 400000 // I2Cの速度（400kHz推奨）
#define MPU_STOCK 4      // MPUで移動平均を取る際の元にする時系列データの個数
MPU6050 mpu;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
float ROLL, PITCH, YAW, yaw_center;
float mpu_read[16];                                                     // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
float mpu_zeros[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // リセット用
float mpu_ave_data[16];                                                 // 上記の移動平均値を入れる
float mpu_result[16];                                                   // 加工後の最新のmpuデータ（二次データ）
float mpu_stock_data[MPU_STOCK][16];                                    // 上記の移動平均値計算用のデータストック
int mpu_stock_count = 0;                                                // 上記の移動平均値計算用のデータストックを輪番させる時の変数
VectorInt16 aa;                                                         // [x, y, z]            加速度センサの測定値
VectorInt16 gyro;                                                       // [x, y, z]            角速度センサの測定値
VectorInt16 mag;                                                        // [x, y, z]            磁力センサの測定値
long temperature;                                                       // センサの温度測定値

/* BNO055用変数 */
#define IMUAHRS_POLLING 10 // IMU,AHRSの問い合わせフレーム間隔
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
double qw, qx, qy, qz; // クオータニオン計算用

/* ICSサーボのインスタンス設定 */
IcsHardSerialClass krs_L(&Serial2, EN_L_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_R(&Serial3, EN_R_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_3(&Serial1, EN_3_PIN, BAUDRATE, TIMEOUT); // 3系もICSの場合

/* サーボのポジション用配列.*/                                       // degreeではなくサーボ値が入る
int s_servo_pos_L[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 15要素
int s_servo_pos_R[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 15要素
int r_servo_pos_L[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 15要素
int r_servo_pos_R[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 15要素

/* 各サーボのマウントありなし */
int idl_mt[15]; // L系統
int idr_mt[15]; // R系統

// (TS-6-9-4) 各サーボの正逆方向補正用配列
float idl_cw[15]; // L系統
float idr_cw[15]; // R系統

/* 各サーボの直立ポーズトリム値 */
int idl_trim[15]; // L系統
int idr_trim[15]; // R系統

/* 各サーボのポジション値 */
float idl_d[15]; // L系統
float idr_d[15]; // R系統

//================================================================================================================
//---- コ マ ン ド 処 理 系 の 関 数 各 種 ---------------------------------------------------------------------------
//================================================================================================================

// +----------------------------------------------------------------------
// | 関数名　　:  trimadjustment()
// +----------------------------------------------------------------------
// | 機能     :  サーボトリム調整. サーボオンで直立静止状態を保つ.
// +----------------------------------------------------------------------
void trimadjustment()
{
    while (true)
    {
        for (int i = 0; i < 15; i++)
        {
            if (idl_mt[i] == 1)
            {
                krs_L.setPos(i, 7500 + (idl_trim[i] * idl_cw[i]));
            }
            if (idr_mt[i] == 1)
            {
                krs_R.setPos(i, 7500 + (idr_trim[i] * idr_cw[i]));
            }
            delayMicroseconds(2);
        }
        delay(100);
        Serial.println("Trim adjst mode.");
    }
}

// +----------------------------------------------------------------------
// | 関数名　　:  setyawcenter()
// +----------------------------------------------------------------------
// | 機能     :  ヨー軸の原点リセット. IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// +----------------------------------------------------------------------
void setyawcenter()
{
    if (IMUAHRS_MOUNT == 1) // MPU6050
    {
        yaw_center = ypr[0] * 180 / M_PI;
        // s_spi_meridim.sval[0] = MSG_SIZE;
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055
    {
        yaw_center = mpu_result[14];
        // s_spi_meridim.sval[0] = MSG_SIZE;
    }
}

// +----------------------------------------------------------------------
// | 関数名　　:  servo_all_off()
// +----------------------------------------------------------------------
// | 機能     :  全サーボオフ
// +----------------------------------------------------------------------
void servo_all_off()
{
    for (int h = 0; h < 5; h++)
    {
        for (int i = 0; i < 15; i++)
        {
            if (idl_mt[i] == 1)
            {
                krs_L.setFree(i);
            }
            if (idr_mt[i] == 1)
            {
                krs_R.setFree(i);
            }
            delayMicroseconds(2);
        }
    }
    delay(100);
    Serial.println("All servos off.");
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// +----------------------------------------------------------------------
// | 関数名　　:  checksum_val(short arr[], int len)
// +----------------------------------------------------------------------
// | 機能     :  配列のチェックサムを算出
// | 　　     :  チェックサムは配列の末尾を除く合計数をビット反転しShort型にしたもの
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  short型. チェックサム値
// +----------------------------------------------------------------------
short checksum_val(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    return short(~cksm);
    // return ~cksm;//★もとの式　動いたら消す
}

// +-------------------------------------------------------------------
// | 関数名　　:  checksum_rslt(short arr[], int len)
// +-------------------------------------------------------------------
// | 機能     :  チェックサムの合否判定
// | 　　     :  配列の末尾を除く合計数をビット反転しShort型にしたものと, 末尾の値を比較
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  bool型. チェックサム値がOKならtrue, NGならfalse
// +------------------------------------------------------------------
bool checksum_rslt(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    if (short(~cksm) == arr[len - 1])
    {
        return true;
    }
    return false;
}

// +----------------------------------------------------------------------
// | 関数名　　:  float2HfShort(float val)
// +----------------------------------------------------------------------
// | 機能　　　:  floatを100倍し小数点以下を四捨五入した整数をshortに収める
// | 引数　　　:  float型. 有効数字　-327.67 ~ 327.67
// | 戻り値　　:  short型. 限界値を超えたら限界値を返す(-32767,32767)
// +----------------------------------------------------------------------
short float2HfShort(float val)
{
    int x = round(val * 100); // floatの小数点以下を四捨五入して整数化
    if (x > 32766)
    {
        x = 32767;
    }
    else if (x < -32766)
    {
        x = -32767;
    }
    return (short)x;
}

// +----------------------------------------------------------------------
// | 関数名　　:  HfDeg2Krs(int hfdegree, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  degree値*100 をKRS単位値に変換. HfはHundredfoldの略.
// | 引数１　　:  float型. degree値
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. KRS単位値（3500-11500）
// | 備考　　　:  0.02度ぐらいからサーボ値には反映される(=0.59で1に繰り上がる)
// +----------------------------------------------------------------------
int HfDeg2Krs(int hfdegree, int n, int cw)
{
    float x = 7500 + n + (hfdegree / 3.375 * cw); //
    if (x > 11500)                                // 上限を設定
    {
        x = 11500;
    }
    else if (x < 3500) //下限を設定
    {
        x = 3500;
    }
    return int(x);
}

// +----------------------------------------------------------------------
// | 関数名　　:  Krs2HfDeg(int krs, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  KRS単位値をdegree値*100に変換. HfはHundredfoldの略.
// | 引数１　　:  int型. サーボ値（KRS単位値）
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. degree値*100
// +----------------------------------------------------------------------
int Krs2HfDeg(int krs, int n, int pn)
{
    float x = (krs - 7500 - n) * 3.375 * pn;
    return int(x);
}

// +----------------------------------------------------------------------
// | 関数名　　:  setupIMUAHRS()
// +----------------------------------------------------------------------
// | 機能     :  MPU6050,BNO055等の初期設定を行う.　IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// | 引数　　　:  なし.
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void setupIMUAHRS()
{
    if (IMUAHRS_MOUNT == 1) // MPU6050
    {

        Wire.begin();
        Wire.setClock(I2C_CLOCK); // 400kHz I2C clock. Comment this line if having compilation difficulties
        mpu.initialize();
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXAccelOffset(-1745);
        mpu.setYAccelOffset(-1034);
        mpu.setZAccelOffset(966);
        mpu.setXGyroOffset(176);
        mpu.setYGyroOffset(-6);
        mpu.setZGyroOffset(-25);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.setDMPEnabled(true);
            packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else
        {
            Serial.print("DMP Initialization failed.");
        }
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055の場合
    {
        /* BNO055の初期化 */
        if (!bno.begin())
        {
            Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
            // while (1)
            //   ;
        }
        else
        {
            Serial.println("BNO055 initialize... OK.");
            delay(50);
            bno.setExtCrystalUse(false);
            delay(10);
        }
        // センサー用スレッド
        delay(10);
    }
    else
    {
        Serial.println("No IMU/AHRS sensor mounted.");
    }
}

// +----------------------------------------------------------------------
// | 関数名　　:  IMUAHRS_getYawPitchRoll()
// +----------------------------------------------------------------------
// | 機能     :  MPU6050,BNO055等の値を格納する.　IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// | 引数　　　:  なし.
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void IMUAHRS_getYawPitchRoll()
{
    if (IMUAHRS_MOUNT == 1) // MPU6050
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        { // 最新のIMU/AHRS情報を取得
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            //加速度の値
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu_read[0] = (float)aa.x;
            mpu_read[1] = (float)aa.y;
            mpu_read[2] = (float)aa.z;

            //ジャイロの値
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            mpu_read[3] = (float)gyro.x;
            mpu_read[4] = (float)gyro.y;
            mpu_read[5] = (float)gyro.z;

            //磁力センサの値
            mpu_read[6] = (float)mag.x;
            mpu_read[7] = (float)mag.y;
            mpu_read[8] = (float)mag.z;

            //重力DMP推定値
            mpu_read[9] = gravity.x;
            mpu_read[10] = gravity.y;
            mpu_read[11] = gravity.z;

            //相対方向DMP推定値
            mpu_read[12] = ypr[2] * 180 / M_PI;                // DMP_ROLL推定値
            mpu_read[13] = ypr[1] * 180 / M_PI;                // DMP_PITCH推定値
            mpu_read[14] = (ypr[0] * 180 / M_PI) - yaw_center; // DMP_YAW推定値

            //温度
            mpu_read[15] = 0; // fifoBufferからの温度取得方法が今のところ不明。

            if (flag_sensor_IMUAHRS_writable)
            {
                memcpy(mpu_result, mpu_read, sizeof(float) * 16);
            }
        }
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055
    {
        // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
        imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        mpu_read[0] = (float)accelermetor.x();
        mpu_read[1] = (float)accelermetor.y();
        mpu_read[2] = (float)accelermetor.z();

        // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
        imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        mpu_read[3] = gyroscope.x();
        mpu_read[4] = gyroscope.y();
        mpu_read[5] = gyroscope.z();

        // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
        imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        mpu_read[6] = magnetmetor.x();
        mpu_read[7] = magnetmetor.y();
        mpu_read[8] = magnetmetor.z();

        // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        mpu_read[12] = euler.y();                    // DMP_ROLL推定値
        mpu_read[13] = euler.z();                    // DMP_PITCH推定値
        mpu_read[14] = euler.x() - yaw_center - 180; // DMP_YAW推定値

        // センサフュージョンの方向推定値のクオータニオン
        imu::Quaternion quat = bno.getQuat();
        qw = quat.w(); // / 16384;
        qx = quat.x(); // / 16384;
        qy = quat.y(); // / 16384;
        qx = quat.z(); // / 16384;

        double ysqr = qy * qy;

        // roll (x-axis rotation)
        double t2 = +2.0 * (qw * qy - qz * qx);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        mpu_read[12] = -asin(t2) * 57.2957795131;

        // pitch (y-axis rotation)
        // double t3 = +2.0 * (qw * qz + qx * qy);
        // double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
        // mpu_read[13] = atan2(t3, t4) * 57.2957795131;

        // yaw (z-axis rotation)
        double t0 = +2.0 * (qw * qx + qy * qz);
        double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
        mpu_read[14] = -atan2(t0, t1) * 57.2957795131 - yaw_center;

        // 調整用数値モニタリング;
        /*
        Serial.print(qx);
        Serial.print(", ");
        Serial.print(qy);
        Serial.print(", ");
        Serial.print(qz);
        Serial.print(", ");
        Serial.print(qw);
        Serial.print(", ");
        Serial.print(mpu_read[12]);
        Serial.print(", ");
        Serial.print(mpu_read[13]);
        Serial.print(", ");
        Serial.println(mpu_read[14]);
        */

        /*
        // キャリブレーションのステータスの取得と表示
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIB Sys:"); Serial.print(system, DEC);
        Serial.print(", Gy"); Serial.print(gyro, DEC);
        Serial.print(", Ac"); Serial.print(accel, DEC);
        Serial.print(", Mg"); Serial.println(mag, DEC);
        */

        if (flag_sensor_IMUAHRS_writable)
        {
            memcpy(mpu_result, mpu_read, sizeof(float) * 16);
        }
    }
}

// +----------------------------------------------------------------------
// | 関数名　　:  joypad_read()
// +----------------------------------------------------------------------
// | 機能     :  Teensy4.0に接続されたJOYPADの値を読みとり、pad_btnに格納
// | 　　　　　:  0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void joypad_read()
{
    if (JOYPAD_MOUNT == 2)
    { // KRR5FH(KRC-5FH)をICS_R系に接続している場合
        joypad_frame_count++;
        if (joypad_frame_count >= JOYPAD_FRAME)
        {
            unsigned short buttonData;
#if 1
            bool ret;
            int joy_a[4];
            ret = krs_R.getKrrAllData(&buttonData, joy_a); // +Hori 20221008
            delayMicroseconds(2);
            if (ret)
#else
            buttonData = krs_R.getKrrButton();
            delayMicroseconds(2);
            if (buttonData != KRR_BUTTON_FALSE) // ボタンデータが受信できていたら
#endif

            {
                button_1 = buttonData;
                pad_btn = 0;
                if ((button_1 & 15) == 15)
                { // 左側十字ボタン全部押しならstart押下とみなす
                    pad_btn += 1;
                }
                else
                {
                    // 左側の十字ボタン
                    pad_btn += (button_1 & 1) * 16 + ((button_1 & 2) >> 1) * 64 + ((button_1 & 4) >> 2) * 32 + ((button_1 & 8) >> 3) * 128;
                }
                if ((button_1 & 368) == 368)
                    pad_btn += 8; // 右側十時ボタン全部押しならselect押下とみなす
                else
                {
                    // 右側十字ボタン
                    pad_btn += ((button_1 & 16) >> 4) * 4096 + ((button_1 & 32) >> 5) * 16384 + ((button_1 & 64) >> 6) * 8192 + ((button_1 & 256) >> 8) * 32768;
                }
                // L1,L2,R1,R2
                pad_btn += ((button_1 & 2048) >> 11) * 2048 + ((button_1 & 4096) >> 12) * 512 + ((button_1 & 512) >> 9) * 1024 + ((button_1 & 1024) >> 10) * 256;

#if 1
                if (joy_a[0] + joy_a[1] + joy_a[2] + joy_a[3])
                {
                    for (int i = 0; i < 4; i++)
                    { // +Hori 20221008
                        pad_analog[i] = (joy_a[i] - 62) << 2;
                        pad_analog[i] = (pad_analog[i] < -127) ? -127 : pad_analog[i];
                        pad_analog[i] = (pad_analog[i] > 127) ? 127 : pad_analog[i];
                    }
                }
                else
                    for (int i = 0; i < 4; i++)
                    {
                        pad_analog[i] = 0;
                    }
#endif
            }
            joypad_frame_count = 0;
        }
    }
}

// +----------------------------------------------------------------------
// | 関数名　　:  set_EEPROM()
// +----------------------------------------------------------------------
// | 機能     :  ここで指定したハードウェア設定内容をEEPROMに書き込む
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void set_EEPROM() //　★ここの設定から再開する
{
    /* 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転） */
    // 1:KONDO_KRS, 2:KONDO RS485, 3:FUTABA_TTL, 4:FUTABARS485,
    // 5:ROBOTIS_TTL,6:ROBOTIS_RS485 7:JRPROPO, 8:HiTECH,
    // 9:未定, 10:PWM
    // 凡例: idl_mt[20] = -3; → FUTABA_TTLサーボを逆転設定でマウント
    eeprom_write.sval[20] = 1; // idl[00]頭ヨー
    eeprom_write.sval[22] = 1; // idl[01]左肩ピッチ
    eeprom_write.sval[24] = 1; // idl[02]左肩ロール
    eeprom_write.sval[26] = 1; // idl[03]左肘ヨー
    eeprom_write.sval[28] = 1; // idl[04]左肘ピッチ
    eeprom_write.sval[30] = 1; // idl[05]左股ヨー
    eeprom_write.sval[32] = 1; // idl[06]左股ロール
    eeprom_write.sval[34] = 1; // idl[07]左股ピッチ
    eeprom_write.sval[36] = 1; // idl[08]左膝ピッチ
    eeprom_write.sval[38] = 1; // idl[09]左足首ピッチ
    eeprom_write.sval[40] = 1; // idl[10]左足首ロール
    eeprom_write.sval[42] = 0; // idl[11]追加テスト用
    eeprom_write.sval[44] = 0; // idl[12]追加テスト用
    eeprom_write.sval[46] = 0; // idl[13]追加テスト用
    eeprom_write.sval[48] = 0; // idl[14]追加テスト用
    eeprom_write.sval[50] = 1; // idr[00]腰ヨー
    eeprom_write.sval[52] = 1; // idr[01]右肩ピッチ
    eeprom_write.sval[54] = 1; // idr[02]右肩ロール
    eeprom_write.sval[56] = 1; // idr[03]右肘ヨー
    eeprom_write.sval[58] = 1; // idr[04]右肘ピッチ
    eeprom_write.sval[60] = 1; // idr[05]右股ヨー
    eeprom_write.sval[62] = 1; // idr[06]右股ロール
    eeprom_write.sval[64] = 1; // idr[07]右股ピッチ
    eeprom_write.sval[66] = 1; // idr[08]右膝ピッチ
    eeprom_write.sval[68] = 1; // idr[09]右足首ピッチ
    eeprom_write.sval[70] = 1; // idr[10]右足首ロール
    eeprom_write.sval[72] = 0; // idr[11]追加テスト用
    eeprom_write.sval[74] = 0; // idr[12]追加テスト用
    eeprom_write.sval[76] = 0; // idr[13]追加テスト用
    eeprom_write.sval[78] = 0; // idr[14]追加テスト用

    /* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */
    // 直立状態になるよう、具体的な数値を入れて現物調整する
    eeprom_write.sval[21] = 0;     // idl[00]頭ヨー
    eeprom_write.sval[23] = -70;   // idl[01]左肩ピッチ
    eeprom_write.sval[25] = -2700; // idl[02]左肩ロール
    eeprom_write.sval[27] = 0;     // idl[03]左肘ヨー
    eeprom_write.sval[29] = 2666;  // idl[04]左肘ピッチ
    eeprom_write.sval[31] = 0;     // idl[05]左股ヨー
    eeprom_write.sval[33] = 0;     // idl[06]左股ロール
    eeprom_write.sval[35] = -40;   // idl[07]左股ピッチ
    eeprom_write.sval[37] = -1720; // idl[08]左膝ピッチ
    eeprom_write.sval[39] = -600;  // idl[09]左足首ピッチ
    eeprom_write.sval[41] = -20;   // idl[10]左足首ロール
    eeprom_write.sval[43] = 0;     // idl[11]追加テスト用
    eeprom_write.sval[45] = 0;     // idl[12]追加テスト用
    eeprom_write.sval[47] = 0;     // idl[13]追加テスト用
    eeprom_write.sval[49] = 0;     // idl[14]追加テスト用
    eeprom_write.sval[51] = 0;     // idr[00]腰ヨー
    eeprom_write.sval[53] = 0;     // idr[01]右肩ピッチ
    eeprom_write.sval[55] = -2650; // idr[02]右肩ロール
    eeprom_write.sval[57] = 0;     // idr[03]右肘ヨー
    eeprom_write.sval[59] = 2666;  // idr[04]右肘ピッチ
    eeprom_write.sval[61] = 0;     // idr[05]右股ヨー
    eeprom_write.sval[63] = 50;    // idr[06]右股ロール
    eeprom_write.sval[65] = -100;  // idr[07]右股ピッチ
    eeprom_write.sval[67] = -1700; // idr[08]右膝ピッチ
    eeprom_write.sval[69] = -600;  // idr[09]右足首ピッチ
    eeprom_write.sval[71] = -70;   // idr[10]右足首ロール
    eeprom_write.sval[73] = 0;     // idr[11]追加テスト用
    eeprom_write.sval[75] = 0;     // idr[12]追加テスト用
    eeprom_write.sval[77] = 0;     // idr[13]追加テスト用
    eeprom_write.sval[79] = 0;     // idr[14]追加テスト用

    // EEPROM書き込み
    for (int i = 0; i < 540; i++) // データを書き込む時はbyte型
    {
        EEPROM.write(i, eeprom_write.bval[i]);
    }
    Serial.println();
}

// +----------------------------------------------------------------------
// | 関数名　　:  sd_setup()
// +----------------------------------------------------------------------
// | 機能     :  SDカードの挿入と起動をチェックし、
// | 　　　　　:  ランダムな4桁のコードの読み書きを行い動作チェックとする.
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void sd_setup()
{
    Serial.print("Checking SD card...");
    delay(100);
    if (!SD.begin(CHIPSELECT_SD))
    {
        Serial.println(" FALIED!");
        delay(500);
        // Serial.println("Retry.");
        // return;
    }
    else
    {
        Serial.println(" OK.");
    }

    if (SD_TEST)
    {
        SD.remove("/test.txt");
        // open the file.
        sdFile = SD.open("/test.txt", FILE_WRITE);
        delayMicroseconds(1); // SPI安定化検証用

        // if the file opened okay, write to it:
        if (sdFile)
        {
            Serial.print("SD card test...");
            randomSeed(analogRead(2) * 10 + analogRead(3) * 2 + analogRead(4));
            int randNumber = random(1000, 9999); // 0から299の乱数を生成
            Serial.print(" Writing code ");
            Serial.print(randNumber);
            sdFile.println(randNumber);
            delayMicroseconds(1); // SPI安定化検証用
            // close the file:
            sdFile.close();
            Serial.print(" and");
        }
        else
        {
            // if the file didn't open, print an error:
            Serial.println("... opening /test.txt FALIED!");
        }
        delayMicroseconds(1); // SPI安定化検証用
        // re-open the file for reading:
        sdFile = SD.open("/test.txt");

        if (sdFile)
        {
            Serial.print(" read code ");
            // read from the file until there's nothing else in it:
            while (sdFile.available())
            {
                Serial.write(sdFile.read());
            }
            // close the file:
            sdFile.close();
        }
        else
        {
            // if the file didn't open, print an error:
            Serial.println("... opening /test.txt FALIED!");
        }
        delay(100);
    }
}

//================================================================================================================
//---- セ ッ ト ア ッ プ -------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
    // EEPROM書き込み
    if (SET_EEPROM)
    {
        set_EEPROM();
    }

    // EEPROM読み込み
    for (int i = 0; i < 540; i++) // データを読み込む時はbyte型
    {
        eeprom_load.bval[i] = EEPROM.read(i);
    }

    //-------------------------------------------------------------------------
    //---- サ ー ボ 設 定  -----------------------------------------------------
    //-------------------------------------------------------------------------
    /* 各サーボのマウントありなし（1:サーボあり、0:サーボなし） */
    for (int i = 0; i < 15; i++)
    {
        if (eeprom_load.sval[i * 2 + 20] != 0)
        {
            idl_mt[i] = 1;
        }
        else
        {
            idl_mt[i] = 0;
        }
        if (eeprom_load.sval[i * 2 + 50] != 0)
        {
            idr_mt[i] = 1;
        }
        else
        {
            idr_mt[i] = 0;
        }
    }

    /* 各サーボの内外回転プラマイ方向補正(1 or -1) */
    for (int i = 0; i < 15; i++)
    {
        if (eeprom_load.sval[i * 2 + 20] >= 0)
        {
            idl_cw[i] = 1;
        }
        else
        {
            idl_cw[i] = -1;
        }
        if (eeprom_load.sval[i * 2 + 50] >= 0)
        {
            idr_cw[i] = 1;
        }
        else
        {
            idr_cw[i] = -1;
        }
    }

    /* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */
    for (int i = 0; i < 15; i++)
    {
        idl_trim[i] = eeprom_load.sval[i * 2 + 21];
        idr_trim[i] = eeprom_load.sval[i * 2 + 51];
    }

    //-------------------------------------------------------------------------
    //---- 起　動　時 設 定 -----------------------------------------------------
    //-------------------------------------------------------------------------

    /* 入出力ピンのモード設定 */
    pinMode(ERR_LED, OUTPUT); // 通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

    /* シリアル設定 */
    Serial.begin(SERIAL_PC); // シリアルモニター表示
    delay(100);

    /* 起動時のインフォメーション表示表示(シリアルモニタ) */
    Serial.println();
    Serial.print("Hi, This is ");
    Serial.println(VERSION);
    Serial.print("PC-Serial speed: ");
    Serial.println(SERIAL_PC);
    Serial.print("Set SPI speed  : ");
    Serial.println(SPI_CLOCK);
    Serial.print("Set I2C speed  : ");
    Serial.println(I2C_CLOCK);

    int value = EEPROM.length(); // EEPROMの長さ
    Serial.print("EEPROM length  : ");
    Serial.println(value); // EEPROMの長さ表示

    Serial.print("Left side Servos mounted : ");
    for (int i = 0; i < 15; i++)
    {
        if (idl_mt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();
    Serial.print("Right side Servos mounted: ");
    for (int i = 0; i < 15; i++)
    {
        if (idr_mt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();

    Serial.print("I2C IMU/AHRS Sensor mounted : ");
    if (IMUAHRS_MOUNT == 0)
    {
        Serial.println("None. ");
    }
    else if (IMUAHRS_MOUNT == 1)
    {
        Serial.print("MPU6050(GY-521) ");
    }
    else if (IMUAHRS_MOUNT == 2)
    {
        Serial.print("MPU9250(GY-6050/GY-9250) ");
    }
    else if (IMUAHRS_MOUNT == 3)
    {
        Serial.print("BNO055 ");
    }
    if (IMUAHRS_MOUNT != 0)
    {
        Serial.print(" freq: ");
        Serial.print(IMUAHRS_FREQ);
        Serial.println(" ms");
    }

    Serial.print("Controll Pad Receiver mounted: ");
    if (JOYPAD_MOUNT == 0)
    {
        Serial.print("None. ");
    }
    else if (JOYPAD_MOUNT == 1)
    {
        Serial.print("SBDBT ");
    }
    else if (JOYPAD_MOUNT == 2)
    {
        Serial.print("KRC-5FH ");
    }
    else if (JOYPAD_MOUNT == 3)
    {
        Serial.print("Merimote PICO ");
    }
    if (JOYPAD_MOUNT != 0)
    {
        Serial.print(" freq: ");
        Serial.print(JOYPAD_FRAME);
        Serial.println(" ms");
    }
    delay(100);

    /* サーボ用シリアル設定 */
    krs_L.begin(); // サーボモータの通信初期設定。Serial2
    krs_R.begin(); // サーボモータの通信初期設定。Serial3
    if (ICS3_MOUNT)
    {
        krs_3.begin(); // サーボモータの通信初期設定。Serial1
    }
    delay(100);

    /* SDカードの初期化 */

    if (SD_MOUNT)
    {
        sd_setup();
    }
    else
    {
        Serial.println("No SD reader mounted.");
    }

    /* SPI通信用DMAの設定 */
    TsyDMASPI0.begin(SS, SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    /* 配列のリセット */
    memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);     //配列要素を0でリセット
    memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);     //配列要素を0でリセット
    memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
    memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
    memset(idl_d, 0, 15);                            //配列要素を0でリセット
    memset(idr_d, 0, 15);                            //配列要素を0でリセット

    /* I2Cに接続したIMU/AHRSセンサの割り込み設定 */
    if (IMUAHRS_MOUNT == 1) // MPU6050の場合
    {
        setupIMUAHRS();
        /* MPU6050の割り込み設定 */
        MsTimer2::set(IMUAHRS_FREQ, IMUAHRS_getYawPitchRoll); // MPUの情報を取得 10msごとにチェック
        MsTimer2::start();
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055の場合
    {
        setupIMUAHRS();
        /* BNO055の割り込み設定 */
        MsTimer2::set(IMUAHRS_FREQ, IMUAHRS_getYawPitchRoll); // AHRSの情報を取得 IMUAHRS_FREQ msごとにチェック
        MsTimer2::start();
    }
    else
    {
        Serial.println("No IMU/AHRS sensor mounted.");
    }

    /* 変数の設定 */
    yaw_center = 0;
    s_spi_meridim.sval[0] = MSG_SIZE; // (マスターコマンド）

    /* 起動時のディレイ用mercちょい足し */
    merc = merc + 3500;
    Serial.println();
    // EEPROMテスト

    /*

    // データ作成
    for (int i = 0; i < 270; i++) // 書き込むデータはshort型で作成
    {
        eeprom_write.sval[i] = 0;
    }

    for (int i = 0; i < 15; i++) // 書き込むデータはshort型で作成
    {
        eeprom_write.sval[i * 2 + 20] = idl_trim[i];
        eeprom_write.sval[i * 2 + 50] = idr_trim[i];
    }

    // 書き込み
    for (int i = 0; i < 540; i++) // データを書き込む時はbyte型
    {
        EEPROM.write(i, eeprom_write.bval[i]);
    }
    Serial.println();
    */

    // EEPROM内容表示
    Serial.println("Display EEPROM:");
    for (int i = 0; i < 270; i++) // 書き込むデータはshort型で作成
    {
        Serial.print("[");
        Serial.print(i);
        Serial.print("]");
        Serial.print(eeprom_load.sval[i]);
        if ((i == 89) or (i == 179))
        {
            Serial.println();
        }
        else
        {
            Serial.print(",");
        }
    }
    Serial.println();
    // 初期設定完了
    Serial.println("Ready. ");
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

    //////// < 1 > E S P 3 2 と の S P I に よ る 送 受 信 処 理  /////////////////////////

    // @[1-1] ESP32とのSPI送受信の実行
    if (ESP32_MOUNT)
    {
        TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

        spi_trial++; // SPI送受信回数のカウント

        // [1-2] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
        // チェックサムがOKならバッファから受信配列に転記
        if (checksum_rslt(r_spi_meridim_dma.sval, MSG_SIZE))
        {
            for (int i = 0; i < MSG_SIZE; i++)
            {
                r_spi_meridim.sval[i] = r_spi_meridim_dma.sval[i];
            }
            spi_ok++;
            r_spi_meridim.bval[MSG_ERR_u] &= B11011111; //エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオフ

            // SPI送信タイミング「待受モード」のコマンドチェック.
            if (r_spi_meridim.sval[0] == 5) //　5ならフラグ下げ. Teensy主導
            {
                flag_spi_initiative_outer = false;
            }
            else if (r_spi_meridim.sval[0] == 6) // 6 ならフラグ上げ. 外部主導
            {
                flag_spi_initiative_outer = true;
            }
        }
        else
        {
            r_spi_meridim.bval[MSG_ERR_u] |= B00100000; //エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオン
        }

        // @[1-3-1] シーケンス番号予想値の生成
        frame_sync_r_expect++;           // フレームカウント予想値を加算
        if (frame_sync_r_expect > 59999) // 予想値が29,999以上ならカウントを-30000に戻す
        {
            frame_sync_r_expect = 0;
        }
      
        // @[1-3-2] 受信シーケンス番号のデータ格納
        frame_sync_r_resv = r_spi_meridim.usval[1];

        // @[1-3-3] シーケンス番号チェック
        if (frame_sync_r_expect == r_spi_meridim.usval[1]) // 受信シーケンス番号の値が予想通りなら,
        {
            r_spi_meridim.bval[MSG_ERR_u] &= 0b11111101; // Meridim[MSG_ERR] 9番ビット:Teensy受信のスキップ検出をサゲる.
        }
        else // 受信シーケンス番号の値が予想と違ったら,
        {
            frame_sync_r_expect = r_spi_meridim.usval[1]; // 現在の受信値を予想結果としてキープ
            r_spi_meridim.bval[MSG_ERR_u] |= 0b00000010;  // Meridim[MSG_ERR] 9番ビット:Teensy受信のスキップ検出をアゲる.
            err_tsy_skip++;
        }

        // [1-4] 通信エラー処理(エラーカウンタへの反映)

        if ((r_spi_meridim.bval[MSG_ERR_u] >> 7) & 0b00000001) // Meridim[88] bit15:PCのESP32からのUDP受信エラー
        {
            err_esp_pc++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u] >> 6) & 0b00000001) // Meridim[88] bit14:ESP32のPCからのUDP受信エラー
        {
            err_pc_esp++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u] >> 5) & 0b00000001) // Meridim[88] bit13:TeensyのESPからのSPI受信エラー
        {
            err_esp_tsy++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u] >> 4) & 0b00000001) // Meridim[88] bit12:ESP32のTeensyからのSPI受信エラー
        {
            err_tsy_esp++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u] >> 2) & 0b00000001) // Meridim[88] bit10:UDP→ESP受信のカウントのスキップ
        {
            err_esp_skip++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u] >> 1) & 0b00000001) // Meridim[88] bit9:ESP→Teensy受信のカウントのスキップ
        {
            err_tsy_skip++;
        }
        if ((r_spi_meridim.bval[MSG_ERR_u]) & 0b00000001) // Meridim[88] bit8:PC受信のカウントのスキップ
        {
            err_pc_skip++;
        }

        //////// < 2 > シ リ ア ル モ ニ タ リ ン グ 表 示 処 理  //////////////////////////////

        // [2-1] //受信データの表示（SPI受信データShort型）

        // [2-2] //受信エラー率の表示

        // [2-3] 全経路のエラー数の表示
        if (monitor_all_error)
        {
            Serial.print("[ERRORs] esp->pc:");
            Serial.print(err_esp_pc);
            Serial.print("  pc->esp:");
            Serial.print(err_pc_esp);
            Serial.print("  esp->tsy:");
            Serial.print(err_esp_tsy);
            Serial.print("  tsy->esp:");
            Serial.print(err_esp_tsy);
            Serial.print("  tsy-skip:");
            Serial.print(err_tsy_skip); //
            Serial.print("  esp-skip:");
            Serial.print(err_esp_skip); //
            Serial.print("  pc-skip:");
            Serial.print(err_pc_skip); //
            Serial.print("  seq:");
            Serial.print(int(frame_sync_r_resv)); //
            Serial.print("  [ERR]:");
            Serial.print(r_spi_meridim.bval[MSG_ERR_u], BIN);
            Serial.println();
        }
    }

    //////// < 3 > 積 み 残 し 処 理  ////////////////////////////////////////////////////
    // → 積み残しがあればここで処理

    //////// < 3.5 > S P I 送 信 の タ イ ミ ン グ を 外 部 に 依 存 す る「待 受 モ ー ド 」の 処 理 ///

    // if ((flag_spi_initiative_outer == false) or (r_spi_meridim_last_cksm != r_spi_meridim.sval[MSG_SIZE - 1])) // チェックサムを比較
    if ((flag_spi_initiative_outer == false) or (frame_sync_r_resv != frame_sync_r_last)) //★★★ シーケンシャルカウンタを比較
    {                                                                                     // SPI送信のタイミングを外部に依存する「待受モード」の場合, 新しいデータが来るまで以下の処理を飛ばして待受する.
        // シーケンス番号と待受フラグのモニタリング(カウンタが同じ場合)
        // Serial.println(frame_sync_r_last);         //前回のカウンタ
        // Serial.println(frame_sync_r_resv);         //今回のカウンタ
        // Serial.println(flag_spi_initiative_outer); //待受モードのフラグ状態
        // Serial.println("***** accepted");          //データ処理の作業の採用通知

        //////// < 4 > 受 信 S P I デ ー タ を 送 信 S P I デ ー タ に 転 記 ////////////////////
        memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MSG_BUFF + 4);

        //////// < 5 > セ ン サ ー 類 読 み 取 り /////////////////////////////////////////////
        // @[5-1] IMU/AHRSの値を取得
        // ※IMU/AHRSについてはタイマー割り込みで別途処理

        //////// < 6 > コ ン ト ロ ー ラ の 読 み 取 り　///////////////////////////////////////
        //[6-1] コントローラの値を取得
        if (JOYPAD_MOUNT == 1)
        { // SBDBTが接続設定されていれば受信チェック（未実装）
            Serial.print("SBDBT connection has not been programmed yet.");
        }
        if (JOYPAD_MOUNT == 2)
        { // KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
            joypad_read();
            r_spi_meridim.sval[15] = 0; //急場しのぎ
            s_spi_meridim.sval[15] = 0; //急場しのぎ
            r_spi_meridim.sval[15] |= pad_btn;
            s_spi_meridim.sval[15] |= pad_btn;
        }

        //////// < 7 > Teensy 内 部 で 位 置 制 御 す る 場 合 の 処 理 /////////////////////////

        // @[7-1] マスターコマンドの判定によりこの工程の実行orスキップを分岐(デフォルトはMeridim配列数である90)

        // コマンド[90]: サーボオン 通常動作

        // コマンド[0]: 全サーボ脱力

        // コマンド[1]: サーボオン 通常動作

        // コマンド[2]: IMU/AHRSのヨー軸リセット
        if (r_spi_meridim.sval[0] == UPDATE_YAW_CENTER)
        {
            setyawcenter();
        }

        // コマンド[3]: トリムモードがオンもしくはコマンド3の時はループ
        if ((trim_adjust == 1) or (r_spi_meridim.sval[0] == ENTER_TRIM_MODE))
        {
            trimadjustment();
        }

        // @[7-2] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
        for (int i = 0; i < 15; i++)
        {
            s_servo_pos_L[i] = HfDeg2Krs(r_spi_meridim.sval[i * 2 + 21], idl_trim[i], idl_cw[i]); //
            s_servo_pos_R[i] = HfDeg2Krs(r_spi_meridim.sval[i * 2 + 51], idr_trim[i], idr_cw[i]); //
        }

        // @[7-3] Teensyによる次回動作の計算

        // リモコンの左十字キー左右で首を左右にふるサンプル
        if (r_spi_meridim.sval[15] == 32)
        {
            s_servo_pos_L[0] = HfDeg2Krs(-3000, idl_trim[0], idl_cw[0]); //
        }
        else if (r_spi_meridim.sval[15] == 128)
        {
            s_servo_pos_L[0] = HfDeg2Krs(3000, idl_trim[0], idl_cw[0]); //
        }

        /*
        if (s_spi_meridim.sval[15] == 32)
        {
            s_servo_pos_L[0] = HfDeg2Krs(-3000, idl_trim[0], idl_cw[0]); //
        }
        else if (s_spi_meridim.sval[15] == 128)
        {
            s_servo_pos_L[0] = HfDeg2Krs(3000, idl_trim[0], idl_cw[0]); //
        }
        Serial.println(r_spi_meridim.sval[15]);
        */
        // @[7-4] センサーデータによる動作へのフィードバック加味

        // @[7-5] 移動時間の決定

        // @[7-6] Teensy内計算による次回動作をMeridim配列に書き込む

        //////// < 8 > サ ー ボ コ マ ン ド の 書 き 込 み /////////////////////////////////////

        // @[8-1] Meridim配列をサーボ命令に変更

        // @[8-2] サーボコマンドの配列に書き込み

        // @[8-3] サーボデータのICS送信および返り値を取得

        for (int i = 0; i < servo_num; i++) // ICS_L系統の処理
        {                                   //接続したサーボの数だけ繰り返す。最大は15
            idl_d[i] = 0;
            if (idl_mt[i])
            {
                if (r_spi_meridim.sval[(i * 2) + 20] == 1) //受信配列のサーボコマンドが1ならPos指定
                {
                    k = krs_L.setPos(i, s_servo_pos_L[i]);
                    if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = s_servo_pos_L[i];
                    }
                }
                else // 1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
                {
                    k = krs_L.setFree(i);
                    if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = s_servo_pos_L[i];
                    }
                }
                idl_d[i] = Krs2HfDeg(k, idl_trim[i], idl_cw[i]);
            }
            delayMicroseconds(2);
            //}

            // for (int i = 0; i < 11; i++) // ICS_R系統の処理
            //{                            //接続したサーボの数だけ繰り返す。最大は15
            idr_d[i] = 0;
            if (idr_mt[i])
            {
                if (r_spi_meridim.sval[(i * 2) + 50] == 1) //受信配列のサーボコマンドが1ならPos指定
                {
                    k = krs_R.setPos(i, s_servo_pos_R[i]);
                    if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = s_servo_pos_R[i];
                    }
                }
                else // 1以外ならとりあえずサーボを脱力し位置を取得
                {
                    k = krs_R.setFree(i);
                    if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = s_servo_pos_R[i];
                    }
                }
                idr_d[i] = Krs2HfDeg(k, idr_trim[i], idr_cw[i]);
            }
            delayMicroseconds(2);
        }

        //////// < 9 > S P I 送 信 用 の Meridim 配 列 を 作 成 す る //////////////////////////

        // @[9-1] マスターコマンドを配列に格納
        // ※ここで特に作業しなければ, 受け取った値をそのままスルー返信する.
        // s_spi_meridim.sval[0] = MSG_SIZE; //デフォルトのマスターコマンドは配列数

        // @[9-2] 移動時間を配列に格納
        // s_spi_meridim.sval[19] = 1 ;// モーションのフレーム数とイージング

        // @[9-3] センサー値を配列に格納
        if (IMUAHRS_MOUNT != 0)
        {
            flag_sensor_IMUAHRS_writable = false;
            s_spi_meridim.sval[2] = float2HfShort(mpu_result[0]);   // IMU/AHRS_acc_x
            s_spi_meridim.sval[3] = float2HfShort(mpu_result[1]);   // IMU/AHRS_acc_y
            s_spi_meridim.sval[4] = float2HfShort(mpu_result[2]);   // IMU/AHRS_acc_z
            s_spi_meridim.sval[5] = float2HfShort(mpu_result[3]);   // IMU/AHRS_gyro_x
            s_spi_meridim.sval[6] = float2HfShort(mpu_result[4]);   // IMU/AHRS_gyro_y
            s_spi_meridim.sval[7] = float2HfShort(mpu_result[5]);   // IMU/AHRS_gyro_z
            s_spi_meridim.sval[8] = float2HfShort(mpu_result[6]);   // IMU/AHRS_mag_x
            s_spi_meridim.sval[9] = float2HfShort(mpu_result[7]);   // IMU/AHRS_mag_y
            s_spi_meridim.sval[10] = float2HfShort(mpu_result[8]);  // IMU/AHRS_mag_z
            s_spi_meridim.sval[11] = float2HfShort(mpu_result[15]); // temperature
            s_spi_meridim.sval[12] = float2HfShort(mpu_result[12]); // DMP_ROLL推定値
            s_spi_meridim.sval[13] = float2HfShort(mpu_result[13]); // DMP_PITCH推定値
            s_spi_meridim.sval[14] = float2HfShort(mpu_result[14]); // DMP_YAW推定値
            flag_sensor_IMUAHRS_writable = true;
        }

        // @[9-4] サーボIDごとにの現在位置もしくは計算結果を配列に格納
        for (int i = 0; i < 15; i++)
        {
            s_spi_meridim.sval[i * 2 + 20] = 0;        // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
            s_spi_meridim.sval[i * 2 + 21] = idl_d[i]; // 仮にここでは最新のサーボ角度degreeを格納
        }
        for (int i = 0; i < 15; i++)
        {
            s_spi_meridim.sval[i * 2 + 50] = 0;        // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
            s_spi_meridim.sval[i * 2 + 51] = idr_d[i]; // 仮にここでは最新のサーボ角度degreeを格納
        }

        // @[9-5] フレームスキップ検出用のカウントをカウントアップして送信用に格納
        frame_sync_s++;           // シーケンシャルカウンタを加算
        if (frame_sync_s > 59999) // 予想値が29,999以上ならカウントを-30000に戻す
        {
            frame_sync_s = 0;
        }
        s_spi_meridim.usval[1] = frame_sync_s;

        // @[9-6] カスタムデータを配列格納

        // @[9-7] チェックサムを計算
        s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);

        // @[9-8] 送信データのSPIバッファへのバイト型書き込み
        for (int i = 0; i < MSG_BUFF; i++)
        {
            s_spi_meridim_dma.bval[i] = s_spi_meridim.bval[i];
        }

        //////// < 11 > フ レ ー ム 終 端 処 理 ///////////////////////////////////////////////

        // if (flag_spi_initiative_outer == false) // SPI待受モード時は消化タイムを無視してベストエフォート
        //{
        //  @[11-1] この時点で１フレーム内に処理が収まっていない時の処理
        curr = (long)millis(); // 現在時刻を更新
        if (curr > merc)
        {                              // 現在時刻がフレーム管理時計を超えていたらアラートを出す
            Serial.print("* delay: "); //シリアルに遅延msを表示
            Serial.println(curr - merc);
            digitalWrite(ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
        }
        else
        {
            digitalWrite(ERR_LED, LOW); // 処理が収まっていればLEDを消灯
        }

        // @[11-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
        curr = (long)millis();
        curr_micro = (long)micros(); // 現在時刻を取得
        // Serial.println(merc * 1000 - curr_micro); // 詳細な残り時間をμ秒単位でシリアル表示
        while (curr < merc)
        {
            curr = (long)millis();
        }

        // @[11-3]フレーム管理時計mercのカウントアップ
        merc = merc + frame_ms;                       // フレーム管理時計を1フレーム分進める
        frame_count = frame_count + frame_count_diff; // サインカーブ等動作用のフレームカウントアップ
        if (frame_count > frame_count_max)            // カウンターが最大値ならゼロリセット
        {
            frame_count = 0;
        }
        //}

        r_spi_meridim_last_cksm = r_spi_meridim.sval[MSG_SIZE - 1]; //前回のチェックサムをキープ
        frame_sync_r_last = frame_sync_r_resv;                      //前回受信したシーケンシャル番号をキープ
        //}
        // else
        //{
        //    // シーケンス番号と待受フラグのモニタリング(カウンタが同じ場合)
        //    Serial.println(frame_sync_r_last);         //前回のカウンタ
        //    Serial.println(frame_sync_r_resv);         //今回のカウンタ
        //    Serial.println(flag_spi_initiative_outer); //待受モードのフラグ状態
        //    Serial.println("----- cancelled");         //データ処理の作業のキャンセル通知
        //    delay(1);                                  // SPI送信のタイミングを外部に依存する「待受モード」の場合、ループ中に1ms挟む.
    }
}
