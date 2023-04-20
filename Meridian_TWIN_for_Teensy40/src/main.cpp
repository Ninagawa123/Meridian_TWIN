
// Meridian_TWIN_for_Teensy_20220828 By Izumi Ninagawa
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト　20230420版
// 220723 内部計算時に degree*100 を単位として使用するように変更
// 220723 センサーの関数を集約
// 220723 サーボオン時にリモコン左十字キー入力で首を左右に振る動作サンプル入り
// 220730 PCからのリモコン受信が有効となるように調整
// 220828 サーボからの受信が-1(タイムアウト)の時、前に取得した情報を使用する（データ飛びや表示のブレを防止）
// 220828 SERVO_NUM_L, SERVO_NUM_R に左右の接続サーボ数の登録を設定
// 220828 左右の接続サーボ数の多い方をservo_numとし、サーボ送信命令も左右交互に実行するよう改定
// 230420 起動時にSDカードの動作チェックを行えるようにした。
// 230420 起動時のメッセージ表記の順番やテキストを若干変更。

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
#define VERSION "Meridian_TWIN_for_Teensy_2023.04.20" // バージョン表示
#define FRAME_DURATION 10                             // 1フレームあたりの単位時間（単位ms）

/* シリアルモニタリング切り替え */
#define MONITOR_ALL_ERROR 0 // Teensyでのシリアル表示:全経路の受信エラー率
#define MONITOR_JOYPAD 0    // Teensyでのシリアル表示:リモコンのデータ

/* マウント有無とピンアサイン */
#define ESP32_MOUNT 1   // ESPの搭載 0:なし(SPI通信およびUDP通信を実施しない), 1:あり
#define IMUAHRS_MOUNT 1 // IMU/AHRSの搭載状況 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
#define IMUAHRS_FREQ 10 // IMU/AHRSのセンサの読み取り間隔(ms)
#define JOYPAD_MOUNT 0  // ジョイパッドの搭載 0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH (※2のみ実装済,MeridianBoardではICS_R系に接続)
#define JOYPAD_FRAME 4  // 上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define ICS3_MOUNT 0    // 半二重サーボ信号の3系のありなし
#define SD_MOUNT 1      // SDカードリーダーのありなし. MeridianBoard Type.Kは有り
#define SD_RWCHECK 0    // 起動時のSDカードリーダーの読み書きチェック
#define CHIPSELECT_SD 9 // SDカードSPI通信用のChipSelectのピン番号
#define SERVO_NUM_L 11  // L系統につないだサーボの数
#define SERVO_NUM_R 11  // R系統につないだサーボの数

/* マスターコマンド定義 */
#define TRIM_ADJUST_MODE 0      // トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
#define UPDATE_YAW_CENTER 10002 // センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 10003   // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

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
#include <Wire.h>                       // MPU-6050のライブラリ導入
#include <SPI.h>                        // SDカード用のライブラリ導入
#include <SD.h>                         // SDカード用のライブラリ導入
#include <TsyDMASPI.h>                  // SPI Master用のライブラリを導入
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050のライブラリ導入
#include <IcsHardSerialClass.h>         // ICSサーボのライブラリ導入
#include <MsTimer2.h>                   // タイマーのライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ
#include <TeensyThreads.h>              // マルチスレッド用のライブラリ

/* 変数一般 */
int spi_ok = 0;                                // 通信のエラーカウント
int spi_trial = 0;                             // 通信のエラーカウント
int k;                                         // 各サーボの計算用変数
int servo_num = max(SERVO_NUM_L, SERVO_NUM_R); // サーボ送受信のループ処理数（L系R系で多い方）
File myFile;                                   // SDカード用

/* フラグ用変数 */
bool flag_sensor_IMUAHRS_writable = true; // メインが結果値を読み取る瞬間、サブスレッドによる書き込みをウェイト

/* Meridim配列用の共用体の設定 */
typedef union // 共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
    short sval[MSG_SIZE + 4];   // short型で90個の配列データを持つ
    uint8_t bval[MSG_BUFF + 4]; // 1バイト単位で180個の配列データを持つ
} UnionData;
UnionData s_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData s_spi_meridim_dma; // SPI送信用の共用体のインスタンスを作成
UnionData r_spi_meridim_dma; // SPI受信用の共用体のインスタンスを作成

/* タイマー管理用の変数 */
long frame_ms = FRAME_DURATION;   // 1フレームあたりの単位時間(ms)
long merc = (long)millis();       // フレーム管理時計の時刻 Meridian Clock.
long curr = (long)millis();       // 現在時刻を取得
long curr_micro = (long)micros(); // 現在時刻を取得
int frame_count = 0;              // サイン計算用の変数
int frame_count_diff = 2;         // サインカーブ動作などのフレームカウントをいくつずつ進めるか
int frame_count_max = 360000;     // フレームカウントの最大値
int joypad_frame_count = 0;       // JOYPADのデータを読みに行くためのフレームカウント
short frame_sync_s = 0;           // フレーム毎に0-199をカウントし、送信用Meridm[88]の下位8ビットに格納
short frame_sync_r_expect = 0;    // フレーム毎に0-199をカウントし、受信値と比較
short frame_sync_r_resv = 0;      // 今フレームに受信したframe_sync_rを格納

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
float ROLL, PITCH, YAW, YAW_ZERO;
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
float yaw_center = 0;

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
// | 関数名　　:  setyaw()
// +----------------------------------------------------------------------
// | 機能     :  ヨー軸の原点リセット. IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// +----------------------------------------------------------------------
void setyaw()
{
    if (IMUAHRS_MOUNT == 1) // MPU6050
    {
        YAW_ZERO = ypr[0] * 180 / M_PI;
        s_spi_meridim.sval[0] = MSG_SIZE;
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055
    {
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
    else if (x < 3500) // 下限を設定
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
            Serial.println("IMU/AHRS DMP Initialization FAILED!");
        }
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055
    {
        // BNO055の初期設定
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

            // 加速度の値
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu_read[0] = (float)aa.x;
            mpu_read[1] = (float)aa.y;
            mpu_read[2] = (float)aa.z;

            // ジャイロの値
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            mpu_read[3] = (float)gyro.x;
            mpu_read[4] = (float)gyro.y;
            mpu_read[5] = (float)gyro.z;

            // 磁力センサの値
            mpu_read[6] = (float)mag.x;
            mpu_read[7] = (float)mag.y;
            mpu_read[8] = (float)mag.z;

            // 重力DMP推定値
            mpu_read[9] = gravity.x;
            mpu_read[10] = gravity.y;
            mpu_read[11] = gravity.z;

            // 相対方向DMP推定値
            mpu_read[12] = ypr[2] * 180 / M_PI;              // DMP_ROLL推定値
            mpu_read[13] = ypr[1] * 180 / M_PI;              // DMP_PITCH推定値
            mpu_read[14] = (ypr[0] * 180 / M_PI) - YAW_ZERO; // DMP_YAW推定値

            // 温度
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

        /*
          // センサフュージョンの方向推定値のクオータニオン
          imu::Quaternion quat = bno.getQuat();
          Serial.print("qW: "); Serial.print(quat.w(), 4);
          Serial.print(" qX: "); Serial.print(quat.x(), 4);
          Serial.print(" qY: "); Serial.print(quat.y(), 4);
          Serial.print(" qZ: "); Serial.println(quat.z(), 4);
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
        threads.delay(IMUAHRS_POLLING);
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
            buttonData = krs_R.getKrrButton();
            delayMicroseconds(2);
            if (buttonData != KRR_BUTTON_FALSE) // ボタンデータが受信できていたら
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
            }
            joypad_frame_count = 0;
        }
    }
}

//================================================================================================================
//---- セ ッ ト ア ッ プ -------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
    //-------------------------------------------------------------------------
    //---- サ ー ボ 設 定  -----------------------------------------------------
    //-------------------------------------------------------------------------
    /* 各サーボのマウントありなし（1:サーボあり、0:サーボなし） */
    idl_mt[0] = 1;  // 頭ヨー
    idl_mt[1] = 1;  // 左肩ピッチ
    idl_mt[2] = 1;  // 左肩ロール
    idl_mt[3] = 1;  // 左肘ヨー
    idl_mt[4] = 1;  // 左肘ピッチ
    idl_mt[5] = 1;  // 左股ヨー
    idl_mt[6] = 1;  // 左股ロール
    idl_mt[7] = 1;  // 左股ピッチ
    idl_mt[8] = 1;  // 左膝ピッチ
    idl_mt[9] = 1;  // 左足首ピッチ
    idl_mt[10] = 1; // 左足首ロール
    idl_mt[11] = 0; // 追加テスト用
    idl_mt[12] = 0; // 追加テスト用
    idl_mt[13] = 0; // 追加テスト用
    idl_mt[14] = 0; // 追加テスト用
    idr_mt[0] = 1;  // 腰ヨー
    idr_mt[1] = 1;  // 右肩ピッチ
    idr_mt[2] = 1;  // 右肩ロール
    idr_mt[3] = 1;  // 右肘ヨー
    idr_mt[4] = 1;  // 右肘ピッチ
    idr_mt[5] = 1;  // 右股ヨー
    idr_mt[6] = 1;  // 右股ロール
    idr_mt[7] = 1;  // 右股ピッチ
    idr_mt[8] = 1;  // 右膝ピッチ
    idr_mt[9] = 1;  // 右足首ピッチ
    idr_mt[10] = 1; // 右足首ロール
    idr_mt[11] = 0; // 追加テスト用
    idr_mt[12] = 0; // 追加テスト用
    idr_mt[13] = 0; // 追加テスト用
    idr_mt[14] = 0; // 追加テスト用

    /* 各サーボの内外回転プラマイ方向補正(1 or -1) */
    idl_cw[0] = 1;  // 頭ヨー
    idl_cw[1] = 1;  // 左肩ピッチ
    idl_cw[2] = 1;  // 左肩ロール
    idl_cw[3] = 1;  // 左肘ヨー
    idl_cw[4] = 1;  // 左肘ピッチ
    idl_cw[5] = 1;  // 左股ヨー
    idl_cw[6] = 1;  // 左股ロール
    idl_cw[7] = 1;  // 左股ピッチ
    idl_cw[8] = 1;  // 左膝ピッチ
    idl_cw[9] = 1;  // 左足首ピッチ
    idl_cw[10] = 1; // 左足首ロール
    idl_cw[11] = 1; // 追加テスト用
    idl_cw[12] = 1; // 追加テスト用
    idl_cw[13] = 1; // 追加テスト用
    idl_cw[14] = 1; // 追加テスト用
    idr_cw[0] = 1;  // 腰ヨー
    idr_cw[1] = 1;  // 右肩ピッチ
    idr_cw[2] = 1;  // 右肩ロール
    idr_cw[3] = 1;  // 右肘ヨー
    idr_cw[4] = 1;  // 右肘ピッチ
    idr_cw[5] = 1;  // 右股ヨー
    idr_cw[6] = 1;  // 右股ロール
    idr_cw[7] = 1;  // 右股ピッチ
    idr_cw[8] = 1;  // 右膝ピッチ
    idr_cw[9] = 1;  // 右足首ピッチ
    idr_cw[10] = 1; // 右足首ロール
    idr_cw[11] = 1; // 追加テスト用
    idr_cw[12] = 1; // 追加テスト用
    idr_cw[13] = 1; // 追加テスト用
    idr_cw[14] = 1; // 追加テスト用

    /* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */
    idl_trim[0] = 0;     // 頭ヨー       /* 直立状態になるよう、具体的な数値を入れて現物調整する */
    idl_trim[1] = -70;   // 左肩ピッチ
    idl_trim[2] = -2700; // 左肩ロール
    idl_trim[3] = 0;     // 左肘ヨー
    idl_trim[4] = 2666;  // 左肘ピッチ
    idl_trim[5] = 0;     // 左股ヨー
    idl_trim[6] = 0;     // 左股ロール
    idl_trim[7] = -40;   // 左股ピッチ
    idl_trim[8] = -1720; // 左膝ピッチ
    idl_trim[9] = -600;  // 左足首ピッチ
    idl_trim[10] = -20;  // 左足首ロール
    idl_trim[11] = 0;    // 追加テスト用
    idl_trim[12] = 0;    // 追加テスト用
    idl_trim[13] = 0;    // 追加テスト用
    idl_trim[14] = 0;    // 追加テスト用
    idr_trim[0] = 0;     // 腰ヨー
    idr_trim[1] = 0;     // 右肩ピッチ
    idr_trim[2] = -2650; // 右肩ロール
    idr_trim[3] = 0;     // 右肘ヨー
    idr_trim[4] = 2666;  // 右肘ピッチ
    idr_trim[5] = 0;     // 右股ヨー
    idr_trim[6] = 50;    // 右股ロール
    idr_trim[7] = -100;  // 右股ピッチ
    idr_trim[8] = -1700; // 右膝ピッチ
    idr_trim[9] = -600;  // 右足首ピッチ
    idr_trim[10] = -70;  // 右足首ロール
    idr_trim[11] = 0;    // 追加テスト用
    idr_trim[12] = 0;    // 追加テスト用
    idr_trim[13] = 0;    // 追加テスト用
    idr_trim[14] = 0;    // 追加テスト用

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
    Serial.print("Set I2C speed: ");
    Serial.println(I2C_CLOCK);
    Serial.print("Set SPI speed: ");
    Serial.println(SPI_CLOCK);
    Serial.print("Left side Servos mounted:  ");
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

    Serial.print("I2C IMU/AHRS Sensor mounted: ");
    if (IMUAHRS_MOUNT == 0)
    {
        Serial.print("None. ");
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
        Serial.print("  freq: ");
        Serial.println(IMUAHRS_FREQ);
    }

    /* サーボ用シリアル設定 */
    krs_L.begin(); // サーボモータの通信初期設定。Serial2
    krs_R.begin(); // サーボモータの通信初期設定。Serial3
    if (ICS3_MOUNT)
    {
        krs_3.begin(); // サーボモータの通信初期設定。Serial1
    }
    delay(100);

    /* I2C接続センサーの設定 */
    if (IMUAHRS_MOUNT == 1) //
    {
        setupIMUAHRS();
    }

    Serial.print("Controll Pad Receiver mounted: ");
    if (JOYPAD_MOUNT == 0)
    {
        Serial.println("None. ");
    }
    else if (JOYPAD_MOUNT == 1)
    {
        Serial.println("SBDBT ");
    }
    else if (JOYPAD_MOUNT == 2)
    {
        Serial.println("KRC-5FH ");
    }
    else if (JOYPAD_MOUNT == 3)
    {
        Serial.println("Merimote PICO ");
    }
    if (JOYPAD_MOUNT != 0)
    {
        Serial.print("  freq: ");
        Serial.println(JOYPAD_FRAME);
    }
    delay(100);

    // SDカードの初期化と読み書きテスト
    if (SD_MOUNT)
    {
        Serial.print("SD card check...");
        delay(100);
        if (!SD.begin(CHIPSELECT_SD))
        {
            Serial.println(" initialization FALIED!");
            delay(500);
        }
        else
        {
            Serial.println(" OK.");
        }

        if (SD_RWCHECK)
        {
            // open the file.
            myFile = SD.open("/tmp.txt", FILE_WRITE);
            delayMicroseconds(1); // SPI安定化検証用

            // if the file opened okay, write to it:
            if (myFile)
            {
                Serial.print("SD card r/w check...");
                // SD書き込みテスト用のランダムな4桁の数字を生成
                randomSeed(long(analogRead(A0) + analogRead(A1) * 2 + analogRead(A2) * 3 + analogRead(A3) * 100 + analogRead(A4) * 103 + analogRead(A5) * 105 + analogRead(A6) * 1000 + analogRead(A7) * 1001 + analogRead(A8) * 1007 + analogRead(A9) * 10000 + analogRead(A10) * 10001 + analogRead(A11) * 10005 + analogRead(A12) * 10007 + analogRead(A13) * 10009)); // 未接続ピンのノイズを利用
                int randNumber = random(1000, 9999);

                Serial.print(" write code ");
                Serial.print(randNumber);
                myFile.println(randNumber);
                delayMicroseconds(1); // SPI安定化検証用
                // close the file:
                myFile.close();
                Serial.print(" and");
                delayMicroseconds(10); // SPI安定化検証用
                // re-open the file for reading:
                myFile = SD.open("/tmp.txt");
                if (myFile)
                {
                    Serial.print(" read code ");
                    while (myFile.available())
                    {
                        Serial.write(myFile.read());
                    }
                    // close the file:
                    myFile.close();
                }
                SD.remove("/tmp.txt");
                // Serial.println("SD /tmp.txt file removed.");
                delay(10);
            }
            else
            {
                // if the file didn't open, print an error:
                Serial.println("Could not open SD test file.");
            }
        }
    }
    else
    {
        Serial.println("No SD reader/writer mounted.");
    }

    /* SPI通信用DMAの設定 */
    TsyDMASPI0.begin(SS, SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    /* 配列のリセット */
    memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    memset(idl_d, 0, 15);                            // 配列要素を0でリセット
    memset(idr_d, 0, 15);                            // 配列要素を0でリセット

    /* I2Cに接続したIMU/AHRSセンサの設定 */
    if (IMUAHRS_MOUNT == 1) // MPU6050の場合
    {
        /* MPU6050の割り込み設定 */
        MsTimer2::set(10, IMUAHRS_getYawPitchRoll); // MPUの情報を取得 10msごとにチェック
        MsTimer2::start();
    }
    else if (IMUAHRS_MOUNT == 3) // BNO055の場合
    {
        /* MPU6050の初期化 */
        if (!bno.begin())
        {
            Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
            // while (1)
            //   ;
        }
        else
        {
            Serial.println("BNO055 mounted.");
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

    /* 変数の設定 */
    YAW_ZERO = 0;
    s_spi_meridim.sval[0] = MSG_SIZE; // (マスターコマンド）

    /* 起動時のディレイ用mercちょい足し */
    merc = merc + 3500;
    Serial.println("Ready. ");
    Serial.println();
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
            r_spi_meridim.bval[MSG_ERR_u] &= B11011111; // エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオフ
        }
        else
        {
            r_spi_meridim.bval[MSG_ERR_u] |= B00100000; // エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオン
        }

        // @[1-3-1] シーケンシャルカウンタ予想値の生成
        frame_sync_r_expect++;           // フレームカウント予想値を加算
        if (frame_sync_r_expect > 29999) // 予想値が29,999以上ならカウントを-30000に戻す
        {
            frame_sync_r_expect = -30000;
        }

        // @[1-3-2] シーケンシャルカウンタチェック
        if (frame_sync_r_expect == r_spi_meridim.sval[1]) // 受信シーケンシャルカウンタの値が予想通りなら,
        {
            r_spi_meridim.bval[MSG_ERR_u] &= 0b11111101; // Meridim[MSG_ERR] 9番ビット:Teensy受信のスキップ検出をサゲる.
        }
        else // 受信シーケンシャルカウンタの値が予想と違ったら,
        {
            frame_sync_r_expect = r_spi_meridim.sval[1]; // 現在の受信値を予想結果としてキープ
            r_spi_meridim.bval[MSG_ERR_u] |= 0b00000010; // Meridim[MSG_ERR] 9番ビット:Teensy受信のスキップ検出をアゲる.
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
    else if (JOYPAD_MOUNT == 2)
    { // KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
        joypad_read();
        r_spi_meridim.sval[15] |= pad_btn;
        s_spi_meridim.sval[15] |= pad_btn;
    }
    else
    {
        pad_btn = r_spi_meridim.sval[15]; // をセットする
    }

    //////// < 7 > Teensy 内 部 で 位 置 制 御 す る 場 合 の 処 理 /////////////////////////

    // @[7-1] マスターコマンドの判定によりこの工程の実行orスキップを分岐(デフォルトはMeridim配列数である90)

    // コマンド[90]: サーボオン 通常動作

    // コマンド[0]: 全サーボ脱力

    // コマンド[1]: サーボオン 通常動作

    // コマンド[2]: IMU/AHRSのヨー軸リセット
    if (r_spi_meridim.sval[0] == UPDATE_YAW_CENTER)
    {
        setyaw();
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
    // Serial.println(r_spi_meridim.sval[15]);

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
    {                                   // 接続したサーボの数だけ繰り返す。最大は15
        idl_d[i] = 0;
        if (idl_mt[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
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
            if (r_spi_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
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
    // s_spi_meridim.sval[0] = MSG_SIZE ;//デフォルトのマスターコマンドは配列数

    // @[9-2] 移動時間を配列に格納
    // s_spi_meridim.sval[1] = 10 ;//(移動時間）

    // @[9-3] センサー値を配列に格納
    if (IMUAHRS_MOUNT == 1)
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
    if (frame_sync_s > 29999) // 予想値が29,999以上ならカウントを-30000に戻す
    {
        frame_sync_s = -30000;
    }
    s_spi_meridim.sval[1] = frame_sync_s;

    // @[9-6] カスタムデータを配列格納

    // @[9-7] チェックサムを計算
    s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);

    // @[9-8] 送信データのSPIバッファへのバイト型書き込み
    for (int i = 0; i < MSG_BUFF; i++)
    {
        s_spi_meridim_dma.bval[i] = s_spi_meridim.bval[i];
    }

    //////// < 11 > フ レ ー ム 終 端 処 理 ///////////////////////////////////////////////

    // @[11-1] この時点で１フレーム内に処理が収まっていない時の処理
    curr = (long)millis(); // 現在時刻を更新
    if (curr > merc)
    {                              // 現在時刻がフレーム管理時計を超えていたらアラートを出す
        Serial.print("* delay: "); // シリアルに遅延msを表示
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
}
