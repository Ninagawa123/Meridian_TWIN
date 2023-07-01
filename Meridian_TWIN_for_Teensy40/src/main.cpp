#define VERSION "Meridian_TWIN_for_Teensy_2023.07.01" // バージョン表示

// Meridian_TWIN_for_Teensy_20220617 By Izumi Ninagawa
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト　20230420版
// 220723 内部計算時に degree*100 を単位として使用するように変更
// 220723 センサーの関数を集約
// 220723 サーボオン時にリモコン左十字キー入力で首を左右に振る動作サンプル入り
// 220730 PCからのリモコン受信が有効となるように調整
// 220828 サーボからの受信が-1(タイムアウト)の時、前に取得した情報を使用する（データ飛びや表示のブレを防止）
// 220828 MOUNT_SERVO_NUM_L, MOUNT_SERVO_NUM_R に左右の接続サーボ数の登録を設定
// 220828 左右の接続サーボ数の多い方をservo_num_maxとし、サーボ送信命令も左右交互に実行するよう改定
// 230420 起動時にSDカードの動作チェックを行えるようにした。
// 230420 起動時のメッセージ表記の順番やテキストを若干変更。
// 230430 内部計算時の単位をdegree*100からdegreeに変更
// 230430 いくつかの基本的な関数をMeridianのライブラリに移動
// 230430 サーボの直立ポーズトリム値をdegreeに変更
// 230430 フローの見通しをよくする目的でsetup()やmain()関数内の処理の多くをローカル関数化
// 230430 変数名 idl_d,idr_dをidl_tgt,idr_tgtにした.
// 230430 変数名 idl_mt,idl_mtをidl_mount,idl_mountにした.
// 230617 ライブラリをESP32側と統合.

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* コンフィグファイルの読み込み */
#include "config.h"

/* ヘッダファイルの読み込み */
#include "main.h"

/* ライブラリ導入 */
#include <Meridian.h>                   // Meridianのライブラリ導入
MERIDIANFLOW::Meridian mrd;             // ライブラリのクラスを mrdという名前でインスタンス化
#include <Wire.h>                       // I2Cのライブラリ導入
#include <SPI.h>                        // SDカード用のライブラリ導入
#include <SD.h>                         // SDカード用のライブラリ導入
#include <TsyDMASPI.h>                  // SPI Master用のライブラリを導入
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050のライブラリ導入
#include <IcsHardSerialClass.h>         // ICSサーボのライブラリ導入
#include <MsTimer2.h>                   // タイマーのライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ
#include <TeensyThreads.h>              // マルチスレッド用のライブラリ

/* ICSサーボのインスタンス設定 */
IcsHardSerialClass krs_L(&Serial2, PIN_EN_L, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass krs_R(&Serial3, PIN_EN_R, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass krs_3(&Serial1, PIN_EN_3, ICS_BAUDRATE, ICS_TIMEOUT); // 3系もICSの場合

/* Meridim配列用の共用体の設定 */
typedef union // 共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
    short sval[MSG_SIZE + 4];           // short型で90個の配列データを持つ
    unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
    uint8_t bval[MSG_BUFF + 4];         // 1バイト単位で180個の配列データを持つ
} UnionData;
UnionData s_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_spi_meridim;     // Meridim配列データ(short型、センサや角度は100倍値)
UnionData s_spi_meridim_dma; // SPI送信用の共用体のインスタンスを作成
UnionData r_spi_meridim_dma; // SPI受信用の共用体のインスタンスを作成

/* システム用 */
// const int MSG_BUFF = MSG_SIZE * 2;                                                     // Meridim配列の長さ（byte換算）
const int MSG_ERR = MSG_SIZE - 2;                                                      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1;                                                 // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;                                                     // エラーフラグの格納場所（下位8ビット）
int k;                                                                                 // 各サーボの計算用変数
File myFile;                                                                           // SDカード用
int servo_num_max = max(max(MOUNT_SERVO_NUM_L, MOUNT_SERVO_NUM_R), MOUNT_SERVO_NUM_3); // サーボ送受信のループ処理数（L系R系で多い方）

/* フラグ用変数 */
bool flag_imuahrs_available = true; // メインが結果値を読み取る瞬間、サブスレッドによる書き込みをウェイト

/* タイマー管理用の変数 */
long frame_ms = FRAME_DURATION;  // 1フレームあたりの単位時間(ms)
long mrd_t_mil = (long)millis(); // フレーム管理時計の時刻 Meridian Clock.
long now_t_mil = (long)millis(); // 現在時刻を取得
long now_t_mic = (long)micros(); // 現在時刻を取得
int frame_count = 0;             // サイン計算用の変数
int frame_count_diff = 2;        // サインカーブ動作などのフレームカウントをいくつずつ進めるか
int frame_count_max = 360000;    // フレームカウントの最大値
int joypad_polling_count = 0;    // JOYPADのデータを読みに行くためのフレームカウント
int mrd_seq_s_increment = 0;     // フレーム毎に0-59999をカウントし、送信
int mrd_seq_r_expect = 0;        // フレーム毎に0-59999をカウントし、受信値と比較

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
unsigned short pad_stick_R = 0;
int pad_stick_R_x = 0;
int pad_stick_R_y = 0;
unsigned short pad_stick_L = 0;
int pad_stick_L_x = 0;
int pad_stick_L_y = 0;
unsigned short pad_stick_L2R2V = 0;
int pad_R2_val = 0;
int pad_L2_val = 0;

/* MPU6050のアドレス、レジスタ設定値 */
MPU6050 mpu6050;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
float imuahrs_yaw_origin;
float mpu_read[16];                                                     // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
float mpu_zeros[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // リセット用
float mpu_ave_data[16];                                                 // 上記の移動平均値を入れる
float mpu_result[16];                                                   // 加工後の最新のmpuデータ（二次データ）
float mpu_stock_data[IMUAHRS_STOCK][16];                                // 上記の移動平均値計算用のデータストック
int mpu_stock_count = 0;                                                // 上記の移動平均値計算用のデータストックを輪番させる時の変数
VectorInt16 aa;                                                         // [x, y, z]            加速度センサの測定値
VectorInt16 gyro;                                                       // [x, y, z]            角速度センサの測定値
VectorInt16 mag;                                                        // [x, y, z]            磁力センサの測定値
long temperature;                                                       // センサの温度測定値

/* BNO055用変数 */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float yaw_center = 0;

/* 各サーボのマウントありなし */
int idl_mount[15] = {IDL_MT0, IDL_MT1, IDL_MT2, IDL_MT3, IDL_MT4, IDL_MT5, IDL_MT6, IDL_MT7, IDL_MT8, IDL_MT9, IDL_MT10, IDL_MT11, IDL_MT12, IDL_MT13, IDL_MT14}; // L系統
int idr_mount[15] = {IDR_MT0, IDR_MT1, IDR_MT2, IDR_MT3, IDR_MT4, IDR_MT5, IDR_MT6, IDR_MT7, IDR_MT8, IDR_MT9, IDR_MT10, IDR_MT11, IDR_MT12, IDR_MT13, IDR_MT14}; // R系統
int id3_mount[15] = {ID3_MT0, ID3_MT1, ID3_MT2, ID3_MT3, ID3_MT4, ID3_MT5, ID3_MT6, ID3_MT7, ID3_MT8, ID3_MT9, ID3_MT10, ID3_MT11, ID3_MT12, ID3_MT13, ID3_MT14}; // 3系統
/* 各サーボのマウント(config.hで設定) */

/* 各サーボの正逆方向補正用配列 */
int idl_cw[15] = {IDL_CW0, IDL_CW1, IDL_CW2, IDL_CW3, IDL_CW4, IDL_CW5, IDL_CW6, IDL_CW7, IDL_CW8, IDL_CW9, IDL_CW10, IDL_CW11, IDL_CW12, IDL_CW13, IDL_CW14}; // L系統
int idr_cw[15] = {IDR_CW0, IDR_CW1, IDR_CW2, IDR_CW3, IDR_CW4, IDR_CW5, IDR_CW6, IDR_CW7, IDR_CW8, IDR_CW9, IDR_CW10, IDR_CW11, IDR_CW12, IDR_CW13, IDR_CW14}; // R系統
int id3_cw[15] = {ID3_CW0, ID3_CW1, ID3_CW2, ID3_CW3, ID3_CW4, ID3_CW5, ID3_CW6, ID3_CW7, ID3_CW8, ID3_CW9, ID3_CW10, ID3_CW11, ID3_CW12, ID3_CW13, ID3_CW14}; // R系統

/* 各サーボの直立ポーズトリム値 */
float idl_trim[15] = {IDL_TRIM0, IDL_TRIM1, IDL_TRIM2, IDL_TRIM3, IDL_TRIM4, IDL_TRIM5, IDL_TRIM6, IDL_TRIM7, IDL_TRIM8, IDL_TRIM9, IDL_TRIM10, IDL_TRIM11, IDL_TRIM12, IDL_TRIM13, IDL_TRIM14}; // L系統
float idr_trim[15] = {IDR_TRIM0, IDR_TRIM1, IDR_TRIM2, IDR_TRIM3, IDR_TRIM4, IDR_TRIM5, IDR_TRIM6, IDR_TRIM7, IDR_TRIM8, IDR_TRIM9, IDR_TRIM10, IDR_TRIM11, IDR_TRIM12, IDR_TRIM13, IDR_TRIM14}; // R系統
float id3_trim[15] = {ID3_TRIM0, ID3_TRIM1, ID3_TRIM2, ID3_TRIM3, ID3_TRIM4, ID3_TRIM5, ID3_TRIM6, ID3_TRIM7, ID3_TRIM8, ID3_TRIM9, ID3_TRIM10, ID3_TRIM11, ID3_TRIM12, ID3_TRIM13, ID3_TRIM14}; // R系統

/* 各サーボのポジション値(degree) */
float idl_tgt[15] = {0};      // L系統の目標値
float idr_tgt[15] = {0};      // R系統の目標値
float id3_tgt[15] = {0};      // R系統の目標値
float idl_tgt_past[15] = {0}; // L系統の前回の値
float idr_tgt_past[15] = {0}; // R系統の前回の値
float id3_tgt_past[15] = {0}; // R系統の前回の値

/* サーボのエラーカウンタ配列.*/
int idl_err[15] = {0}; // 15要素
int idr_err[15] = {0}; // 15要素
int id3_err[15] = {0}; // 15要素

//================================================================================================================
//---- セ ッ ト ア ッ プ -------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
    //-------------------------------------------------------------------------
    //---- 起　動　時 設 定 -----------------------------------------------------
    //-------------------------------------------------------------------------

    /* 入出力ピンのモード設定 */
    pinMode(PIN_ERR_LED, OUTPUT); // 通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

    /* シリアル設定 */
    Serial.begin(SERIAL_PC_BPS); // シリアルモニター表示
    delay(100);

    /* 起動時のインフォメーション表示表示(シリアルモニタ) */
    mrd.print_tsy_hello(VERSION, SPI_SPEED, I2C_SPEED);

    /* マウント設定されたサーボのIDを表示 */
    mrd.print_servo_mounts(idl_mount, idr_mount, id3_mount);

    /* IMU/AHRSタイプの表示 */
    mrd.print_imuahrs(MOUNT_IMUAHRS, IMUAHRS_POLLING);

    /* サーボ用シリアル設定 */
    krs_L.begin(); // サーボモータの通信初期設定。Serial2
    krs_R.begin(); // サーボモータの通信初期設定。Serial3
    if (MOUNT_ICS3)
    {
        krs_3.begin(); // サーボモータの通信初期設定。Serial1
    }
    delay(100);

    /* I2C接続センサーの設定 */
    if (MOUNT_IMUAHRS == 1) // MPU6050
    {
        setupIMUAHRS();
    }

    /* コントロールパッドの種類を表示 */
    mrd.print_controlpad(MOUNT_JOYPAD, JOYPAD_POLLING);
    delay(100);

    /* SDカードの初期化と読み書きテスト */
    check_sd();

    /* SPI通信用DMAの設定 */
    TsyDMASPI0.begin(SS, SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));

    /* 配列のリセット */
    memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット

    /* I2Cに接続したIMU/AHRSセンサをスタート */
    imuahrs_start();

    /* 変数の設定 */
    imuahrs_yaw_origin = 0;
    s_spi_meridim.sval[0] = MSG_SIZE; // (マスターコマンド）

    /* 起動時のディレイ用mercちょい足し */
    mrd_t_mil = mrd_t_mil + 3500;
    Serial.println("-) Meridian TWIN system on side Teensy now flows. (-");
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

    //////// < 1 > E S P 3 2 と の S P I に よ る 送 受 信 処 理  /////////////////////////

    // @[1-1] ESP32とのSPI送受信の実行
    if (MOUNT_ESP32)
    {
        TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

        // [1-2] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
        if (mrd.cksm_rslt(r_spi_meridim_dma.sval, MSG_SIZE))
        { // チェックサムがOKならバッファから受信配列に転記
            for (int i = 0; i < MSG_SIZE; i++)
            {
                r_spi_meridim.sval[i] = r_spi_meridim_dma.sval[i];
            }
            r_spi_meridim.bval[MSG_ERR_u] &= B11011111; // [MSG_ERR] 13番ビット[TeensyのESPからのSPI受信エラー検出]をサゲる.
        }
        else // チェックサムがNGならバッファから転記せず前回のデータを使用する
        {
            r_spi_meridim.bval[MSG_ERR_u] |= B00100000; // [MSG_ERR] 13番ビット[TeensyのESPからのSPI受信エラー検出]をアゲる.
        }

        // @[1-3] シーケンス番号チェック
        mrd_seq_r_expect = mrd.seq_predict_num(mrd_seq_r_expect); // シーケンス番号予想値の生成
        if (mrd.seq_compare_nums(mrd_seq_r_expect, int(r_spi_meridim.usval[MRD_SEQENTIAL])))
        {
            r_spi_meridim.bval[MSG_ERR_u] &= 0b11111101; // [MSG_ERR] 9番ビット[Teensy受信のスキップ検出]をサゲる.
        }
        else // 受信シーケンシャルカウンタの値が予想と違ったら
        {
            mrd_seq_r_expect = int(r_spi_meridim.usval[MRD_SEQENTIAL]); // 現在の受信値を予想結果としてキープ
            r_spi_meridim.bval[MSG_ERR_u] |= 0b00000010;                // Meridim[MSG_ERR] 9番ビット[Teensy受信のスキップ検出]をアゲる.
            err_tsy_skip++;
        }

        // [1-4] 通信エラー処理(エラーカウンタへの反映)
        countup_errors();

        //////// < 2 > シ リ ア ル モ ニ タ リ ン グ 表 示 処 理  //////////////////////////////
        // [2-1] 全経路のエラー数の表示
        if (monitor_all_error)
        {
            print_error_monitor();
        }
    }

    //////// < 3 > 積 み 残 し 処 理  ////////////////////////////////////////////////////
    // [3-1] 積み残しがあればここで処理

    //////// < 4 > 受 信 S P I デ ー タ を 送 信 S P I デ ー タ に 転 記 ////////////////////
    // [4-1] 受信データを送信データに転記
    memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MSG_BUFF + 4);

    //////// < 5 > セ ン サ ー 類 読 み 取 り /////////////////////////////////////////////
    // [5-1] IMU/AHRSについてはタイマー割り込みで別途処理

    //////// < 6 > コ ン ト ロ ー ラ の 読 み 取 り　///////////////////////////////////////
    // [6-1] コントローラの値を取得して送信データに格納する
    if (MOUNT_JOYPAD == 1)
    { // SBDBTが接続設定されていれば受信チェック（未実装）
        Serial.print("SBDBT connection has not been programmed yet.");
    }
    else if (MOUNT_JOYPAD == 2)
    { // KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
        pad_array.ui64val[0] = joypad_read(MOUNT_JOYPAD, pad_array.ui64val[0], JOYPAD_POLLING, JOYPAD_REFRESH);
        r_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.usval[0];
        s_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.usval[0];
    }
    else
    {
        pad_array.usval[0] = r_spi_meridim.sval[15]; // をセットする
    }
    if (MONITOR_JOYPAD)
    {
        mrd.monitor_joypad(pad_array.usval);
    }

    //////// < 7 > Teensy 内 部 で 位 置 制 御 す る 場 合 の 処 理 /////////////////////////
    // @[7-1] マスターコマンドの判定により工程の実行orスキップを分岐
    execute_MasterCommand();

    // @[7-2] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
    for (int i = 0; i < servo_num_max; i++)
    {
        idl_tgt_past[i] = idl_tgt[i];                       // 前回のdegreeをキープ
        idr_tgt_past[i] = idr_tgt[i];                       // 前回のdegreeをキープ
        idl_tgt[i] = r_spi_meridim.sval[i * 2 + 21] * 0.01; // 通常のdegreeが一旦入る
        idr_tgt[i] = r_spi_meridim.sval[i * 2 + 51] * 0.01; // 通常のdegreeが一旦入る
    }

    // @[7-3] Teensyによる次回動作の計算
    // リモコンの左十字キー左右で首を30度左右にふるサンプル
    if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 32)
    {
        idl_tgt[0] = -30.0; // -30度
    }
    else if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 128)
    {
        idl_tgt[0] = 30.0; // +30度
    }

    // @[7-4] センサーデータによる動作へのフィードバック加味

    // @[7-5] 移動時間の決定

    // @[7-6] Teensy内計算による次回動作をMeridim配列に書き込む

    //////// < 8 > サ ー ボ コ マ ン ド の 書 き 込 み /////////////////////////////////////

    // @[8-1] Meridim配列をサーボ命令に変更

    // @[8-2] サーボコマンドの配列に書き込み

    // @[8-3] サーボデータのICS送信および返り値を取得

    //////// < 9 > サ ー ボ 動 作 の 実 行 /////////////////////////////////////////////
    // @ [9-1] サーボ命令の実行およびサーボ角度戻り値の取得
    for (int i = 0; i < servo_num_max; i++) // ICS_L系統の処理
    {                                       // 接続したサーボの数だけ繰り返す。最大は15
        if (idl_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = krs_L.setPos(i, mrd.Deg2Krs(idl_tgt[i], idl_trim[i], idl_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(idl_tgt_past[i], idl_trim[i], idl_cw[i]);
                    idl_err[i]++;
                    if (idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
                        mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    idl_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
            {
                k = krs_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
                if (k == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(idl_tgt_past[i], idl_trim[i], idl_cw[i]);
                    idl_err[i]++;
                    if (idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
                        mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    idl_err[i] = 0;
                }
            }
            idl_tgt[i] = mrd.Krs2Deg(k, idl_trim[i], idl_cw[i]);
        }
        delayMicroseconds(2);

        if (idr_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = krs_R.setPos(i, mrd.Deg2Krs(idr_tgt[i], idr_trim[i], idr_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(idr_tgt_past[i], idr_trim[i], idr_cw[i]);
                    idr_err[i]++;
                    if (idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
                        mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    idr_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得
            {
                k = krs_R.setFree(i);
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(idr_tgt_past[i], idr_trim[i], idr_cw[i]);
                    idr_err[i]++;
                    if (idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
                        mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    idr_err[i] = 0;
                }
            }
            idr_tgt[i] = mrd.Krs2Deg(k, idr_trim[i], idr_cw[i]);
        }
        delayMicroseconds(2);
    }

    //////// < 10 > S P I 送 信 用 の Meridim 配 列 を 作 成 す る //////////////////////////
    // @[10-1] マスターコマンドを配列に格納
    s_spi_meridim.sval[0] = MSG_SIZE; // デフォルトのマスターコマンドは配列数

    // @[10-2] 移動時間を配列に格納
    // s_spi_meridim.sval[1] = 10 ;//(移動時間）

    // @[10-3] センサー値を配列に格納
    imuahrs_store();

    // @[10-4] サーボIDごとにの現在位置もしくは計算結果を配列に格納
    for (int i = 0; i < 15; i++)
    {
        s_spi_meridim.sval[i * 2 + 20] = 0;                             // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
        s_spi_meridim.sval[i * 2 + 21] = mrd.float2HfShort(idl_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
    }
    for (int i = 0; i < 15; i++)
    {
        s_spi_meridim.sval[i * 2 + 50] = 0;                             // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
        s_spi_meridim.sval[i * 2 + 51] = mrd.float2HfShort(idr_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
    }

    // @[10-5] Meridimのシーケンス番号をカウントアップして送信用に格納
    mrd_seq_s_increment = mrd.seq_increase_num(mrd_seq_s_increment);
    s_spi_meridim.usval[1] = mrd_seq_s_increment;

    // @[10-6] カスタムデータを配列格納

    // @[10-7] チェックサムを計算
    s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);

    // @[10-8] 送信データのSPIバッファへのバイト型書き込み
    for (int i = 0; i < MSG_BUFF; i++)
    {
        s_spi_meridim_dma.bval[i] = s_spi_meridim.bval[i];
    }

    //////// < 11 > フ レ ー ム 終 端 処 理 ///////////////////////////////////////////////
    // @[11-1] この時点で１フレーム内に処理が収まっていない時の処理
    now_t_mil = (long)millis(); // 現在時刻を更新
    if (now_t_mil > mrd_t_mil)
    {                              // 現在時刻がフレーム管理時計を超えていたらアラートを出す
        Serial.print("* delay: "); // シリアルに遅延msを表示
        Serial.println(now_t_mil - mrd_t_mil);
        digitalWrite(PIN_ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
    }
    else
    {
        digitalWrite(PIN_ERR_LED, LOW); // 処理が収まっていればLEDを消灯
    }

    // @[11-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
    now_t_mil = (long)millis();
    now_t_mic = (long)micros(); // 現在時刻を取得
    while (now_t_mil < mrd_t_mil)
    {
        now_t_mil = (long)millis();
    }

    // @[11-3]フレーム管理時計mercのカウントアップ
    mrd_t_mil = mrd_t_mil + frame_ms;             // フレーム管理時計を1フレーム分進める
    frame_count = frame_count + frame_count_diff; // サインカーブ等動作用のフレームカウントアップ
    if (frame_count > frame_count_max)            // カウンターが最大値ならゼロリセット
    {
        frame_count = 0;
    }
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Initialize sensors like MPU6050, BNO055, and others.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setupIMUAHRS()
{
    if (MOUNT_IMUAHRS == 1) // MPU6050
    {

        Wire.begin();
        Wire.setClock(I2C_SPEED); // 400kHz I2C clock. Comment this line if having compilation difficulties
        mpu6050.initialize();
        devStatus = mpu6050.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu6050.setXAccelOffset(-1745);
        mpu6050.setYAccelOffset(-1034);
        mpu6050.setZAccelOffset(966);
        mpu6050.setXGyroOffset(176);
        mpu6050.setYGyroOffset(-6);
        mpu6050.setZGyroOffset(-25);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
            mpu6050.CalibrateAccel(6);
            mpu6050.CalibrateGyro(6);
            mpu6050.setDMPEnabled(true);
            packetSize = mpu6050.dmpGetFIFOPacketSize();
        }
        else
        {
            Serial.print("IMU/AHRS DMP Initialization FAILED!");
        }
    }
    else if (MOUNT_IMUAHRS == 3) // BNO055
    {
        // BNO055の初期設定(未実装)
    }
    Serial.println();
}

/**
 * @brief Store values for MPU6050, BNO055, and other sensors.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void IMUAHRS_getYawPitchRoll()
{
    if (MOUNT_IMUAHRS == 1) // MPU6050
    {
        if (mpu6050.dmpGetCurrentFIFOPacket(fifoBuffer))
        { // Get new data
            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
            mpu6050.dmpGetGravity(&gravity, &q);
            mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // acceleration values
            mpu6050.dmpGetAccel(&aa, fifoBuffer);
            mpu_read[0] = (float)aa.x;
            mpu_read[1] = (float)aa.y;
            mpu_read[2] = (float)aa.z;

            // gyro values
            mpu6050.dmpGetGyro(&gyro, fifoBuffer);
            mpu_read[3] = (float)gyro.x;
            mpu_read[4] = (float)gyro.y;
            mpu_read[5] = (float)gyro.z;

            // magnetic field values
            mpu_read[6] = (float)mag.x;
            mpu_read[7] = (float)mag.y;
            mpu_read[8] = (float)mag.z;

            // Estimated gravity DMP value.
            mpu_read[9] = gravity.x;
            mpu_read[10] = gravity.y;
            mpu_read[11] = gravity.z;

            // Estimated heading value using DMP.
            mpu_read[12] = ypr[2] * 180 / M_PI;                        // Estimated DMP_ROLL
            mpu_read[13] = ypr[1] * 180 / M_PI;                        // Estimated DMP_PITCH
            mpu_read[14] = (ypr[0] * 180 / M_PI) - imuahrs_yaw_origin; // Estimated DMP_YAW

            // Temperature
            mpu_read[15] = 0; // Not implemented.

            if (flag_imuahrs_available)
            {
                memcpy(mpu_result, mpu_read, sizeof(float) * 16);
            }
        }
    }
    else if (MOUNT_IMUAHRS == 3) // BNO055
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
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh)
{
    if (mount_joypad == 2)
    { // KRR5FH(KRC-5FH)をICS_R系に接続している場合
        joypad_polling_count++;
        if (joypad_polling_count >= polling)
        {
            static bool isFirstCall = true; // 初回の呼び出しフラグ
            if (isFirstCall)
            {
                Serial.println("KRC-5FH successfully connected. ");
                isFirstCall = false; // 初回の呼び出しフラグをオフにする
            }
            unsigned short buttonData;
            unsigned short pad_btn_tmp = 0;

            buttonData = krs_R.getKrrButton();
            delayMicroseconds(2);
            if (buttonData != KRR_BUTTON_FALSE) // ボタンデータが受信できていたら
            {
                int button_1 = buttonData;

                if ((button_1 & 15) == 15)
                { // 左側十字ボタン全部押しなら start押下とみなす
                    pad_btn_tmp += 1;
                }
                else
                {
                    // 左側の十字ボタン
                    pad_btn_tmp += (button_1 & 1) * 16 + ((button_1 & 2) >> 1) * 64 + ((button_1 & 4) >> 2) * 32 + ((button_1 & 8) >> 3) * 128;
                }
                if ((button_1 & 368) == 368)
                    pad_btn_tmp += 8; // 右側十字ボタン全部押しなら select押下とみなす
                else
                {
                    // 右側十字ボタン
                    pad_btn_tmp += ((button_1 & 16) >> 4) * 4096 + ((button_1 & 32) >> 5) * 16384 + ((button_1 & 64) >> 6) * 8192 + ((button_1 & 256) >> 8) * 32768;
                }
                // L1,L2,R1,R2
                pad_btn_tmp += ((button_1 & 2048) >> 11) * 2048 + ((button_1 & 4096) >> 12) * 512 + ((button_1 & 512) >> 9) * 1024 + ((button_1 & 1024) >> 10) * 256;
            }
            /* 共用体用の64ビットの上位16ビット部をボタンデータとして書き換える */
            uint64_t updated_val;
            if (joypad_reflesh)
            {
                updated_val = (pre_val & 0xFFFFFFFFFFFF0000) | (static_cast<uint64_t>(pad_btn_tmp)); // 上位16ビット index[0]
            }
            else
            {
                updated_val = (pre_val) | (static_cast<uint64_t>(pad_btn_tmp));
            }
            // updated_val = (updated_val & 0x0000FFFFFFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 48); // 下位16ビット index[3]
            // updated_val = (updated_val & 0xFFFF0000FFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 32); // 上位33-48ビット index[2]
            // updated_val = (updated_val & 0xFFFFFFFF0000FFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 16); // 上位17-32ビット index[1]
            joypad_polling_count = 0;
            return updated_val;
        }
        else
        {
            return pre_val;
        }
    }
    else
    {
        return pre_val;
    }
}

/**
 * @brief Initializing and performing read/write tests for an SD card.
 *
 */
void check_sd()
{
    if (MOUNT_SD)
    {
        Serial.print("SD card check...");
        delay(100);
        if (!SD.begin(PIN_CHIPSELECT_SD))
        {
            Serial.println(" initialization FALIED!");
            delay(500);
        }
        else
        {
            Serial.println(" OK.");
        }

        if (CHECK_SD_RW)
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
}

/**
 * @brief Displaying the error detection count on Teensy's serial interface.
 *
 */
void print_error_monitor()
{
    Serial.print("[ERRs] esp>pc:");
    Serial.print(err_esp_pc);
    Serial.print(" pc>esp:");
    Serial.print(err_pc_esp);
    Serial.print(" esp>tsy:");
    Serial.print(err_esp_tsy);
    Serial.print(" tsy>esp:");
    Serial.print(err_esp_tsy);
    Serial.print(" tsySkip:");
    Serial.print(err_tsy_skip); //
    Serial.print(" espSkip:");
    Serial.print(err_esp_skip); //
    Serial.print(" pcSkip:");
    Serial.print(err_pc_skip); //
    Serial.print(" seq:");
    Serial.print(int(mrd_seq_r_expect)); //
    Serial.print(" [u]:");
    Serial.print(r_spi_meridim.bval[MSG_ERR_u], BIN);
    Serial.println();
}

/**
 * @brief Counting up the communication error detection count.
 *
 */
void countup_errors()
{
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
}

/**
 * @brief Starting the IMU sensor or AHRS sensor.
 *
 */
void imuahrs_start()
{
    if (MOUNT_IMUAHRS == 1) // MPU6050の場合
    {
        /* MPU6050の割り込み設定 */
        MsTimer2::set(IMUAHRS_POLLING, IMUAHRS_getYawPitchRoll); // MPUの情報を取得 10msごとにチェック
        MsTimer2::start();
    }
    else if (MOUNT_IMUAHRS == 3) // BNO055の場合
    {
        /* BNO055の初期化 */
        if (!bno.begin())
        {
            Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        }
        else
        {
            Serial.println("BNO055 mounted.");
            delay(50);
            bno.setExtCrystalUse(false);
            delay(10);
        }
        // データの取得はセンサー用スレッドで実行
        delay(10);
    }
    else
    {
        Serial.println("No IMU/AHRS sensor mounted.");
    }
}

/**
 * @brief Storing the values of the IMU sensor and AHRS sensor in an array.
 *
 */
void imuahrs_store()
{
    if (MOUNT_IMUAHRS == 1)
    {
        flag_imuahrs_available = false;
        s_spi_meridim.sval[2] = mrd.float2HfShort(mpu_result[0]);   // IMU/AHRS_acc_x
        s_spi_meridim.sval[3] = mrd.float2HfShort(mpu_result[1]);   // IMU/AHRS_acc_y
        s_spi_meridim.sval[4] = mrd.float2HfShort(mpu_result[2]);   // IMU/AHRS_acc_z
        s_spi_meridim.sval[5] = mrd.float2HfShort(mpu_result[3]);   // IMU/AHRS_gyro_x
        s_spi_meridim.sval[6] = mrd.float2HfShort(mpu_result[4]);   // IMU/AHRS_gyro_y
        s_spi_meridim.sval[7] = mrd.float2HfShort(mpu_result[5]);   // IMU/AHRS_gyro_z
        s_spi_meridim.sval[8] = mrd.float2HfShort(mpu_result[6]);   // IMU/AHRS_mag_x
        s_spi_meridim.sval[9] = mrd.float2HfShort(mpu_result[7]);   // IMU/AHRS_mag_y
        s_spi_meridim.sval[10] = mrd.float2HfShort(mpu_result[8]);  // IMU/AHRS_mag_z
        s_spi_meridim.sval[11] = mrd.float2HfShort(mpu_result[15]); // temperature
        s_spi_meridim.sval[12] = mrd.float2HfShort(mpu_result[12]); // DMP_ROLL推定値
        s_spi_meridim.sval[13] = mrd.float2HfShort(mpu_result[13]); // DMP_PITCH推定値
        s_spi_meridim.sval[14] = mrd.float2HfShort(mpu_result[14]); // DMP_YAW推定値
        flag_imuahrs_available = true;
    }
}

//================================================================================================================
//---- Command processing ----------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Execute mastercommands.
 *
 */
void execute_MasterCommand()
{
    // コマンド[90]: サーボオン 通常動作

    // コマンド[0]: 全サーボ脱力
    if (r_spi_meridim.sval[MRD_MASTER] == 0)
    {
        servo_all_off();
    }

    // コマンド[1]: サーボオン 通常動作

    // コマンド[2]: IMU/AHRSのヨー軸リセット
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_UPDATE_YAW_CENTER)
    {
        setyaw();
    }

    // コマンド[3]: トリムモード（既存のものは廃止し、検討中）

    // コマンド[4]: 通信エラーサーボIDのクリア
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_CLEAR_SERVO_ERROR_ID)
    {
        s_spi_meridim.bval[MSG_ERR_l] = 0;
    }
}

/**
 * @brief Resetting the origin of the yaw axis.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setyaw()
{
    if (MOUNT_IMUAHRS == 1) // MPU6050
    {
        imuahrs_yaw_origin = ypr[0] * 180 / M_PI;
        s_spi_meridim.sval[0] = MSG_SIZE;
    }
    else if (MOUNT_IMUAHRS == 3) // BNO055
    {
    }
}

/**
 * @brief Powering off all servos.
 *
 */
void servo_all_off()
{
    for (int h = 0; h < 5; h++)
    {
        for (int i = 0; i < 15; i++)
        {
            if (idl_mount[i] == 1)
            {
                krs_L.setFree(i);
            }
            if (idr_mount[i] == 1)
            {
                krs_R.setFree(i);
            }
            delayMicroseconds(2);
        }
    }
    delay(100);
    Serial.println("All servos off.");
}
