#define VERSION "Meridian_TWIN_for_Teensy_2024.04.30" // バージョン表示

// Meridian_TWIN_for_Teensy_20240422 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト
// 20240414 第二次リファクタリング
// 20240414 SPI通信を1フレームあたり2回,往復実行するように修正.
// 20240414 システム用の変数を構造化
// 20240422 変数名修正

//================================================================================================================
//---- 初期設定  --------------------------------------------------------------------------------------------------
//================================================================================================================

/* コンフィグファイルの読み込み */
#include "config.h"

/* ヘッダファイルの読み込み */
#include "main.h"
// #include "servomanager.h"

/* ライブラリ導入 */
#include <TeensyThreads.h>
#include <Meridian.h>                   // Meridianのライブラリ導入
MERIDIANFLOW::Meridian mrd;             // ライブラリのクラスを mrdという名前でインスタンス化
#include <Wire.h>                       // I2C用
#include <SPI.h>                        // SDカードやSPI通信用
#include <SD.h>                         // SDカード用
#include <TsyDMASPI.h>                  // SPI通信Master用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <IcsHardSerialClass.h>         // ICSサーボ用
// #include <MsTimer2.h>                   // タイマー用
#include <Adafruit_BNO055.h> // 9軸センサBNO055用
#include <IntervalTimer.h>

/* システム用 */
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
File myFile;                           // SDカード用
MrdCheck mrdck;                        // システム管理用
MrdFlags flag;                         // フラグ用
MrdTimer tmr;                          // タイマー管理用
MrdErr err;                            // エラーカウント用
MrdMonitor monitor;                    // モニタリング設定用
ServoId sv;                            // サーボデータ用
UnionPad pad_array = {0};              // リモコン値格納用の配列
UnionPadWire pad_i2c = {0};
PadValue pad; // リモコンの入力結果
short pad_btn_past = 0;
short pad_cksm_past = 0;

AhrsValue ahrs;                                         // 6軸or9軸センサーの値
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055用変数
// IntervalTimer ahrsTimer;
IntervalTimer wireTimer0;
IntervalTimer wireTimer1;

/* Meridim配列用の共用体の設定 */
UnionData s_spi_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
UnionData r_spi_meridim;       // Meridim配列データ受信用
UnionData s_spi_meridim_dma;   // SPI送信DMA用
UnionData r_spi_meridim_dma;   // SPI受信DMA用
UnionData s_spi_meridim_dummy; // SPI送信ダミー用

/* ICSサーボのインスタンス設定 */
IcsHardSerialClass krs_L(&Serial2, PIN_EN_L, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass krs_R(&Serial3, PIN_EN_R, ICS_BAUDRATE, ICS_TIMEOUT);
IcsHardSerialClass krs_3(&Serial1, PIN_EN_3, ICS_BAUDRATE, ICS_TIMEOUT); // 3系もICSの場合

//================================================================================================================
//---- SET UP ----------------------------------------------------------------------------------------------------
//================================================================================================================

void setup()
{
    // std::thread th1(thread_wire, 1);
    // th1.detach();

    /* 入出力ピンのモード設定 */
    pinMode(PIN_ERR_LED, OUTPUT); // 通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

    /* 配列のリセット */
    memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    s_spi_meridim_dummy.sval[0] = MCMD_DUMMY_DATA;   // ダミーデータの設定
    s_spi_meridim.sval[0] = MSG_SIZE;                // (マスターコマンド）

    /* SPI通信用DMAの設定 */
    TsyDMASPI0.begin(SS, SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));

    /* シリアル設定 */
    Serial.begin(SERIAL_PC_BPS); // シリアルモニター表示
    delay(100);

    /* 起動時のシリアルモニタ情報表示 */
    mrd.print_tsy_hello(VERSION, SPI_SPEED, I2C_SPEED);               // 規定文
    mrd.print_servo_mounts(sv.idl_mount, sv.idr_mount, sv.id3_mount); // マウントされたサーボIDの表示
    mrd.print_imuahrs(MOUNT_IMUAHRS, IMUAHRS_POLLING);                // 6軸9軸センサのタイプ

    /* サーボ用UART設定 */
    krs_L.begin(); // サーボモータの通信初期設定. Serial2
    krs_R.begin(); // サーボモータの通信初期設定. Serial3
    if (MOUNT_ICS3)
    {
        krs_3.begin(); // サーボモータの通信初期設定. Serial1
    }
    delay(100);

    /* コントロールパッドの種類を表示 */
    mrd.print_controlpad(MOUNT_JOYPAD, JOYPAD_POLLING);
    delay(100);

    /* SDカードの初期化と読み書きテスト */
    check_sd();

    /* I2Cに接続したIMU/AHRSセンサをスタート */
    setupWire();

    if (flag.wire0_activate)
    {
        wireTimer0.begin(wire_manager0, 10000); // インターバルはマイクロ秒指定
                                                // wireTimer.priority(90);
    }
    if (flag.wire1_activate)
    {
        wireTimer1.begin(wire_manager1, 10000); // インターバルはマイクロ秒指定
                                                // wireTimer.priority(180);
    }

    /* 起動時のディレイ用tmr.mrd_milちょい足し */
    tmr.mrd_mil += 5000;
    Serial.println("-) Meridian TWIN system on side Teensy now flows. (-");
}

//================================================================================================================
//---- MAIN LOOP -------------------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{
    //------------------------------------------------------------------------------------
    //---- [ 1 ] ESP32とのSPIよる送受信処理 -------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[1]", MONITOR_FLOW); // 動作チェック用シリアル表示

    // @[1-1] ESP32とのSPI送受信の実行
    if (MOUNT_ESP32)
    {
        // SPIのデータを受信。送信はダミーデータ。
        TsyDMASPI0.transfer(s_spi_meridim_dummy.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

        // [1-2] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
        if (mrd.cksm_rslt(r_spi_meridim_dma.sval, MSG_SIZE))
        { // チェックサムがOKならバッファから受信配列に転記
            memcpy(r_spi_meridim.bval, r_spi_meridim_dma.bval, MSG_BUFF);
            r_spi_meridim.bval[MSG_ERR_u] &= B11011111; // [MSG_ERR] 13番ビット[TeensyのESPからのSPI受信エラー検出]をサゲる.
        }
        else // チェックサムがNGならバッファから転記せず前回のデータを使用する
        {
            r_spi_meridim.bval[MSG_ERR_u] |= B00100000; // [MSG_ERR] 13番ビット[TeensyのESPからのSPI受信エラー検出]をアゲる.
        }

        // @[1-3] シーケンス番号チェック
        mrdck.seq_r_expect = mrd.seq_predict_num(mrdck.seq_r_expect); // シーケンス番号予想値の生成
        if (mrd.seq_compare_nums(mrdck.seq_r_expect, int(r_spi_meridim.usval[MRD_SEQENTIAL])))
        {
            r_spi_meridim.bval[MSG_ERR_u] &= 0b11111101; // [MSG_ERR] 9番ビット[Teensy受信のスキップ検出]をサゲる.
            flag.mrd_spi_rcvd = true;                    // SPI受信フラグを下げる
        }
        else // シーケンス番号の値が予想と違ったら
        {
            mrdck.seq_r_expect = int(r_spi_meridim.usval[MRD_SEQENTIAL]); // 現在の受信値を予想結果としてキープ
            r_spi_meridim.bval[MSG_ERR_u] |= 0b00000010;                  // Meridim[MSG_ERR] 9番ビット[Teensy受信のスキップ検出]をアゲる.
            err.tsy_skip++;
        }

        // @[1-4] 通信エラー処理(エラーカウンタへの反映)
        countup_errors();

        //------------------------------------------------------------------------------------
        //---- [ 2 ] シリアルモニタリング表示処理 -------------------------------------------------
        //------------------------------------------------------------------------------------
        mrd.monitor_check_flow("[2]", MONITOR_FLOW); // 動作チェック用シリアル表示
        // @[2-1] 全経路のエラー数の表示
        if (monitor.all_error)
        {
            print_error_monitor();
        }
    }

    //------------------------------------------------------------------------------------
    //---- [ 3 ] 積み残し処理 --------------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[3]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[3-1] 積み残しがあればここで処理

    //------------------------------------------------------------------------------------
    //---- [ 4 ] 受信SPIデータを送信SPIデータに転記 -------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[4]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[4-1] 受信データを送信データに転記
    memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MSG_BUFF + 4);

    //------------------------------------------------------------------------------------
    //---- [ 5 ] MastarCommand group1 の処理 -------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[5]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[5-1] マスターコマンドの判定により工程の実行orスキップを分岐

    if (flag.mrd_spi_rcvd)
    {
        execute_MasterCommand_1();
    }

    // @[5-2]EEPROMやSDへの読み書き処理はおそらくここで実施.
    // execute_MasterCommand()関数内でフラグを立て, フラグによって分岐.

    //------------------------------------------------------------------------------------
    //---- [ 6 ] センサー類読み取り ---------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[6]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[6-1] IMU/AHRSについてはタイマー割り込みで等で別途処理

    //------------------------------------------------------------------------------------
    //---- [ 7 ] コントローラの読み取り ------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[7]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[7-1] コントローラの値を取得して送信データに格納する
    if (MOUNT_JOYPAD == 1)
    { // SBDBTが接続設定されていれば受信チェック（未実装）
        Serial.print("SBDBT connection has not been programmed yet.");
    }
    else if (MOUNT_JOYPAD == 2)
    { // KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
        pad_array.ui64val[0] = joypad_read(MOUNT_JOYPAD, pad_array.ui64val[0], JOYPAD_POLLING, JOYPAD_REFRESH);
        r_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
        s_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
    }
    else if (MOUNT_JOYPAD == 3)
    {
        for (int i = 0; i < 4; i++)
        {
            r_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
            s_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
        }
        //r_spi_meridim.usval[MRD_CONTROL_L2R2ANALOG] = pad_array.usval[3];
        //s_spi_meridim.usval[MRD_CONTROL_L2R2ANALOG] = pad_array.usval[3];
    }
    else
    {
        pad_array.usval[0] = r_spi_meridim.sval[15]; // をセットする
    }

    if (MONITOR_JOYPAD)
    {
        mrd.monitor_joypad(pad_array.usval);
    }

    //------------------------------------------------------------------------------------
    //---- [ 8 ] MastarCommand group2 の処理 -------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[8]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[8-1] マスターコマンドの判定により工程の実行orスキップを分岐

    if (flag.mrd_spi_rcvd)
    {
        execute_MasterCommand_2();
    }

    //------------------------------------------------------------------------------------
    //---- [ 9 ] Teensy内部で位置制御する場合の処理 -------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[9]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[9-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
    for (int i = 0; i < sv.num_max; i++)
    {
        sv.idl_tgt_past[i] = sv.idl_tgt[i];                    // 前回のdegreeをキープ
        sv.idr_tgt_past[i] = sv.idr_tgt[i];                    // 前回のdegreeをキープ
        sv.idl_tgt[i] = r_spi_meridim.sval[i * 2 + 21] * 0.01; // 通常のdegreeが一旦入る
        sv.idr_tgt[i] = r_spi_meridim.sval[i * 2 + 51] * 0.01; // 通常のdegreeが一旦入る
    }

    // @[9-2] Teensyによる次回動作の計算
    // リモコンの左十字キー左右で首を30度左右にふるサンプル
    if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 32)
    {
        sv.idl_tgt[0] = -30.0; // -30度
    }
    else if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 128)
    {
        sv.idl_tgt[0] = 30.0; // +30度
    }

    // @[9-3] センサーデータによる動作へのフィードバック加味

    // @[9-4] 移動時間の決定

    // @[9-5] Teensy内計算による次回動作をMeridim配列に書き込む

    //------------------------------------------------------------------------------------
    //---- [ 10 ] サーボコマンドの書き込み ----------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[10]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[10-1] Meridim配列をサーボ命令に変更

    // @[10-2] サーボコマンドの配列に書き込み

    // @[10-3] サーボデータのICS送信および返り値を取得

    //------------------------------------------------------------------------------------
    //---- [ 11 ] サーボ動作の実行 -----------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[11]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @ [11-1] サーボ命令の実行およびサーボ角度戻り値の取得
    for (int i = 0; i < sv.num_max; i++) // ICS_L系統の処理
    {                                    // 接続したサーボの数だけ繰り返す. 最大は15
        int _k = 0;
        if (sv.idl_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                _k = krs_L.setPos(i, mrd.Deg2Krs(sv.idl_tgt[i], sv.idl_trim[i], sv.idl_cw[i]));
                if (_k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    _k = mrd.Deg2Krs(sv.idl_tgt_past[i], sv.idl_trim[i], sv.idl_cw[i]);
                    sv.idl_err[i]++;
                    if (sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
                        mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idl_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得. 手持ちの最大は15
            {
                _k = krs_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
                if (_k == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    _k = mrd.Deg2Krs(sv.idl_tgt_past[i], sv.idl_trim[i], sv.idl_cw[i]);
                    sv.idl_err[i]++;
                    if (sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
                        mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idl_err[i] = 0;
                }
            }
            sv.idl_tgt[i] = mrd.Krs2Deg(_k, sv.idl_trim[i], sv.idl_cw[i]);
        }
        delayMicroseconds(2);

        if (sv.idr_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                _k = krs_R.setPos(i, mrd.Deg2Krs(sv.idr_tgt[i], sv.idr_trim[i], sv.idr_cw[i]));
                if (_k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    _k = mrd.Deg2Krs(sv.idr_tgt_past[i], sv.idr_trim[i], sv.idr_cw[i]);
                    sv.idr_err[i]++;
                    if (sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
                        mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idr_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得
            {
                _k = krs_R.setFree(i);
                if (_k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    _k = mrd.Deg2Krs(sv.idr_tgt_past[i], sv.idr_trim[i], sv.idr_cw[i]);
                    sv.idr_err[i]++;
                    if (sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
                        mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idr_err[i] = 0;
                }
            }
            sv.idr_tgt[i] = mrd.Krs2Deg(_k, sv.idr_trim[i], sv.idr_cw[i]);
        }
        delayMicroseconds(2);
    }

    //------------------------------------------------------------------------------------
    //---- [ 12 ] SPI送信用のMeridim配列を作成する -------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[12]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[12-1] マスターコマンドを配列に格納
    s_spi_meridim.sval[0] = MSG_SIZE; // デフォルトのマスターコマンドは配列数

    // @[12-2] 移動時間を配列に格納
    // s_spi_meridim.sval[1] = 10 ;//(移動時間）

    // @[12-3] センサー値を配列に格納
    imuahrs_store(s_spi_meridim.sval, ahrs.result, MOUNT_IMUAHRS);

    // @[12-4] サーボIDごとにの現在位置もしくは計算結果を配列に格納
    for (int i = 0; i < 15; i++)
    {
        s_spi_meridim.sval[i * 2 + 20] = 0;                                // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
        s_spi_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.idl_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
    }
    for (int i = 0; i < 15; i++)
    {
        s_spi_meridim.sval[i * 2 + 50] = 0;                                // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
        s_spi_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.idr_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
    }

    // @[12-5] Meridimのシーケンス番号をカウントアップして送信用に格納
    mrdck.seq_s_increment = mrd.seq_increase_num(mrdck.seq_s_increment);
    s_spi_meridim.usval[1] = mrdck.seq_s_increment;

    // @[12-6] カスタムデータを配列格納

    // @[12-7] チェックサムを計算
    s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);

    // @[12-8] 送信データのSPIバッファへのバイト型書き込み
    memcpy(s_spi_meridim_dma.bval, s_spi_meridim.bval, MSG_BUFF);

    //------------------------------------------------------------------------------------
    //---- [ 13 ] SPI送信 -----------------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[13]", MONITOR_FLOW); // 動作チェック用シリアル表示
    TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

    //------------------------------------------------------------------------------------
    //---- [ 14 ] フレーム終端処理 ----------------------------------------------------------
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[14]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[14-2] この時点で１フレーム内に処理が収まっていない時の処理
    if (!flag.udp_board_passive) // パッシブモードの時は末端処理を行わない
    {
        tmr.now_mil = (long)millis(); // 現在時刻を更新
        if (tmr.now_mil > tmr.mrd_mil)
        {                              // 現在時刻がフレーム管理時計を超えていたらアラートを出す
            Serial.print("* delay: "); // シリアルに遅延msを表示
            Serial.println(tmr.now_mil - tmr.mrd_mil);
            digitalWrite(PIN_ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
        }
        else
        {
            digitalWrite(PIN_ERR_LED, LOW); // 処理が収まっていればLEDを消灯
        }

        // @[14-3] この時点で時間が余っていたら時間消化. 時間がオーバーしていたらこの処理を自然と飛ばす.
        tmr.now_mil = (long)millis();
        tmr.now_mic = (long)micros(); // 現在時刻を取得
        while (tmr.now_mil < tmr.mrd_mil)
        {
            tmr.now_mil = (long)millis();
            threads.delay(1);
        }
    }
    else
    {
        threads.delay(1); // 負荷低減のため1msのディレイ
    }

    // @[14-1] 必要に応じてフレーム管理時計tmr.mrd_milを現在時刻にリセット
    if (flag.reset_meridian_time)
    {
        tmr.now_mil = (long)millis();
        tmr.mrd_mil = tmr.now_mil;
        flag.reset_meridian_time = false;
        Serial.print(tmr.now_mil);
        Serial.print(" now:mrc ");
        Serial.print(tmr.mrd_mil);
        Serial.println(" Found 10007!!!");
    }

    // @[14-4]フレーム管理時計tmr.mrd_milのカウントアップ
    tmr.mrd_mil = tmr.mrd_mil + tmr.frame_ms;             // フレーム管理時計を1フレーム分進める
    tmr.loop_count = tmr.loop_count + tmr.loop_count_dlt; // サインカーブ等動作用のフレームカウントアップ
    if (tmr.loop_count > tmr.loop_count_max)              // カウンターが最大値ならゼロリセット
    {
        tmr.loop_count = 0;
    }
    mrd.monitor_check_flow("\n", MONITOR_FLOW); // 動作チェック用シリアル表示
    flag.mrd_spi_rcvd = false;                  // SPI受信フラグを下げる
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Initialize sensors like MPU6050, BNO055, and others.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setupWire()
{
    if (MOUNT_IMUAHRS > 0) // 何らかのセンサを搭載
    {
        flag.wire0_activate = true; // wire0をアクティブに
    }

    if (MOUNT_JOYPAD == 3) // Merimoteを搭載
    {
        flag.wire1_activate = true; // wire1をアクティブに
    }

    if (flag.wire0_activate)
    {
        Wire.begin();
        Wire.setClock(I2C_SPEED); // 400kHz I2C clock. Comment this line if having compilation difficulties
    }

    if (flag.wire1_activate)
    {
        Wire1.setSDA(17);
        Wire1.setSCL(16);
        Wire1.begin();
        Wire1.setClock(100000);
    }

    if (MOUNT_IMUAHRS == 1) // MPU6050
    {
        ahrs.mpu6050.initialize();
        ahrs.devStatus = ahrs.mpu6050.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        ahrs.mpu6050.setXAccelOffset(-1745);
        ahrs.mpu6050.setYAccelOffset(-1034);
        ahrs.mpu6050.setZAccelOffset(966);
        ahrs.mpu6050.setXGyroOffset(176);
        ahrs.mpu6050.setYGyroOffset(-6);
        ahrs.mpu6050.setZGyroOffset(-25);

        // make sure it worked (returns 0 if so)
        if (ahrs.devStatus == 0)
        {
            ahrs.mpu6050.CalibrateAccel(6);
            ahrs.mpu6050.CalibrateGyro(6);
            ahrs.mpu6050.setDMPEnabled(true);
            ahrs.packetSize = ahrs.mpu6050.dmpGetFIFOPacketSize();
        }
        else
        {
            Serial.print("IMU/AHRS DMP Initialization FAILED!");
        }
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
            // ahrsTimer.begin(IMUAHRS_getYawPitchRoll, int(IMUAHRS_POLLING * 1000)); // インターバルはマイクロ秒指定
        }
        // データの取得はセンサー用スレッドで実行?
    }
    else
    {
        Serial.println("No IMU/AHRS sensor mounted.");
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
        if (ahrs.mpu6050.dmpGetCurrentFIFOPacket(ahrs.fifoBuffer))
        { // Get new data
            ahrs.mpu6050.dmpGetQuaternion(&ahrs.q, ahrs.fifoBuffer);
            ahrs.mpu6050.dmpGetGravity(&ahrs.gravity, &ahrs.q);
            ahrs.mpu6050.dmpGetYawPitchRoll(ahrs.ypr, &ahrs.q, &ahrs.gravity);

            // acceleration values
            ahrs.mpu6050.dmpGetAccel(&ahrs.aa, ahrs.fifoBuffer);
            ahrs.read[0] = (float)ahrs.aa.x;
            ahrs.read[1] = (float)ahrs.aa.y;
            ahrs.read[2] = (float)ahrs.aa.z;

            // gyro values
            ahrs.mpu6050.dmpGetGyro(&ahrs.gyro, ahrs.fifoBuffer);
            ahrs.read[3] = (float)ahrs.gyro.x;
            ahrs.read[4] = (float)ahrs.gyro.y;
            ahrs.read[5] = (float)ahrs.gyro.z;

            // magnetic field values
            ahrs.read[6] = (float)ahrs.mag.x;
            ahrs.read[7] = (float)ahrs.mag.y;
            ahrs.read[8] = (float)ahrs.mag.z;

            // Estimated gravity DMP value.
            ahrs.read[9] = ahrs.gravity.x;
            ahrs.read[10] = ahrs.gravity.y;
            ahrs.read[11] = ahrs.gravity.z;

            // Estimated heading value using DMP.
            ahrs.read[12] = ahrs.ypr[2] * 180 / M_PI;                     // Estimated DMP_ROLL
            ahrs.read[13] = ahrs.ypr[1] * 180 / M_PI;                     // Estimated DMP_PITCH
            ahrs.read[14] = (ahrs.ypr[0] * 180 / M_PI) - ahrs.yaw_origin; // Estimated DMP_YAW

            // Temperature
            ahrs.read[15] = 0; // Not implemented.

            if (flag.imuahrs_available)
            {
                memcpy(ahrs.result, ahrs.read, sizeof(float) * 16);
            }
        }
    }
    else if (MOUNT_IMUAHRS == 3) // BNO055
    {
        // BNO055とTeensy4.0の相性が悪いという問題は未解決です.
    }
}

/**
 * @brief Receive input data from the gamepad and return it in PS2/3 gamepad array format.
 *
 * @param mount_joypad  Gamepad type (currently only 2: KRC-5FH).
 * @param pre_val Previous received value (8 bytes, assuming union data).
 * @param polling Frame count for inquiry frequency.
 * @param joypad.reflesh 1:To reset the JOYPAD's received button data to 0 with this device
 *                       0:perform logical addition without resetting .(usually 1)
 * @return uint64_t
 */
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh)
{
    if (mount_joypad == 2)
    { // KRR5FH(KRC-5FH)をICS_R系に接続している場合
        tmr.joypad_polling_count++;
        if (tmr.joypad_polling_count >= polling)
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

                if (JOYPAD_GENERALIZE)
                {

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
                else
                {
                    pad_btn_tmp = button_1;
                }
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
            tmr.joypad_polling_count = 0;
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
    Serial.print(err.esp_pc);
    Serial.print(" pc>esp:");
    Serial.print(err.pc_esp);
    Serial.print(" esp>tsy:");
    Serial.print(err.esp_tsy);
    Serial.print(" tsy>esp:");
    Serial.print(err.esp_tsy);
    Serial.print(" tsySkip:");
    Serial.print(err.tsy_skip); //
    Serial.print(" espSkip:");
    Serial.print(err.esp_skip); //
    Serial.print(" pcSkip:");
    Serial.print(err.pc_skip); //
    Serial.print(" seq:");
    Serial.print(int(mrdck.seq_r_expect)); //
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
        err.esp_pc++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u] >> 6) & 0b00000001) // Meridim[88] bit14:ESP32のPCからのUDP受信エラー
    {
        err.pc_esp++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u] >> 5) & 0b00000001) // Meridim[88] bit13:TeensyのESPからのSPI受信エラー
    {
        err.esp_tsy++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u] >> 4) & 0b00000001) // Meridim[88] bit12:ESP32のTeensyからのSPI受信エラー
    {
        err.tsy_esp++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u] >> 2) & 0b00000001) // Meridim[88] bit10:UDP→ESP受信のカウントのスキップ
    {
        err.esp_skip++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u] >> 1) & 0b00000001) // Meridim[88] bit9:ESP→Teensy受信のカウントのスキップ
    {
        err.tsy_skip++;
    }
    if ((r_spi_meridim.bval[MSG_ERR_u]) & 0b00000001) // Meridim[88] bit8:PC受信のカウントのスキップ
    {
        err.pc_skip++;
    }
}

/*
 * @brief Starting the IMU sensor or AHRS sensor.
 *
 */
void wire_manager0()
{
    IMUAHRS_getYawPitchRoll();
}

void wire_manager1()
{
    get_merimote_i2c();
}

/**
 * @brief Storing the values of the IMU sensor and AHRS sensor in an array.
 *
 */
void imuahrs_store(short sensorValues[], float storageArray[], int type)
{
    if (type == 1)
    {
        flag.imuahrs_available = false;
        sensorValues[2] = mrd.float2HfShort(storageArray[0]);   // IMU/AHRS_acc_x
        sensorValues[3] = mrd.float2HfShort(storageArray[1]);   // IMU/AHRS_acc_y
        sensorValues[4] = mrd.float2HfShort(storageArray[2]);   // IMU/AHRS_acc_z
        sensorValues[5] = mrd.float2HfShort(storageArray[3]);   // IMU/AHRS_gyro_x
        sensorValues[6] = mrd.float2HfShort(storageArray[4]);   // IMU/AHRS_gyro_y
        sensorValues[7] = mrd.float2HfShort(storageArray[5]);   // IMU/AHRS_gyro_z
        sensorValues[8] = mrd.float2HfShort(storageArray[6]);   // IMU/AHRS_mag_x
        sensorValues[9] = mrd.float2HfShort(storageArray[7]);   // IMU/AHRS_mag_y
        sensorValues[10] = mrd.float2HfShort(storageArray[8]);  // IMU/AHRS_mag_z
        sensorValues[11] = mrd.float2HfShort(storageArray[15]); // temperature
        sensorValues[12] = mrd.float2HfShort(storageArray[12]); // DMP_ROLL推定値
        sensorValues[13] = mrd.float2HfShort(storageArray[13]); // DMP_PITCH推定値
        sensorValues[14] = mrd.float2HfShort(storageArray[14]); // DMP_YAW推定値
        flag.imuahrs_available = true;
    }
    else if (type == 3)
    {
        // BNO055とTeensy4.0の相性は未解決問題です.
    }
}

//================================================================================================================
//---- Command processing ----------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Execute mastercommands group1.
 *
 */
void execute_MasterCommand_1()
{
    // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

    // コマンド:MCMD_ENTER_EEPROM_WRITE_MODE (10009) EEPROMの書き込みモードスタート
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_ENTER_EEPROM_WRITE_MODE)
    {
        flag.eeprom_write_mode = true;   // 書き込みモードのフラグを上げる
        flag.reset_meridian_time = true; // フレームの管理時計をリセットフラグを上げる
    }
}

/**
 * @brief Execute mastercommands group2.
 *
 */
void execute_MasterCommand_2()
{
    // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

    // コマンド:[0] 全サーボ脱力
    if (r_spi_meridim.sval[MRD_MASTER] == 0)
    {
        servo_all_off();
    }

    // コマンド:[1] サーボオン 通常動作

    // コマンド:MCMD_UPDATE_YAW_CENTER (10002) IMU/AHRSのヨー軸リセット
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_UPDATE_YAW_CENTER)
    {
        setyaw();
    }

    // コマンド:MCMD_ENTER_TRIM_MODE (10003) トリムモードに入る（既存のものは廃止し, 検討中）

    // コマンド:MCMD_CLEAR_SERVO_ERROR_ID (10004) 通信エラーサーボIDのクリア
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_CLEAR_SERVO_ERROR_ID)
    {
        s_spi_meridim.bval[MSG_ERR_l] = 0;
    }

    // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE)
    {
        flag.udp_board_passive = false;  // UDP送信をアクティブモードに
        flag.reset_meridian_time = true; // フレームの管理時計をリセットフラグを上げる
    }

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE)
    {
        flag.udp_board_passive = true;   // UDP送信をパッシブモードに
        flag.reset_meridian_time = true; // フレームの管理時計をリセットフラグを上げる
    }

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10007) フレーム管理時計tmr.mrd_milを現在時刻にリセット
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_RESET_MRD_TIMER)
    {
        flag.reset_meridian_time = true; // フレームの管理時計をリセットフラグを上げる
    }

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10008) ボードの末端処理を指定時間だけ止める.
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_STOP_BOARD_DURING)
    {
        flag.stop_board_during = true; // ボードの処理停止フラグを上げる
        // ボードの末端処理をmeridim[2]ミリ秒だけ止める.
        Serial.print("Stop teensy's processing during ");
        Serial.print(int(r_spi_meridim.sval[MRD_STOP_FRAMES_MS]));
        Serial.println(" ms.");
        for (int i = 0; i < int(r_spi_meridim.sval[MRD_STOP_FRAMES_MS]); i++)
        {
            delay(1);
        }
        flag.stop_board_during = false;  // ボードの処理停止フラグを下げる
        flag.reset_meridian_time = true; // フレームの管理時計をリセットフラグを上げる
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
        ahrs.yaw_origin = ahrs.ypr[0] * 180 / M_PI;
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
            if (sv.idl_mount[i] == 1)
            {
                krs_L.setFree(i);
            }
            if (sv.idr_mount[i] == 1)
            {
                krs_R.setFree(i);
            }
            delayMicroseconds(2);
        }
    }
    delay(2);
    Serial.println("All servos off.");
}

void get_merimote_i2c()
{
    int _i = 0;
    Wire1.requestFrom(I2C_MERIMOTE_ADDR, 10);
    while (Wire1.available())
    {                                    // バッファにデータがある間
        pad_i2c.bval[_i] = Wire1.read(); // バッファから1バイト読み取り
        _i++;
    }

    if (mrd.cksm_val(pad_i2c.sval, JOYPAD_I2C_LEN) == pad_i2c.sval[4])
    {
        for (int i = 0; i < 4; i++)
        {
            pad_array.sval[i] = pad_i2c.sval[i];
        }
        //Serial.print(pad_array.ubval[6]);
        //Serial.print(",");
        //Serial.println(pad_array.ubval[7]);
    }
    else
    {
        Serial.println("Merimote:NG");
    }
    pad_btn_past = pad_i2c.sval[0];
}
