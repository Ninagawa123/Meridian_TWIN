#ifndef __MERIDIAN_MAIN__
#define __MERIDIAN_MAIN__

#define VERSION "Meridian_TWIN_for_Teensy_2024.05.05" // バージョン表示

// Meridian_TWIN_for_Teensy By Izumi Ninagawa & Meridian Project
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト
// 20240414 第二次リファクタリング
// 20240414 SPI通信を1フレームあたり2回,往復実行するように修正.変数を構造化
// 20240503 第三次リファクタリング
// 20240502 モジュールごとにファイルを分割
// ※デフォルト値がIポーズかTポーズかを決定するフラグか変数が必要

// 引数をもっていること. （設定系の構造体が先頭(トリムとか, マウント, 設定？), meridimの送受信, OPTION）
// option:スルーモード, やりたいことは, SDアクセス, EEPROMアクセスなど
// （古いデータで上書きしちゃうけど, 上書き自体をしないでほしい. ）

//================================================================================================================
//---- 初期設定  --------------------------------------------------------------------------------------------------
//================================================================================================================

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_eeprom.h"
#include "mrd_move.h"
#include "mrd_msg.h"
#include "mrd_pad.h"
#include "mrd_sd.h"
#include "mrd_servo.h"
#include "mrd_wire0.h"
#include "mrd_wire1.h"

bool spi_trans = MOUNT_ESP32;                          // SPI(ESP32)の送受信をするかどうか
const unsigned long frameTime = FRAME_DURATION * 1000; // フレーム時間（マイクロ秒）
unsigned long frameStartTime = micros();
unsigned long frameEndTime = micros();
unsigned long elapsedTime;
long frameDeley = 0; // フレーム時間（マイクロ秒）
long timeToNextFrame;

//================================================================================================================
//---- SET UP ----------------------------------------------------------------------------------------------------
//================================================================================================================

void setup()
{
    // 入出力ピンのモード設定
    pinMode(PIN_ERR_LED, OUTPUT); // 通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

    // 配列のリセット
    memset(s_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);     // 配列要素を0でリセット
    memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); // 配列要素を0でリセット
    s_spi_meridim_dummy.sval[0] = MCMD_DUMMY_DATA;   // ダミーデータの設定
    s_spi_meridim.sval[0] = MSG_SIZE;                // (マスターコマンド）

    // SPI通信用DMAの設定
    TsyDMASPI0.begin(SS, SPISettings(SPI0_SPEED, MSBFIRST, SPI_MODE3));
    delay(500);

    // シリアル設定
    Serial.begin(SERIAL_PC_BPS); // シリアルモニター表示
    delay(100);

    //  起動時のシリアルモニタ情報表示
    mrd_msg_print_tsy_hello(VERSION);
    mrd_msg_print_imuahrs(MOUNT_IMUAHRS); // 6軸9軸センサのタイプ

    // I2Cに接続したセンサやリモコンをスタート
    mrd_wire0_startIntervalTimer(mrd_wire0_setup(MOUNT_IMUAHRS, I2C0_SPEED, ahrs));
    mrd_msg_print_mounted_pad(); // コントロールパッドの種類を表示
    flg.wire1_init = mrd_wire1_setup(MOUNT_JOYPAD, I2C1_SPEED);
    mrd_wire1_startIntervalTimer(pad_array, pad_i2c);
    delay(100);

    // サーボ用UART設定
    mrd_servos_begin(L, MOUNT_L_SERVO_TYPE);                          // サーボモータの通信初期設定. Serial2
    mrd_servos_begin(R, MOUNT_R_SERVO_TYPE);                          // サーボモータの通信初期設定. Serial3
    mrd_servos_begin(C, MOUNT_C_SERVO_TYPE);                          // サーボモータの通信初期設定. Serial1
    mrd.print_servo_mounts(sv.idl_mount, sv.idr_mount, sv.IDC_mount); // マウントされたサーボIDの表示
    delay(100);

    // SDカードの初期化と読み書きテスト
    mrd_sd_init(MOUNT_SD, PIN_CHIPSELECT_SD);
    mrd_sd_check(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW);

    Serial.println("*CHECK*");

    // EEPROMの読み書きテスト
    mrd_eeprom_check(EEPROM_CHECK);

    // 起動時
    mrd_msg_charging(CHARGE_TIME); // ボード搭載のキャパシタチャージ時間
    mrd_msg_flow_start();
    flg.mrd_spi_rcvd = false; // SPI受信フラグをクリア
}

//================================================================================================================
//---- MAIN LOOP -------------------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{
    frameStartTime = micros(); // フレーム開始時刻を取得

    //------------------------------------------------------------------------------------
    //  [ 1 ] ESP32とのSPIよる送受信処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[1]", MONITOR_FLOW); // 動作チェック用シリアル表示

    // @[1-1] ESP32とのSPI送受信の実行
    if (spi_trans)
    {
        // SPIのデータを受信。送信はダミーデータ。
        TsyDMASPI0.transfer(s_spi_meridim_dummy.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

        // [1-2] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
        if (mrd.cksm_rslt(r_spi_meridim_dma.sval, MSG_SIZE))
        { // チェックサムがOKならバッファから受信配列に転記
            memcpy(r_spi_meridim.bval, r_spi_meridim_dma.bval, MSG_BUFF);
            clearBit16(r_spi_meridim.usval[MSG_ERR], 13); // TeensyのESPからのSPI受信エラー検出(13番ビット)をクリア
        }
        else // チェックサムがNGならバッファから転記せず前回のデータを使用する
        {
            setBit16(r_spi_meridim.usval[MSG_ERR], 13); // TeensyのESPからのSPI受信エラー検出(13番ビット)をセット
        }

        // @[1-3] シーケンス番号チェック
        mrdck.seq_r_expect = mrd.seq_predict_num(mrdck.seq_r_expect); // シーケンス番号予想値の生成
        if (mrd.seq_compare_nums(mrdck.seq_r_expect, int(r_spi_meridim.usval[MRD_SEQENTIAL])))
        {
            clearBit16(r_spi_meridim.usval[MSG_ERR], 9); // Teensy受信のスキップ検出(9番ビット)をクリア
            flg.mrd_spi_rcvd = true;                     // SPI受信フラグをセット
        }
        else // シーケンス番号の値が予想と違ったら
        {
            mrdck.seq_r_expect = int(r_spi_meridim.usval[MRD_SEQENTIAL]); // 現在の受信値を予想結果としてキープ
            setBit16(r_spi_meridim.usval[MSG_ERR], 9);                    // Teensy受信のスキップ検出(9番ビット)をセット
            flg.mrd_spi_rcvd = false;                                     // SPI受信フラグをクリア
        }

        // @[1-4] 通信エラー処理(エラーカウンタへの反映)
        countup_errors();

        //------------------------------------------------------------------------------------
        //  [ 2 ] シリアルモニタリング表示処理
        //------------------------------------------------------------------------------------
        mrd.monitor_check_flow("[2]", MONITOR_FLOW); // 動作チェック用シリアル表示
        // @[2-1] 全経路のエラー数の表示
        if (monitor.all_error)
            mrd_msg_print_error_monitor();
    }

    //------------------------------------------------------------------------------------
    //  [ 3 ] MastarCommand group1 の処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[3]", MONITOR_FLOW); // 動作チェック用シリアル表示
                                                 // @[3-1] マスターコマンドの判定により工程の実行orスキップを分岐

    execute_MasterCommand_1(flg.mrd_spi_rcvd); // Meridmを正しく受信した時のみ実行

    // @[3-2]EEPROMやSDへの読み書き処理はおそらくここで実施.
    // execute_MasterCommand()関数内でフラグを立て, フラグによって分岐.

    //------------------------------------------------------------------------------------
    //  [ 4 ] 積み残し処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[4]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[4-1] 積み残しがあればここで処理

    //------------------------------------------------------------------------------------
    //  [ 5 ] 受信SPIデータを送信SPIデータに転記 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[5]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[5-1] 受信データを送信データに転記
    memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MSG_BUFF + 4);

    //------------------------------------------------------------------------------------
    //  [ 6 ] センサー類読み取り 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[6]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[6-3] センサー値を配列に格納
    mrd_wire0_write_ahrs_data(MOUNT_IMUAHRS, s_spi_meridim.sval, ahrs.result);

    //------------------------------------------------------------------------------------
    //  [ 7 ] コントローラの読み取り 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[7]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[7-1] コントローラの値を取得して送信データに格納する
    mrd_pad_reader(MOUNT_JOYPAD, JOYPAD_INTERVAL, JOYPAD_BUTTON_MARGE);

    //------------------------------------------------------------------------------------
    //  [ 8 ] Teensy内部で位置制御する場合の処理  
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[8]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[8-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
    for (int i = 0; i < sv.num_max; i++)
    {
        sv.idl_tgt_past[i] = sv.idl_tgt[i];                    // 前回のdegreeをキープ
        sv.idr_tgt_past[i] = sv.idr_tgt[i];                    // 前回のdegreeをキープ
        sv.idl_tgt[i] = r_spi_meridim.sval[i * 2 + 21] * 0.01; // 通常のdegreeが一旦入る
        sv.idr_tgt[i] = r_spi_meridim.sval[i * 2 + 51] * 0.01; // 通常のdegreeが一旦入る
    }

    // @[8-2] Teensyによる次回動作の計算
    // リモコンの左十字キー左右で首を30度左右にふるサンプル
    if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 32)
    {
        sv.idl_tgt[0] = -30.0; // -30度
    }
    else if (r_spi_meridim.sval[MRD_CONTROL_BUTTONS] == 128)
    {
        sv.idl_tgt[0] = 30.0; // +30度
    }

    // @[8-3] センサーデータによる動作へのフィードバック加味

    // @[8-4] 移動時間の決定

    // @[8-5] Teensy内計算による次回動作をMeridim配列に書き込む

    //------------------------------------------------------------------------------------
    //  [ 9 ] MastarCommand group2 の処理 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[9]", MONITOR_FLOW); // 動作チェック用シリアル表示
                                                 // @[8-1] マスターコマンドの判定により工程の実行orスキップを分岐
    // @[9-1] マスターコマンドの判定により工程の実行orスキップを分岐

    // if (flg.mrd_spi_rcvd)
    //{
    // execute_MasterCommand_2();
    execute_MasterCommand_2(flg.mrd_spi_rcvd);
    //}

    //------------------------------------------------------------------------------------
    //  [ 10 ] サーボコマンドの書き込み 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[10]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[10-1] Meridim配列をサーボ命令に変更

    // @[10-2] サーボコマンドの配列に書き込み

    // @[10-3] サーボデータのICS送信および返り値を取得

    //------------------------------------------------------------------------------------
    //   [ 11 ] サーボ動作の実行 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[11]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @ [11-1] サーボ命令の実行およびサーボ角度戻り値の取得
    mrd_servos_drive(s_spi_meridim, MOUNT_L_SERVO_TYPE, MOUNT_R_SERVO_TYPE, MOUNT_C_SERVO_TYPE);

    //------------------------------------------------------------------------------------
    //  [ 12 ] SPI送信用のMeridim配列を作成する 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[12]", MONITOR_FLOW); // 動作チェック用シリアル表示
    // @[12-1] マスターコマンドを配列に格納
    s_spi_meridim.sval[0] = MSG_SIZE; // デフォルトのマスターコマンドは配列数

    // @[12-2] 移動時間を配列に格納
    // s_spi_meridim.sval[1] = 10 ;//(移動時間）

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
    // s_spi_meridim.usval[MRD_SEQENTIAL] = mrdck.seq_s_increment;
    s_spi_meridim.usval[MRD_SEQENTIAL] = u_short(mrdck.seq_s_increment);

    // s_spi_meridim.usval[MRD_SEQENTIAL] = u_short(mrd.seq_increase_num(mrdck.seq_s_increment));
    //   @[12-6] カスタムデータを配列格納

    // @[12-7] チェックサムを計算
    s_spi_meridim.sval[MSG_SIZE - 1] = mrd.cksm_val(s_spi_meridim.sval, MSG_SIZE);

    // @[12-8] 送信データのSPIバッファへのバイト型書き込み
    memcpy(s_spi_meridim_dma.bval, s_spi_meridim.bval, MSG_BUFF);

    //------------------------------------------------------------------------------------
    //   [ 13 ] SPI送信 
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[13]", MONITOR_FLOW); // 動作チェック用シリアル表示
    TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

    //------------------------------------------------------------------------------------
    //   [ 14 ] フレーム終端処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[14]", MONITOR_FLOW); // 動作チェック用シリアル表示
    frameEndTime = micros();
    elapsedTime = frameEndTime - frameStartTime;
    timeToNextFrame = frameTime - elapsedTime + frameDeley;

    if (!flg.udp_board_passive) // パッシブモードの時は末端処理を行わない
    {
        // @[14-1] 処理が早く終わった場合, 残り時間を待機
        if (timeToNextFrame > 0)
        {
            // 1ms以上の余裕がある場合はdelayを使用して休む（熱対策など用）
            if (timeToNextFrame >= 1000)
            {
                delay(timeToNextFrame / 1000); // 整数部分のミリ秒だけdelay
                timeToNextFrame %= 1000;       // 余りのマイクロ秒
            }

            // 残りのマイクロ秒単位の端数で厳密に待機
            if (timeToNextFrame > 0)
            {
                delayMicroseconds(timeToNextFrame);
            }
            frameDeley = 0;
        }
        else
        {
            frameDeley = timeToNextFrame; // 新しい遅延時間を蓄積
            Serial.print("deley: ");      // シリアルに遅延msを表示
            Serial.print(-float(frameDeley) / 1000000, 3);
            Serial.println(" sec");
            digitalWrite(PIN_ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
        }
    }
    else
    {
        // @[14-2] この時点で１フレーム内に処理が収まっていない時の処理
        delay(1); // 待機処理をスキップするが, 負荷低減のため1msのディレイ
    }

    // @[14-3] 必要に応じてフレーム管理時計tmr.mrd_milを現在時刻にリセット
    if (flg.reset_meridian_time)
    {
        frameDeley = 0;
        Serial.println("Delay time reset.");
        flg.reset_meridian_time = false;
    }
    mrd.monitor_check_flow("\n", MONITOR_FLOW); // 動作チェック用シリアル表示
    flg.mrd_spi_rcvd = false;                   // SPI受信フラグをクリア
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// ビットをセットする関数（16ビット用）
void setBit16(uint16_t &byte, uint16_t bitPosition)
{
    byte |= (1 << bitPosition);
}

// ビットをクリアする関数（16ビット用）
void clearBit16(uint16_t &byte, uint16_t bitPosition)
{
    byte &= ~(1 << bitPosition);
}

// ビットをセットする関数（8ビット用）
void setBit8(uint8_t &value, uint8_t bitPosition)
{
    value |= (1 << bitPosition);
}

// ビットをクリアする関数（8ビット用）
void clearBit8(uint8_t &value, uint8_t bitPosition)
{
    value &= ~(1 << bitPosition);
}

void countup_errors()
{
    auto incrementError = [&](int bitPosition, int &errorCounter)
    {
        if ((r_spi_meridim.bval[MSG_ERR_u] >> bitPosition) & 0b00000001)
        {
            errorCounter++;
        }
    };

    incrementError(7, err.esp_pc);   // PCのESP32からのUDP受信エラー
    incrementError(6, err.pc_esp);   // ESP32のPCからのUDP受信エラー
    incrementError(5, err.esp_tsy);  // TeensyのESPからのSPI受信エラー
    incrementError(4, err.tsy_esp);  // ESP32のTeensyからのSPI受信エラー
    incrementError(2, err.esp_skip); // UDP→ESP受信のカウントのスキップ
    incrementError(1, err.tsy_skip); // ESP→Teensy受信のカウントのスキップ
    incrementError(0, err.pc_skip);  // PC受信のカウントのスキップ
}

//================================================================================================================
//---- Command processing ----------------------------------------------------------------------------------------
//================================================================================================================

bool execute_MasterCommand_1(bool exe_flg_mrd_spi_rcvd)
{
    if (!exe_flg_mrd_spi_rcvd)
    {
        return false;
    }

    // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

    // コマンド:MCMD_CLEAR_SERVO_ERROR_ID (10004) 通信エラーサーボIDのクリア
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_CLEAR_SERVO_ERROR_ID)
    {
        r_spi_meridim.bval[MSG_ERR_l] = 0;
        s_spi_meridim.bval[MSG_ERR_l] = 0;
        for (int i = 0; i < IDL_MAX; i++)
        {
            sv.idl_err[i] = 0;
        }
        for (int i = 0; i < IDR_MAX; i++)
        {
            sv.idr_err[i] = 0;
        }
        Serial.println("Servo Error ID reset.");
    }

    // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE)
    {
        flg.udp_board_passive = false;  // UDP送信をアクティブモードに
        flg.reset_meridian_time = true; // フレームの管理時計をリセットフラグをセット
    }

    // コマンド:MCMD_ENTER_EEPROM_WRITE_MODE (10009) EEPROMの書き込みモードスタート
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_ENTER_EEPROM_WRITE_MODE)
    {
        flg.eeprom_write_mode = true;   // 書き込みモードのフラグをセット
        flg.reset_meridian_time = true; // フレームの管理時計をリセットフラグをセット
    }

    return true;
}

bool execute_MasterCommand_2(bool exe_flg_mrd_spi_rcvd)
{
    if (!exe_flg_mrd_spi_rcvd)
    {
        return false;
    }
    // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

    // コマンド:[0] 全サーボ脱力
    if (r_spi_meridim.sval[MRD_MASTER] == 0)
    {
        mrd_servos_all_off(s_spi_meridim);
    }

    // コマンド:[1] サーボオン 通常動作

    // コマンド:MCMD_UPDATE_YAW_CENTER (10002) IMU/AHRSのヨー軸リセット
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_UPDATE_YAW_CENTER)
    {
        ahrs.yaw_origin = mrd_wire0_setyaw(ahrs.ypr[0]);
    }

    // コマンド:MCMD_ENTER_TRIM_MODE (10003) トリムモードに入る（既存のものは廃止し, 検討中）

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE)
    {
        flg.udp_board_passive = true;   // UDP送信をパッシブモードに
        flg.reset_meridian_time = true; // フレームの管理時計をリセットフラグをセット
    }

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10007) フレーム管理時計tmr.mrd_milを現在時刻にリセット
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_RESET_MRD_TIMER)
    {
        flg.reset_meridian_time = true; // フレームの管理時計をリセットフラグをセット
    }

    // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10008) ボードの末端処理を指定時間だけ止める.
    if (r_spi_meridim.sval[MRD_MASTER] == MCMD_STOP_BOARD_DURING)
    {
        flg.stop_board_during = true; // ボードの処理停止フラグをセット
        // ボードの末端処理をmeridim[2]ミリ秒だけ止める.
        Serial.print("Stop teensy's processing during ");
        Serial.print(int(r_spi_meridim.sval[MRD_STOP_FRAMES_MS]));
        Serial.println(" ms.");
        for (int i = 0; i < int(r_spi_meridim.sval[MRD_STOP_FRAMES_MS]); i++)
        {
            delay(1);
        }
        flg.stop_board_during = false;  // ボードの処理停止フラグをクリア
        flg.reset_meridian_time = true; // フレームの管理時計をリセットフラグをセット
    }
    return true;
}

#endif
