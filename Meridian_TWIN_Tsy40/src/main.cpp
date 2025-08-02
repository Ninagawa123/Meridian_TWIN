#ifndef __MERIDIAN_MAIN__
#define __MERIDIAN_MAIN__

#define VERSION "Meridian_TWIN_for_Teensy_v1.1.1_2025.08.02" // バージョン表示

// Meridian_TWIN_for_Teensy By Izumi Ninagawa & Meridian Project
// MIT Licenced.
// Meridan TWIN Teensy4.0用スクリプト
// 20240414 第二次リファクタリング
// 20240414 SPI通信を1フレームあたり2回,往復実行するように修正.変数を構造化
// 20240503 第三次リファクタリング
// 20240502 モジュールごとにファイルを分割
// 20240812 変数, 関数, ファイル名等を大幅に変更.
// 20240819 EEPROMのコードは調整中.
// 20250505 トリム調整モードに対応. (Meridian_console.py連携)
// 20250801 SDメモリの起動順を修正

//================================================================================================================
//  初期設定
//================================================================================================================

// ヘッダファイルの読み込み
#include "config.h"

#include "main.h"
#include "mrd_command.h"
#include "mrd_disp.h"
#include "mrd_eeprom.h"
#include "mrd_hwtimer.h"
#include "mrd_move.h"
#include "mrd_pad.h"
#include "mrd_sd.h"
#include "mrd_servo.h"
#include "mrd_util.h"
#include "mrd_wire0.h"
#include "mrd_wire1.h"

//================================================================================================================
//  SET UP
//================================================================================================================

void setup() {
  // シリアルモニターの設定
  Serial.begin(SERIAL_PC_BPS); //

  // シリアルモニターの確立待ち
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < SERIAL_PC_TIMEOUT)) { // タイムアウトもチェック
    delay(1);
  }

  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // 通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

  // ボード搭載のコンデンサの充電時間として待機
  mrd_disp.charging(CHARGE_TIME);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_twin_tsy(VERSION, SERIAL_PC_BPS, SPI0_SPEED, I2C0_SPEED, I2C1_SPEED);

  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX),
                   mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数

  for (int i = 0; i <= sv.num_max; i++) { // configで設定した値を反映させる
    sv.ixl_mount[i] = IXL_MT[i];
    sv.ixr_mount[i] = IXR_MT[i];
    sv.ixc_mount[i] = IXC_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixc_id[i] = IXC_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixc_cw[i] = IXC_CW[i];
    sv.ixl_trim[i] = IXL_TRIM[i];
    sv.ixr_trim[i] = IXR_TRIM[i];
    sv.ixc_trim[i] = IXC_TRIM[i];
  };

  // サーボUARTの通信速度の表示
  mrd_disp.servo_bps_3lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R, SERVO_BAUDRATE_C);

  // サーボ用UART設定
  mrd_servos_begin(L, MOUNT_SERVO_TYPE_L); // サーボモータの通信初期設定. Serial2
  mrd_servos_begin(R, MOUNT_SERVO_TYPE_R); // サーボモータの通信初期設定. Serial3
  mrd_servos_begin(C, MOUNT_SERVO_TYPE_C); // サーボモータの通信初期設定. Serial1
  mrd_disp.servo_protcol(L, MOUNT_SERVO_TYPE_L);
  mrd_disp.servo_protcol(R, MOUNT_SERVO_TYPE_R);
  mrd_disp.servo_protcol(C, MOUNT_SERVO_TYPE_C);

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_3lines(sv);

  // EEPROMにconfigのサーボ設定値を書き込む場合
  if (EEPROM_SET) {
    Serial.println("Set EEPROM data from config.");
    // 書き込みデータの作成と書き込み
    if (
        mrd_eeprom_write(mrd_eeprom_make_data_from_config(sv), EEPROM_PROTECT, Serial)) {
      Serial.println("Write EEPROM succeed.");
    } else {
      Serial.println("Write EEPROM failed.");
    };
  }

  // EEPROMからサーボ設定の内容を読み込んで反映する場合
  if (EEPROM_LOAD) {
    mrd_eeprom_load_servosettings(sv, true, Serial);
  }

  // 配列のリセット
  memset(s_spi_meridim.bval, 0, MRDM_BYTE + 4);     // 配列要素を0でリセット
  memset(r_spi_meridim.bval, 0, MRDM_BYTE + 4);     // 配列要素を0でリセット
  memset(s_spi_meridim_dma.bval, 0, MRDM_BYTE + 4); // 配列要素を0でリセット
  memset(r_spi_meridim_dma.bval, 0, MRDM_BYTE + 4); // 配列要素を0でリセット
  s_spi_meridim_dummy.sval[0] = MCMD_DUMMY_DATA;    // ダミーデータの設定
  s_spi_meridim.sval[0] = MRDM_LEN;                 // (マスターコマンド）

  // I2C0をスタート(主にセンサ)
  mrd_disp.imuahrs(MOUNT_IMUAHRS); // 6軸9軸センサタイプの表示
  mrd_wire0_intervaltimer_start(mrd_wire0_setup(MOUNT_IMUAHRS, I2C0_SPEED, ahrs));

  // I2C0をスタート(主にリモコン)
  mrd_disp.mounted_pad(MOUNT_PAD); // コントロールパッドの種類を表示
  mrd_wire1_startIntervalTimer(mrd_wire1_setup(MOUNT_PAD, I2C1_SPEED, Serial));

  // SDカードの初期化と読み書きテスト
  mrd_sd.init(MOUNT_SD, PIN_CHIPSELECT_SD);
  mrd_sd.check_rw(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW);

  // ハードウェアタイマーを開始
  mrd_hwtimer_start_tsy(FRAME_DURATION); // ハードウェアタイマーを指定msでカウントアップ

  // フラグ調整
  flg.spi_trans = MODE_SPI_TRANS & MOUNT_ESP32; // SPI(ESP32)の送受信をするかどうか
  flg.spi_rcvd = false;                         // SPI受信完了フラグをサゲる

  // SPI通信用DMAの設定と開始
  TsyDMASPI0.begin(SS, SPISettings(SPI0_SPEED, MSBFIRST, SPI_MODE3));

  // 起動時
  mrd_disp.flow_start_twin_tsy();
}

//================================================================================================================
//  MAIN LOOP
//================================================================================================================
void loop() {

  // 計算用ループカウンタのカウントアップ
  tmr.count_loop += tmr.count_loop_dlt;
  if (tmr.count_loop > tmr.count_loop_max) {
    tmr.count_loop = 0;
  }

  //------------------------------------------------------------------------------------
  //  [ 1 ] ESP32とのSPIよる送受信処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", monitor.flow); // 動作チェック用シリアル表示

  // @[1-1] ESP32とのSPI送受信の実行
  if (flg.spi_trans) {
    // SPIのデータを受信. 送信はダミーデータ.
    TsyDMASPI0.transfer(s_spi_meridim_dummy.bval, r_spi_meridim_dma.bval, MRDM_BYTE + 4);

    // [1-2] ESP32からのSPI受信データチェックサム確認
    if (mrd.cksm_rslt(r_spi_meridim_dma.sval, MRDM_LEN)) //
    {                                                    // チェックサムがOKならバッファから受信配列に転記
      memcpy(r_spi_meridim.bval, r_spi_meridim_dma.bval, MRDM_BYTE);
      mrd_clear_bit16(r_spi_meridim.usval[MRD_ERR], ERRBIT_13_ESP_TSY); // エラービットをサゲる
      mrd.monitor_check_flow("csOK", monitor.flow);                     // 動作チェック用シリアル表示
    } else                                                              // チェックサムがNGならバッファから転記せず前回のデータを使用する
    {
      mrd.monitor_check_flow("cs *NG* ", monitor.flow);               // 動作チェック用シリアル表示
      mrd_set_bit16(r_spi_meridim.usval[MRD_ERR], ERRBIT_13_ESP_TSY); // エラービットをアゲる
    }

    // @[1-3] シーケンス番号チェック
    mrdsq.r_expect = mrd.seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成
    if (mrd.seq_compare_nums(mrdsq.r_expect, int(r_spi_meridim.usval[MRD_SEQ]))) {
      // 受信シーケンス番号の値が予想と合致なら
      mrd_clear_bit16(r_spi_meridim.usval[MRD_ERR], ERRBIT_9_BOARD_SKIP); // エラービットをサゲる
      flg.spi_rcvd = true;                                                // SPI受信フラグをアゲる
    } else {
      // 受信シーケンス番号の値が予想と違うなら
      mrdsq.r_expect = int(r_spi_meridim.usval[MRD_SEQ]);               // 現在の受信値を予想値として更新
      mrd_set_bit16(r_spi_meridim.usval[MRD_ERR], ERRBIT_9_BOARD_SKIP); // エラービットをアゲる
      flg.spi_rcvd = false;                                             // SPI受信フラグをサゲる
    }

    // @[1-4] 通信エラー処理(エラーカウンタへの反映)
    mrd_countup_errs();

    // @[1-end] この時点で r_spi_meridim にespから届いたチェック済みの最新データが格納されている.
    // @        ただし, チェックサム後にエラーフラグを更新している場合がある.

    //------------------------------------------------------------------------------------
    //  [ 2 ] シリアルモニタリング表示処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[2]", monitor.flow); // 動作チェック用シリアル表示

    // @[2-1] 全経路のエラー数の表示
    if (monitor.all_err) {
      mrd_disp.err_monitor(r_spi_meridim, err, mrdsq);
    }

    //------------------------------------------------------------------------------------
    //  [ 3 ] MastarCommand group1 の処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[3]", monitor.flow); // 動作チェック用シリアル表示

    // @[3-1] マスターコマンドの実行
    // execute_master_command_1(r_spi_meridim, flg.spi_rcvd); // Meridmを正しく受信した時のみ実行
    execute_master_command_1(r_spi_meridim, flg.spi_rcvd, sv, Serial);

    // @[3-2]EEPROMやSDへの読み書き処理はおそらくここで実施.
    // execute_MasterCommand()関数内でフラグを立て, フラグによって分岐.

    //------------------------------------------------------------------------------------
    //  [ 4 ] 積み残し処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[4]", monitor.flow); // 動作チェック用シリアル表示
    // @[4-1] 積み残しがあればここで処理

    // @[4-end] ここまでの処理は r_spi_meridim に受信したデータに基づいて行う.

    //------------------------------------------------------------------------------------
    //  [ 5 ] 受信SPIデータを送信SPIデータに転記
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[5]", monitor.flow); // 動作チェック用シリアル表示
    // @[5-1] 受信データを送信データに転記
    memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MRDM_BYTE + 4);

    // @[5-end] ここで送信データのベースである s_spi_meridim の準備ができ,
    //          以降は s_spi_meridim に基づいて処理を進める.

    //------------------------------------------------------------------------------------
    //  [ 6 ] センサー類読み取り
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[6]", monitor.flow); // 動作チェック用シリアル表示

    // @[6-3] 最新のセンサー値を配列に格納する
    mrd_meriput90_ahrs(s_spi_meridim, ahrs.result);

    //------------------------------------------------------------------------------------
    //  [ 7 ] リモコンの読み取り
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[7]", monitor.flow); // 動作チェック用シリアル表示

    // @[7-1] コントローラの値を取得する
    pad_new = mrd_pad_reader(MOUNT_PAD, PAD_INTERVAL);

    // @[7-2] コントローラの値をmeridimに格納する
    mrd_meriput90_pad(MOUNT_PAD, s_spi_meridim, pad_new, PAD_BUTTON_MARGE);

    // @[7-end] ここでs_spi_meridim にリモコンやセンサのデータが格納完了.

    //------------------------------------------------------------------------------------
    //  [ 8 ] Teensy内部で位置制御する場合の処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[8]", monitor.flow); // 動作チェック用シリアル表示
    // @[8-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
    for (int i = 0; i < sv.num_max; i++) {
      sv.ixl_tgt_past[i] = sv.ixl_tgt[i];                    // 前回のdegreeをキープ
      sv.ixr_tgt_past[i] = sv.ixr_tgt[i];                    // 前回のdegreeをキープ
      sv.ixl_tgt[i] = r_spi_meridim.sval[i * 2 + 21] * 0.01; // 通常のdegreeが一旦入る
      sv.ixr_tgt[i] = r_spi_meridim.sval[i * 2 + 51] * 0.01; // 通常のdegreeが一旦入る
    }

    // @[8-2] Teensyによる次回動作の計算
    // 以下はリモコンの左十字キー左右でL系統0番サーボ（首部）を30度左右にふるサンプル
    if (s_spi_meridim.sval[MRD_PAD_BUTTONS] == 32) {
      sv.ixl_tgt[0] = -30.00; // -30度
    } else if (s_spi_meridim.sval[MRD_PAD_BUTTONS] == 128) {
      sv.ixl_tgt[0] = 30.00; // +30度
    }

    // @[8-3] センサーデータによる動作へのフィードバック加味

    // @[8-4] 移動時間の決定

    // @[8-5] Teensy内計算による次回動作をサーボ配列に書き込む

    // @[8-end] ここでの最新の制御命令データはサーボ配列(sv)のパラメータとなる.

    //------------------------------------------------------------------------------------
    //  [ 9 ] MastarCommand group2 の処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[9]", monitor.flow); // 動作チェック用シリアル表示

    // @[9-1] マスターコマンドの判定により工程の実行orスキップを分岐
    // execute_master_command_2(s_spi_meridim, flg.spi_rcvd);
    execute_master_command_2(s_spi_meridim, flg.spi_rcvd, sv, Serial);

    //------------------------------------------------------------------------------------
    //  [ 10 ] サーボコマンドの書き込み
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[10]", monitor.flow); // 動作チェック用シリアル表示

    // @[10-1] Meridim配列のサーボコマンドを調べ,優先度を比較する.(強制トルクオフなど)
    //         必要に応じてサーボ配列(sv)の命令を上書きする.

    //------------------------------------------------------------------------------------
    //  [ 11 ] サーボ動作の実行
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[11]", monitor.flow); // 動作チェック用シリアル表示

    // @[11-1] サーボコマンド用の配列に基づき, サーボ命令の実行およびサーボ角度戻り値の取得
    mrd_servos_drive(s_spi_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, MOUNT_SERVO_TYPE_C, sv);

    // @[11-end] サーボの動作結果が s_spi_meridim に格納完了している.

    //------------------------------------------------------------------------------------
    //  [ 12 ] MastarCommand group3 の処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[12]", monitor.flow); // 動作チェック用シリアル表示

    // @[12-1] マスターコマンドの判定により工程の実行orスキップを分岐, 場合により特殊命令でs_spi_meridim上書き
    execute_master_command_3(s_spi_meridim, flg.spi_rcvd, sv, flg.meridim_special, Serial);

    //------------------------------------------------------------------------------------
    //  [ 13 ] SPI送信用のMeridim配列を作成する
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[13]", monitor.flow); // 動作チェック用シリアル表示

    if (flg.meridim_special == false) { // 特殊Meridim配列の送信フラグが立っていなければ以下の通常処理
      // @[13-1] マスターコマンドを配列に格納
      s_spi_meridim.sval[0] = MRDM_LEN; // デフォルトのマスターコマンドは配列数

      // @[13-2] サーボIndexごとにの現在位置もしくは計算結果を配列に格納
      for (int i = 0; i < 15; i++) {
        s_spi_meridim.sval[i * 2 + 20] = 0; // 仮に各サーボのコマンドを脱力&ポジション返信に設定
        s_spi_meridim.sval[i * 2 + 21] =
            mrd.float2HfShort(sv.ixl_tgt[i]); // 最新のサーボ角度degreeを格納
        s_spi_meridim.sval[i * 2 + 50] = 0;   // 仮に各サーボのコマンドを脱力&ポジション返信に設定
        s_spi_meridim.sval[i * 2 + 51] =
            mrd.float2HfShort(sv.ixr_tgt[i]); // 最新のサーボ角度degreeを格納
      }

      // @[13-3] エラーが出たサーボのインデックス番号を格納
      s_spi_meridim.ubval[MRD_ERR_l] = mrd_servos_make_errcode_3lines(sv);

      // @[13-4] 移動時間を配列に格納
      s_spi_meridim.sval[1] = 10; //(移動時間）
    }

    // @[13-5] Meridimのシーケンス番号をカウントアップして送信用に格納
    mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
    s_spi_meridim.usval[MRD_SEQ] = u_short(mrdsq.s_increment);

    // @[13-6] 必要に応じてカスタムデータを配列格納

    // @[13-7] チェックサムを追加
    s_spi_meridim.sval[MRDM_LEN - 1] = mrd.cksm_val(s_spi_meridim.sval, MRDM_LEN);

    // @[13-8] 送信データのSPIバッファへのバイト型書き込み
    memcpy(s_spi_meridim_dma.bval, s_spi_meridim.bval, MRDM_BYTE);

    // @[13-9] 特殊Meridim配列の送信フラグが立っていれば下げる
    flg.meridim_special = false;

    //------------------------------------------------------------------------------------
    //  [ 14 ] SPI送信
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[14]", monitor.flow); // 動作チェック用シリアル表示

    // @[14-1] SPI送受信の実行
    TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MRDM_BYTE + 4);

    // @[14-end] SPI送信が完了. ここで受信したデータは前回のものであるため, 一旦無視する.

    //------------------------------------------------------------------------------------
    //   [ 15 ] フレーム終端処理
    //------------------------------------------------------------------------------------
    mrd.monitor_check_flow("[15]", monitor.flow); // 動作チェック用シリアル表示

    if (!flg.udp_board_passive) // パッシブモードの時は末端処理を行わない
    {
      // @[15-1] count_timerがcount_frameに追いつくまで待機
      tmr.count_frame++;

      while (true) {
        unsigned long hwtimer_tmp = mrd_hwtimer_read_tsy();
        if (hwtimer_tmp > tmr.count_frame) {
          digitalWrite(PIN_ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
          // シリアルに遅延msを表示
          mrd_disp.frame_delay(hwtimer_tmp - tmr.count_frame, monitor.frame_delay);
        }
        if (hwtimer_tmp >= tmr.count_frame) {
          break;
        }
        delay(1);
      }
    }

    // @[15-2] 必要に応じてフレームの遅延累積時間をリセット
    if (flg.count_frame_reset) {

      __disable_irq(); // 割り込みを無効化
      tmr.count_frame = mrd_hwtimer_read_tsy() + 1;
      __enable_irq(); // 割り込みを再度有効化

      Serial.println("Reset frame counter.");
      flg.count_frame_reset = false;
    }
    mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
    flg.spi_rcvd = false;                       // SPI受信フラグをサゲる
  }
}

//================================================================================================================
//  関 数 各 種
//================================================================================================================

/// @brief エラーカウントをインクリメントする関数.
/// 特定のビット位置に基づいて各種エラーをチェックし, 対応するエラーカウンターを増加させます.
/// @return なし.
void mrd_countup_errs() {
  auto increment_err = [&](int bit_position, int &err_counter) {
    if ((r_spi_meridim.bval[MRD_ERR_u] >> bit_position) & 0b00000001) {
      err_counter++;
    }
  };
  increment_err(7, err.esp_pc);   // PCのESP32からのUDP受信エラー
  increment_err(6, err.pc_esp);   // ESP32のPCからのUDP受信エラー
  increment_err(5, err.esp_tsy);  // TeensyのESPからのSPI受信エラー
  increment_err(4, err.tsy_esp);  // ESP32のTeensyからのSPI受信エラー
  increment_err(2, err.esp_skip); // UDP→ESP受信のカウントのスキップ
  increment_err(1, err.tsy_skip); // ESP→Teensy受信のカウントのスキップ
  increment_err(0, err.pc_skip);  // PC受信のカウントのスキップ
}

#endif // __MERIDIAN_MAIN__
