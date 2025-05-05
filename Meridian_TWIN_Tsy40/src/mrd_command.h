#ifndef __MERIDIAN_COMMAND_H__
#define __MERIDIAN_COMMAND_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include "mrd_eeprom.h"
#include "mrd_servo.h"
#include "mrd_wire0.h"

//==================================================================================================
//  コマンド処理
//==================================================================================================

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, Stream &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    a_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) {
      a_sv.ixl_err[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      a_sv.ixr_err[i] = 0;
    }
    String msg_tmp = "cmd: reset servo error id.[" + String(MCMD_ERR_CLEAR_SERVO_ID) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に(デフォルト)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROMの書き込みモードスタート
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    flg.eeprom_write_mode = true; // 書き込みモードのフラグをセット
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_SAVE_TRIM (10101) EEPROMに現在のサーボ値をTRIM値として書き込む
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_SAVE_TRIM) {
    String msg_tmp = "cmd: set EEPROM data from current trim.[" + String(MCMD_EEPROM_SAVE_TRIM) + "]";
    a_serial.println(msg_tmp);

    // 空のUnionEEPROM構造体を作成し初期化
    UnionEEPROM array_tmp = {0};

    // サーボの設定情報をsaval[1]にコピー
    for (int i = 0; i < a_sv.num_max; i++) {
      array_tmp.saval[1][20 + i * 2] = a_meridim.sval[20 + i * 2];
      array_tmp.saval[1][21 + i * 2] = a_meridim.sval[21 + i * 2];
      array_tmp.saval[1][50 + i * 2] = a_meridim.sval[50 + i * 2]; // - int(sv.ixl_trim[i] * 100);
      array_tmp.saval[1][51 + i * 2] = a_meridim.sval[51 + i * 2]; // - int(sv.ixr_trim[i] * 100);
    }

    // 書き込みデータの作成と書き込み
    if (mrd_eeprom_write(array_tmp, EEPROM_PROTECT, a_serial)) {
      a_serial.println("write EEPROM succeed.");
    } else {
      a_serial.println("write EEPROM failed.");
      return false;
    }
    return true;
  }

  // コマンド:MCMD_EEPROM_LOAD_TRIM (10102) EEPROMからTRIM値を読み込んで設定
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_LOAD_TRIM) {
    // EEPROMのデータを展開する
    mrd_eeprom_load_servosettings(a_sv, true, a_serial);

    // サーボをEEPROMのTRIM値で補正されたHOME(原点)に移動する
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統の目標値を原点に
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統の目標値を原点に
      a_sv.ixl_tgt[i] = 0;                           //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ動作を実行する
    if (!MODE_TSY_STANDALONE) {
      mrd_servos_drive(a_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, MOUNT_SERVO_TYPE_C, a_sv);
    }

    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, Stream &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servos_all_off(a_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    ahrs.yaw_origin = mrd_wire0_setyaw(ahrs.ypr[0]);
    String msg_tmp = "cmd: caliblate sensor's yaw.[" + String(MCMD_SENSOR_YAW_CALIB) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に(SSH的な動作)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    String msg_tmp = "cmd: enter passive mode.[" + String(MCMD_BOARD_TRANSMIT_PASSIVE) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET) (10007) フレームカウンタを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_BOARD_STOP_DURING (10008) ボードの末端処理を指定時間だけ止める.
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_STOP_DURING) {
    flg.stop_board_during = true; // ボードの処理停止フラグをセット
    // ボードの末端処理をmeridim[2]ミリ秒だけ止める.

    String msg_tmp = "cmd: stop ESP32's processing during " + String(int(a_meridim.sval[MRD_STOP_FRAMES])) + " ms.[" + String(MCMD_BOARD_TRANSMIT_PASSIVE) + "]";
    Serial.println(msg_tmp);

    for (int i = 0; i < int(a_meridim.sval[MRD_STOP_FRAMES]); i++) {
      delay(1);
    }
    flg.stop_board_during = false; // ボードの処理停止フラグをクリア
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief Master Commandの第3群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_3(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, bool &a_flg_special, Stream &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servos_all_off(a_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_START_TRIM_SETTING (10100) TRIM設定のスタート(Meridian_console連携)
  if (a_meridim.sval[MRD_MASTER] == MCMD_START_TRIM_SETTING) {

    // EEPROMのデータを展開する
    mrd_eeprom_load_servosettings(a_sv, true, a_serial);

    // サーボをEEPROMのTRIM値で補正されたHOME(原点)に移動する
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統の目標値を原点に
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統の目標値を原点に
      a_sv.ixl_tgt_past[i] = a_sv.ixl_tgt[i];        // 前回のdegreeをキープ
      a_sv.ixr_tgt_past[i] = a_sv.ixr_tgt[i];
      a_sv.ixl_tgt[i] = 0; //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ動作を実行する
    if (!MODE_TSY_STANDALONE) {
      mrd_servos_drive(a_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, MOUNT_SERVO_TYPE_C, a_sv);
    }

    // サーボの目標値として現在のTRIM値をセットする
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = a_sv.ixl_trim[i];
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = a_sv.ixr_trim[i];
    }

    // サーボのTRIM値をゼロリセットする
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_sv.ixl_trim[i] = 0;
      a_sv.ixr_trim[i] = 0;
    }

    // サーボ動作を実行する. サーボはTRIM値を0としつつ, tgtとしてこれまでのTRIM値の角度をキープする
    if (!MODE_TSY_STANDALONE) {
      mrd_servos_drive(a_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, MOUNT_SERVO_TYPE_C, a_sv); // サーボ動作を実行する
    }

    // サーボの設定値とTRIM値をPCに送信する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    a_serial.println("send:");
    for (int i = 0; i < MRDM_LEN; i++) {
      a_serial.print(a_meridim.sval[i]);
      a_serial.print(",");
    }
    a_serial.println();

    a_flg_special = true;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[1][*] to PC.[" + String(MCMD_START_TRIM_SETTING) + "]";
    Serial.println(msg_tmp);

    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10200) EEPROMの[0][*]をボードからPCにMeridimで送信する
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA0) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[0][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA0;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[0][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA0) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10201) EEPROMの[1][*]をボードからPCにMeridimで送信する
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA1) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[1][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA1) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10202) EEPROMの[2][*]をボードからPCにMeridimで送信する
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA2) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[2][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA2;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[2][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA2) + "]";
    Serial.println(msg_tmp);
    return true;
  }

  return false;
}

#endif // __MERIDIAN_COMMAND_H__