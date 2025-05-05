#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_module/sv_dxl2.h"
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_ftc.h"
#include "mrd_module/sv_ics.h"
#include "mrd_module/sv_xbus.h"

//================================================================================================================
//  Servo 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定する.
/// @param a_line UART通信ライン（L, R, またはC）.
/// @param mrd_servo_type サーボのタイプを示す整数値.
/// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
bool mrd_servos_begin(UartLine a_line, int mrd_servo_type) {
  if (mrd_servo_type > 0) {
    switch (mrd_servo_type) {
    case 1: // single PWM
      break;
    case 11: // I2C_PCA9685 to PWM
      break;
    case 21: // RSxTTL (FUTABA)
      break;
    case 31: // DYNAMIXEL Protocol 1.0
      break;
    case 32: // DYNAMIXEL Protocol 2.0
      break;
    case 43: // ICS3.5/3.6(KONDO,KRS)
      if (a_line == L)
        ics_L.begin();
      else if (a_line == R)
        ics_R.begin();
      else if (a_line == C)
        ics_C.begin();
      break;
    case 44: // PMX(KONDO)
      break;
    case 51: // XBUS(JR PROPO)
      break;
    case 61: // STS(FEETECH)
      break;
    case 62: // SCS(FEETECH)
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  サーボ通信フォーメーションの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボにコマンドを分配する.
/// @param a_meridim meridim配列.
/// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_servos_drive(Meridim90Union a_meridim, int sv_L_type, int sv_R_type, int sv_C_type, ServoParam &a_sv) {
  if (sv_L_type == 43 && sv_R_type == 43) // ICSサーボがL系R系に設定されていた場合はLR均等送信を実行
  {
    mrd_servos_drive_ics_double(a_meridim, a_sv);
    return true;
  } else {
    return false;
  }
}

//------------------------------------------------------------------------------------
//  EEPROM等から読み込んだサーボパラメータの適用
//------------------------------------------------------------------------------------
/// @brief EEPROM等に格納されているデータを読み出し、各サーボの設定に割り当てる.
/// @param a_eeprom_data EEPROMから読み出したデータを格納したUnionEEPROM配列.
/// @param a_sv サーボパラメータの構造体.
/// @return サーボの設定を反映したUnionEEPROM配列を返す.
void mrd_servos_load_param_lite(UnionEEPROM a_eeprom_data, ServoParam &a_sv) {
  for (int i = 0; i < 15; i++) {
    // 左サーボの情報を読み出し
    a_sv.ixl_mount[i] = (a_eeprom_data.usaval[1][20 + i * 2] >> 15) & 0x01;
    a_sv.ixl_id[i] = (a_eeprom_data.usaval[1][20 + i * 2] >> 8) & 0x7F;
    a_sv.ixl_cw[i] = (a_eeprom_data.usaval[1][20 + i * 2] & (1 << 8)) ? -1 : 1;
    a_sv.ixl_trim[i] = mrd.HfShort2float(short(a_eeprom_data.saval[1][21 + i * 2]));

    // 右サーボの情報を読み出し
    a_sv.ixr_mount[i] = (a_eeprom_data.usaval[1][50 + i * 2] >> 15) & 0x01;
    a_sv.ixr_id[i] = (a_eeprom_data.usaval[1][50 + i * 2] >> 8) & 0x7F;
    a_sv.ixr_cw[i] = (a_eeprom_data.usaval[1][50 + i * 2] & (1 << 8)) ? -1 : 1;
    a_sv.ixr_trim[i] = mrd.HfShort2float(short(a_eeprom_data.saval[1][51 + i * 2]));
  }
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
/// @param a_sv サーボパラメータの構造体.
/// @return uint8_tで番号を返す.
///         100-149(L系統 0-49),200-249(R系統 0-49),150-199(C系統 0-49)
uint8_t mrd_servos_make_errcode_3lines(ServoParam a_sv) {
  uint8_t servo_ix_tmp = 0;
  for (int i = 0; i < 15; i++) {
    if (a_sv.ixl_stat[i]) {
      servo_ix_tmp = uint8_t(i + 100);
    }
    if (a_sv.ixr_stat[i]) {
      servo_ix_tmp = uint8_t(i + 200);
    }
    if (a_sv.ixc_stat[i]) {
      servo_ix_tmp = uint8_t(i + 150);
    }
  }
  return servo_ix_tmp;
}

/// @brief すべてのサーボモーターをオフ（フリー状態）に設定する.
bool mrd_servos_all_off(Meridim90Union a_meridim) {
  for (int i = 0; i < 15; i++) {
    a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
    a_meridim.sval[i * 2 + 50] = 0; //
  }
  Serial.println("All servos off.");
  return true;
}

#endif //__MERIDIAN_SERVO_DISTRIBUTOR_H__
