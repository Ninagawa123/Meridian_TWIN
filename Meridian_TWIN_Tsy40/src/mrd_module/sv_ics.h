#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "config.h"
#include "main.h"
#include "mrd_disp.h"
#include "gs2d_krs.h"

//================================================================================================================
//  KONDO ICSサーボ関連の処理
//================================================================================================================

/// @brief ICSサーボの実行処理を行う関数
/// @param a_servo_id サーボのインデックス番号
/// @param a_cmnd サーボのコマンド
/// @param a_tgt サーボの目標位置
/// @param a_tgt_past 前回のサーボの目標位置
/// @param a_tgt_trim サーボの補正値
/// @param a_cw サーボの回転方向補正値
/// @param a_err_cnt サーボのエラーカウント
/// @param a_stat サーボのステータス
/// @param ics サーボクラスのインスタンス
float mrd_servo_process_ics(int a_servo_id, int a_cmnd, float a_tgt, float a_tgt_past, int a_trim,
                            int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsHardSerialClass &ics) {
  int val_tmp = 0;
  if (a_cmnd == 1) { // コマンドが1ならPos指定
    val_tmp = ics.setPos(a_servo_id, mrd.Deg2Krs(a_tgt, a_trim, a_cw));
  } else { // コマンドが0等なら脱力して値を取得
    val_tmp = ics.setFree(a_servo_id);
  }

  if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
    val_tmp = mrd.Deg2Krs(a_tgt_past, a_trim, a_cw);
    a_err_cnt++;
    if (a_err_cnt >= SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
      a_err_cnt = SERVO_LOST_ERR_WAIT;
      a_stat = 1;
    }
  } else {
    a_err_cnt = 0;
    a_stat = 0;
  }

  return mrd.Krs2Deg(val_tmp, a_trim, a_cw);
}

/// @brief ICSサーボを駆動する関数
/// @param a_meridim Meridimデータの参照
/// @param a_sv サーボパラメータの配列
void mrd_servo_drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv) {
  for (int i = 0; i < a_sv.num_max; i++) {
    // L系統サーボの処理
    if (a_sv.ixl_mount[i]) {
      a_sv.ixl_tgt[i] = mrd_servo_process_ics(
          a_sv.ixl_id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl_tgt[i], a_sv.ixl_tgt_past[i],
          a_sv.ixl_trim[i], a_sv.ixl_cw[i], a_sv.ixl_err[i], a_sv.ixl_stat[i], ics_L);
    }
    // R系統サーボの処理
    if (a_sv.ixr_mount[i]) {
      a_sv.ixr_tgt[i] = mrd_servo_process_ics(
          a_sv.ixr_id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr_tgt[i], a_sv.ixr_tgt_past[i],
          a_sv.ixr_trim[i], a_sv.ixr_cw[i], a_sv.ixr_err[i], a_sv.ixr_stat[i], ics_R);
    }
    delayMicroseconds(1); //Teensyの場合には必要かも
  }
}

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__