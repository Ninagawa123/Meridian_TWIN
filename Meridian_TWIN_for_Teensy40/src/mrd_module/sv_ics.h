#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "main.h"
#include "config.h"

//================================================================================================================
//---- KONDO ICSサーボ関連の処理  -----------------------------------------------------------------------------------
//================================================================================================================

void mrd_sv_drive_ics_double(Meridim90Union mrd_s_meridim)
{
    for (int i = 0; i < sv.num_max; i++) // ICS_L系統の処理
    {                                    // 接続したサーボの数だけ繰り返す. 最大は15
        int k = 0;
        if (sv.idl_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = ics_L.setPos(i, mrd.Deg2Krs(sv.idl_tgt[i], sv.idl_trim[i], sv.idl_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.idl_tgt_past[i], sv.idl_trim[i], sv.idl_cw[i]);
                    if (sv.idl_err[i] < 100)
                    {
                        sv.idl_err[i]++;
                    }
                    if (sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.ubval[MSG_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで）
                        mrd_msg_servo_error(L, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idl_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得. 手持ちの最大は15
            {
                k = ics_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
                if (k == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.idl_tgt_past[i], sv.idl_trim[i], sv.idl_cw[i]);
                    sv.idl_err[i]++;
                    if (sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.ubval[MSG_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで）
                        mrd_msg_servo_error(L, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idl_err[i] = 0;
                }
            }
            sv.idl_tgt[i] = mrd.Krs2Deg(k, sv.idl_trim[i], sv.idl_cw[i]);
        }
        delayMicroseconds(2);

        if (sv.idr_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = ics_R.setPos(i, mrd.Deg2Krs(sv.idr_tgt[i], sv.idr_trim[i], sv.idr_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.idr_tgt_past[i], sv.idr_trim[i], sv.idr_cw[i]);
                    if (sv.idr_err[i] < 10)
                    {
                        sv.idr_err[i]++;
                    }
                    if (sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.ubval[MSG_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); // Meridim[MSG_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで）
                        mrd_msg_servo_error(R, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idr_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得
            {
                k = ics_R.setFree(i);
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.idr_tgt_past[i], sv.idr_trim[i], sv.idr_cw[i]);
                    sv.idr_err[i]++;
                    if (sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_spi_meridim.ubval[MSG_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); // Meridim[MSG_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで）
                        mrd_msg_servo_error(R, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.idr_err[i] = 0;
                }
            }
            sv.idr_tgt[i] = mrd.Krs2Deg(k, sv.idr_trim[i], sv.idr_cw[i]);
        }
        delayMicroseconds(2);
    }
}

#endif
