#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "main.h"
#include "config.h"

//================================================================================================================
//  KONDO ICSサーボ関連の処理
//================================================================================================================

void mrd_sv_drive_ics_double(Meridim90Union mrd_s_meridim)
{
    for (int i = 0; i < sv.num_max; i++) // ICS_L系統の処理
    {                                    // 接続したサーボの数だけ繰り返す. 最大は15
        int k = 0;
        if (sv.ixl_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = ics_L.setPos(sv.ixl_id[i], mrd.Deg2Krs(sv.ixl_tgt[i], sv.ixl_trim[i], sv.ixl_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.ixl_tgt_past[i], sv.ixl_trim[i], sv.ixl_cw[i]);
                    if (sv.ixl_err[i] < 100)
                    {
                        sv.ixl_err[i]++;
                    }
                    if (sv.ixl_err[i] >= SERVO_LOST_ERR_WAIT)
                    {
                        s_spi_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); // Meridim[MRD_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで）
                        mrd_disp.servo_err(L, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.ixl_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得. 手持ちの最大は15
            {
                k = ics_L.setFree(sv.ixl_id[i]); // サーボからの返信信号を受け取れていれば値を更新
                if (k == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.ixl_tgt_past[i], sv.ixl_trim[i], sv.ixl_cw[i]);
                    sv.ixl_err[i]++;
                    if (sv.ixl_err[i] >= SERVO_LOST_ERR_WAIT)
                    {
                        s_spi_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); // Meridim[MRD_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで）
                        mrd_disp.servo_err(L, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.ixl_err[i] = 0;
                }
            }
            sv.ixl_tgt[i] = mrd.Krs2Deg(k, sv.ixl_trim[i], sv.ixl_cw[i]);
        }
        delayMicroseconds(2);

        if (sv.ixr_mount[i])
        {
            if (r_spi_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = ics_R.setPos(sv.ixr_id[i], mrd.Deg2Krs(sv.ixr_tgt[i], sv.ixr_trim[i], sv.ixr_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.ixr_tgt_past[i], sv.ixr_trim[i], sv.ixr_cw[i]);
                    if (sv.ixr_err[i] < 10)
                    {
                        sv.ixr_err[i]++;
                    }
                    if (sv.ixr_err[i] >= SERVO_LOST_ERR_WAIT)
                    {
                        s_spi_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); // Meridim[MRD_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで）
                        mrd_disp.servo_err(R, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.ixr_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得
            {
                k = ics_R.setFree(sv.ixr_id[i]);
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(sv.ixr_tgt_past[i], sv.ixr_trim[i], sv.ixr_cw[i]);
                    sv.ixr_err[i]++;
                    if (sv.ixr_err[i] >= SERVO_LOST_ERR_WAIT)
                    {
                        s_spi_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); // Meridim[MRD_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで）
                        mrd_disp.servo_err(R, i, MONITOR_SERVO_ERR);
                    }
                }
                else
                {
                    sv.ixr_err[i] = 0;
                }
            }
            sv.ixr_tgt[i] = mrd.Krs2Deg(k, sv.ixr_trim[i], sv.ixr_cw[i]);
        }
        delayMicroseconds(2);
    }
}

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
