#ifndef __MERIDIAN_JOYPAD_H__
#define __MERIDIAN_JOYPAD_H__

#include "config.h"
#include "main.h"

//================================================================================================================
//---- JOYPAD 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  各種パッドからの読み取り処理
//------------------------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新します. 
/// @param pre_val 前回のジョイパッドの状態を保持する64ビット整数. 
/// @param mrd_pad_interval 読み取り間隔（ミリ秒）. 
/// @param mrd_pad_marge マージフラグ. trueの場合, 古いデータと新しいデータをマージします. 
/// @return 更新されたジョイパッドの状態を64ビット整数で返します. 
uint64_t mrd_pad_read_krc(uint64_t pre_val, uint mrd_pad_interval, bool mrd_pad_marge)
{
    // KRR5FH(KRC-5FH)をICS_R系に接続している場合
    static unsigned long last_time = 0; // 最後に関数が呼ばれた時間を記録
    unsigned long current_time = millis();

    if (current_time - last_time >= mrd_pad_interval)
    {
        static bool isFirstCall = true; // 初回の呼び出しフラグ
        unsigned short buttonData;
        unsigned short pad_btn_tmp = 0;
        bool ret;
        int joy_a[4];
        ret = ics_R.getKrrAllData(&buttonData, joy_a);
        // buttonData = ics_R.getKrrButton();
        delayMicroseconds(2);
        // if (buttonData != KRR_BUTTON_FALSE) // ボタンデータが受信できていたら
        if (ret) // ボタンデータが受信できていたら

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

                int8_t pad_analog[4];
                if (joy_a[0] + joy_a[1] + joy_a[2] + joy_a[3])
                {
                    for (int i = 0; i < 4; i++)
                    {
                        pad_analog[i] = (joy_a[i] - 62) << 2;
                        pad_analog[i] = (pad_analog[i] < -127) ? -127 : pad_analog[i];
                        pad_analog[i] = (pad_analog[i] > 127) ? 127 : pad_analog[i];
                    }
                }
                else
                    for (int i = 0; i < 4; i++)
                    {
                        pad_analog[i] = 0;
                    }
            }
            else
            {
                pad_btn_tmp = button_1;
            }

            if (isFirstCall)
            {
                Serial.println("KRC-5FH successfully connected. ");
                isFirstCall = false; // 初回の呼び出しフラグをオフにする
            }
        }
        // 共用体用の64ビットの上位16ビット部をボタンデータとして書き換える
        uint64_t updated_val;
        if (mrd_pad_marge)
        {
            updated_val = (pre_val & 0xFFFF000000000000) | (static_cast<uint64_t>(pad_btn_tmp)); // 上位16ビット index[0]
        }
        else
        {
            updated_val = (pre_val) | (static_cast<uint64_t>(pad_btn_tmp));
        }
        // updated_val = (updated_val & 0x0000FFFFFFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 48); // 下位16ビット index[3]
        // updated_val = (updated_val & 0xFFFF0000FFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 32); // 上位33-48ビット index[2]
        // updated_val = (updated_val & 0xFFFFFFFF0000FFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 16); // 上位17-32ビット index[1]
        // tmr.JOYPAD_INTERVAL_count = 0;
        last_time = current_time; // 最後の実行時間を更新
        return updated_val;
    }
    return pre_val;
}

//------------------------------------------------------------------------------------
//  各種パッド読み取りへの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたジョイパッドタイプに応じてジョイパッドのデータを読み取り, データを管理します. 
/// @param mrd_pad_type 使用するジョイパッドのタイプを示す列挙型です（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）. 
/// @param mrd_pad_interval ジョイパッドのデータ読み取り間隔（ミリ秒）. 
/// @param mrd_pad_marge データをマージするかどうかのブール値. trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きします. 
/// @return ジョイパッドの接続がプログラムされていないタイプの場合はfalseを返し, それ以外の場合はtrueを返します. 
bool mrd_pad_reader(PadReceiverType mrd_pad_type, uint mrd_pad_interval, bool mrd_pad_marge)
{
    if (mrd_pad_type == MERIMOTE) // merimote
    {
        for (int i = 0; i < 4; i++)
        {
            if (mrd_pad_marge)
            {
                r_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] |= pad_array.sval[i];
                s_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] |= pad_array.sval[i];
            }
            else
            {
                r_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
                s_spi_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
            }
        }
    }
    else if (mrd_pad_type == BLUERETRO) // merimote
    {
        Serial.print("BLUERETRO connection has not been programmed yet.");
        return false;
    }
    else if (mrd_pad_type == SBDBT) // SBDBT
    {
        Serial.print("SBDBT connection has not been programmed yet.");
        return false;
    }
    else if (mrd_pad_type == KRR5FH) // KRC-5FH+KRR-5FH
    {
        pad_array.ui64val[0] = mrd_pad_read_krc(pad_array.ui64val[0], mrd_pad_interval, mrd_pad_marge);
        r_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
        s_spi_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
    }
    else
    {
        pad_array.usval[0] = r_spi_meridim.sval[15]; // をセットする
    }

    if (MONITOR_JOYPAD)
    {
        mrd.monitor_joypad(pad_array.usval);
    }
    return true;
}


//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------


#endif
