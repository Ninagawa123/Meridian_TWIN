#ifndef __MERIDIAN_WIRE1_H__
#define __MERIDIAN_WIRE1_H__

#include "config.h"
#include "main.h"

IntervalTimer wireTimer1;

//================================================================================================================
//---- I2C wire0 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  リモコンのデータ取得処理 
//------------------------------------------------------------------------------------

/// @brief MerimoteのI2C通信を通じてジョイパッドデータを読み取ります. 
/// @param mrd_pad_array 処理後のジョイパッドデータを格納する構造体です. 
/// @param mrd_pad_i2c I2C通信から読み取った生データを格納する構造体です. 
/// @return 正常にデータが読み取れた場合はtrueを, そうでない場合はfalseを返します. 
bool mrd_wire1_read_merimote_i2c(PadUnion &mrd_pad_array, PadUnionWire &mrd_pad_i2c)
{
    int i = 0;
    Wire1.requestFrom(I2C1_MERIMOTE_ADDR, JOYPAD_LEN * 2);
    while (Wire1.available())
    {                                       // バッファにデータがある間
        mrd_pad_i2c.bval[i] = Wire1.read(); // バッファから1バイト読み取り
        i++;
        if (i > JOYPAD_LEN * 2)
        {
            break;
        }
    }

    if (mrd.cksm_val(mrd_pad_i2c.sval, sizeof(mrd_pad_i2c.sval)))
    {
        for (int i = 0; i < JOYPAD_LEN; i++)
        {
            mrd_pad_array.sval[i] = mrd_pad_i2c.sval[i];
        }
        return true;
    }
    else
    {
        Serial.println("Merimote:NG");
        return false;
    }
    return false;
}

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief 指定されたジョイパッドタイプに基づいてWire1 I2C通信を設定します. 
/// @param mrd_mount_joypad ジョイパッドのタイプを示す列挙型です. Merimoteがサポートされています. 
/// @param mrd_i2c1_speed I2C通信のクロック速度です. 
/// @return ジョイパッドが正しく初期化され, データの読み取りに成功した場合はtrueを, それ以外の場合はfalseを返します. 
bool mrd_wire1_setup(PadReceiverType mrd_mount_joypad, int mrd_i2c1_speed)
{
    if (mrd_mount_joypad == MERIMOTE) // Merimoteを搭載
    {
        Serial.print("Initializing wire1 I2C...");
        Wire1.setSDA(17);
        Wire1.setSCL(16);
        Wire1.begin();
        Wire1.setClock(mrd_i2c1_speed);
    }
    else
    {
        return false;
    }
    bool rslt = mrd_wire1_read_merimote_i2c(pad_array, pad_i2c);
    if (rslt)
    {
        Serial.print("Merimote OK.");
    }
    return rslt;
}

//------------------------------------------------------------------------------------
//  IntervalTimerTimerのスタート
//------------------------------------------------------------------------------------

/// @brief MerimoteのデータをI2C経由で読み取る関数を実行します. 
void mrd_wire1_run() // ※IntervalTimer用の関数のためvoidのみ可
{
    mrd_wire1_read_merimote_i2c(pad_array, pad_i2c);
}

/// @brief インターバルタイマーを開始して, 定期的にMerimoteデータの読み取りを行います. 
/// @param mrd_pad_array ジョイパッドデータを格納する構造体です. 
/// @param mrd_pad_i2c I2C通信からの生データを格納する構造体です. 
/// @return タイマーの初期化が成功した場合はtrue, それ以外の場合はfalseを返します. 
bool mrd_wire1_startIntervalTimer(PadUnion mrd_pad_array, PadUnionWire mrd_pad_i2c)
{
    if (flg.wire1_init)
    {
        wireTimer1.begin(mrd_wire1_run, 10000); // インターバルはマイクロ秒指定
        // wireTimer.priority(180);
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif
