#ifndef __MERIDIAN_WIRE1_H__
#define __MERIDIAN_WIRE1_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

IntervalTimer wireTimer1;

//================================================================================================================
//  I2C wire0 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  リモコンのデータ取得処理
//------------------------------------------------------------------------------------

/// @brief MerimoteのI2C通信を通じてジョイパッドデータを読み取る.
/// @param a_pad_array 処理後のジョイパッドデータを格納する構造体.
/// @param a_pad_i2c I2C通信から読み取った生データを格納する構造体.
/// @param a_len a_pad_i2cの長さ. デフォルトは5.
/// @return 正常にデータが読み取れた場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire1_read_merimote_i2c(PadUnion &a_pad_array, PadUnion &a_pad_i2c, int a_len) {
  int i_tmp = 0;
  Wire1.requestFrom(I2C1_MERIMOTE_ADDR, a_len * 2);
  while (Wire1.available()) {         // バッファにデータがある間
    a_pad_i2c.bval[i_tmp] = Wire1.read(); // バッファから1バイト読み取り
    i_tmp++;
    if (i_tmp > a_len * 2) {
      break;
    }
  }

  if (mrd.cksm_val(a_pad_i2c.sval, a_len)) {
    for (int i = 0; i < a_len; i++) {
      a_pad_array.sval[i] = a_pad_i2c.sval[i];
    }
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief 指定されたジョイパッドタイプに基づいてWire1 I2C通信を設定する.
/// @param a_type ジョイパッドのタイプを示す列挙型. Merimoteがサポートされている.
/// @param a_i2c_bps I2C通信のクロック速.
/// @param a_serial メッセージ表示用のハードウェアシリアル.
/// @return ジョイパッドが正しく初期化され, データの読み取りに成功した場合はtrueを,
/// それ以外の場合はfalseを返す.
bool mrd_wire1_setup(PadType a_type, int a_i2c_bps, Stream &a_serial) {
  if (a_type == MERIMOTE) // Merimoteを搭載
  {
    // Serial.print("Initializing wire1 I2C...");
    Wire1.setSDA(17);
    Wire1.setSCL(16);
    Wire1.begin();
    Wire1.setClock(a_i2c_bps);
  } else {
    return false;
  }
  bool rslt = mrd_wire1_read_merimote_i2c(pad_array, pad_i2c, PAD_LEN);
  if (rslt) {
    a_serial.print("Merimote OK.");
  }
  return rslt;
}

//------------------------------------------------------------------------------------
//  IntervalTimerTimerのスタート
//------------------------------------------------------------------------------------

/// @brief MerimoteのデータをI2C経由で読み取る関数を実行する.
void mrd_wire1_run() // ※IntervalTimer用の関数のためvoidのみ可
{
  mrd_wire1_read_merimote_i2c(pad_array, pad_i2c, PAD_LEN);
}

/// @brief インターバルタイマーを開始して, 定期的にMerimoteデータの読み取りを行う.
/// @param a_wire1_init 初期化の合否.
/// @return タイマーの初期化と開始が成功した場合はtrue, それ以外の場合はfalseを返す.
bool mrd_wire1_startIntervalTimer(bool a_wire1_init) {
  if (a_wire1_init) {
    wireTimer1.begin(mrd_wire1_run, 10000); // インターバルはマイクロ秒指定
    // wireTimer.priority(180);
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif // __MERIDIAN_WIRE1_H__
