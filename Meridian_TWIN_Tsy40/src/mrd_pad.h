#ifndef __MERIDIAN_JOYPAD_H__
#define __MERIDIAN_JOYPAD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

//================================================================================================================
//  JOYPAD 関連の処理
//================================================================================================================

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = //
    {0,    64,   32,  128,  1,  4,  2,  8,                //
     1024, 4096, 512, 2048, 16, 64, 32, 256};             //

//------------------------------------------------------------------------------------
//  タイプ別のJOYPAD読み込み処理
//------------------------------------------------------------------------------------

//----------------------------------------------------------------------
// KRC-5FHの読み込み
//----------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
/// @param a_interval 読み取り間隔（ミリ秒）.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
uint64_t mrd_pad_read_krc(uint a_interval, IcsHardSerialClass &a_ics) {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  int8_t pad_analog_tmp[4] = {0};  // アナログ入力のデータ組み立て用
  //static int calib[4] = {0};       // アナログスティックのキャリブレーション値

  static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= a_interval) {
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機がデータを受信成功したか
    rcvd_tmp = ics_R.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);

    if (rcvd_tmp) // リモコンデータが受信できていたら
    {
      // ボタンデータの処理
      int button_tmp = krr_button_tmp; // 受信ボタンデータの読み込み用

      if (PAD_GENERALIZE) {            // ボタンデータの一般化処理
        if ((button_tmp & 15) == 15) { // 左側十字ボタン全部押しなら select押下とみなす
          pad_common_tmp += 1;
          button_tmp &= 0b1111111111110000; // 左十字ボタンのクリア
        }

        if ((button_tmp & 368) == 368) {
          pad_common_tmp += 8; // 右側十字ボタン全部押しなら start押下とみなす
          button_tmp &= 0b1111111010001111; // 右十字ボタンのクリア
        }

        // ボタン値の変換(一般化)
        for (int i = 0; i < 16; i++) {
          uint16_t mask_tmp = 1 << i;
          if (PAD_TABLE_KRC5FH_TO_COMMON[i] & button_tmp) {
            pad_common_tmp |= mask_tmp;
          }
        }
        pad_common_tmp &= 0b11111111111111001; // 2と4のビットのクリア(謎のデータ調整)

        // アナログ入力データの処理
        if (krr_analog_tmp[0] + krr_analog_tmp[1] + krr_analog_tmp[2] + krr_analog_tmp[3]) {
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = (krr_analog_tmp[i] - 62) << 2;
            pad_analog_tmp[i] = (pad_analog_tmp[i] < -127) ? -127 : pad_analog_tmp[i];
            pad_analog_tmp[i] = (pad_analog_tmp[i] > 127) ? 127 : pad_analog_tmp[i];
          }
        } else
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = 0;
          }
      } else {
        pad_common_tmp = button_tmp; // ボタンの変換なし生値を使用
      }
    }

    // アナログスティックのキャリブレーション
    // [WIP]

    // データの組み立て
    uint64_t updated_val_tmp = 0;
    updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 16;
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 24;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[2] & 0xFF) << 32;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[3] & 0xFF) << 40;

    last_time_tmp = current_time_tmp; // 最後の実行時間を更新
    pre_val_tmp = updated_val_tmp;
    return updated_val_tmp;
  }
  return pre_val_tmp;
}

//------------------------------------------------------------------------------------
//  各種パッド読み取りへの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたジョイパッドタイプに応じてジョイパッドのデータを読み取る.
/// @param a_type 使用するジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
/// @param a_interval ジョイパッドのデータ読み取り間隔（ミリ秒）.
/// @return PAD受信値を共用体(PadUnion)データで返す.
/// @note 関数内で外部変数ics_Rを使用.
PadUnion mrd_pad_reader(PadType a_type, uint a_interval) {
  PadUnion pad_array_tmp = {0};
  if (a_type == KRR5FH) // KRC-5FH+KRR-5FH
  {
    pad_array_tmp.ui64val[0] = mrd_pad_read_krc(a_interval, ics_R);
    return pad_array_tmp;
  } else if (a_type == MERIMOTE) // merimote
  {
    for (int i = 0; i < 4; i++) {
      pad_array_tmp.sval[i] = pad_array.sval[i];
    }
  } else if (a_type == BLUERETRO) // BLUERETRO(未実装)
  {
    return pad_array_tmp;
  } else if (a_type == SBDBT) // SBDBT(未実装)
  {
    return pad_array_tmp;
  }
  return pad_array_tmp;
}

//------------------------------------------------------------------------------------
//  meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列にPADデータを書き込む.
/// @param a_type 使用するジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @param a_pad_array PAD受信値の格納用配列.
/// @param a_marge PADボタンデータをマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
bool mrd_meriput90_pad(PadType a_type, Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge) {

  if (a_type == PC) {
    return false;
  }

  // ボタンデータの処理 (マージ or 上書き)
  if (a_marge) {
    a_meridim.usval[MRD_PAD_BUTTONS] |= a_pad_array.usval[0];
  } else {
    a_meridim.usval[MRD_PAD_BUTTONS] = a_pad_array.usval[0];
  }

  // アナログ入力データの処理 (上書きのみ)
  for (int i = 1; i < 4; i++) {
    a_meridim.usval[MRD_PAD_BUTTONS + i] = a_pad_array.usval[i];
  }
  return true;
}

#endif // __MERIDIAN_JOYPAD_H__
