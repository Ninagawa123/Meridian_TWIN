#ifndef __MERIDIAN_JOYPAD_H__
#define __MERIDIAN_JOYPAD_H__

// コンフィグファイルの読み込み
#include "config.h"

// ヘッダファイルの読み込み
#include "main.h"
#include "mrd_bt.h"

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

void mrd_bt_wiimote_receive(PadUnion &a_pad_array); // 関数の予約

//================================================================================================================
//  JOYPAD 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  各種パッドからの読み取り処理
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//  各種パッド読み取りへの分岐
//------------------------------------------------------------------------------------
/// @brief 指定されたジョイパッドタイプに応じてデータを読み取り, 関数外のmeridim配列に代入する.
/// @param a_pad_array pad値の格納用配列.
/// @param a_pad_type ジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
/// @param a_marge ボタン値をマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
/// @param a_monitor データをシリアルモニタに表示するかどうか.
/// @return ジョイパッドが未対応のタイプの場合はfalseを返し, それ以外はtrueを返す.
bool mrd_pad_reader(PadUnion &a_pad_array, PadReceiverType a_pad_type, bool a_marge, bool a_monitor) {
  if (a_pad_type == WIIMOTE) // Wiimote
  {
    mrd_bt_wiimote_receive(a_pad_array);
    if (MONITOR_PAD) {
      mrd.monitor_joypad(pad_array.usval);
    }
  } else {
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief meridim配列の適切な箇所にジョイパッド値を書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @param a_marge ボタン値をマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
/// @param a_pad ジョイパッド値の共用体.
/// @return 常にtrueを返す.
bool mrd_writedim90_pad(Meridim90Union &a_meridim, bool a_marge, PadUnion a_pad) {
  if (a_marge) { // ボタン値を受信Meridimとマージする
    a_meridim.usval[MRD_CONTROL_BUTTONS] = a_meridim.usval[MRD_CONTROL_BUTTONS] | a_pad.usval[0];
  } else { // ボタン値をマージしない（新しい値で更新する）
    a_meridim.usval[MRD_CONTROL_BUTTONS] = a_pad.usval[0];
  }

  for (int i = 1; i < 4; i++) { // アナログ値の転記
    a_meridim.usval[i + MRD_CONTROL_BUTTONS] = a_pad.usval[i];
  }
  return true;
}

#endif // __MERIDIAN_JOYPAD_H__
