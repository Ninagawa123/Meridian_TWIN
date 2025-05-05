#ifndef __MERIDIAN_UTILITY_H__
#define __MERIDIAN_UTILITY_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

//================================================================================================================
// Utility ごく小規模な汎用関数
//================================================================================================================

/// @brief 配列の中で0以外が入っている最大のIndexを求める.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex. すべて0の場合は1を反す.
int mrd_max_used_index(const int a_arr[], int a_size) {
  int max_index_tmp = 0;
  for (int i = 0; i < a_size; ++i) {
    if (a_arr[i] != 0) {
      max_index_tmp = i;
    }
  }
  return max_index_tmp + 1;
}

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
inline void mrd_set_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
inline void mrd_clear_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

/// @brief 指定された位置のビットをセットする(8ビット変数用).
/// @param value ビットをセットする8ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から7).
/// @return なし.
inline void mrd_set_bit8(uint8_t &value, uint8_t a_bit_pos) { value |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(8ビット変数用).
/// @param value ビットをクリアする8ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から7).
/// @return なし.
inline void mrd_clear_bit8(uint8_t &value, uint8_t a_bit_pos) { value &= ~(1 << a_bit_pos); }

/// @brief 任意の整数値から、任意幅のビット列を取り出す汎用関数.
/// @tparam T 型テンプレート.任意の整数型(符号付き・符号無しどちらでも可)
/// @param value 抽出元となる値.
/// @param pos   取り出し開始位置(LSB＝0, 右から数え, 最初は0番)
/// @param len   取り出すビット幅
/// @return unsigned 取り出したビット列(0〜 2^len−1 の範囲)
template <class T> // 型テンプレート
unsigned mrd_slice_bits(T value, unsigned pos, unsigned len) {
  return (static_cast<unsigned>(value) >> pos) & ((1u << len) - 1u);
}

/// @brief 任意の整数値の指定範囲のビット列に値を設定する汎用関数.
/// @tparam T 型テンプレート.任意の整数型(符号付き・符号無しどちらでも可)
/// @tparam U 設定する値の型(通常は整数型)
/// @param value 設定先となる値.
/// @param pos 設定開始位置(LSB＝0, 右から数え, 最初は0番)
/// @param len 設定するビット幅
/// @param new_value 設定したい値(0〜 2^len−1 の範囲)
/// @return T 設定後の値
template <class T>
T mrd_set_bits(T value, unsigned pos, unsigned len, unsigned new_value) {
  unsigned mask = ((1u << len) - 1u);                         // マスクを作成.指定範囲のビットを1で埋める
  unsigned masked_new_value = new_value & mask;               // マスクして範囲内に収める
  T cleared = value & ~(static_cast<T>(mask) << pos);         // 既存の値から指定範囲のビットをクリア
  return cleared | (static_cast<T>(masked_new_value) << pos); // 新しい値を設定
}

/// @brief 列挙型(L,R,C)から文字列を取得する関数.
/// @param a_line 列挙型 enum UartLine
/// @return 列挙型の内容に応じて文字列"L","R","C"返す.
const char *mrd_get_line_name(UartLine a_line) {
  switch (a_line) {
  case L:
    return "L";
  case R:
    return "R";
  case C:
    return "C";
  default:
    return "Unknown";
  }
}

/// @brief 数値をシリアルモニタ表示するときにパディングする.
/// @param num 表示したい値.
/// @param total_width 桁数.
/// @param rac_Width 小数点以下の桁数(0ならば小数点以下非表示).
/// @param show_plus +記号の有無.
/// @return 整形されたストリング.
String mrd_pddstr(float num, int total_width, int frac_width, bool show_plus = true) {
  char buf[30];
  char sign = (num < 0) ? '-' : (show_plus ? '+' : '\0');
  if (num < 0)
    num = -num;

  // 小数あり/なし
  if (frac_width) {
    if (sign)
      sprintf(buf, "%c%d.%0*d", sign, (int)num, frac_width, (int)((num - (int)num) * pow(10, frac_width) + 0.5));
    else
      sprintf(buf, "%d.%0*d", (int)num, frac_width, (int)((num - (int)num) * pow(10, frac_width) + 0.5));
  } else {
    if (sign)
      sprintf(buf, "%c%d", sign, (int)(num + 0.5));
    else
      sprintf(buf, "%d", (int)(num + 0.5));
  }

  // パディング
  String result = "";
  int pad = total_width - strlen(buf);
  for (int i = 0; i < pad; i++)
    result += ' ';
  result += buf;
  return result;
}

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------
// タイマーの開始状態を保持するための静的変数
static unsigned long timeout_start = 0;
static bool flg_timer_started = false; // タイマーが開始されたかどうかのフラグ

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit) {
  // タイマーが開始されていない場合は現在の時間を記録してタイマーを開始
  if (!flg_timer_started) {
    timeout_start = millis();
    flg_timer_started = true; // タイムアウト監視開始フラグをアゲる
  }

  unsigned long current_time = millis(); // 現在の時間を取得

  if (current_time - timeout_start >= a_limit) { // 指定された時間が経過しているかチェック
    flg_timer_started = false;                   // タイムアウト監視開始フラグをサゲる
    return true;                                 // 指定された時間が経過していれば true を返す
  }

  return false; // まだ時間が経過していなければ false を返す
}

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset() {
  flg_timer_started = false; // タイマーをリセットして次回の呼び出しに備える
}

//------------------------------------------------------------------------------------
//  meriput / meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_meriput90_cksm(Meridim90Union &a_meridim, int len = 90) {
  int a_cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    a_cksm += int(a_meridim.sval[i]);
  }
  a_meridim.sval[len - 1] = short(~a_cksm);
  return true;
}

#endif //__MERIDIAN_UTILITY_H__
