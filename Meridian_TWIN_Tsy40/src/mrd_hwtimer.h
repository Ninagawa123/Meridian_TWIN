#ifndef __MERIDIAN_HARDWARE_TIMER_H__
#define __MERIDIAN_HARDWARE_TIMER_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

//================================================================================================================
//  HardwareTimer関連の処理
//================================================================================================================
// グローバル変数の宣言
volatile unsigned long hwtimer_counter = 0; // ハードウェアタイマーのフレーム管理用カウント

// ハードウェアタイマーの定義
IntervalTimer hwtimer;

/// @brief hwtimer_counterを保護しつつ1ずつインクリメント
inline void hwtimer_tick() {
  __disable_irq(); // 割り込みを無効化
  hwtimer_counter++;
  __enable_irq(); // 割り込みを再度有効化
}

/// @brief 指定msでカウントアップするハードウェアタイマーを開始
/// @param a_frame_duration ハードウェアタイマーをカウントアップする間隔(ms)
inline void mrd_hwtimer_start_tsy(int a_frame_duration) {
  hwtimer.begin(hwtimer_tick, a_frame_duration * 1000); // タイマー周期をマイクロ秒で指定
}

/// @brief count_timerの安全な読み取り関数
inline unsigned long mrd_hwtimer_read_tsy() {
  unsigned long timer_tmp = 0;
  __disable_irq(); // 割り込みを無効化
  timer_tmp = hwtimer_counter;
  __enable_irq(); // 割り込みを再度有効化
  return timer_tmp;
}

#endif // __MERIDIAN_HARDWARE_TIMER_H__
