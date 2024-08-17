#ifndef __MERIDIAN_BT_PAD_H__
#define __MERIDIAN_BT_PAD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー
ESP32Wiimote wiimote;

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

//================================================================================================================
//  JOYPAD 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  各種パッドからの読み取り処理
//------------------------------------------------------------------------------------

/// @brief Wiiリモコンからの入力データを受信し, 処理する.
/// @param a_pad_array pad値の格納用配列.
/// @param a_pad_type ジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
/// @param a_wiimote wiimoteのインスタンス.
/// @param a_marge ボタン値をマージするかどうかのブール値.
void mrd_bt_wiimote_receive(int a_type, PadUnion &a_pad_array, ESP32Wiimote a_wiimote) {
  static int calib_l1x = 0;
  static int calib_l1y = 0;
  a_wiimote.task();
  a_wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);
  flg.bt_busy = true;
  ButtonState button_tmp;
  // AccelState accel_tmp;
  NunchukState nunchuk_tmp;

  if (a_wiimote.available() > 0) {
    button_tmp = a_wiimote.getButtonState();
    nunchuk_tmp = a_wiimote.getNunchukState();

    a_pad_array.usval[0] = 0;

    // ボタン値の変換(一般化)
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_SOLO[i] & button_tmp)) ||
          (!PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_ORIG[i] & button_tmp))) {
        a_pad_array.usval[0] |= mask_tmp;
      }
    }

    if (button_tmp & BUTTON_C) {
      if (PAD_GENERALIZE) {
        a_pad_array.usval[0] |= 1024;
      } else {
        a_pad_array.usval[0] |= 8192;
      }
    }

    if (button_tmp & BUTTON_Z) {
      if (PAD_GENERALIZE) {
        a_pad_array.usval[0] |= 2048;
      } else {
        a_pad_array.usval[0] |= 16384;
      }
    }

    if (button_tmp & BUTTON_HOME) { // スティックのキャリブレーション
      calib_l1x = nunchuk_tmp.xStick - 127;
      calib_l1y = nunchuk_tmp.yStick - 127;
    }

    a_pad_array.usval[1] =
        ((nunchuk_tmp.xStick - calib_l1x - 127) * 256 + (nunchuk_tmp.yStick - calib_l1y - 127));
  }
  flg.bt_busy = false;
}

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
bool mrd_pad_reader(PadUnion &a_pad_array, PadType a_pad_type, bool a_marge, bool a_monitor) {
  if (a_pad_type == WIIMOTE) // Wiimote
  {
    mrd_bt_wiimote_receive(a_pad_type, a_pad_array, wiimote);
    if (a_monitor) {
      mrd.monitor_joypad(a_pad_array.usval);
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
    a_meridim.usval[MRD_PAD_BUTTONS] = a_meridim.usval[MRD_PAD_BUTTONS] | a_pad.usval[0];
  } else { // ボタン値をマージしない（新しい値で更新する）
    a_meridim.usval[MRD_PAD_BUTTONS] = a_pad.usval[0];
  }

  for (int i = 1; i < 4; i++) { // アナログ値の転記
    a_meridim.usval[i + MRD_PAD_BUTTONS] = a_pad.usval[i];
  }
  return true;
}

/// @brief Bluetoothの設定を行い, 条件に応じてWiiコントローラの接続を開始する.
bool mrd_bt_settings(int a_mount_pad, int a_timeout, ESP32Wiimote &a_wiimote, int a_led,
                     HardwareSerial &a_serial) {
  // Wiiコントローラの接続開始
  if (a_mount_pad == 5) {
    a_serial.println("Try to connect Wiimote Single...");
  } else if (a_mount_pad == 6) {
    a_serial.println("Try to connect Wiimote + Nunchuk...");
  }
  if ((a_mount_pad == 5) or (a_mount_pad == 6)) {
    a_wiimote.init();
    uint16_t count_tmp = 0;
    unsigned long start_time = millis();
    while (!a_wiimote.available()) {
      // タイムアウトチェック
      if (millis() - start_time >= a_timeout) {
        digitalWrite(a_led, LOW);
        a_serial.println("Wiimote connection timed out.");
        return false;
      }
      a_wiimote.task();
      count_tmp++;
      if (count_tmp < 500) {
        digitalWrite(a_led, HIGH);
      } else {
        digitalWrite(a_led, LOW);
      }
      if (count_tmp > 1000) {
        a_serial.print(".");
        count_tmp = 0;
      }
      delay(1); // 1ms秒待機して再チェック
    }
    digitalWrite(a_led, HIGH);
    a_serial.println("Wiimote successfully connected. ");
    return true;
  }
  digitalWrite(a_led, LOW);
  return false;
}

//================================================================================================================
//   Bluetooth用スレッド
//================================================================================================================

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチン.
/// @param args この関数に渡される引数ですが, 現在は使用されていません.
void Core0_BT_r(void *args) { // サブCPU(Core0)で実行するプログラム
  while (true) {              // Bluetooth待受用の無限ループ
    if (!flg.udp_busy)        // UDP使用中を避ける
    {
      mrd_pad_reader(pad_array, MOUNT_PAD, PAD_BUTTON_MARGE, MONITOR_PAD);
      delay((PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 0); // PAD_INTERVAL-1ms(最小値0)
    }
    delay(1); // 1ms
  }
}

#endif // __MERIDIAN_BT_PAD_H__
