#ifndef __MERIDIAN_BLUETOOTH_H__
#define __MERIDIAN_BLUETOOTH_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_pad.h"

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー
ESP32Wiimote wiimote;

//================================================================================================================
//  JOYPAD 関連の処理
//================================================================================================================
/// @brief Wiiリモコンからの入力データを受信し, 処理する.
void mrd_bt_wiimote_receive(PadUnion &a_pad_array) {
  wiimote.task();
  flg.bt_busy = true;
  if (wiimote.available() > 0) {
    static bool first_call_tmp = true; // 初回の呼び出しフラグ
    if (first_call_tmp) {
      Serial.println("Wiimote successfully connected. ");
      first_call_tmp = false; // 初回の呼び出しフラグをオフにする
    }
    uint16_t button = wiimote.getButtonState();
    a_pad_array.usval[0] = 0;
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_SOLO[i] & button)) ||
          (!PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_ORIG[i] & button))) {
        a_pad_array.usval[0] |= mask_tmp;
      }
    }
    a_pad_array.usval[0] = a_pad_array.usval[0]; // short型で4個の配列データを持つ
  }
  if (MOUNT_PAD == 6) {
    // NunchukState nunchuk = wiimote.getNunchukState();
    //  int calib_l1x = 5;  // キャリブレーション値
    //   int calib_l1y = -6; // キャリブレーション値
    //    pad.stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 +
    //    calib_l1y));
    //     if (nunchuk.cBtn == 1)
    //       pad_array.usval[0] |= (0b00000100 * 256) + 0b00000000;
    //      if (nunchuk.zBtn == 1)
    //        pad_array.usval[0] |= (0x00000001 * 256) + 0b00000000;
    //       pad_array.sval[1] = pad.stick_L_x * 256 + pad.stick_L_y;
    //       pad_array.sval[2] = pad.stick_R_x * 256 + pad.stick_R_y;
    //       pad_array.sval[3] = pad.L2_val * 256 + pad.R2_val;
  }
  flg.bt_busy = false;
}

/// @brief Bluetoothの設定を行い, 条件に応じてWiiコントローラの接続を開始する.
bool mrd_bt_settings(int a_mount_pad) {
  // Wiiコントローラの接続開始
  if ((a_mount_pad == 5)) {
    Serial.println("Try to connect Wiimote single ...");
    wiimote.init();
    return true;
  } else if ((a_mount_pad == 6)) {
    Serial.println("Try to connect Wiimote + analog...");
    wiimote.init();
    return true;
  }
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
      delay((PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 0); // interval ms
    }
    delay(1); // 1ms
  }
}

#endif // __MERIDIAN_BLUETOOTH_H__
