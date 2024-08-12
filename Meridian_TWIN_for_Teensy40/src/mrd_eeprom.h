#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <EEPROM.h>

//================================================================================================================
//  EEPROM関連の処理
//================================================================================================================
// 注:EEPROM.beginはESP32では必要だが, Teensyでは不要.

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_BYTE];                // 1バイト単位で540個のデータを持つ
  int16_t saval[6][int(EEPROM_BYTE / 6)];   // short型で6*90個の配列データを持つ
  uint16_t sauval[6][int(EEPROM_BYTE / 6)]; // ushort型で6*90個の配列データを持つ
  int16_t baval[6][int(EEPROM_BYTE / 3)];   // byte型で6*180個の配列データを持つ
  uint8_t bauval[6][int(EEPROM_BYTE / 3)];  // ubyte型で6*180個の配列データを持つ
  int16_t sval[int(EEPROM_BYTE / 2)];       // short型で270個のデータを持つ
  uint16_t usval[int(EEPROM_BYTE / 2)];     // short型で270個のデータを持つ
} UnionEEPROM;
UnionEEPROM eeprom_write_data; // EEPROM書き込み用
UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

//------------------------------------------------------------------------------------
//  EEPROM 読み込み
//------------------------------------------------------------------------------------

/// @brief EEPROMの内容を読み込んで返す.
/// @param a_len_byte EEPROMの使用サイズ.
/// @param a_serial メッセージ表示用のハードウェアシリアル.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_load(int a_len_byte, Stream &a_serial) {
  if (a_len_byte > EEPROM.length()) // EEPROMのサイズを超えないようチェック
  {
    a_len_byte = EEPROM.length();
    a_serial.println("Error: EEPROM address out of range.");
  }

  UnionEEPROM data_tmp;
  for (int i = 0; i < a_len_byte; i++) // データを読み込む時はbyte型
  {
    data_tmp.bval[i] = EEPROM.read(i);
  }
  return data_tmp;
}

//------------------------------------------------------------------------------------
//  EEPROM ダンプ出力
//------------------------------------------------------------------------------------

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
/// @param a_data EEPROM用の配列データ.
/// @param a_len_byte EEPROMのサイズ(byte).
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @param a_serial メッセージ表示用のハードウェアシリアル.
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_serial(UnionEEPROM a_data, int a_len_byte, int a_bhd, Stream &a_serial) {
  a_serial.print("EEPROM Length ");
  a_serial.print(EEPROM.length()); // EEPROMの長さ表示
  a_serial.print(", Used Length ");
  a_serial.print(a_len_byte); // EEPROMの使用サイズ表示(BYTE)
  a_serial.println("byte, 16bit Dump;");
  int k = 0;
  char hex_tmp[5];
  for (int i = 0; i < int(a_len_byte / 2); i++) // 読み込むデータはshort型で作成
  {
    k++;
    if (a_bhd == Bin) {
      // 16桁のビット列を表示
      for (int bit = 15; bit >= 0; bit--) {
        a_serial.print((a_data.usval[i] >> bit) & 1);
      }
    } else if (a_bhd == Hex) {
      sprintf(hex_tmp, "%04X", (uint16_t)a_data.sval[i]); // 符号なし16ビットにキャストして表示
      a_serial.print(hex_tmp);
    } else {
      a_serial.print(a_data.sval[i], DEC);
    }

    if (a_bhd == Bin) {
      if (k % 90 == 0) {
        a_serial.println(",\n");
        k = 0;
      } else if (k % 20 == 0) {
        a_serial.println("/");
      } else if (k % 5 == 0) {
        a_serial.println();
      } else {
        a_serial.print(" ");
      }
    } else {
      if (k % 90 == 0) {
        a_serial.println(",");
        k = 0;
      } else if (k % 10 == 0) {
        a_serial.print((k % 20 == 0) ? "\n" : "  ");
      } else {
        a_serial.print(" ");
      }
    }
  }
  return true;
}

//------------------------------------------------------------------------------------
//  EEPROM データ作成
//------------------------------------------------------------------------------------

/// @brief config.hにあるサーボの諸設定からEEPROM格納用の配列データを作成する.
/// @return config.hから作成したEEPROM格納用の配列データを返す.
UnionEEPROM mrd_eeprom_make_data_from_config() {
  UnionEEPROM array_tmp = {0};
  for (int i = 0; i < 15; i++) {
    // 左サーボの情報を格納
    array_tmp.sauval[1][20 + i * 2] =
        ((sv.ixl_mount[i] != 0) << 15) |     // bit16: マウント有無 (0以外なら1)
        ((sv.ixl_id[i] & 0x7F) << 8) |       // bit15-9: ID (7ビット)
        ((sv.ixl_cw[i] == -1 ? 1 : 0) << 8); // bit8: 正逆 (1ビット)
    // 各サーボの直立デフォルト値 degree
    array_tmp.saval[1][21 + i * 2] = short(mrd.float2HfShort(sv.ixl_trim[i]));

    // 右サーボの情報を格納
    array_tmp.sauval[1][50 + i * 2] =
        ((sv.ixl_mount[i] != 0) << 15) |     // bit16: マウント有無 (0以外なら1)
        ((sv.ixl_id[i] & 0x7F) << 8) |       // bit15-9: ID (7ビット)
        ((sv.ixl_cw[i] == -1 ? 1 : 0) << 8); // bit8: 正逆 (1ビット)
    // 各サーボの直立デフォルト値 degree
    array_tmp.saval[1][51 + i * 2] = short(mrd.float2HfShort(sv.ixr_trim[i]));
  };
  return array_tmp;
}

//------------------------------------------------------------------------------------
//  EEPROM 書き込み
//------------------------------------------------------------------------------------

/// @brief EEPROMを0でフォーマットする
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_len_byte EEPROMの使用サイズ.
/// @param a_serial メッセージ表示用のハードウェアシリアル.
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
bool mrd_eeprom_zero_format(bool a_flg_protect, int a_len_byte, Stream &a_serial) {
  a_serial.println("Try to EEPROM zero format...");

  if (a_flg_protect) { // EEPROM書き込み実施フラグをチェック
    return false;
  }
  if (flg.eeprom_protect) // config.hのEEPROM書き込みプロテクトをチェック
  {
    a_serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
    return false;
  }

  // EEPROM書き込み
  for (int i = 0; i < EEPROM_BYTE; i++) // データを書き込む時はbyte型
  {
    if (i >= EEPROM.length()) // EEPROMのサイズを超えないようチェック
    {
      a_serial.println("Error: EEPROM address out of range.");
      return false;
    }
    // 書き込みデータがEEPROM内のデータと違う場合のみ書き込みをセット
    EEPROM.write(i, 0);
  }
  a_serial.println("EEPROM zero format finished.");
  return true;
}

/// @brief EEPROMにEEPROM格納用の配列データを書き込む.
/// @param a_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_serial メッセージ表示用のハードウェアシリアル.
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
bool mrd_eeprom_write(UnionEEPROM a_data, bool a_flg_protect, Stream &a_serial) {
  if (a_flg_protect) { // EEPROM書き込み実施フラグをチェック
    return false;
  }
  if (flg.eeprom_protect) // config.hのEEPROM書き込みプロテクトをチェック
  {
    a_serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
    return false;
  }

  // EEPROM書き込み
  byte old_value_tmp;                   // EEPROMにすでに書き込んであるデータ
  bool flg_renew_tmp = false;           // 書き込みコミットを実施するかのフラグ
  for (int i = 0; i < EEPROM_BYTE; i++) // データを書き込む時はbyte型
  {
    if (i >= EEPROM.length()) // EEPROMのサイズを超えないようチェック
    {
      a_serial.println("Error: EEPROM address out of range.");
      return false;
    }
    old_value_tmp = EEPROM.read(i);
    // 書き込みデータがEEPROM内のデータと違う場合のみ書き込みをセット
    if (old_value_tmp != a_data.bval[i]) {
      EEPROM.write(i, a_data.bval[i]);
      flg_renew_tmp = true;
    }
  }
  if (flg_renew_tmp) // 変更箇所があれば書き込みを実施
  {
    // EEPROM.commit(); // 書き込みを確定にcommitがESP32では必要.Teensyでは不要,
    a_serial.print("Value updated ");
    return true;
  } else {
    a_serial.print("Same value ");
  }
  return false;
}

//------------------------------------------------------------------------------------
//  EEPROM データ作成→書き込み
//------------------------------------------------------------------------------------

/// @brief EEPROMにサーボの設定とデフォルト位置を保存する.
/// @param a_data EEPROM書き込み用の配列データ.
/// @param a_len_byte EEPROMの使用サイズ.
/// @return UnionEEPROM のフォーマットで配列を返す.
bool mrd_eeprom_set(UnionEEPROM a_data, int a_len_byte) //
{
  if (flg.eeprom_set) {
    // a_data = mrd_eeprom_make_data_from_config();

    // EEPROM書き込み
    for (int i = 0; i < a_len_byte; i++) // データを書き込む時はbyte型
    {
      EEPROM.write(i, a_data.bval[i]);
    }
    // Serial.println();
    return true;
  }
  return false;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_len_byte EEPROMのサイズ(byte).
/// @param a_flg_do 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(int a_len_byte, bool a_flg_do, int a_bhd, Stream &a_serial) {
  if (a_flg_do) {
    mrd_eeprom_dump_serial(mrd_eeprom_load(a_len_byte, a_serial), a_len_byte, a_bhd, a_serial);
    return true;
  }
  return false;
}

/// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力する.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_a_flg_do EEPROMの読み書きチェックを実施するかのブール値.
/// @param a_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, それ以外はfalseを返す.
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, int a_len_byte, bool a_flg_do,
                                 bool a_protect, int a_bhd, Stream &a_serial) {
  if (!a_flg_do) // EEPROMの読み書きチェックを実施するか
  {
    return false;
  }

  // EEPROM書き込みを実行
  a_serial.println("Try to write EEPROM: ");
  mrd_eeprom_dump_serial(a_write_data, a_len_byte, a_bhd, a_serial); // 書き込み内容をダンプ表示

  if (mrd_eeprom_write(a_write_data, a_protect, a_serial)) {
    a_serial.println("...Write OK.");
  } else {
    a_serial.println("...Write failed.");
    return false;
  }

  // EEPROM読み込みを実行
  a_serial.println("Read EEPROM: ");
  UnionEEPROM a_data_tmp;
  mrd_eeprom_load(a_len_byte, a_serial);
  mrd_eeprom_dump_serial(a_data_tmp, a_len_byte, a_bhd, a_serial); // 読み込み内容をダンプ表示
  a_serial.println("...Read completed.");

  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief EEPROMから任意のshort型データを読み込む.
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~89).
/// @return short型データを返す.
short mrd_eeprom_load_short(int index_y, int index_x) {
  return short(EEPROM.read(index_y * 90 + index_x));
}

/// @brief EEPROMから任意のbyte型データを読み込む.
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~179).
/// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
/// @return byte型データを返す.
int8_t mrd_eeprom_load_byte(int index_y, int index_x, int low_high) //
{
  return int8_t(EEPROM.read(index_y * 180 + index_x * 2 + low_high));
}

#endif
