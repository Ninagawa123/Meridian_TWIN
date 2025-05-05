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
  uint8_t bval[EEPROM_BYTE]; // 1バイト単位で540個のデータを持つ
  int16_t saval[3][90];      // short型で3*90個の配列データを持つ
  uint16_t usaval[3][90];    // unsigned short型で3*90個の配列データを持つ
  int16_t sval[270];         // short型で270個のデータを持つ
  uint16_t usval[270];       // unsigned short型で270個のデータを持つ
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

/// @brief サーボ設定構造体からEEPROM格納用の配列データを作成する
/// @param a_sv サーボ設定を保持する構造体
/// @return EEPROM格納用の配列データ(UnionEEPROM型)
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv) {
  UnionEEPROM array_tmp = {0};

  for (int i = 0; i < 15; i++) {
    // 各サーボのマウント有無と方向(正転・逆転)
    uint16_t l_tmp = 0;
    uint16_t r_tmp = 0;

    // bit0 : マウント
    if (sv.ixl_mount[i])
      l_tmp |= 0x0001;
    if (sv.ixr_mount[i])
      r_tmp |= 0x0001;

    // bit1-7 : サーボ ID
    l_tmp |= static_cast<uint16_t>(sv.ixl_id[i] & 0x7F) << 1;
    r_tmp |= static_cast<uint16_t>(sv.ixr_id[i] & 0x7F) << 1;

    // bit8 : サーボ回転方向 (正転なら1, 逆転なら0)
    if (sv.ixl_cw[i] > 0)
      l_tmp |= 0x0100;
    if (sv.ixr_cw[i] > 0)
      r_tmp |= 0x0100;

    // サーボのマウント有無, ID, 回転方向のデータ格納
    array_tmp.saval[1][20 + i * 2] = l_tmp;
    array_tmp.saval[1][50 + i * 2] = r_tmp;

    // 各サーボの直立デフォルト角度(degree → float short*100)の格納
    array_tmp.saval[1][21 + i * 2] = mrd.float2HfShort(a_sv.ixl_trim[i]);
    array_tmp.saval[1][51 + i * 2] = mrd.float2HfShort(a_sv.ixr_trim[i]);
  }
  return array_tmp;
}

/// @brief 現在のサーボ位置からEEPROM格納用のトリムデータを作成し、EEPROM配列に入れる.
/// @param a_data EEPROM書き込み用の配列データ.
/// @param a_sv サーボパラメータの構造体.
/// @return EEPROM格納用の配列データを返す.
UnionEEPROM mrd_eeprom_make_trim_from_current_lite(UnionEEPROM a_data, ServoParam a_sv) {
  for (int i = 0; i < 15; i++) {
    // L系統サーボの直立デフォルト値 degree
    a_data.saval[1][21 + i * 2] = short(mrd.float2HfShort(a_sv.ixl_tgt[i] - a_sv.ixl_trim[i]));
    // R系統サーボの直立デフォルト値 degree
    a_data.saval[1][51 + i * 2] = short(mrd.float2HfShort(a_sv.ixr_tgt[i] - a_sv.ixr_trim[i]));
  };
  return a_data;
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
    // a_data = mrd_eeprom_make_data_from_config_lite();

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

/// @brief EEPROMの内容を読み込んで返す.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_read() {
  UnionEEPROM read_data_tmp = {0};      // ゼロ初期化を明示的に行う
  for (int i = 0; i < EEPROM_BYTE; i++) // データを読み込む時はbyte型
  {
    read_data_tmp.bval[i] = EEPROM.read(i);
  }
  return read_data_tmp;
}

/// @brief EEPROMの内容を読み込みサーボ値構造体に反映する.
/// @param a_sv サーボ設定を保持する構造体.
/// @param a_monitor シリアルモニタへのデータ表示.
/// @param a_serial 出力先シリアルの指定.
/// @return 終了時にtrueを返す.
bool mrd_eeprom_load_servosettings(ServoParam &a_sv, bool a_monitor, Stream &a_serial) {
  a_serial.println("Load and set servo settings from EEPROM.");
  UnionEEPROM array_tmp = mrd_eeprom_read();
  for (int i = 0; i < a_sv.num_max; i++) {
    // 各サーボのマウント有無
    a_sv.ixl_mount[i] = static_cast<bool>(array_tmp.saval[1][20 + i * 2] & 0x0001); // bit0:マウント有無
    a_sv.ixr_mount[i] = static_cast<bool>(array_tmp.saval[1][50 + i * 2] & 0x0001); // bit0:マウント有無
    // 各サーボの実サーボ呼び出しID番号
    a_sv.ixl_id[i] = static_cast<uint8_t>(array_tmp.saval[1][20 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    a_sv.ixr_id[i] = static_cast<uint8_t>(array_tmp.saval[1][50 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    // 各サーボの回転方向(正転・逆転)
    a_sv.ixl_cw[i] = static_cast<int8_t>((array_tmp.saval[1][20 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    a_sv.ixr_cw[i] = static_cast<int8_t>((array_tmp.saval[1][50 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    // 各サーボの直立デフォルト角度,トリム値(degree小数2桁までを100倍した値で格納されているものを展開)
    a_sv.ixl_trim[i] = array_tmp.saval[1][21 + i * 2] * 0.01f;
    a_sv.ixr_trim[i] = array_tmp.saval[1][51 + i * 2] * 0.01f;

    if (a_monitor) {
      a_serial.print("L-idx:");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(sv.ixl_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(sv.ixl_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(sv.ixl_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.print(mrd_pddstr(sv.ixl_trim[i], 7, 2, true));
      a_serial.print("  R-idx: ");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(sv.ixr_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(sv.ixr_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(sv.ixr_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.println(mrd_pddstr(sv.ixr_trim[i], 7, 2, true));
    }
  }
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

#endif // __MERIDIAN_EEPROM_H__
