#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

#include "config.h"
#include "main.h"
#include <EEPROM.h> // EEPROMのライブラリ導入

// EEPROM読み書き用
typedef union // 共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
    short sval[270];   // short型で270個の配列データを持つ
    uint8_t bval[540]; // 1バイト単位で540個の配列データを持つ
} UnionEEPROM;
UnionEEPROM eeprom_write; // EEPROM書き込み用
UnionEEPROM eeprom_load;  // EEPROM読み込み用

bool mrd_eeprom_set = EEPROM_SET;

//================================================================================================================
//---- EEPROM関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief EEPROMの初期化を試みるスタブ関数です. 現在は何も行わず, 常にfalseを返します. 
/// @return 常にfalseを返します. 
bool mrd_eeprom_init()
{
    return false;
}

/// @brief EEPROMにサーボの設定とデフォルト位置を保存します. 
bool mrd_set_eeprom() //
{
    if (mrd_eeprom_set)
    {
        /* 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転） */
        // 0: マウントなし
        // 01: Single PWM
        // 11: I2C_PCA9685 to PWM
        // 21: FUTABA_RSxTTL
        // 31: DYNAMIXEL Protocol 1.0
        // 32: DYNAMIXEL Protocol 2.0
        // 43: KONDO_ICS 3.5/3.6
        // 44: KONDO_PMX
        // 51: JRPROPO_XBUS
        // 61: FEETECH_STS
        // 62: FEETECH_SCS
        // 凡例: idl_mt[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
        eeprom_write.sval[20] = 43; // idl[00]頭ヨー
        eeprom_write.sval[22] = 43; // idl[01]左肩ピッチ
        eeprom_write.sval[24] = 43; // idl[02]左肩ロール
        eeprom_write.sval[26] = 43; // idl[03]左肘ヨー
        eeprom_write.sval[28] = 43; // idl[04]左肘ピッチ
        eeprom_write.sval[30] = 43; // idl[05]左股ヨー
        eeprom_write.sval[32] = 43; // idl[06]左股ロール
        eeprom_write.sval[34] = 43; // idl[07]左股ピッチ
        eeprom_write.sval[36] = 43; // idl[08]左膝ピッチ
        eeprom_write.sval[38] = 43; // idl[09]左足首ピッチ
        eeprom_write.sval[40] = 43; // idl[10]左足首ロール
        eeprom_write.sval[42] = 0;  // idl[11]追加テスト用
        eeprom_write.sval[44] = 0;  // idl[12]追加テスト用
        eeprom_write.sval[46] = 0;  // idl[13]追加テスト用
        eeprom_write.sval[48] = 0;  // idl[14]追加テスト用
        eeprom_write.sval[50] = 43; // idr[00]腰ヨー
        eeprom_write.sval[52] = 43; // idr[01]右肩ピッチ
        eeprom_write.sval[54] = 43; // idr[02]右肩ロール
        eeprom_write.sval[56] = 43; // idr[03]右肘ヨー
        eeprom_write.sval[58] = 43; // idr[04]右肘ピッチ
        eeprom_write.sval[60] = 43; // idr[05]右股ヨー
        eeprom_write.sval[62] = 43; // idr[06]右股ロール
        eeprom_write.sval[64] = 43; // idr[07]右股ピッチ
        eeprom_write.sval[66] = 43; // idr[08]右膝ピッチ
        eeprom_write.sval[68] = 43; // idr[09]右足首ピッチ
        eeprom_write.sval[70] = 43; // idr[10]右足首ロール
        eeprom_write.sval[72] = 0;  // idr[11]追加テスト用
        eeprom_write.sval[74] = 0;  // idr[12]追加テスト用
        eeprom_write.sval[76] = 0;  // idr[13]追加テスト用
        eeprom_write.sval[78] = 0;  // idr[14]追加テスト用

        // 各サーボの直立デフォルト値 degree
        // 直立状態になるよう, 具体的な数値を入れて現物調整する
        eeprom_write.sval[21] = 0;      // idl[00]頭ヨー
        eeprom_write.sval[23] = -91.10; // idl[01]左肩ピッチ
        eeprom_write.sval[25] = -2700;  // idl[02]左肩ロール
        eeprom_write.sval[27] = 0;      // idl[03]左肘ヨー
        eeprom_write.sval[29] = 89.98;  // idl[04]左肘ピッチ
        eeprom_write.sval[31] = 0;      // idl[05]左股ヨー
        eeprom_write.sval[33] = 0;      // idl[06]左股ロール
        eeprom_write.sval[35] = -1.35;  // idl[07]左股ピッチ
        eeprom_write.sval[37] = -58.05; // idl[08]左膝ピッチ
        eeprom_write.sval[39] = -20.25; // idl[09]左足首ピッチ
        eeprom_write.sval[41] = -0.68;  // idl[10]左足首ロール
        eeprom_write.sval[43] = 0;      // idl[11]追加テスト用
        eeprom_write.sval[45] = 0;      // idl[12]追加テスト用
        eeprom_write.sval[47] = 0;      // idl[13]追加テスト用
        eeprom_write.sval[49] = 0;      // idl[14]追加テスト用
        eeprom_write.sval[51] = 0;      // idr[00]腰ヨー
        eeprom_write.sval[53] = 0;      // idr[01]右肩ピッチ
        eeprom_write.sval[55] = -89.44; // idr[02]右肩ロール
        eeprom_write.sval[57] = 0;      // idr[03]右肘ヨー
        eeprom_write.sval[59] = 89.98;  // idr[04]右肘ピッチ
        eeprom_write.sval[61] = 0;      // idr[05]右股ヨー
        eeprom_write.sval[63] = 1.69;   // idr[06]右股ロール
        eeprom_write.sval[65] = -3.38;  // idr[07]右股ピッチ
        eeprom_write.sval[67] = -57.38; // idr[08]右膝ピッチ
        eeprom_write.sval[69] = -20.25; // idr[09]右足首ピッチ
        eeprom_write.sval[71] = -2.36;  // idr[10]右足首ロール
        eeprom_write.sval[73] = 0;      // idr[11]追加テスト用
        eeprom_write.sval[75] = 0;      // idr[12]追加テスト用
        eeprom_write.sval[77] = 0;      // idr[13]追加テスト用
        eeprom_write.sval[79] = 0;      // idr[14]追加テスト用

        // EEPROM書き込み
        for (int i = 0; i < 540; i++) // データを書き込む時はbyte型
        {
            EEPROM.write(i, eeprom_write.bval[i]);
        }
        Serial.println();
        return true;
    }
    return false;
}

// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力します. 
/// @param mrd_eeprom_check EEPROMのチェックを行うかどうかのブール値. 
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, チェックを行わなかった場合はfalseを返します. 
bool mrd_eeprom_check(bool mrd_eeprom_check)
{
    // EEPROM書き込み
    if (mrd_eeprom_check)
    {
        Serial.print("Set EEPROM...");

        mrd_set_eeprom();

        Serial.println("Completed.");
        Serial.println("Read EEPROM...");
        // EEPROM読み込み
        for (int i = 0; i < 540; i++) // データを読み込む時はbyte型
        {
            eeprom_load.bval[i] = EEPROM.read(i);
        }

        int value = EEPROM.length(); // EEPROMの長さ
        Serial.print("EEPROM length  : ");
        Serial.println(value); // EEPROMの長さ表示

        // データ作成
        for (int i = 0; i < 270; i++) // 書き込むデータはshort型で作成
        {
            eeprom_write.sval[i] = 0;
        }

        for (int i = 0; i < 15; i++) // 書き込むデータはshort型で作成
        {
            eeprom_write.sval[i * 2 + 20] = short(i);
            eeprom_write.sval[i * 2 + 50] = short(i);
        }

        // 書き込み
        for (int i = 0; i < 540; i++) // データを書き込む時はbyte型
        {
            EEPROM.write(i, eeprom_write.bval[i]);
        }
        Serial.println();

        // EEPROM内容表示
        Serial.println("Display EEPROM:");
        // Serial.print("[0]");
        for (int i = 0; i < 270; i++) // 書き込むデータはshort型で作成
        {
            // Serial.print("/");
            Serial.print(eeprom_load.sval[i]);
            if ((i == 89) or (i == 179))
            {
                Serial.println();
            }
            else
            {
                Serial.print("/");
            }
        }
        Serial.println();
        // 初期設定完了
        Serial.println("Ready. ");
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif
