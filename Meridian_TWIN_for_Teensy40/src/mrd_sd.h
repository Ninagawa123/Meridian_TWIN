#ifndef __MERIDIAN_SD_H__
#define __MERIDIAN_SD_H__

#include "config.h"
#include "main.h"
#include <SD.h> // SDカード用
File myFile;    // SDカード用

//================================================================================================================
//---- SDメモリ 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  初期化処理
//------------------------------------------------------------------------------------

/// @brief SDカードの初期化を試みます. SDカードがマウントされているか, 及びチップ選択ピンの設定に基づきます. 
/// @param mrd_sd_mount SDカードがマウントされているかどうかのブール値. 
/// @param mrd_sd_chipselect_pin SDカードのチップ選択ピン番号. 
/// @return SDカードの初期化が成功した場合はtrueを, 失敗またはSDカードがマウントされていない場合はfalseを返します. 
bool mrd_sd_init(bool mrd_sd_mount, int mrd_sd_chipselect_pin)
{
    if (mrd_sd_mount)
    {
        Serial.print("Initializing SD card ...");
        delay(100);
        if (!SD.begin(mrd_sd_chipselect_pin))
        {
            Serial.println("Card failed, or not present.");
            delay(100);
            return false;
        }
        else
        {
            Serial.println(" OK.");
            delay(100);
            return true;
        }
    }
    Serial.println("SD not mounted.");
    delay(100);
    return false;
}

//------------------------------------------------------------------------------------
//  リードライトテスト
//------------------------------------------------------------------------------------

/// @brief SDカードの読み書き機能をテストします. SDカードがマウントされ, 読み書きのチェックが要求された場合のみテストを実行します. 
/// @param mrd_sd_mount SDカードがマウントされているかどうかのブール値. 
/// @param mrd_sd_chipselect_pin SDカードのチップ選択ピン番号. 
/// @param mrd_sd_check_rw SDカードの読み書きをチェックするかどうかのブール値. 
/// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返します. 
bool mrd_sd_check(bool mrd_sd_mount, int mrd_sd_chipselect_pin, bool mrd_sd_check_rw)
{
    if (mrd_sd_mount && mrd_sd_check_rw)
    {
        myFile = SD.open("/test.txt", FILE_WRITE);
        delay(1); // SPI安定化検証用

        if (myFile)
        {
            Serial.print("Checking SD card r/w...");
            // SD書き込みテスト用のランダムな4桁の数字を生成
            randomSeed(long(analogRead(A0) + analogRead(A1) * 2 + analogRead(A2) * 3 + analogRead(A3) * 100 + analogRead(A4) * 103 + analogRead(A5) * 105 + analogRead(A6) * 1000 + analogRead(A7) * 1001 + analogRead(A8) * 1007 + analogRead(A9) * 10000 + analogRead(A10) * 10001 + analogRead(A11) * 10005 + analogRead(A12) * 10007 + analogRead(A13) * 10009)); // 未接続ピンのノイズを利用
            int randNumber = random(1000, 9999);

            Serial.print(" write code ");
            Serial.print(randNumber);
            // ファイルへの書き込みを実行
            myFile.println(randNumber);
            delayMicroseconds(1); // SPI安定化検証用
            myFile.close();
            Serial.print(" and");
            delayMicroseconds(10); // SPI安定化検証用
            // ファイルからの読み込みを実行
            myFile = SD.open("/test.txt");
            if (myFile)
            {
                Serial.print(" read code ");
                while (myFile.available())
                {
                    Serial.write(myFile.read());
                }
                myFile.close();
            }
            SD.remove("/test.txt");
            delay(10);
            return true;
        }
        else
        {
            Serial.println("Could not open SD test.txt file.");
        }
    }
    return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif
