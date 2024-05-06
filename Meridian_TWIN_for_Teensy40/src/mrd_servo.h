#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

#include "config.h"
#include "main.h"
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_dxl2.h"
#include "mrd_module/sv_ics.h"
#include "mrd_module/sv_xbus.h"
#include "mrd_module/sv_ftc.h"

//================================================================================================================
//---- Servo 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定します. 
/// @param line UART通信ライン（L, R, またはC）. 
/// @param mrd_servo_type サーボのタイプを示す整数値. 
/// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返します. 
bool mrd_servos_begin(Uart_line line, int mrd_servo_type)
{
    if (mrd_servo_type > 0)
    {
        Serial.print("Set UART_");
        Serial.print(getLineName(line));
        Serial.print(" protocol : ");

        switch (mrd_servo_type)
        {
        case 1:
            Serial.print("single PWM");
            Serial.println(" - Not supported yet.");
            break;
        case 11:
            Serial.print("I2C_PCA9685 to PWM");
            Serial.println(" - Not supported yet.");
            break;
        case 21:
            Serial.print("RSxTTL (FUTABA)");
            Serial.println(" - Not supported yet.");
            break;
        case 31:
            Serial.print("DYNAMIXEL Protocol 1.0");
            Serial.println(" - Not supported yet.");
            break;
        case 32:
            Serial.print("DYNAMIXEL Protocol 2.0");
            Serial.println(" - Not supported yet.");
            break;
        case 43:
            if (line == L)
                ics_L.begin(); // サーボモータの通信初期設定. Serial2
            else if (line == R)
                ics_R.begin(); // サーボモータの通信初期設定. Serial3
            else if (line == C)
                ics_C.begin(); // サーボモータの通信初期設定. Serial1
            Serial.println("ICS3.5/3.6(KONDO,KRS)");
            break;
        case 44:
            Serial.print("PMX(KONDO)");
            Serial.println(" - Not supported yet.");
            break;
        case 51:
            Serial.print("XBUS(JR PROPO)");
            Serial.println(" - Not supported yet.");
            break;
        case 61:
            Serial.print("STS(FEETECH)");
            Serial.println(" - Not supported yet.");
            break;
        case 62:
            Serial.print("SCS(FEETECH)");
            Serial.println(" - Not supported yet.");
            break;
        default:
            Serial.println(" Not defined. ");
            break;
        }
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------------
//  サーボ通信フォーメーションの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボにコマンドを分配します. 
/// @param mrd_s_meridim サーボの動作パラメータを含む構造体. 
/// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返します. 
bool mrd_servos_drive(Meridim90Union mrd_s_meridim, int sv_L_type, int sv_R_type, int sv_C_type)
{
    if (sv_L_type == 43 && sv_R_type == 43) // ICSサーボがL系R系に設定されていた場合はLR均等送信を実行
    {
        mrd_sv_drive_ics_double(mrd_s_meridim);
        return true;
    }
    else
    {
        return false;
    }
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief すべてのサーボモーターをオフ（フリー状態）に設定します. 
bool mrd_servos_all_off(Meridim90Union mrd_s_meridim)
{
    for (int i = 0; i < 15; i++)
    {
        mrd_s_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
        mrd_s_meridim.sval[i * 2 + 50] = 0; //
    }
    Serial.println("All servos off.");
    return true;
}

#endif
