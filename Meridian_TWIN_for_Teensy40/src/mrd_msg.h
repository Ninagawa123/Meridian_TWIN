#ifndef __MERIDIAN_MESSAGE_H__
#define __MERIDIAN_MESSAGE_H__

#include "config.h"
#include "main.h"

//================================================================================================================
//---- Message 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  起動時メッセージ
//------------------------------------------------------------------------------------

/// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力します. 
/// @param version システムのバージョンを示す文字列. 
void mrd_msg_print_tsy_hello(String version)
{
    Serial.println();
    Serial.print("Hi, This is ");
    Serial.println(version);
    Serial.print("Set PC-USB speed: ");
    Serial.println(SERIAL_PC_BPS);
    Serial.print("Set SPI0 speed  : ");
    Serial.println(SPI0_SPEED);
    Serial.print("Set i2c0 speed  : ");
    Serial.println(I2C0_SPEED);
    Serial.print("Set i2c1 speed  : ");
    Serial.println(I2C1_SPEED);
}

/// @brief システムに接続設定したジョイパッドのタイプを表示します. 
void mrd_msg_print_mounted_pad()
{
    Serial.print("Pad Receiver mounted : ");

    if (MOUNT_JOYPAD == MERIMOTE)
    {
        Serial.println("Merimote.");
    }
    else if (MOUNT_JOYPAD == BLUERETRO)
    {
        Serial.println("BlueRetro.");
    }
    else if (MOUNT_JOYPAD == SBDBT)
    {
        Serial.println("SBDBT.");
    }
    else if (MOUNT_JOYPAD == KRR5FH)
    {
        Serial.println("KRC-5FH.");
    }
    else
    {
        Serial.println("None (PC).");
    }
}

/// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力します. 
void mrd_msg_print_error_monitor()
{
    Serial.print("[ERRs] esp>pc:");
    Serial.print(err.esp_pc);
    Serial.print(" pc>esp:");
    Serial.print(err.pc_esp);
    Serial.print(" esp>tsy:");
    Serial.print(err.esp_tsy);
    Serial.print(" tsy>esp:");
    Serial.print(err.esp_tsy);
    Serial.print(" tsySkip:");
    Serial.print(err.tsy_skip); //
    Serial.print(" espSkip:");
    Serial.print(err.esp_skip); //
    Serial.print(" pcSkip:");
    Serial.print(err.pc_skip); //
    Serial.print(" seq:");
    Serial.print(int(mrdck.seq_r_expect)); //
    Serial.print(" [u]:");
    Serial.print(r_spi_meridim.bval[MSG_ERR_u], BIN);
    Serial.println();
}

/// @brief システムに接続されているIMU/AHRSセンサーのタイプを表示します. 
/// @param mrd_msg_imuahrs_type 接続されているセンサーのタイプを示す列挙型です. 
void mrd_msg_print_imuahrs(ImuAhrsType mrd_msg_imuahrs_type)
{
    Serial.print("IMU/AHRS Sensor mounted: ");

    switch (mrd_msg_imuahrs_type)
    {
    case NO_IMU:
        Serial.println("None. ");
        break;
    case MPU6050_IMU:
        Serial.println("MPU6050(GY-521) ");
        break;
    case MPU9250_IMU:
        Serial.println("MPU9250(GY-6050/GY-9250) ");
        break;
    case BNO055_AHRS:
        Serial.println("BNO055 ");
        break;
    default:
        break;
    }
}

/// @brief 指定された秒数だけキャパシタの充電プロセスを示すメッセージを表示します. 
/// @param sec 充電プロセスの期間を秒単位で指定します. 
void mrd_msg_charging(int sec)
{
    Serial.print("Charging the capacitor.");
    for (int i = 0; i < sec;i++)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println();
}

/// @brief システムの動作開始を示すメッセージを出力します. 
void mrd_msg_flow_start()
{
    Serial.println();
    Serial.println("-) Meridian TWIN system on side Teensy now flows. (-");
}

//------------------------------------------------------------------------------------
//  イベントメッセージ
//------------------------------------------------------------------------------------

/// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示します. 
/// @param mrd_line サーボモーターが接続されているUARTライン（L, R, C）. 
/// @param mrd_num エラーが発生しているサーボの番号. 
/// @param mrd_disp_sv_error エラーメッセージを表示するかどうかのブール値. 
/// @return エラーメッセージが表示された場合はtrueを, 表示されなかった場合はfalseを返します. 
bool mrd_msg_servo_error(Uart_line mrd_line, int mrd_num, bool mrd_disp_sv_error)
{
    if (mrd_disp_sv_error)
    {
        Serial.print("Found servo err ");
        if (mrd_line == L)
        {
            Serial.print("L_");
            Serial.println(mrd_num);
            return true;
        }
        else if (mrd_line == R)
        {
            Serial.print("R_");
            Serial.println(mrd_num);
            return true;
        }
        else if (mrd_line == C)
        {
            Serial.print("C_");
            Serial.println(mrd_num);
            return true;
        }
    }
    return false;
}

#endif