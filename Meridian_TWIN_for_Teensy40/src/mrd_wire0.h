#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

#include "config.h"
#include "main.h"

IntervalTimer wireTimer0;

//================================================================================================================
//---- I2C wire0 関連の処理  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数です. 
///        MPU6050, MPU9250, BNO055に対応していますが, MPU9250とBNO055は現在実装されていません. 
/// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされます. 
void mrd_wire0_read_ahrs_i2c() // ※wireTimer0.beginの引数のためvoid必須
{
    if (MOUNT_IMUAHRS == MPU6050_IMU) // MPU6050
    {
        if (ahrs.mpu6050.dmpGetCurrentFIFOPacket(ahrs.fifoBuffer))
        { // Get new data
            ahrs.mpu6050.dmpGetQuaternion(&ahrs.q, ahrs.fifoBuffer);
            ahrs.mpu6050.dmpGetGravity(&ahrs.gravity, &ahrs.q);
            ahrs.mpu6050.dmpGetYawPitchRoll(ahrs.ypr, &ahrs.q, &ahrs.gravity);

            // acceleration values
            ahrs.mpu6050.dmpGetAccel(&ahrs.aa, ahrs.fifoBuffer);
            ahrs.read[0] = (float)ahrs.aa.x;
            ahrs.read[1] = (float)ahrs.aa.y;
            ahrs.read[2] = (float)ahrs.aa.z;

            // gyro values
            ahrs.mpu6050.dmpGetGyro(&ahrs.gyro, ahrs.fifoBuffer);
            ahrs.read[3] = (float)ahrs.gyro.x;
            ahrs.read[4] = (float)ahrs.gyro.y;
            ahrs.read[5] = (float)ahrs.gyro.z;

            // magnetic field values
            ahrs.read[6] = (float)ahrs.mag.x;
            ahrs.read[7] = (float)ahrs.mag.y;
            ahrs.read[8] = (float)ahrs.mag.z;

            // Estimated gravity DMP value.
            ahrs.read[9] = ahrs.gravity.x;
            ahrs.read[10] = ahrs.gravity.y;
            ahrs.read[11] = ahrs.gravity.z;

            // Estimated heading value using DMP.
            ahrs.read[12] = ahrs.ypr[2] * 180 / M_PI;                     // Estimated DMP_ROLL
            ahrs.read[13] = ahrs.ypr[1] * 180 / M_PI;                     // Estimated DMP_PITCH
            ahrs.read[14] = (ahrs.ypr[0] * 180 / M_PI) - ahrs.yaw_origin; // Estimated DMP_YAW

            // Temperature
            ahrs.read[15] = 0; // Not implemented.

            if (flg.imuahrs_available)
            {
                memcpy(ahrs.result, ahrs.read, sizeof(float) * 16);
            }
        }
    }
    else if (MOUNT_IMUAHRS == MPU9250_IMU) // MPU9250
    {
        ; //
    }
    else if (MOUNT_IMUAHRS == BNO055_AHRS) // BNO055
    {
        ; //
    }
}

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定します. 
/// @param mrd_i2c0_speed I2C通信のクロック速度です. 
void mrd_wire0_init_common(int mrd_i2c0_speed)
{
    Serial.print("Initializing wire0 I2C...");
    Wire.begin();
    Wire.setClock(mrd_i2c0_speed);
}

/// @brief MPU6050センサーのDMP（デジタルモーションプロセッサ）を初期化し, ジャイロスコープと加速度センサーのオフセットを設定します. 
/// @param mrd_w_ahrs AHRSの値を保持する構造体. 
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返します. 
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &mrd_w_ahrs)
{
    mrd_w_ahrs.mpu6050.initialize();
    mrd_w_ahrs.devStatus = mrd_w_ahrs.mpu6050.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mrd_w_ahrs.mpu6050.setXAccelOffset(-1745);
    mrd_w_ahrs.mpu6050.setYAccelOffset(-1034);
    mrd_w_ahrs.mpu6050.setZAccelOffset(966);
    mrd_w_ahrs.mpu6050.setXGyroOffset(176);
    mrd_w_ahrs.mpu6050.setYGyroOffset(-6);
    mrd_w_ahrs.mpu6050.setZGyroOffset(-25);

    // make sure it worked (returns 0 if so)
    if (mrd_w_ahrs.devStatus == 0)
    {
        mrd_w_ahrs.mpu6050.CalibrateAccel(6);
        mrd_w_ahrs.mpu6050.CalibrateGyro(6);
        mrd_w_ahrs.mpu6050.setDMPEnabled(true);
        mrd_w_ahrs.packetSize = mrd_w_ahrs.mpu6050.dmpGetFIFOPacketSize();
        Serial.println("MPU6050 OK.");
        return true;
    }
    Serial.println("IMU/AHRS DMP Initialization FAILED!");
    return false;
}

/// @brief BNO055センサーの初期化を試みます. 
/// @param mrd_w_ahrs AHRSの値を保持する構造体. 
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返します. 
///         現在, この関数は常にfalseを返すように設定されています. 
bool mrd_wire0_init_bno055(AhrsValue &mrd_w_ahrs)
{
    /*
    else if (mount_ahrs == 3) // BNO055の場合
    {
        // BNO055の初期化
        if (!bno.begin())
        {
            Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        }
        else
        {
            Serial.println("BNO055 mounted.");
            delay(50);
            bno.setExtCrystalUse(false);
            delay(10);
            // _ahrsTimer.begin(IMU_ahrs_getYawPitchRoll, int(IMU_ahrs_POLLING * 1000)); // インターバルはマイクロ秒指定
        }
        // データの取得はセンサー用スレッドで実行?
    }
    */
    return false;
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います. 
/// @param mrd_imuahrs_type 使用するセンサのタイプを示す列挙型です（MPU6050, MPU9250, BNO055）. 
/// @param mrd_i2c0_speed I2C通信のクロック速度です. 
/// @param mrd_w_ahrs AHRSの値を保持する構造体. 
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返します. 
bool mrd_wire0_setup(ImuAhrsType mrd_imuahrs_type, int mrd_i2c0_speed, AhrsValue &mrd_w_ahrs)
{
    if (mrd_imuahrs_type > 0) // 何らかのセンサを搭載
    {
        mrd_wire0_init_common(mrd_i2c0_speed);
    }

    if (mrd_imuahrs_type == MPU6050_IMU) // MPU6050
    {
        return mrd_wire0_init_mpu6050_dmp(mrd_w_ahrs);
    }
    else if (mrd_imuahrs_type == MPU9250_IMU) // MPU9250の場合
    {
        // mrd_wire_init_mpu9250_dmp(mrd_w_ahrs)
        return false;
    }
    else if (mrd_imuahrs_type == BNO055_AHRS) // BNO055の場合
    {
        // mrd_wire0_init_bno055(mrd_w_ahrs)
        return false;
    }

    Serial.println("No IMU/AHRS sensor mounted.");
    return false;
}

//------------------------------------------------------------------------------------
//  IntervalTimerTimerのスタート
//------------------------------------------------------------------------------------

/// @brief AHRSセンサーからデータを読み取るための関数を実行します. 
void mrd_wire0_run() // ※IntervalTimer用の関数のためvoidのみ可
{
    mrd_wire0_read_ahrs_i2c();
}

/// @brief インターバルタイマーを設定して, 定期的にAHRSセンサーからのデータ読み取りを行うための関数を実行します. 
/// @param mrd_wr_check タイマーを開始するかどうかの条件を示すブール値です. 
/// @return タイマーが正しく開始された場合はtrue, それ以外の場合はfalseを返します. 
bool mrd_wire0_startIntervalTimer(bool mrd_wr_check)
{
    if (mrd_wr_check)
    {
        bool rlst = wireTimer0.begin(mrd_wire0_run, IMUAHRS_INTERVAL * 1000); // インターバルはマイクロ秒指定
                                                                              // wireTimer.priority(90);
        return rlst;
    }
    return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータをメリディム配列に格納します. 
/// @param mrd_imuahrs_type 使用するセンサのタイプを示す列挙型です（MPU6050, MPU9250, BNO055）. 
/// @param mrd_w_meridim メリディム配列. 計測データを格納します. 
/// @param mrd_w_ahrs_result AHRSから読み取った結果を格納した配列. 
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返します. 
bool mrd_wire0_write_ahrs_data(ImuAhrsType mrd_imuahrs_type, short mrd_w_meridim[], float mrd_w_ahrs_result[])
{
    if (mrd_imuahrs_type == MPU6050_IMU)
    {
        flg.imuahrs_available = false;
        mrd_w_meridim[2] = mrd.float2HfShort(mrd_w_ahrs_result[0]);   // IMU/AHRS_acc_x
        mrd_w_meridim[3] = mrd.float2HfShort(mrd_w_ahrs_result[1]);   // IMU/AHRS_acc_y
        mrd_w_meridim[4] = mrd.float2HfShort(mrd_w_ahrs_result[2]);   // IMU/AHRS_acc_z
        mrd_w_meridim[5] = mrd.float2HfShort(mrd_w_ahrs_result[3]);   // IMU/AHRS_gyro_x
        mrd_w_meridim[6] = mrd.float2HfShort(mrd_w_ahrs_result[4]);   // IMU/AHRS_gyro_y
        mrd_w_meridim[7] = mrd.float2HfShort(mrd_w_ahrs_result[5]);   // IMU/AHRS_gyro_z
        mrd_w_meridim[8] = mrd.float2HfShort(mrd_w_ahrs_result[6]);   // IMU/AHRS_mag_x
        mrd_w_meridim[9] = mrd.float2HfShort(mrd_w_ahrs_result[7]);   // IMU/AHRS_mag_y
        mrd_w_meridim[10] = mrd.float2HfShort(mrd_w_ahrs_result[8]);  // IMU/AHRS_mag_z
        mrd_w_meridim[11] = mrd.float2HfShort(mrd_w_ahrs_result[15]); // temperature
        mrd_w_meridim[12] = mrd.float2HfShort(mrd_w_ahrs_result[12]); // DMP_ROLL推定値
        mrd_w_meridim[13] = mrd.float2HfShort(mrd_w_ahrs_result[13]); // DMP_PITCH推定値
        mrd_w_meridim[14] = mrd.float2HfShort(mrd_w_ahrs_result[14]); // DMP_YAW推定値
        flg.imuahrs_available = true;
        return true;
    }
    else if (mrd_imuahrs_type == MPU9250_IMU)
    {
        //
        return false;
    }
    else if (mrd_imuahrs_type == BNO055_AHRS)
    {
        //
        return false;
    }
    return false;
}

/// @brief 現在のヨー角を基準値として設定し, 適切なセンサの単位に変換します. 
/// @param yaw 現在のヨー角をラジアン単位で指定します. 
/// @return 変換されたヨー角を度単位で返します. 対応していないセンサの場合は0を返します. 
float mrd_wire0_setyaw(float yaw)
{
    if (MOUNT_IMUAHRS == MPU6050_IMU) // MPU6050
    {
        return yaw * 180 / M_PI;
    }
    else if (MOUNT_IMUAHRS == MPU9250_IMU) // MPU9250
    {
        return 0;
    }
    else if (MOUNT_IMUAHRS == BNO055_AHRS) // BNO055
        return 0;
    {
        return 0;
    }
}

#endif
