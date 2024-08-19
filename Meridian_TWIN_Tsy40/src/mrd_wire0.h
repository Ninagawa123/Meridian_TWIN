#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

IntervalTimer wireTimer0;

//================================================================================================================
//  I2C wire0 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
///        MPU6050, MPU9250, BNO055に対応予定だが, MPU9250とBNO055は現在実装されていない.
///        各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
void mrd_wire0_read_ahrs_i2c() // ※wireTimer0.beginの引数のためvoid必須
{
  if (MOUNT_IMUAHRS == MPU6050_IMU) // MPU6050
  {
    if (ahrs.mpu6050.dmpGetCurrentFIFOPacket(ahrs.fifoBuffer)) { // Get new data
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

      if (flg.imuahrs_available) {
        memcpy(ahrs.result, ahrs.read, sizeof(float) * 16);
      }
    }
  } else if (MOUNT_IMUAHRS == MPU9250_IMU) // MPU9250
  {
    ;                                      //
  } else if (MOUNT_IMUAHRS == BNO055_AHRS) // BNO055
  {
    ; //
  }
}

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c_bps I2C通信のクロック速度.
void mrd_wire0_init_common(int a_i2c_bps) {
  Serial.print("Initializing wire0 I2C...");
  Wire.begin();
  Wire.setClock(a_i2c_bps);
}

/// @brief MPU6050センサーのDMP（デジタルモーションプロセッサ）を初期化し,
/// ジャイロスコープと加速度センサーのオフセットを設定する.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &a_ahrs) {
  a_ahrs.mpu6050.initialize();
  a_ahrs.devStatus = a_ahrs.mpu6050.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  a_ahrs.mpu6050.setXAccelOffset(-1745);
  a_ahrs.mpu6050.setYAccelOffset(-1034);
  a_ahrs.mpu6050.setZAccelOffset(966);
  a_ahrs.mpu6050.setXGyroOffset(176);
  a_ahrs.mpu6050.setYGyroOffset(-6);
  a_ahrs.mpu6050.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (a_ahrs.devStatus == 0) {
    a_ahrs.mpu6050.CalibrateAccel(6);
    a_ahrs.mpu6050.CalibrateGyro(6);
    a_ahrs.mpu6050.setDMPEnabled(true);
    a_ahrs.packetSize = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサーの初期化を試みる.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
///         現在, この関数は常にfalseを返すように設定されている.
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs) {
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
          // _ahrsTimer.begin(IMU_ahrs_getYawPitchRoll, int(IMU_ahrs_POLLING * 1000)); //
  インターバルはマイクロ秒指定
      }
      // データの取得はセンサー用スレッドで実行?
  }
  */
  return false;
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行う.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型（MPU6050, MPU9250, BNO055）.
/// @param a_i2c_bps I2C通信のクロック速.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c_bps, AhrsValue &a_ahrs) {
  if (a_imuahrs_type > 0) // 何らかのセンサを搭載
  {
    mrd_wire0_init_common(a_i2c_bps);
  }

  if (a_imuahrs_type == MPU6050_IMU) // MPU6050
  {
    return mrd_wire0_init_mpu6050_dmp(a_ahrs);
  } else if (a_imuahrs_type == MPU9250_IMU) // MPU9250の場合
  {
    // mrd_wire_init_mpu9250_dmp(a_ahrs)
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) // BNO055の場合
  {
    // mrd_wire0_init_bno055(a_ahrs)
    return false;
  }
  // Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  IntervalTimerのスタート
//------------------------------------------------------------------------------------

/// @brief AHRSセンサーからデータを読み取るための関数を実行する.
void mrd_wire0_run() // ※IntervalTimer用の関数のためvoidのみ可
{
  mrd_wire0_read_ahrs_i2c();
}

/// @brief インターバルタイマーを設定し,
/// 定期的にAHRSセンサーからのデータ読み取りを行う関数を実行する.
/// @param a_wr_check タイマーを開始するかどうかの条件を示すブール値.
/// @return タイマーが正しく開始された場合はtrue, それ以外の場合はfalseを返す.
bool mrd_wire0_intervaltimer_start(bool a_wr_check) {
  if (a_wr_check) {
    bool rlst =
        wireTimer0.begin(mrd_wire0_run, IMUAHRS_INTERVAL * 1000); // インターバルはマイクロ秒指定
                                                                  // wireTimer.priority(90);
    return rlst;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  meridimへの書き込み処理
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む.
/// @param a_type 使用するセンサのタイプを示す列挙（MPU6050, MPU9250, BNO055）.
/// @param a_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool mrd_meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[]) {
  // if (a_type == MPU6050_IMU) {
  flg.imuahrs_available = false;
  a_meridim.sval[2] = mrd.float2HfShort(a_ahrs_result[0]);   // IMU/AHRS_acc_x
  a_meridim.sval[3] = mrd.float2HfShort(a_ahrs_result[1]);   // IMU/AHRS_acc_y
  a_meridim.sval[4] = mrd.float2HfShort(a_ahrs_result[2]);   // IMU/AHRS_acc_z
  a_meridim.sval[5] = mrd.float2HfShort(a_ahrs_result[3]);   // IMU/AHRS_gyro_x
  a_meridim.sval[6] = mrd.float2HfShort(a_ahrs_result[4]);   // IMU/AHRS_gyro_y
  a_meridim.sval[7] = mrd.float2HfShort(a_ahrs_result[5]);   // IMU/AHRS_gyro_z
  a_meridim.sval[8] = mrd.float2HfShort(a_ahrs_result[6]);   // IMU/AHRS_mag_x
  a_meridim.sval[9] = mrd.float2HfShort(a_ahrs_result[7]);   // IMU/AHRS_mag_y
  a_meridim.sval[10] = mrd.float2HfShort(a_ahrs_result[8]);  // IMU/AHRS_mag_z
  a_meridim.sval[11] = mrd.float2HfShort(a_ahrs_result[15]); // temperature
  a_meridim.sval[12] = mrd.float2HfShort(a_ahrs_result[12]); // DMP_ROLL推定値
  a_meridim.sval[13] = mrd.float2HfShort(a_ahrs_result[13]); // DMP_PITCH推定値
  a_meridim.sval[14] = mrd.float2HfShort(a_ahrs_result[14]); // DMP_YAW推定値
  flg.imuahrs_available = true;
  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 現在のヨー角を基準値として設定し, 適切なセンサの単位に変換する.
/// @param yaw 現在のヨー角をラジアン単位で指定する.
/// @return 変換されたヨー角を度単位で返す. 対応していないセンサの場合は0を返す.
float mrd_wire0_setyaw(float yaw) {
  if (MOUNT_IMUAHRS == MPU6050_IMU) // MPU6050
  {
    return yaw * 180 / M_PI;
  } else if (MOUNT_IMUAHRS == MPU9250_IMU) // MPU9250
  {
    return 0;
  } else if (MOUNT_IMUAHRS == BNO055_AHRS) // BNO055
    return 0;
  { return 0; }
}

#endif // __MERIDIAN_WIRE0_H__
