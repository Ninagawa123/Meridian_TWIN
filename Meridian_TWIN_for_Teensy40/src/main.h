#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <cstdint>

/**
 * @brief Initialize sensors like MPU6050, BNO055, and others.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setupIMUAHRS();

/**
 * @brief Store values for MPU6050, BNO055, and other sensors.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void IMUAHRS_getYawPitchRoll();

/**
 * @brief Receive input data from the gamepad and return it in PS2/3 gamepad array format.
 *
 * @param mount_joypad  Gamepad type (currently only 2: KRC-5FH).
 * @param pre_val Previous received value (8 bytes, assuming union data).
 * @param polling Frame count for inquiry frequency.
 * @param joypad_reflesh 1:To reset the JOYPAD's received button data to 0 with this device
 *                       0:perform logical addition without resetting .(usually 1)
 * @return uint64_t
 */
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh);

/**
 * @brief Initializing and performing read/write tests for an SD card.
 *
 */
void check_sd();

/**
 * @brief Displaying the error detection count on Teensy's serial interface.
 *
 */
void print_error_monitor();

/**
 * @brief Counting up the communication error detection count.
 *
 */
void countup_errors();

/**
 * @brief Starting the IMU sensor or AHRS sensor.
 *
 */
void imuahrs_start();

/**
 * @brief Storing the values of the IMU sensor and AHRS sensor in an array.
 *
 */
void imuahrs_store();

/**
 * @brief Execute mastercommands.
 *
 */
void execute_MasterCommand();

/**
 * @brief Resetting the origin of the yaw axis.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 */
void setyaw();

/**
 * @brief Powering off all servos.
 *
 */
void servo_all_off();

#endif
