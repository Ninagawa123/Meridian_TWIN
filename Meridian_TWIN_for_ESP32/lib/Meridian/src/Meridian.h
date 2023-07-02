/**
 *  @file Meridian.h
 *  @brief This is a library for controlling the communication system of humanoid robots.
 *         It manages multiple servo motors and 9-axis sensors at a frequency of 100 Hz and
 *         provides real-time status information to a PC.
 *         Created by Izumi Ninagawa on October 30, 2022.
 *         MIT license.
 *         Detalis: https://github.com/Ninagawa123/Meridian_TWIN
 *  @author Izumi Ninagawa
 *  @date 2023/07/02
 *  @version 1.0.0
 *  @copyright &copy; Izumi Ninagawa & Project Meridian
 */

#ifndef Meridian_h
#define Meridian_h
#include "Arduino.h"

namespace arduino
{
  namespace robotics
  {
    namespace espteensy
    {
      namespace meridian
      {
        class Meridian
        {
        public:
          /**
           * @brief Evaluate checksum of Meridim.
           *
           * @param[in] arr[] Meridim array
           * @param[in] len Length of array
           * @return true Check OK
           * @return false Check NG
           */
          bool cksm_rslt(short arr[], int len);

          /**
           * @brief calculate checksum of Meridim
           *
           * @param[in] arr Meridim array
           * @param[in] len Length of array
           * @return short checksum value
           */
          short cksm_val(short arr[], int len);

          /**
           * @brief Increase sequence number from imput.
           *
           * @param[in] previous_seq_num Previous sequence number.
           * @return int, Expected sequence number. (0 to 59,999)
           */
          int seq_increase_num(int previous_seq_num);

          /**
           * @brief Compare expected seq number and received seq number.(0 to 59,000)
           *
           * @param[in] predict_seq_num Predict sequence number.
           * @param[in] received_seq_num Received sequence number.
           * @return true OK
           * @return false NG
           */
          bool seq_compare_nums(int predict_seq_num, int received_seq_num);

          /**
           * @brief Generate expected sequence number from imput.
           *
           * @param[in] previous_seq_num Previous sequence number.
           * @return int, Expected sequence number. (0 to 59,999)
           */
          int seq_predict_num(int previous_seq_num);

          /**
           * @brief Short type value divided by 100 to float type.
           *
           * @param[in] val Source value
           * @return float Source value / 100
           */
          float HfShort2float(short val);

          /**
           * @brief Evaluate checksum of Meridim.
           *
           * @param[in] arr[] Meridim array
           * @param[in] len Length of array
           * @return true Check OK
           * @return false Check NG
           */
          short float2HfShort(float val);

          /**
           * @brief Degree value to Kondo's KRS Servo value.
           *
           * @param[in] degree Source degree value
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return int, Kondo's KRS Servo value（3500-11500）
           */
          int Deg2Krs(float degree, float trim, int cw);

          /**
           * @brief Hundredfold degree value to Kondo's KRS Servo value.
           *
           * @param[in] hfdegree Source degree value * 100
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return Kondo's KRS Servo value（3500-11500）
           */
          int HfDeg2Krs(int hfdegree, float trim, int cw);

          /**
           * @brief Kondo's KRS Servo value to degree value.
           *
           * @param[in] krs Source Kondo's KRS Servo value（3500-11500）
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return float, degree
           */
          float Krs2Deg(int krs, float trim, int cw);

          /**
           * @brief Kondo's KRS Servo value to hundredfold degree value.
           *
           * @param[in] krs Source Kondo's KRS Servo value（3500-11500）
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return int, degree * 100
           */
          int Krs2HfDeg(int krs, float trim, int cw);

          /**
           * @brief Degree value to Futaba's RSxx Servo value.
           *
           * @param[in] degree Source degree
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return int, Futaba's RSxx Servo value（-1600 to 1600）
           */
          int Deg2RSxx(float degree, float trim, int cw);

          /**
           * @brief Hundredfold degree value to Futaba's RSxx Servo value.
           *
           * @param[in] degree Source degree value * 100
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return int, Futaba's RSxx Servo value（-1600 to 1600）
           */
          int HfDeg2RSxx(int degree, float trim, int cw);

          /**
           * @brief Futaba's RSxx Servo value to degree value.
           *
           * @param[in] rsxx Source Futaba's RSxx Servo value（-1600 to 1600）
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return float, degree
           */
          float RSxx2Deg(int rsxx, float trim, int cw);

          /**
           * @brief Futaba's RSxx Servo value to hundredfold degree value.
           *
           * @param[in] rsxx Source futaba's RSxx Servo value（-1600 to 1600）
           * @param[in] trim Trim degree value
           * @param[in] cw Correction value for direction of rotation（+1 or -1）
           * @return int, degree * 100
           */
          int RSxx2HfDeg(int rsxx, float trim, int cw);

          /**
           * @brief Print version, I2C speed, SPI speed.
           *
           * @param[in] version This　Merisian's VERSION
           * @param[in] spi_speed SPI speed
           * @param[in] i2c_speed I2C speed
           */
          void print_tsy_hello(String version, int spi_speed, int i2c_speed);

          /**
           * @brief Print wake-up massage to serial.
           *
           * @param[in] version VESRION of Meridian.
           * @param[in] serial_pc_bps PC-Serial speed.
           * @param[in] wifi_ap_ssid Wi-Fi AP adress to connect.
           */
          void print_esp_hello_start(String version, String serial_pc_bps, String wifi_ap_ssid);

          /**
           * @brief Print Wi-Fi status to serial.
           *
           * @param[in] wifi_send_ip Send Wi-Fi IP. (PC's IP adress)
           * @param[in] wifi_localip Wi-Fi IP of this device.
           * @param[in] fixed_ip_addr Fixed Wi-Fi IP of this device. (If mode_fixed_ip is True)
           * @param[in] mode_fixed_ip Use fixed Wi-Fi IP or Not.
           */
          void print_esp_hello_ip(String wifi_send_ip, String wifi_localip, String fixed_ip_addr, bool mode_fixed_ip);

          /**
           * @brief Print IMU/AHRS sensor's type to serial.
           *
           * @param[in] imuahrs_mount imuahrs_mount number
           *                          0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
           * @param[in] imuahrs_freq Frequency to calling imu/ahrs via I2C.
           */
          void print_imuahrs(int imuahrs_mount, int imuahrs_freq);

          /**
           * @brief Print mounted servomotor's id.
           *
           * @param[in] idl_svmt Left side servos arrey.
           * @param[in] idr_svmt Right side servos arrey.
           * @param[in] id3_svmt 3rd side servos arrey.
           */
          void print_servo_mounts(int idl_svmt[], int idr_svmt[], int id3_svmt[]);

          /**
           * @brief Print mounted control pad.
           *
           * @param[in] pad_mount Type of control pad.
           * @param[in] pad_freq Freqency of calling control pad.(every ms)
           */
          void print_controlpad(int pad_mount, int pad_freq);

          /**
           * @brief Show text massage if monitor_flow is true. This is for debagging.
           *
           * @param[in] text Text message to display.
           * @param[in] monitor_flow True:on ,False;off
           */
          void monitor_check_flow(const String &text, bool monitor_flow);

          /**
           * @brief Display gamepad keys.
           *
           * @param[in] arr array[3]
           */
          void monitor_joypad(uint16_t *arr);

          /**
           * @brief Show servo error id.
           *
           * @param[in] text String. "L","R","C",etc
           * @param[in] num Servo id.
           * @param[in] monitor_servo_error True:on ,False;off
           */
          void monitor_servo_error(const String &text, int num, bool monitor_servo_error);
        };
      } // Meridian
    }   // espteensy
  }     // robotics
} // arduino

#ifndef ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_BEGIN
#define ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_BEGIN \
  namespace arduino                                         \
  {                                                         \
    namespace robotics                                      \
    {                                                       \
      namespace espteensy                                   \
      {                                                     \
        namespace meridian                                  \
        {
#endif

#ifndef ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_END
#define ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_END \
  }                                                       \
  }                                                       \
  }                                                       \
  }
#endif

namespace MERIDIANFLOW = arduino::robotics::espteensy::meridian;

#endif // Meridian_h
