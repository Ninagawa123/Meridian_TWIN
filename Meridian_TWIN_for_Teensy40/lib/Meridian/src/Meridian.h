/*Meridian.h
  This is a library to drive a communication system for humanoid robots.
  It operates numerous servo motors and 9-axis sensors at 100 Hz and links status information with a PC in real time.
  Created by Ninagawa123. October 30, 2022.
  MIT license.*/

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
          short cksm_val(short arr[], int len);
          bool cksm_rslt(short arr[], int len);
          short float2HfShort(float val);
          float HfShort2float(short val);
          int Deg2Krs(float degree, float trim, int cw);
          float Krs2Deg(int krs, float trim);
          int HfDeg2Krs(int hfdegree, float trim, int cw);
          int Krs2HfDeg(int krs, float trim, int cw);
          float RSxx2Deg(int rsxx, float trim, int cw);
          int Deg2RSxx(float degree, float trim, int cw);
          int RSxx2HfDeg(int rsxx, float trim, int cw);
          int HfDeg2RSxx(int degree, float trim, int cw);
          void print_hello_tsy(String version, int spi_speed, int i2c_speed);
          void print_servo_mounts(int idl_svmt[], int idr_svmt[]);
          void print_controlpad(int pad_mount, int pad_freq);
          void print_imuahrs(int imuahrs_mount, int imuahrs_freq);
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