#include "Meridian.h"

ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_BEGIN

// +----------------------------------------------------------------------
// Meridim is an array of Meridian System to control robots.
// A standard Meridian array consists of 90 Short type values.
// Detalis: https://github.com/Ninagawa123/Meridian_TWIN
// +----------------------------------------------------------------------

// +----------------------------------------------------------------------
// | func name : cksm_val(short arr[], int len)
// +----------------------------------------------------------------------
// | function  : calculate checksum of Meridim
// | argument1 : short, Meridim array
// | argument2 : int,   Length of array
// | return    : short, checksum value
// +----------------------------------------------------------------------
short Meridian::cksm_val(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    return short(~cksm);
}

// +----------------------------------------------------------------------
// | func name : cksm_rslt(short arr[], int len)
// +----------------------------------------------------------------------
// | function  : evaluate checksum of Meridim
// | argument1 : short, Meridim array
// | argument2 : int,   Length of array
// | return    : bool,  OK is True, NG is False
// +----------------------------------------------------------------------
bool Meridian::cksm_rslt(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    if (short(~cksm) == arr[len - 1])
    {
        return true;
    }
    return false;
}

// +----------------------------------------------------------------------
// | func name : float2HfShort(float val)
// +----------------------------------------------------------------------
// | function  : Float type value multiplied by 100 to short type
// | argument  : float, -327.67 to 327.67
// | return    : short, -32767 to 32767, Returns a value within the limit
// +----------------------------------------------------------------------
short Meridian::float2HfShort(float val)
{
    int x = round(val * 100);
    if (x > 32766)
    {
        x = 32767;
    }
    else if (x < -32766)
    {
        x = -32767;
    }
    return (short)x;
}

// +----------------------------------------------------------------------
// | func name : HfShort2float(short val)
// +----------------------------------------------------------------------
// | function  : Short type value divided by 100 for float type
// | argument  : short
// | return    : float
// +----------------------------------------------------------------------
float Meridian::HfShort2float(short val)
{
    return (float)val / 100;
}

// +----------------------------------------------------------------------
// | func name : Krs2Deg(int krs, int n, int pn)
// +----------------------------------------------------------------------
// | function  : Kondo's KRS Servo value to degree value.
// | argument1 : int, Kondo's KRS Servo value（3500-11500）
// | argument2 : float, Degree trim value
// | return    : float, degree
// +----------------------------------------------------------------------
float Meridian::Krs2Deg(int krs, float trim)
{
    float x = ((krs - 7500 - (trim * 29.62963)) * 0.03375);
    return float(x);
}

// +----------------------------------------------------------------------
// | func name : Deg2Krs(int hfdegree, float n, int pn)
// +----------------------------------------------------------------------
// | function  : Degree value to Kondo's KRS Servo value.
// | argument1 : float, degree
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : int, Kondo's KRS Servo value（3500-11500）
// +----------------------------------------------------------------------
int Meridian::Deg2Krs(float degree, float trim, int cw)
{
    float x = 7500 + (trim * 29.6296) + (degree * 29.6296 * cw); //
    if (x > 11500)                                               // 上限を設定
    {
        x = 11500;
    }
    else if (x < 3500) // 下限を設定
    {
        x = 3500;
    }
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : HfDeg2Krs(int hfdegree, int n, int pn)
// +----------------------------------------------------------------------
// | function  : Hundredfold degree value to Kondo's KRS Servo value.
// | argument1 : int, degree * 100
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : int, Kondo's KRS Servo value（3500-11500）
// +----------------------------------------------------------------------
int Meridian::HfDeg2Krs(int hfdegree, float trim, int cw)
{
    float x = 7500 + (trim * 29.6296) + (hfdegree * 0.296296 * cw); //
    if (x > 11500)                                                  // 上限を設定
    {
        x = 11500;
    }
    else if (x < 3500) // 下限を設定
    {
        x = 3500;
    }
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : Krs2HfDeg(int krs, float n, int pn)
// +----------------------------------------------------------------------
// | function  : Kondo's KRS Servo value to hundredfold degree value.
// | argument1 : int, Kondo's KRS Servo value（3500-11500）
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : int, degree * 100
// +----------------------------------------------------------------------
int Meridian::Krs2HfDeg(int krs, float n, int cw)
{
    float x = (krs - 7500 - (n * 29.62963)) * 3.375 * cw;
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : RSxx2Deg(int RSxx, float n, int cw)
// +----------------------------------------------------------------------
// | function  : Futaba's RSxx Servo value to degree value.
// | argument1 : int, Futaba's RSxx Servo value（-1600 to 1600）
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : float, degree
// +----------------------------------------------------------------------
float Meridian::RSxx2Deg(int rsxx, float trim, int cw)
{
    float x = (rsxx - (trim * 10)) * 0.1 * cw;
    return float(x);
}

// +----------------------------------------------------------------------
// | func name : RSxx2HfDeg(int RSxx, float n, int cw)
// +----------------------------------------------------------------------
// | function  : Futaba's RSxx Servo value to hundredfold degree value.
// | argument1 : int, Futaba's RSxx Servo value（-1600 to 1600）
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : float, degree
// +----------------------------------------------------------------------
int Meridian::RSxx2HfDeg(int rsxx, float trim, int cw)
{
    float x = (rsxx - (trim * 10)) * 10 * cw;
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : Deg2RSxx(float degree, float n, int cw)
// +----------------------------------------------------------------------
// | function  : Degree value to Futaba's RSxx Servo value.
// | argument1 : float, degree
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : int, Futaba's RSxx Servo value（-1600 to 1600）
// +----------------------------------------------------------------------
int Meridian::Deg2RSxx(float degree, float trim, int cw)
{
    float x = (degree + trim) * 10 * cw; //
    if (x > 1600)                        // 上限を設定
    {
        x = 1600;
    }
    else if (x < -1600) // 下限を設定
    {
        x = -1600;
    }
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : HfDeg2RSxx(int degree, float n, int cw)
// +----------------------------------------------------------------------
// | function  : Hundredfold degree value to Futaba's RSxx Servo value.
// | argument1 : int, degree
// | argument2 : float, Degree trim value
// | argument3 : int, correction value for direction of rotation（+1 or -1）
// | return    : int, Futaba's RSxx Servo value（-1600 to 1600）
// +----------------------------------------------------------------------
int Meridian::HfDeg2RSxx(int degree, float trim, int cw)
{
    float x = (degree + trim) * 0.1 * cw; //
    if (x > 1600)                         // 上限を設定
    {
        x = 1600;
    }
    else if (x < -1600) // 下限を設定
    {
        x = -1600;
    }
    return int(x);
}

// +----------------------------------------------------------------------
// | func name : print_hello_tsy(String version,int imuahrs_mount, int imuahrs_freq)
// +----------------------------------------------------------------------
// | function  : print version, I2C speed, SPI speed.
// | argument1 : String version.
// | argument3 : int, SPI speeed.
// | argument2 : int, I2C speeed.
// | return    : none.
// +----------------------------------------------------------------------
void Meridian::print_hello_tsy(String version, int spi_speed, int i2c_speed, int pcserial_speed)
{
    Serial.println();
    Serial.print("Hi, This is ");
    Serial.println(version);
    delay(100);
    Serial.print("PC Serial    : ");
    Serial.print(pcserial_speed);
    Serial.println(" bps");
    Serial.print("Set SPI speed: ");
    Serial.print(spi_speed);
    Serial.println(" Hz");
    Serial.print("Set I2C speed: ");
    Serial.print(i2c_speed);
    Serial.println(" Hz");
}

// +----------------------------------------------------------------------
// | func name : print_servo_mounts(int idl_svmt[], int idr_svmt[])
// +----------------------------------------------------------------------
// | function  : print mounted servomotor's id.
// | argument1 : int, left side servos arrrey.
// | argument2 : int, right side servos arrrey.
// | return    : none.
// +----------------------------------------------------------------------
void Meridian::print_servo_mounts(int idl_svmt[], int idr_svmt[])
{
    Serial.print("Left side Servos mounted:  ");
    for (int i = 0; i < 15; i++)
    {
        if (idl_svmt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();
    Serial.print("Right side Servos mounted: ");
    for (int i = 0; i < 15; i++)
    {
        if (idr_svmt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();
}

// +----------------------------------------------------------------------
// | func name : print_controlpad(int pad_mount, int pad_freq)
// +----------------------------------------------------------------------
// | function  : print mounted control pad.
// | argument1 : int, type of control pad.
// | argument2 : int, freqency of calling control pad.(every ms)
// | return    : none.
// +----------------------------------------------------------------------

void Meridian::print_controlpad(int pad_mount, int pad_freq)
{
    Serial.print("Controll Pad Receiver mounted: ");

    switch (pad_mount)
    {
    case 0:
        Serial.println("None. ");
        break;
    case 1:
        Serial.println("SBDBT ");
        break;
    case 2:
        Serial.println("KRC-5FH ");
        break;
    case 3:
        Serial.println("Merimote PICO ");
        break;
    default:
        break;
    }

    if (pad_mount != 0)
    {
        Serial.print("  freq: ");
        Serial.println(pad_freq);
    }

    delay(100);
}

// +----------------------------------------------------------------------
// | func name : print_imuahrs(int imuahrs_mount, int imuahrs_freq)
// +----------------------------------------------------------------------
// | function  : print imu/ahrs type to serial.
// | argument1 : int, imuahrs_mount number
// |           : 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// | argument2 : int, frequency to calling imu/ahrs via I2C.
// | return    : none.
// +----------------------------------------------------------------------
void Meridian::print_imuahrs(int imuahrs_mount, int imuahrs_freq)
{
    Serial.print("I2C IMU/AHRS Sensor mounted: ");

    switch (imuahrs_mount)
    {
    case 0:
        Serial.print("None. ");
        break;
    case 1:
        Serial.print("MPU6050(GY-521) ");
        break;
    case 2:
        Serial.print("MPU9250(GY-6050/GY-9250) ");
        break;
    case 3:
        Serial.print("BNO055 ");
        break;
    default:
        break;
    }

    if (imuahrs_mount != 0)
    {
        Serial.print("  freq: ");
        Serial.println(imuahrs_freq);
    }
}

// +----------------------------------------------------------------------
// | func name : predict_seq_num(int previous_seq_num)
// +----------------------------------------------------------------------
// | function  : Generate expected sequence number from imput.
// |           : range 0 to 600,000
// | argument  : int, previous sequence number.
// | return    : int, expected sequence number.
// +----------------------------------------------------------------------
int Meridian::predict_seq_num(int previous_seq_num)
{
    int x = previous_seq_num;
    x++;
    if (x > 59999) // 予想値が59,999以上ならカウントを0に戻す
    {
        x = 0;
    }
    return x;
}

// +----------------------------------------------------------------------
// | func name : compare_seq_nums(int previous_seq_num)
// +----------------------------------------------------------------------
// | function  : Generate expected sequence number from imput.
// |           : range 0 to 600,000
// | argument  : int, previous sequence number.
// | return    : bool
// +----------------------------------------------------------------------
bool Meridian::predict_seq_nums(int predict_seq_num, int received_seq_num)
{
    if (predict_seq_num == received_seq_num)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// +----------------------------------------------------------------------
// | func name : convertStringToIPAddress(const char *ip_str)
// +----------------------------------------------------------------------
// | function  : Convert a period-separated IP address string to an IPAddress object.
// | argument  : const char*, period-separated IP address string.
// | return    : IPAddress object.
// +----------------------------------------------------------------------
IPAddress Meridian::makeIPAddress(const char *ip_str)
{
    int a, b, c, d;
    sscanf(ip_str, "%d.%d.%d.%d", &a, &b, &c, &d);
    return IPAddress(a, b, c, d);
}

ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_END
