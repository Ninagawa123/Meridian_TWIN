#ifndef __MERIDIAN_DISPLEY_MESSAGE_H__
#define __MERIDIAN_DISPLEY_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_util.h"

//================================================================================================================
//  シリアルモニタリング出力関連の処理
//================================================================================================================

class MrdMsgHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdMsgHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  起動時メッセージ
  //------------------------------------------------------------------------------------

  /// @brief 指定された秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
  void charging(int duration_ms) {
    m_serial.print("Charging the capacitor.");
    for (int i = 0; i < duration_ms; i++) {
      if (i % 100 == 0) {
        m_serial.print(".");
      }
      delay(1);
    }
    m_serial.println();
  }

  /// @brief システムのバージョン情報と通信速度の設定をシリアルモニタに出力する.
  /// @param a_version システムのバージョンを示す文字列.
  /// @param a_pc PCとの有線シリアル接続速度(bps).
  /// @param a_spi0 SPI0の通信速度(Hz).
  /// @param a_i2c0 I2C0の通信速度(Hz).
  /// @param a_i2c1 I2C1の通信速度(Hz).
  void hello_tsy(String a_version, int a_pc, int a_spi0, int a_i2c0, int a_i2c1) {
    m_serial.println();
    m_serial.print("Hi, This is ");
    m_serial.println(a_version);
    m_serial.print("Set PC-USB speed: ");
    m_serial.println(a_pc);
    m_serial.print("Set SPI0 speed  : ");
    m_serial.println(a_spi0);
    m_serial.print("Set i2c0 speed  : ");
    m_serial.println(a_i2c0);
    m_serial.print("Set i2c1 speed  : ");
    m_serial.println(a_i2c1);
  }

  /// @brief マウント設定したジョイパッドのタイプをシリアルモニタに出力する.
  /// @param a_mount_pad パッドの定義(PC,MERIMOTE,BLUERETRO,SBDBT,KRR5FH,WIIMOTE)
  void mounted_pad(int a_mount_pad) {
    m_serial.print("Pad Receiver mounted : ");
    if (a_mount_pad == MERIMOTE) {
      m_serial.println("Merimote.");
    } else if (a_mount_pad == BLUERETRO) {
      m_serial.println("BlueRetro.");
    } else if (a_mount_pad == SBDBT) {
      m_serial.println("SBDBT.");
    } else if (a_mount_pad == KRR5FH) {
      m_serial.println("KRC-5FH.");
    } else {
      m_serial.println("None (PC).");
    }
  }

  /// @brief マウント設定したサーボのbpsをシリアルモニタに出力する.
  /// @param a_servo_l L系統のサーボbps.
  /// @param a_servo_r R系統のサーボbps.
  /// @param a_servo_c C系統のサーボbps.
  void servo_bps_3lines(int a_servo_l, int a_servo_r, int a_servo_c) {
    m_serial.print("Set UART_L ");
    m_serial.print(a_servo_l);
    m_serial.println(" bps");
    m_serial.print("Set UART_R ");
    m_serial.print(a_servo_r);
    m_serial.println(" bps");
    m_serial.print("Set UART_C ");
    m_serial.print(a_servo_c);
    m_serial.println(" bps");
  }

  /// @brief 指定されたUARTラインのサーボIDを表示する.
  /// @param a_label UARTラインのラベル.
  /// @param a_max サーボの最大数.
  /// @param a_mount サーボのマウント状態を示す配列.
  /// @param a_id サーボIDの配列.
  void print_servo_ids(const char *a_label, int a_max, int *a_mount, const int *a_id) {
    m_serial.print(a_label);
    for (int i = 0; i <= a_max; i++) {
      if (a_mount[i] != 0) {
        if (a_id[i] < 10) {
          m_serial.print(" ");
        }
        m_serial.print(a_id[i]);
      } else {
        m_serial.print("__");
      }
      m_serial.print(" ");
    }
    m_serial.println();
  }

  /// @brief マウントされているサーボのIDを表示する.
  /// @param a_sv サーボパラメータの構造体.
  void servo_mounts_3lines(ServoParam a_sv) {
    print_servo_ids("UART_L Servos mounted: ", a_sv.num_max, a_sv.ixl_mount, a_sv.ixl_id);
    print_servo_ids("UART_R Servos mounted: ", a_sv.num_max, a_sv.ixr_mount, a_sv.ixr_id);
    print_servo_ids("UART_C Servos mounted: ", a_sv.num_max, a_sv.ixc_mount, a_sv.ixc_id);
  }

  /// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを表示する.
  /// @param a_line UART通信ライン（L, R, C）.
  /// @param a_servo_type サーボのタイプを示す整数値.
  void servo_protcol(UartLine a_line, int a_servo_type) {
    if (a_servo_type > 0) {
      m_serial.print("Set UART_");
      m_serial.print(mrd_get_line_name(a_line));
      m_serial.print(" protocol : ");

      switch (a_servo_type) {
      case 1:
        m_serial.print("single PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 11:
        m_serial.print("I2C_PCA9685 to PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 21:
        m_serial.print("RSxTTL (FUTABA)");
        m_serial.println(" - Not supported yet.");
        break;
      case 31:
        m_serial.print("DYNAMIXEL Protocol 1.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 32:
        m_serial.print("DYNAMIXEL Protocol 2.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 43:
        m_serial.println("ICS3.5/3.6(KONDO,KRS)");
        break;
      case 44:
        m_serial.print("PMX(KONDO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 51:
        m_serial.print("XBUS(JR PROPO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 61:
        m_serial.print("STS(FEETECH)");
        m_serial.println(" - Not supported yet.");
        break;
      case 62:
        m_serial.print("SCS(FEETECH)");
        m_serial.println(" - Not supported yet.");
        break;
      default:
        m_serial.println(" Not defined. ");
        break;
      }
    }
  }

  /// @brief システム内の各経路の通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
  /// @param a_meridim Meridim配列の共用体.
  /// @param a_err エラーカウント管理用の構造体
  /// @param a_mrdsq シーケンス番号管理用の構造体.
  void err_monitor(const Meridim90Union &a_meridim, const MrdErr &a_err, const MrdSq &a_mrdsq) {
    m_serial.print("[ERRs] esp>pc:");
    m_serial.print(a_err.esp_pc);
    m_serial.print(" pc>esp:");
    m_serial.print(a_err.pc_esp);
    m_serial.print(" esp>tsy:");
    m_serial.print(a_err.esp_tsy);
    m_serial.print(" tsy>esp:");
    m_serial.print(a_err.esp_tsy);
    m_serial.print(" tsySkip:");
    m_serial.print(a_err.tsy_skip);
    m_serial.print(" espSkip:");
    m_serial.print(a_err.esp_skip);
    m_serial.print(" pcSkip:");
    m_serial.print(a_err.pc_skip);
    m_serial.print(" seq:");
    m_serial.print(int(a_mrdsq.r_expect));
    m_serial.print(" [u]:");
    m_serial.print(a_meridim.bval[MRD_ERR_u], BIN);
    m_serial.println();
  }

  /// @brief システムに接続されているIMU/AHRSセンサーのタイプを表示する.
  /// @param a_type 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  void imuahrs(ImuAhrsType a_type) {
    m_serial.print("IMU/AHRS Sensor mounted: ");
    switch (a_type) {
    case NO_IMU:
      m_serial.println("None.");
      break;
    case MPU6050_IMU:
      m_serial.println("MPU6050(GY-521)");
      break;
    case MPU9250_IMU:
      m_serial.println("MPU9250(GY-6050/GY-9250)");
      break;
    case BNO055_AHRS:
      m_serial.println("BNO055");
      break;
    default:
      break;
    }
  }

  /// @brief システムの動作開始を示すメッセージを出力する.
  void flow_start() {
    m_serial.println();
    m_serial.println("-) Meridian TWIN system on side Teensy now flows. (-");
  }

  //------------------------------------------------------------------------------------
  //  イベントメッセージ
  //------------------------------------------------------------------------------------
  /// @brief 遅延時間をシリアルモニタに出力する.
  /// @param a_delay_time 表示する値.
  /// @param a_flg_disp 表示を実施するかどうか.
  void frame_delay(unsigned long a_delay_time, bool a_flg_disp) {
    if (a_flg_disp) {
      m_serial.print("deley: ");
      m_serial.print(a_delay_time);
      m_serial.println("ms");
    }
  }

  /// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示する.
  /// @param a_line サーボモーターが接続されているUARTライン（L, R, C）.
  /// @param a_num エラーが発生しているサーボの番号.
  /// @param a_flg_disp エラーメッセージを表示するかどうかのブール値.
  /// @return エラーメッセージが表示された場合はtrueを, 表示されなかった場合はfalseを返す.
  bool servo_err(UartLine a_line, int a_num, bool a_flg_disp) {
    if (a_flg_disp) {
      m_serial.print("Found servo err ");
      if (a_line == L) {
        m_serial.print("L_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == R) {
        m_serial.print("R_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == C) {
        m_serial.print("C_");
        m_serial.println(a_num);
        return true;
      }
    }
    return false;
  }
};

#endif // __MERIDIAN_DISPLEY_MESSAGE_H__