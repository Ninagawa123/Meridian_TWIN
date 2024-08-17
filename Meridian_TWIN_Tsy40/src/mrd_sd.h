#ifndef __MERIDIAN_SD_H__
#define __MERIDIAN_SD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <SD.h> // SDカード用
File sdfile;    // SDカード用

//================================================================================================================
//  SDメモリ 関連の処理
//================================================================================================================

class MrdSdHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdSdHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  初期化処理
  //------------------------------------------------------------------------------------

  /// @brief SDカードの初期化を試みる.
  /// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
  /// @param a_chipselect SDカードのチップ選択ピン番号.
  /// @return SDカードの初期化が成功した場合はtrue, 失敗またはマウントされていない場合はfalse.
  bool init(bool a_sd_mount, int a_chipselect) {
    if (!a_sd_mount) {
      m_serial.println("SD not mounted.");
      delay(100);
      return false;
    }

    const int max_attempts_tmp = 5; // 最大再試行回数
    m_serial.print("Initializing SD card.");

    for (int attempt = 0; attempt < max_attempts_tmp; attempt++) {
      m_serial.print(".");
      delay(100);

      if (SD.begin(a_chipselect)) {
        m_serial.println(" OK.");
        return true;
      } else {
        delay(100); // 再試行までの待機時間
      }
    }

    m_serial.println(" failed.");
    return false;
  }

  //------------------------------------------------------------------------------------
  //  SDリードライトテスト
  //------------------------------------------------------------------------------------

  /// @brief SDカードの読み書き機能をテストする.
  /// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
  /// @param a_chipselect SDカードのチップ選択ピン番号.
  /// @param a_flg_rw SDカードの読み書きをチェックするかどうかのブール値.
  /// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool check_rw(bool a_sd_mount, int a_chipselect, bool a_flg_rw) {
    if (a_sd_mount && a_flg_rw) {
      sdfile = SD.open("/test.txt", FILE_WRITE);
      delay(1); // SPI安定化検証用

      if (sdfile) {
        m_serial.print("Checking SD card r/w...");
        // SD書き込みテスト用のランダムな4桁の数字を生成
        randomSeed(long(analogRead(A0) + analogRead(A1) * 1000 +
                        analogRead(A2) * 1000000)); // 未接続ピンのノイズを利用
        int randNumber = random(1000, 9999);

        m_serial.print(" write code ");
        m_serial.print(randNumber);
        // ファイルへの書き込みを実行
        sdfile.println(randNumber);
        delayMicroseconds(1); // SPI安定化検証用
        sdfile.close();
        m_serial.print(" and");
        delayMicroseconds(10); // SPI安定化検証用
        // ファイルからの読み込みを実行
        sdfile = SD.open("/test.txt");
        if (sdfile) {
          m_serial.print(" read code ");
          while (sdfile.available()) {
            m_serial.write(sdfile.read());
          }
          sdfile.close();
        }
        SD.remove("/test.txt");
        delay(10);
        return true;
      } else {
        m_serial.println("Could not open SD test.txt file.");
      }
    }
    return false;
  }

  //------------------------------------------------------------------------------------
  //  各種オペレーション
  //------------------------------------------------------------------------------------
};

#endif // __MERIDIAN_SD_H__