#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <Arduino.h>

//================================================================================================================
//---- クラス・構造体・共用体  --------------------------------------------------------------------------------------
//================================================================================================================

#define MSG_SIZE 90

// Meridim配列用の共用体の設定
typedef union
{
    short sval[MSG_SIZE + 2];
    unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
    uint8_t bval[(MSG_SIZE * 2) + 4];
} UnionData;

// リモコン用変数
typedef union
{
    short sval[4];       // short型で4個の配列データを持つ
    uint16_t usval[4];   // 上記のunsigned short型
    int8_t bval[8];      // 上記のbyte型
    uint8_t ubval[8];    // 上記のunsigned byte型
    uint64_t ui64val[1]; // 上記のunsigned int16型
                         // button, pad.stick_L_x:pad.stick_L_y,
                         // pad.stick_R_x:pad.stick_R_y, pad.L2_val:pad.R2_val
} UnionPad;

struct PadValue // リモコンの値
{
    unsigned short btn = 0;
    int stick_R = 0;
    int stick_R_x = 0;
    int stick_R_y = 0;
    int stick_L = 0;
    int stick_L_x = 0;
    int stick_L_y = 0;
    int stick_V = 0;
    int R2_val = 0;
    int L2_val = 0;
};

// フラグ関連変数
struct MrdFlags
{
    bool spi_rcvd = true;           // SPIの真データ受信判定
    bool udp_rcvd = true;           // UDPスレッドでの受信完了フラグ
    bool udp_busy = false;          // UDPスレッドでの受信中フラグ（送信抑制）
    bool bt_busy = false;           // UDPスレッドでの受信中フラグ（送信抑制）
    bool udp_board_passive = false; // UDP受信のパッシブモード（0:デフォルト, 1:パッシブ/PC側が通信周期制御）
};

// システム管理用の変数
struct MrdCheck
{
    int seq_r_expect = 0; // フレーム毎に前回受信値に+１として受信値と比較（0~60000)
    int seq_r_past = -1;  // 前回のシーケンス番号受信値
};

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/// @brief 文字列形式のIPv4アドレスをIPAddressオブジェクトに変換します.
/// @param ip_str ドット10進記法で記述されたIPv4アドレスを含む文字列.
/// @return IPAddressオブジェクトが返されます.
IPAddress makeIPAddress(const char *ip_str);

/// @brief Bluetoothデバイスアドレスを文字列形式に変換します.
/// @param bda Bluetoothデバイスアドレスを表す6バイトの配列.
/// @param str 変換されたアドレスを格納するための文字列バッファ.
/// @param size 文字列バッファのサイズ.
/// @return 変換に成功した場合は文字列バッファへのポインタを, 
//          失敗（例えばバッファのサイズが不足している場合）した場合はNULLを返します.
char *bda2str(const uint8_t *bda, char *str, size_t size);

/// @brief Wiiリモコンからの入力データを受信し, 処理します.
void pad_wiimote_receive();

/// @brief UDPを使用してデータを送信します.
void udp_send();

/// @brief UDPを介して受信されたデータを読み込みます.
void udp_receive();

/// @brief Bluetoothデバイスの設定を行い, 接続を試みます.
void bt_settings();

/// @brief サブCPU (Core0) でBluetoothデバイスからのデータ受信を管理する関数.
/// @param args この関数に渡される引数ですが, 現在は使用されていません.
void Core0_BT_r(void *args);

/// @brief サブCPU (Core1) でSPI通信を管理するための関数.
/// @param args この関数に渡される引数ですが, 現在は使用されていません.
void Core1_SPI_r(void *args);

#endif
