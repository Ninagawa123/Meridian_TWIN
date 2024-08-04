#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// コンフィグファイルの読み込み
#include "config.h"

// ヘッダファイルの読み込み
#include "main.h"
#include "keys.h"

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp; // wifi設定

//================================================================================================================
//  Wifi 関連の処理  -------------------------------------------------------------------------------------------
//================================================================================================================

/// @brief wifiを初期化する.
/// @param a_ssid WifiアクセスポイントのSSID.
/// @param a_pass Wifiアクセスポイントのパスワード.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_wifi_init(const char *a_ssid, const char *a_pass)
{
    WiFi.disconnect(true, true); // WiFi接続をリセット
    delay(100);
    WiFi.begin(a_ssid, a_pass); // Wifiに接続
    int i=0;
    while (WiFi.status() != WL_CONNECTED)
    { // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
        i++;
        delay(50); // 接続が完了するまでループで待つ
        if (i > 200)
        { // 10秒でタイムアウト
            Serial.println("Wifi init TIMEOUT.");
            return false;
        }
    }
    udp.begin(UDP_RESV_PORT);
    return true;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len)
{
    if (udp.parsePacket() >= a_len) // データの受信バッファ確認
    {
        udp.read(a_meridim_bval, a_len); // データの受信
        return true;
    }
    return false;
}

/// @brief 第一引数のMeridim配列のデータをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTに送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @return 送信完了時にtrueを返す.
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len)
{
    udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケットの開始
    udp.write(a_meridim_bval, a_len);
    udp.endPacket(); // UDPパケットの終了
    return true;
}

#endif // __MERIDIAN_WIFI_H__
