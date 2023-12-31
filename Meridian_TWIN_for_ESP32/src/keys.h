/* Wifiアクセスポイントの設定 */
#define WIFI_AP_SSID "xxxxxxxxx"       // アクセスポイントのWIFI_AP_SSID
#define WIFI_AP_PASS "xxxxxxxxx"       // アクセスポイントのパスワード
#define WIFI_SEND_IP "192.168.1.x" // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define UDP_SEND_PORT 22222         // 送り先のポート番号
#define UDP_RESV_PORT 22224         // このESP32のポート番号

/* ESP32のIPアドレスを固定する場合は下記の4項目を設定 */
#define MODE_FIXED_IP 0 // IPアドレスを固定するか（0:NO, 1:YES）
#define FIXED_IP_ADDR "192. 168. 1. xx"    // ESP32のIPアドレスを固定する場合のESPのIPアドレス
#define FIXED_IP_GATEWAY "192. 168. 1. xx" // ESP32のIPアドレスを固定する場合のルーターのゲートウェイ
#define FIXED_IP_SUBNET "255. 255. 255. 0" // ESP32のIPアドレスを固定する場合のサブネット

/* リモコンの設定 */
#define BT_MAC_ADDR "xx:xx:xx:xx:xx:xx" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
                                        // PS4リモコン(Bluetooth)を使わない場合は不要.
