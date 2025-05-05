
#ifndef __MERIDIAN_CONFIG_H__
#define __MERIDIAN_CONFIG_H__

//================================================================================================================
//  MERIDIAN TWIN ESP32の配線
//================================================================================================================
//   ESP32devkitC  -  Teensy4.0 or PIN
//   [3.3v]        -> x
//   [EN]          -> x
//   [SENSOR_VP]   -> x
//   [SENSOR_VN]   -> x
//   [34]          -> x
//   [35]          -> x
//   [32]          -> x
//   [33]          -> x
//   [25]          -> x
//   [26]          -> x
//   [27]          -> x
//   [14]          -> SPI/SD_SCK (Teensy[13]) & SD_CLK (SD[5])
//   [12]          -> MISO (Teensy[12])
//   [GND1]        -> x
//   [13]          -> MOSI (Teensy[11])
//   [D2]          -> x
//   [D3]          -> x
//   [CMD]         -> x
//   [5V]          -> 5V

//   [GND3]        -> GND
//   [23]          -> x
//   [22]          -> x
//   [TXD0]        -> x
//   [RXD0]        -> x
//   [21]          -> x
//   [GND2]        -> x
//   [19]          -> x
//   [18]          -> x
//   [05]          -> x
//   [17]          -> x
//   [16]          -> x
//   [04]          -> x
//   [00]          -> x
//   [02]          -> x
//   [15]          -> SPI_CS (Teensy[10])
//   [D1]          -> x
//   [D0]          -> x
//   [CLK]         -> x

//================================================================================================================
//  Meridim90配列 一覧表
//================================================================================================================
// [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
// [01]      シーケンス番号
// [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
// [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
// [11]      IMU/AHRS:temp                   温度
// [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
// [15]      ボタンデータ1
// [16]      ボタンアナログ1 (Stick Left)
// [17]      ボタンアナログ2 (Stick Right)
// [18]      ボタンアナログ3 (L2,R2 アナログ)
// [19]      モーション設定 (フレーム数)
// [20]      サーボID LO  コマンド
// [21]      サーボID LO  データ値
// [...]     ...
// [48]      サーボID L14 コマンド
// [49]      サーボID L14 データ値
// [50]      サーボID RO  コマンド
// [51]      サーボID RO  データ値
// [...]     ...
// [78]      サーボID R14 コマンド
// [79]      サーボID R14 データ値
// [80]-[MRDM_LEN-3] free (Meridim90では[87]まで)
// [MRDM_LEN-2] ERROR CODE
// [MRDM_LEN-1] チェックサム

// Meridimの基本設定
#define MRDM_LEN       90  // Meridim配列の長さ設定 (デフォルトは90)
#define FRAME_DURATION 10  // 1フレームあたりの単位時間 (単位ms)
#define CHARGE_TIME    200 // 起動時のコンデンサチャージ待機時間 (単位ms)

// 動作モード
#define MODE_UDP_RECEIVE 1 // PCからのデータ受信 (0:OFF, 1:ON, 通常は1)
#define MODE_UDP_SEND    1 // PCへのデータ送信 (0:OFF, 1:ON, 通常は1)

// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define MODE_FIXED_IP 0 // IPアドレスを固定するか (0:NO, 1:YES)
#define UDP_TIMEOUT   4 // UDPの待受タイムアウト (単位ms,推奨値0)

// EEPROMの設定
#define EEPROM_BYTE    540 // 使用するEEPROMのサイズ(バイト)
#define EEPROM_SET     0   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_PROTECT 0   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD    0   // 起動時にEEPROMの内容を諸設定にロードする(未導入)
#define EEPROM_DUMP    1   // 起動時のEEPROM内容のダンプ表示
#define EEPROM_STYLE   Hex // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)

// 動作チェックモード
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

// シリアルモニタリング
#define MONITOR_FLOW              0     // シリアルモニタでフローを表示 (0:OFF, 1:ON)
#define MONITOR_ERR_ALL           0     // 全経路の受信エラー率を表示
#define MONITOR_SEQ               0     // シリアルモニタでシーケンス番号チェックを表示 (0:OFF, 1:ON)
#define MONITOR_PAD               0     // シリアルモニタでリモコンのデータを表示 (0:OFF, 1:ON)
#define MONITOR_SUPPRESS_DURATION 20000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

// SPI設定
#define SPI0_SPEED 6000000 // SPI通信の速度 (6000000kHz推奨)

// PC接続関連設定
#define SERIAL_PC_BPS     1000000 // PCとのシリアル速度 (モニタリング表示用)
#define SERIAL_PC_TIMEOUT 2000    // PCとのシリアル接続確立タイムアウト(ms)

// JOYPAD関連設定
#define MOUNT_PAD        NONE  // ESP32へのジョイパッドの搭載 NONE:0, PC:0, WIIMOTE:5, WIIMOTE_C:6
#define PAD_INIT_TIMEOUT 10000 // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL     10    // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 0     // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1     // ジョイパッドの入力値をPS系に一般化する

// ピンアサイン
#define PIN_ERR_LED       25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L          33 // サーボL系統のENピン
#define PIN_EN_R          4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define PIN_I2C0_SDA      22 // I2CのSDAピン
#define PIN_I2C0_SCL      21 // I2CのSCLピン
#define PIN_LED_BT        23 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)

//-------------------------------------------------------------------------
//  固定値, マスターコマンド定義
//-------------------------------------------------------------------------
#define MCMD_DUMMY_DATA             -32768 // SPI送受信用のダミーデータ判定用
#define MCMD_TEST_VALUE             -32767 // テスト用の仮設変数
#define MCMD_SENSOR_YAW_CALIB       10002  // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_SENSOR_ALL_CALIB       10003  // センサの3軸について現在値を原点としてリセット
#define MCMD_ERR_CLEAR_SERVO_ID     10004  // 通信エラーのサーボのIDをクリア(MRD_ERR_l)
#define MCMD_BOARD_TRANSMIT_ACTIVE  10005  // ボードが定刻で送信を行うモード (PC側が受信待ち)
#define MCMD_BOARD_TRANSMIT_PASSIVE 10006  // ボードが受信を待ち返信するモード (PC側が定刻送信)
#define MCMD_FRAMETIMER_RESET       10007  // フレームタイマーを現在時刻にリセット
#define MCMD_BOARD_STOP_DURING      10008  // ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める
#define MCMD_EEPROM_ENTER_WRITE     10009  // EEPROM書き込みモードのスタート
#define MCMD_EEPROM_EXIT_WRITE      10010  // EEPROM書き込みモードの終了
#define MCMD_EEPROM_ENTER_READ      10011  // EEPROM読み出しモードのスタート
#define MCMD_EEPROM_EXIT_READ       10012  // EEPROM読み出しモードの終了
#define MCMD_SDCARD_ENTER_WRITE     10013  // SDCARD書き込みモードのスタート
#define MCMD_SDCARD_EXIT_WRITE      10014  // SDCARD書き込みモードの終了
#define MCMD_SDCARD_ENTER_READ      10015  // SDCARD読み出しモードのスタート
#define MCMD_SDCARD_EXIT_READ       10016  // SDCARD読み出しモードの終了

//-------------------------------------------------------------------------
//  Meridim90 配列アクセス対応キー
//-------------------------------------------------------------------------
#define MRD_MASTER        0  // マスターコマンド
#define MRD_SEQ           1  // シーケンス番号
#define MRD_ACC_X         2  // 加速度センサX値
#define MRD_ACC_Y         3  // 加速度センサY値
#define MRD_ACC_Z         4  // 加速度センサZ値
#define MRD_GYRO_X        5  // ジャイロセンサX値
#define MRD_GYRO_Y        6  // ジャイロセンサY値
#define MRD_GYRO_Z        7  // ジャイロセンサZ値
#define MRD_MAG_X         8  // 磁気コンパスX値
#define MRD_MAG_Y         9  // 磁気コンパスY値
#define MRD_MAG_Z         10 // 磁気コンパスZ値
#define MRD_TEMP          11 // 温度センサ値
#define MRD_DIR_ROLL      12 // DMP推定ロール方向値
#define MRD_DIR_PITCH     13 // DMP推定ピッチ方向値
#define MRD_DIR_YAW       14 // DMP推定ヨー方向値
#define MRD_PAD_BUTTONS   15 // リモコンの基本ボタン値
#define MRD_PAD_STICK_L   16 // リモコンの左スティックアナログ値
#define MRD_PAD_STICK_R   17 // リモコンの右スティックアナログ値
#define MRD_PAD_L2R2VAL   18 // リモコンのL2R2ボタンアナログ値
#define MRD_MOTION_FRAMES 19 // モーション設定のフレーム数
#define MRD_STOP_FRAMES   19 // ボード停止時のフレーム数(MCMD_STOP_BOARD_DURINGで指定)

#define C_HEAD_Y_CMD     20 // 頭ヨーのコマンド
#define C_HEAD_Y_VAL     21 // 頭ヨーの値
#define L_SHOULDER_P_CMD 22 // 左肩ピッチのコマンド
#define L_SHOULDER_P_VAL 23 // 左肩ピッチの値
#define L_SHOULDER_R_CMD 24 // 左肩ロールのコマンド
#define L_SHOULDER_R_VAL 25 // 左肩ロールの値
#define L_ELBOW_Y_CMD    26 // 左肘ヨーのコマンド
#define L_ELBOW_Y_VAL    27 // 左肘ヨーの値
#define L_ELBOW_P_CMD    28 // 左肘ピッチのコマンド
#define L_ELBOW_P_VAL    29 // 左肘ピッチの値
#define L_HIPJOINT_Y_CMD 30 // 左股ヨーのコマンド
#define L_HIPJOINT_Y_VAL 31 // 左股ヨーの値
#define L_HIPJOINT_R_CMD 32 // 左股ロールのコマンド
#define L_HIPJOINT_R_VAL 33 // 左股ロールの値
#define L_HIPJOINT_P_CMD 34 // 左股ピッチのコマンド
#define L_HIPJOINT_P_VAL 35 // 左股ピッチの値
#define L_KNEE_P_CMD     36 // 左膝ピッチのコマンド
#define L_KNEE_P_VAL     37 // 左膝ピッチの値
#define L_ANKLE_P_CMD    38 // 左足首ピッチのコマンド
#define L_ANKLE_P_VAL    39 // 左足首ピッチの値
#define L_ANKLE_R_CMD    40 // 左足首ロールのコマンド
#define L_ANKLE_R_VAL    41 // 左足首ロールの値
#define L_SERVO_IX11_CMD 42 // 追加サーボ用のコマンド
#define L_SERVO_IX11_VAL 43 // 追加サーボ用の値
#define L_SERVO_IX12_CMD 44 // 追加サーボ用のコマンド
#define L_SERVO_IX12_VAL 45 // 追加サーボ用の値
#define L_SERVO_IX13_CMD 46 // 追加サーボ用のコマンド
#define L_SERVO_IX13_VAL 47 // 追加サーボ用の値
#define L_SERVO_IX14_CMD 48 // 追加サーボ用のコマンド
#define L_SERVO_IX14_VAL 49 // 追加サーボ用の値

#define C_WAIST_Y_CMD    50 // 腰ヨーのコマンド
#define C_WAIST_Y_VAL    51 // 腰ヨーの値
#define R_SHOULDER_P_CMD 52 // 右肩ピッチのコマンド
#define R_SHOULDER_P_VAL 53 // 右肩ピッチの値
#define R_SHOULDER_R_CMD 54 // 右肩ロールのコマンド
#define R_SHOULDER_R_VAL 55 // 右肩ロールの値
#define R_ELBOW_Y_CMD    56 // 右肘ヨーのコマンド
#define R_ELBOW_Y_VAL    57 // 右肘ヨーの値
#define R_ELBOW_P_CMD    58 // 右肘ピッチのコマンド
#define R_ELBOW_P_VAL    59 // 右肘ピッチの値
#define R_HIPJOINT_Y_CMD 60 // 右股ヨーのコマンド
#define R_HIPJOINT_Y_VAL 61 // 右股ヨーの値
#define R_HIPJOINT_R_CMD 62 // 右股ロールのコマンド
#define R_HIPJOINT_R_VAL 63 // 右股ロールの値
#define R_HIPJOINT_P_CMD 64 // 右股ピッチのコマンド
#define R_HIPJOINT_P_VAL 65 // 右股ピッチの値
#define R_KNEE_P_CMD     66 // 右膝ピッチのコマンド
#define R_KNEE_P_VAL     67 // 右膝ピッチの値
#define R_ANKLE_P_CMD    68 // 右足首ピッチのコマンド
#define R_ANKLE_P_VAL    69 // 右足首ピッチの値
#define R_ANKLE_R_CMD    70 // 右足首ロールのコマンド
#define R_ANKLE_R_VAL    71 // 右足首ロールの値
#define R_SERVO_IX11_CMD 72 // 追加テスト用のコマンド
#define R_SERVO_IX11_VAL 73 // 追加テスト用の値
#define R_SERVO_IX12_CMD 74 // 追加テスト用のコマンド
#define R_SERVO_IX12_VAL 75 // 追加テスト用の値
#define R_SERVO_IX13_CMD 76 // 追加テスト用のコマンド
#define R_SERVO_IX13_VAL 77 // 追加テスト用の値
#define R_SERVO_IX14_CMD 78 // 追加テスト用のコマンド
#define R_SERVO_IX14_VAL 79 // 追加テスト用の値

#define MRD_USERDATA_80 80 // ユーザー定義用
#define MRD_USERDATA_81 81 // ユーザー定義用
#define MRD_USERDATA_82 82 // ユーザー定義用
#define MRD_USERDATA_83 83 // ユーザー定義用
#define MRD_USERDATA_84 84 // ユーザー定義用
#define MRD_USERDATA_85 85 // ユーザー定義用
#define MRD_USERDATA_86 86 // ユーザー定義用
#define MRD_USERDATA_87 87 // ユーザー定義用
// #define MRD_ERR         88 // エラーコード (MRDM_LEN - 2)
// #define MRD_CKSM        89 // チェックサム (MRDM_LEN - 1)

// エラービット MRD_ERR_CODEの上位8bit分
#define ERRBIT_15_ESP_PC       15 // ESP32 → PC のUDP受信エラー (0:エラーなし、1:エラー検出)
#define ERRBIT_14_PC_ESP       14 // PC → ESP32 のUDP受信エラー
#define ERRBIT_13_ESP_TSY      13 // ESP32 → TeensyのSPI受信エラー
#define ERRBIT_12_TSY_ESP      12 // Teensy → ESP32 のSPI受信エラー
#define ERRBIT_11_BOARD_DELAY  11 // Teensy or ESP32の処理ディレイ (末端で捕捉)
#define ERRBIT_10_UDP_ESP_SKIP 10 // PC → ESP32 のUDPフレームスキップエラー
#define ERRBIT_9_BOARD_SKIP    9  // PC → ESP32 → Teensy のフレームスキップエラー(末端で捕捉)
#define ERRBIT_8_PC_SKIP       8  // Teensy → ESP32 → PC のフレームスキップエラー(末端で捕捉)

#endif // __MERIDIAN_CONFIG_H__
