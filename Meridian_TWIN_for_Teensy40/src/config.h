//================================================================================================================
//---- Teensy4.0 の 配 線 / ピンアサイン ----------------------------------------------------------------------------
//================================================================================================================
/*
  [GND]               -> GND
  [00] RX1, CRX2      -> ICS_3rd_TX
  [01] TX1, CTX2      -> ICS_3rd_RX
  [02]                -> LED（lights up when the processing time is not within the specified time.）
  [03]                -> (NeoPixel?)
  [04]                -> (NeoPixel?)
  [05]                -> ICS_Right_EN
  [06]                -> ICS_Left_EN
  [07] RX2            -> ICS_Left_TX
  [08] TX2            -> ICS_Left_RX
  [09]                -> SD_DAT3/CD (SD[2])
  [10] CS             -> SPI_CS (ESP32[15])
  [11] MOSI           -> SPI/SD_MOSI (ESP32[13]) & SD_CMD (SD[3])
  [12] MISO           -> SPI/SD_MISO (ESP32[12]) & SD_DAT0 (SD[7])
  [Vin]               -> 5V
  [AGND]              ->
  [3.3v]              -> IMU/AHRS 3.3Vin & SD_VDD (SD[4])
  [23] CRX1           -> ICS_3rd_EN
  [22] CTX1           ->
  [21] RX5            ->
  [20] TX5            ->
  [19] I2C-SCL0       -> IMU/AHRS SCL
  [18] I2C-SDA0       -> IMU/AHRS SDA
  [17] TX4, I2C-SDA1  -> (PC/Raspi etc.)
  [16] RX4, I2C-SCL1  -> (PC/Raspi etc.)
  [15] RX3            -> ICS_Right_TX
  [14] TX3            -> ICS_Right_RX
  [13] SCK(CRX1)      -> SPI/SD_SCK (ESP32[14]) & SD_CLK (SD[5])
*/

//================================================================================================================
//---- サーボIDとロボット部位、軸との対応表 ----------------------------------------------------------------------------
//================================================================================================================
/*
  ID    Parts-Axis　＜ICS_Left_Upper SIO1,SIO2＞
  [L00] 頭ヨー
  [L01] 左肩ピッチ
  [L02] 左肩ロール
  [L03] 左肘ヨー
  [L04] 左肘ピッチ
  [L05] -
  ID    Parts-Axis　＜ICS_Left_Lower SIO3,SIO4＞
  [L06] 左股ロール
  [L07] 左股ピッチ
  [L08] 左膝ピッチ
  [L09] 左足首ピッチ
  [L10] 左足首ロール
  ID    Parts-Axis  ＜ICS_Right_Upper SIO5,SIO6＞
  [R00] 腰ヨー
  [R01] 右肩ピッチ
  [R02] 右肩ロール
  [R03] 右肘ヨー
  [R04] 右肘ピッチ
  [R05] -
  ID    Parts-Axis  ＜ICS_Right_Lower SIO7,SIO8＞
  [R06] 右股ロール
  [R07] 右股ピッチ
  [R08] 右膝ピッチ
  [R09] 右足首ピッチ
  [R10] 右足首ロール
*/

//================================================================================================================
//---- Meridim配列 一覧表 ------------------------------------------------------------------------------------------
//================================================================================================================
/*
  [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
  [01]      シーケンシャルカウンタ
  [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
  [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
  [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
  [11]      IMU/AHRS:temp                   温度
  [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
  [15]      ボタンデータ1
  [16]      ボタンアナログ1（Stick Left）
  [17]      ボタンアナログ2（Stick Right）
  [18]      ボタンアナログ3（L2,R2 アナログ）
  [19]      モーション設定（フレーム数/イージング）
  [20]      サーボID LO  コマンド
  [21]      サーボID LO  データ値
  ...
  [48]      サーボID L14 コマンド
  [49]      サーボID L14 データ値
  [50]      サーボID RO  コマンド
  [51]      サーボID RO  データ値
  ...
  [78]      サーボID R14 コマンド
  [79]      サーボID R14 データ値
  [80]-[MSG_SIZE-3] free (Meridim90では[87]まで)
  [MSG_SIZE-2] ERROR CODE
  [MSG_SIZE-1] チェックサム
*/

/* Meridimの基本設定 */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define FRAME_DURATION 10 // 1フレームあたりの単位時間（単位ms）

/* シリアルモニタリング */
#define MONITOR_ALL_ERROR 0 // Teensyでのシリアル表示:全経路の受信エラー率
#define MONITOR_JOYPAD 0    // Teensyでのシリアル表示:リモコンのデータ

/* 動作チェックモード */
#define CHECK_SD_RW 0 // 起動時のSDカードリーダーの読み書きチェック

/* 各種ハードウェアのマウント有無 */
#define MOUNT_ESP32 1        // ESPの搭載 0:なし(SPI通信およびUDP通信を実施しない), 1:あり
#define MOUNT_SD 1           // SDカードリーダーのありなし. MeridianBoard Type.Kは有り
#define MOUNT_IMUAHRS 1      // IMU/AHRSの搭載状況 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
#define MOUNT_ICS3 0         // 半二重サーボ信号の3系のありなし
#define MOUNT_JOYPAD 2       // ジョイパッドの搭載 0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH (※2のみ実装済,MeridianBoardではICS_R系に接続)
#define MOUNT_SERVO_NUM_L 11 // L系統につないだサーボの総数
#define MOUNT_SERVO_NUM_R 11 // R系統につないだサーボの総数
#define MOUNT_SERVO_NUM_3 0  // 3系統につないだサーボの総数
// PC接続関連設定
#define SERIAL_PC_BPS 6000000 // PCとのシリアル速度（モニタリング表示用）
// SPI設定
#define SPI_SPEED 6000000 // SPI通信の速度（6000000kHz推奨）
// I2C設定, I2Cセンサ関連設定
#define I2C_SPEED 400000   // I2Cの速度（400kHz推奨）
#define IMUAHRS_POLLING 10 // IMU/AHRSのセンサの読み取り間隔(ms)
#define IMUAHRS_STOCK 4    // MPUで移動平均を取る際の元にする時系列データの個数
// サーボ関連設定
#define ICS_BAUDRATE 1250000 // ICSサーボの通信速度1.25M
#define ICS_TIMEOUT 3        // ICS返信待ちのタイムアウト時間。通信できてないか確認する場合には1000ぐらいに設定するとよい
// JOYPAD関連設定
#define JOYPAD_FRAME 4 // 上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)

/* 固定値, マスターコマンド定義 */
#define TRIM_ADJUST_MODE 0           // トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
#define MCMD_UPDATE_YAW_CENTER 10002 // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_ENTER_TRIM_MODE 10003   // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

/* ピンアサイン */
#define PIN_ERR_LED 2       // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L 6          // ICSサーボ信号の左系のENピン番号（固定）
#define PIN_EN_R 5          // ICSサーボ信号の右系のENピン番号（固定）
#define PIN_EN_3 23         // 半二重サーボ信号の3系のENピン番号（固定）
#define PIN_CHIPSELECT_SD 9 // SDカードSPI通信用のChipSelectのピン番号

//-------------------------------------------------------------------------
//---- サ ー ボ 設 定  -----------------------------------------------------
//-------------------------------------------------------------------------
/* 各サーボのマウントありなし（1:サーボあり、0:サーボなし） */
#define IDL_MT0 1  // 頭ヨー
#define IDL_MT1 1  // 左肩ピッチ
#define IDL_MT2 1  // 左肩ロール
#define IDL_MT3 1  // 左肘ヨー
#define IDL_MT4 1  // 左肘ピッチ
#define IDL_MT5 1  // 左股ヨー
#define IDL_MT6 1  // 左股ロール
#define IDL_MT7 1  // 左股ピッチ
#define IDL_MT8 1  // 左膝ピッチ
#define IDL_MT9 1  // 左足首ピッチ
#define IDL_MT10 1 // 左足首ロール
#define IDL_MT11 0 // 追加テスト用
#define IDL_MT12 0 // 追加テスト用
#define IDL_MT13 0 // 追加テスト用
#define IDL_MT14 0 // 追加テスト用

#define IDR_MT0 1  // 腰ヨー
#define IDR_MT1 1  // 右肩ピッチ
#define IDR_MT2 1  // 右肩ロール
#define IDR_MT3 1  // 右肘ヨー
#define IDR_MT4 1  // 右肘ピッチ
#define IDR_MT5 1  // 右股ヨー
#define IDR_MT6 1  // 右股ロール
#define IDR_MT7 1  // 右股ピッチ
#define IDR_MT8 1  // 右膝ピッチ
#define IDR_MT9 1  // 右足首ピッチ
#define IDR_MT10 1 // 右足首ロール
#define IDR_MT11 0 // 追加テスト用
#define IDR_MT12 0 // 追加テスト用
#define IDR_MT13 0 // 追加テスト用
#define IDR_MT14 0 // 追加テスト用

/* 各サーボの内外回転プラスマイナス方向補正(1 or -1) */
#define IDL_CW0 1  // 頭ヨー
#define IDL_CW1 1  // 左肩ピッチ
#define IDL_CW2 1  // 左肩ロール
#define IDL_CW3 1  // 左肘ヨー
#define IDL_CW4 1  // 左肘ピッチ
#define IDL_CW5 1  // 左股ヨー
#define IDL_CW6 1  // 左股ロール
#define IDL_CW7 1  // 左股ピッチ
#define IDL_CW8 1  // 左膝ピッチ
#define IDL_CW9 1  // 左足首ピッチ
#define IDL_CW10 1 // 左足首ロール
#define IDL_CW11 1 // 追加テスト用
#define IDL_CW12 1 // 追加テスト用
#define IDL_CW13 1 // 追加テスト用
#define IDL_CW14 1 // 追加テスト用
#define IDR_CW0 1  // 腰ヨー
#define IDR_CW1 1  // 右肩ピッチ
#define IDR_CW2 1  // 右肩ロール
#define IDR_CW3 1  // 右肘ヨー
#define IDR_CW4 1  // 右肘ピッチ
#define IDR_CW5 1  // 右股ヨー
#define IDR_CW6 1  // 右股ロール
#define IDR_CW7 1  // 右股ピッチ
#define IDR_CW8 1  // 右膝ピッチ
#define IDR_CW9 1  // 右足首ピッチ
#define IDR_CW10 1 // 右足首ロール
#define IDR_CW11 1 // 追加テスト用
#define IDR_CW12 1 // 追加テスト用
#define IDR_CW13 1 // 追加テスト用
#define IDR_CW14 1 // 追加テスト用

/* 各サーボの直立デフォルト値(degree) 直立状態になるよう、具体的な数値を入れて現物調整する */
#define IDL_TRIM0 0        // 頭ヨー
#define IDL_TRIM1 -2.3625  // 左肩ピッチ
#define IDL_TRIM2 -91.125  // 左肩ロール
#define IDL_TRIM3 0        // 左肘ヨー
#define IDL_TRIM4 89.9775  // 左肘ピッチ
#define IDL_TRIM5 0        // 左股ヨー
#define IDL_TRIM6 0        // 左股ロール
#define IDL_TRIM7 -1.35    // 左股ピッチ
#define IDL_TRIM8 -58.05   // 左膝ピッチ
#define IDL_TRIM9 -20.25   // 左足首ピッチ
#define IDL_TRIM10 -0.675  // 左足首ロール
#define IDL_TRIM11 0       // 追加テスト用
#define IDL_TRIM12 0       // 追加テスト用
#define IDL_TRIM13 0       // 追加テスト用
#define IDL_TRIM14 0       // 追加テスト用
#define IDR_TRIM0 0        // 腰ヨー
#define IDR_TRIM1 0        // 右肩ピッチ
#define IDR_TRIM2 -89.4375 // 右肩ロール
#define IDR_TRIM3 0        // 右肘ヨー
#define IDR_TRIM4 89.9775  // 右肘ピッチ
#define IDR_TRIM5 0        // 右股ヨー
#define IDR_TRIM6 1.6875   // 右股ロール
#define IDR_TRIM7 -3.375   // 右股ピッチ
#define IDR_TRIM8 -57.375  // 右膝ピッチ
#define IDR_TRIM9 -20.25   // 右足首ピッチ
#define IDR_TRIM10 -2.3625 // 右足首ロール
#define IDR_TRIM11 0       // 追加テスト用
#define IDR_TRIM12 0       // 追加テスト用
#define IDR_TRIM13 0       // 追加テスト用
#define IDR_TRIM14 0       // 追加テスト用
