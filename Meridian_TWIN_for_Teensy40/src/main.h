
#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

//================================================================================================================
//---- コ マ ン ド 処 理 系 の 関 数 各 種 ---------------------------------------------------------------------------
//================================================================================================================

// +----------------------------------------------------------------------
// | 関数名　　:  trimadjustment()
// +----------------------------------------------------------------------
// | 機能     :  サーボトリム調整. サーボオンで直立静止状態を保つ.
// +----------------------------------------------------------------------
void trimadjustment();

// +----------------------------------------------------------------------
// | 関数名　　:  setyaw()
// +----------------------------------------------------------------------
// | 機能     :  ヨー軸の原点リセット. IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// +----------------------------------------------------------------------
void setyaw();

// +----------------------------------------------------------------------
// | 関数名　　:  servo_all_off()
// +----------------------------------------------------------------------
// | 機能     :  全サーボオフ
// +----------------------------------------------------------------------
void servo_all_off();

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// +----------------------------------------------------------------------
// | 関数名　　:  setupIMUAHRS()
// +----------------------------------------------------------------------
// | 機能     :  MPU6050,BNO055等の初期設定を行う.　IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// | 引数　　　:  なし.
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void setupIMUAHRS();

// +----------------------------------------------------------------------
// | 関数名　　:  IMUAHRS_getYawPitchRoll()
// +----------------------------------------------------------------------
// | 機能     :  MPU6050,BNO055等の値を格納する.　IMUAHRS_MOUNTで機種判別.
// | 　　　　　:  0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
// | 引数　　　:  なし.
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void IMUAHRS_getYawPitchRoll();

// +----------------------------------------------------------------------
// | 関数名　　:  joypad_read()
// +----------------------------------------------------------------------
// | 機能     :  Teensy4.0に接続されたJOYPADの値を読みとり、pad_btnに格納
// | 　　　　　:  0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH
// | 戻り値　　:  なし.
// +----------------------------------------------------------------------
void joypad_read();

// +----------------------------------------------------------------------
// | 関数名　　:  check_sd()
// +----------------------------------------------------------------------
// | 機能     :  SDカードの初期化と読み書きテスト
// +----------------------------------------------------------------------
void check_sd();

// +----------------------------------------------------------------------
// | 関数名　　:  print_error_monitor()
// +----------------------------------------------------------------------
// | 機能     :  エラー検出数をTeensyのシリアルに表示する
// +----------------------------------------------------------------------
void print_error_monitor();

// +----------------------------------------------------------------------
// | 関数名　　:  countup_errors()
// +----------------------------------------------------------------------
// | 機能     :  通信エラー検出数をカウントアップする
// +----------------------------------------------------------------------
void countup_errors();

// +----------------------------------------------------------------------
// | 関数名　　:  imuahrs_start()
// +----------------------------------------------------------------------
// | 機能     :  imu/ahrsを開始する
// +----------------------------------------------------------------------
void imuahrs_start();

// +----------------------------------------------------------------------
// | 関数名　　:  imuahrs_store()
// +----------------------------------------------------------------------
// | 機能     :  imu/ahrsのセンサー値を配列に格納する
// +----------------------------------------------------------------------
void imuahrs_store();

// +----------------------------------------------------------------------
// | 関数名　　:  move_servos_krs()
// +----------------------------------------------------------------------
// | 機能     :  krsサーボを動作させ、戻り値を配列に格納する
// +----------------------------------------------------------------------
void move_servos_krs();

// +----------------------------------------------------------------------
// | 関数名　　:  imuahrs_store()
// +----------------------------------------------------------------------
// | 機能     :  imu/ahrsのセンサー値を配列に格納する
// +----------------------------------------------------------------------

void execute_MasterCommand();

#endif