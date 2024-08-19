/*
* @file    gs2d_serial.h
* @author
* @date    2021/01/26
* @brief   Serial Base Class
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
/* USER INCLUDE CODE START */

// TODO : シリアル送受信に必要なファイルをインクルードする

/* USER INCLUDE CODE END */

/* Variables -----------------------------------------------------------------*/
class GS2DSerial
{
public:
	GS2DSerial() {}
	~GS2DSerial() {}

	int open(void)
	{
		/* USER OPEN CODE START */

		// TODO : シリアルポート初期化処理
		// RS485の場合はTXENのIOも初期化

		/* USER OPEN CODE END */
		return 0;
	}

	void close(void)
	{
		/* USER CLOSE CODE START */

		// TODO : シリアルポートを閉じる処理

		/* USER CLOSE CODE END */
		return;
	}

	int isConnected(void)
	{
		/* USER IS_CONNECTED CODE START */

		// TODO : シリアルポートの接続状態を返す関数
		// 開いている時に１，閉じている時に0を返す

		/* USER IS_CONNECTED CODE END */
		return 0;
	}

	int read(void)
	{
		/* USER READ CODE START */

		// TODO : データがバッファにあればデータを、無ければ -1 を返す

		/* USER READ CODE END */

		return -1;
	}

	int write(unsigned char* data, unsigned char size)
	{
		/* USER WRITE CODE START */

		// TODO : データ送信関数
		// RS485のTXENもここで操作

		/* USER WRITE CODE END */
		return 0;
	}

	unsigned long long int time(void)
	{
		/* USER TIME CODE START */

		// TODO : ms単位で開始からの経過時間を返す

		/* USER TIME CODE END */
		return 0;
	}
};
