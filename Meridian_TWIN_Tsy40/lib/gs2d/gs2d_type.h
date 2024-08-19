/*
* @file    gs2d_type.h
* @author
* @date    2021/01/26
* @brief
*
* マルチスレッド処理を行った際、データ入出力にエラーが起こる場合は
* 以下のクラスを環境向けにスレッドセーフに組み直す必要があるかも知れません。
* CircularBuffer, ThreadSafeType
*
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <inttypes.h>

namespace gs2d
{
	// gs2d用エラーコード
	enum {
		TimeoutError = 0x01,
		ResponseError = 0x02,
		ProtocolError = 0x04,
		NotSupportError = 0x00,
		BadInputError = 0x08,
		SystemError = 0x10
	};

	/* Classes -------------------------------------------------------------------*/
		// gs2dで扱う浮動小数点の型
	using gFloat = float;

	// Int, Floatの戻り地に対応するためのクラス
	class EventDataType
	{
	private:
		int32_t intData;
		gFloat floatData;
		bool   flag;

	public:
		EventDataType(gFloat data) { intData = (int32_t)data; floatData = data; flag = true; }
		EventDataType(int32_t data) { intData = data; floatData = (gFloat)data; flag = false; }
		EventDataType() { intData = 0; floatData = 0; flag = false; }
		~EventDataType() {}

		explicit operator int32_t(void) { return flag ? floatData : intData; }
		explicit operator gFloat(void) { return flag ? floatData : intData; }

		void set(int32_t data) { intData = data; floatData = (gFloat)data; flag = false; }
		void set(gFloat data) { intData = (int32_t)data; floatData = data; flag = true; }
	};

	// コールバックの引数用クラス
	class CallbackEventArgs
	{
	public:
		// ID, エラー状態、データ(int or gFloat)
		uint8_t id;
		uint8_t status;
		EventDataType data;

		// 各種コンストラクタ
		CallbackEventArgs(uint8_t id, uint8_t status, gFloat data) :id(id), status(status), data(data) {}
		CallbackEventArgs(uint8_t id, uint8_t status, int32_t data) : id(id), status(status), data(data) {}
		CallbackEventArgs(uint8_t id, uint8_t status, EventDataType data) : id(id), status(status) { this->data = data; }
		CallbackEventArgs(uint8_t status) : id(0), status(status), data((int32_t)0) {}
		~CallbackEventArgs() {}
	};

	// gs2d用サーキュラバッファクラス
	template<class T, unsigned char bufferSize = 50>
	class CircularBuffer
	{
	private:
		T buffer[bufferSize];
		volatile uint8_t writePos = 0;
		volatile uint8_t readPos = 0;
		uint8_t size;

	public:
		CircularBuffer() : size(bufferSize) {}
		virtual ~CircularBuffer() {}

		inline bool push(T data)
		{
			uint8_t tempPos = (uint8_t)((writePos + 1) % size);
			if (tempPos == readPos) return false;
			buffer[writePos++] = data;
			writePos %= size;
			return true;
		}

		inline T pop(void)
		{
			uint8_t tmpPos = readPos;
			if (writePos != readPos) {
				readPos++; readPos %= size;
			}
			return buffer[tmpPos];
		}

		bool isEmpty() { return (writePos == readPos); }
	};

	// スレッドセーフにする必要があるかもしれないので、念の為
	template<class T>
	class Gs2dType
	{
	private:
		volatile T variable;

	public:
		inline void set(T data) { variable = data; }
		inline T get(void) { return variable; }

		Gs2dType(T data) { variable = data; }
		~Gs2dType() {}
	};

	// 受信データ処理関数の型
	using ResponseProcess = EventDataType(*)(int32_t);

	// コールバック関数の型
	using CallbackType = void(*)(CallbackEventArgs);
}
