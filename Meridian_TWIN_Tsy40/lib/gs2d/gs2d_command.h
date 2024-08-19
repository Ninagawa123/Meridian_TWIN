/*
* @file    gs2d_command.h
* @author
* @date    2021/01/26
* @brief
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gs2d_type.h"
#include <inttypes.h>

namespace gs2d
{
	/* Variables -----------------------------------------------------------------*/
	// コマンドバッファの型
	template<unsigned int commandSize = 50>
	class CommandBufferType
	{
	public:
		uint8_t data[commandSize];
		uint8_t length = 0;
		uint8_t count = 0;
		ResponseProcess responseProcess;
		CallbackType callback;

		CommandBufferType(uint8_t* data, uint8_t length, uint8_t count = 1, ResponseProcess response = 0, CallbackType callback = 0)
		{
			memcpy(this->data, data, length);
			this->length = length;
			this->count = count;
			responseProcess = response;
			this->callback = callback;
		}
		CommandBufferType() {}
		~CommandBufferType() {}
	};

	template<class SerialClass, unsigned int bufferSize = 20, unsigned int commandSize = 50>
	class CommandHandler
	{
	protected:
		// 受信タイムアウト
		uint16_t receiveTimeout = 2000; // msec

		// シリアルポート
		SerialClass serialPort;

		// コマンドバッファ
		CircularBuffer<CommandBufferType<commandSize>, bufferSize> commandStack;
		CommandBufferType<commandSize> currentCommand;

		// 受信データ
		uint8_t response[100];
		uint8_t responsePos = 0;

		// 継承先の関数
		virtual bool isComplete(uint8_t* data, uint8_t length) = 0;
		virtual void dataReceivedEvent(uint8_t* data, uint8_t length, uint8_t status) = 0;

		// 処理ステータス
		Gs2dType<bool> isTrafficFree;

		// タイマ
		uint64_t startTime = 0;

		//
		CommandHandler() : isTrafficFree(true), currentCommand() { serialPort.open(); }
		virtual ~CommandHandler() { serialPort.close(); }

		// コマンド追加関数
		void addCommand(uint8_t* data, uint8_t length, ResponseProcess response, CallbackType callback, uint8_t count)
		{
			CommandBufferType<commandSize> command(data, length, count, response, callback);
			commandStack.push(command);
			if (isTrafficFree.get()) sendCommand();
		}

		// コマンド送信
		void sendCommand()
		{
			// コマンドを取り出して送信
			currentCommand = commandStack.pop();
			serialPort.write(currentCommand.data, currentCommand.length);

			// リスナ初期化
			// 受信個数が0個でも無視
			if (currentCommand.count == 0) {
				// コマンドがたまっている場合は送信
				if (!commandStack.isEmpty()) sendCommand();
				else isTrafficFree.set(true);
			}
			else {
				responsePos = 0;
				isTrafficFree.set(false);
				startTime = serialPort.time();
			}
		}

		// リスナー関数
		void listener(void)
		{
			// 無意味なデータを無視
			if (isTrafficFree.get()) return;

			if (serialPort.isConnected())
			{
				// タイムアウト確認
				if (serialPort.time() > startTime + receiveTimeout)
				{
					// タイムアウトを通知
					dataReceivedEvent(0, 0, TimeoutError);

					// コマンドがたまっている場合は送信
					if (!commandStack.isEmpty()) sendCommand();
					else isTrafficFree.set(true);

					return;
				}

				// データが無ければ終了
				int data = serialPort.read();
				if (data == -1) return;

				// 受信データ保存
				response[responsePos++] = data;

				// TODO:エラー処理
				if (responsePos >= 100) responsePos = 0;

				// 受信完了チェック
				if (isComplete(response, responsePos))
				{
					dataReceivedEvent(response, responsePos, 0);

					// 全サーボ受信完了チェック
					currentCommand.count--;
					if (currentCommand.count == 0) {
						// コマンドがたまっている場合は送信
						if (!commandStack.isEmpty()) sendCommand();
						else isTrafficFree.set(true);
					}
					else {
						responsePos = 0;
					}
				}
			}
		}
	};
}
