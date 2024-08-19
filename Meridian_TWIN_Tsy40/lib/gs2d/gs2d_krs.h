/*
* @file    gs2d_krs.h
* @author
* @date    2021/01/31
* @brief
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gs2d_driver.h"
#include "gs2d_command.h"
#include "crc16.h"
#include <vector>

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	template<class SerialClass, unsigned int bufferSize = 2, unsigned int commandSize = 70>
	class KRS : public CommandHandler<SerialClass, bufferSize, commandSize>, public Driver
	{
	private:
		// 受信データバッファ
		EventDataType responseData;
		Gs2dType<bool> isReceived;

		// EEPROM読み込み用
		struct KRSTarget {
			uint8_t address;
			uint8_t length;
		};
		CircularBuffer<KRSTarget, bufferSize> targetStack;
		KRSTarget currentTarget;

	    

		// EEPROM用に関数二つをoverride
		void addCommand(uint8_t* data, uint8_t length, ResponseProcess response, CallbackType callback, KRSTarget target = { 0, 0 })
		{
			CommandBufferType<commandSize> command(data, length, 1, response, callback);
			this->commandStack.push(command);
			targetStack.push(target);
			if (this->isTrafficFree.get()) sendCommand();
		}

		void sendCommand()
		{
			this->currentCommand = this->commandStack.pop();
			currentTarget = targetStack.pop();
			this->serialPort.write(this->currentCommand.data, this->currentCommand.length);

			this->responsePos = 0;
			this->isTrafficFree.set(false);
			this->startTime = this->serialPort.time();
		}

		// 受信データバッファ
		struct KRSRom {
			uint8_t id;
			uint8_t data[64];
		};

		std::vector<KRSRom> eepromBuf;

		// 受信完了チェック関数
		bool isComplete(uint8_t* data, uint8_t length)
		{
			if (length == 0) return false;

			uint8_t header = (uint8_t)((data[0] & 0b11100000) >> 5);

			// ヘッダの値からデータ長を判定
			switch (header)
			{
			case 0:
			case 4:
				return (length >= 3);
			case 1:
				if (length < 2) return false;
				switch (data[1])
				{
				case 0: return (length >= 66);
				case 5: return (length >= 4);
				default: return (length >= 3);
				}
			case 2:
				if (length < 2) return false;
				switch (data[1])
				{
				case 0: return (length >= 2);
				default: return (length >= 3);
				}
			case 7: return true;
			}
			return false;
		}

		// ID不正チェック関数
		bool checkId(uint8_t id)
		{
			if (id < 0 || id > 31) return false;
			return true;
		}

		// 受信イベント関数
		void dataReceivedEvent(uint8_t* data, uint8_t length, uint8_t status)
		{
			uint32_t tmp = 0;

			// エラーステータスを更新
			this->errorBits |= status;

			// エラーがあれば強制的に完了処理
			if (this->errorBits != 0) {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(this->errorBits);
					this->currentCommand.callback(e);
					if (operatingMode) return;
				}
				responseData.set((int32_t)0);
				isReceived.set(true);
				return;
			}

			// パラメータ範囲を抽出
			uint8_t command = (uint8_t)((data[0] & 0b11100000) >> 5);
			uint8_t id = (uint8_t)((data[0] & 0b00011111));
			KRSRom rom;

			switch (command)
			{
			case 0:
			case 4:	tmp = data[2] + (data[1] << 7); break;
			case 7: tmp = data[1] & 0x1F; break;
			case 1:
				switch (data[1]) {
				case 0:

					for (uint8_t t = 0; t < eepromBuf.size(); t++) {
						if (eepromBuf[t].id == id)
						{
							memcpy(eepromBuf[t].data, data + 2, 64);
							for (uint8_t i = 0; i < currentTarget.length; i++) {
								tmp += (eepromBuf[t].data[currentTarget.address + i] << (4 * (currentTarget.length - i - 1)));
							}
							break;
						}

						if (t == eepromBuf.size() - 1) {
							memcpy(rom.data, data + 2, 64);
							eepromBuf.push_back(rom);

							for (uint8_t i = 0; i < currentTarget.length; i++) {
								tmp += (eepromBuf[t].data[currentTarget.address + i] << (4 * (currentTarget.length - i - 1)));
							}
							break;
						}
					}

					break;
				case 5: tmp = data[3] + (data[2] << 7); break;
				default: tmp = data[2]; break;
				}
				break;
			default: tmp = 0; break;
			}

			// エラーがなければ完了処理
			if (this->currentCommand.responseProcess) {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(data[0] & 0b11111, this->errorBits, this->currentCommand.responseProcess(tmp));
					this->currentCommand.callback(e);
					if (!operatingMode) isReceived.set(true);
				}
				else {
					responseData = (this->currentCommand.responseProcess(tmp));
					isReceived.set(true);
				}
			}
			else {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(data[0] & 0b11111, this->errorBits, (int32_t)tmp);
					this->currentCommand.callback(e);
					if (!operatingMode) isReceived.set(true);
				}
				else {
					responseData.set((int32_t)tmp);
					isReceived.set(true);
				}
			}
		}

		EventDataType getFunction(uint8_t* command, uint8_t length, ResponseProcess responseProcess = 0, CallbackType callback = 0, KRSTarget target = { 0, 0 })
		{
			// コールバックが無い且つ同期モードの時のみバス待ち
			if (!operatingMode || callback == 0) {
				while (!this->isTrafficFree.get());
				this->isReceived.set(false);
			}

			// Clear Error
			this->errorBits = 0;

			// コマンドを送信
			this->addCommand(command, length, responseProcess, callback, target);

			// 不必要なら空データを返す
			if (operatingMode && callback != 0) return EventDataType((int32_t)0);

			// 同期モードの時はリスナを起動
			while (!isReceived.get())
			{
				if (!this->operatingMode) this->listener();
			}

			return responseData;
		}
		void spin() { this->listener(); }

		bool isRomDataAvailable(uint8_t id)
		{
			for (uint8_t t = 0; t < eepromBuf.size(); t++) {
				if (eepromBuf[t].id == id) return true;
			}
			return false;
		}

		uint8_t* getRomData(uint8_t id)
		{
			for (uint8_t t = 0; t < eepromBuf.size(); t++) {
				if (eepromBuf[t].id == id)
				{
					return eepromBuf[t].data;
				}
			}
		}

		// ------------------------------------------------------------------------------------------
		static void defaultWriteCallback(CallbackEventArgs e) {  }
		static EventDataType temperatureProcess(int32_t data) { return EventDataType((int32_t)(100 - (data - 30) / 1.425)); }
		static EventDataType currentProcess(int32_t data) { return EventDataType((int32_t)((data < 63) ? data * 100 : (data - 64) * 100)); }
		static EventDataType positionProcess(int32_t data) { return EventDataType((gFloat)((7500 - data) / 29.629)); }
		static EventDataType baudrateProcess(int32_t data)
		{
			switch (data)
			{
			case 0x0A: return EventDataType((int32_t)(115200));
			case 0x01: return EventDataType((int32_t)(62000));
			case 0x00:return EventDataType((int32_t)(1250000));
			default: return EventDataType((int32_t)(0));
			}
		}

		// ------------------------------------------------------------------------------------------
	public:
		KRS() : responseData(EventDataType((int32_t)0)), isReceived(false) {}
		~KRS() {}
		// ------------------------------------------------------------------------------------------
		// General
		uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, CallbackType callback)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			if (address + length > 64) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, 0, callback, { (uint8_t)address, length });
		}
		void writeMemory(uint8_t id, uint16_t address, uint32_t data, uint8_t length)
		{
			if (!checkId(id)) { badInput(); return; }

			// 一度もEEPROMを読み込んでいない場合は終了
			if (!isRomDataAvailable(id)) { this->errorBits |= SystemError; return; }
			if (address + length > 64) { badInput(); return; }

			uint8_t *eepromData = getRomData(id);

			// EEPROMデータを更新
			for (uint8_t i = 0; i < length; i++) eepromData[address + i] = ((data >> (8 * i)) & 0xFF);

			uint8_t command[66];
			command[0] = (0b11000000 | id); command[1] = 0;
			memcpy(command + 2, eepromData, 64);
			getFunction(command, 66, 0, defaultWriteCallback);
		}

		// Ping
		uint16_t ping(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, 0, callback, { 0, 2 });
		}

		// Torque
		uint8_t readTorqueEnable(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeTorqueEnable(uint8_t id, uint8_t torque) { notSupport(); }

		// Temperature
		uint16_t readTemperature(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x04 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, temperatureProcess, callback);
		}

		// Current
		int32_t readCurrent(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x03 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, currentProcess, callback);
		}

		// Voltage
		gFloat readVoltage(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }

		// Target Position
		gFloat readTargetPosition(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeTargetPosition(uint8_t id, gFloat position)
		{
			uint8_t command[3] = { (0b10000000 | id), 0, 0 };

			if (position < -135) position = -135;
			else if (position > 135) position = 135;

			uint16_t tch = (uint16_t)(7500 - 29.629 * position);

			command[1] = ((tch >> 7) & 0x7F);
			command[2] = ((tch) & 0x7F);
			getFunction(command, 3, 0, defaultWriteCallback);
		}


		void writeTargetPosition(uint8_t id, gFloat position, gFloat *currentPosition)
		{
			uint8_t command[3] = { (0b10000000 | id), 0, 0 };

			if (position < -135) position = -135;
			else if (position > 135) position = 135;

			uint16_t tch = (uint16_t)(7500 - 29.629 * position);

			command[1] = ((tch >> 7) & 0x7F);
			command[2] = ((tch) & 0x7F);
			*currentPosition = (gFloat)getFunction(command, 3, positionProcess, 0);
		}

		// Current Position
		gFloat readCurrentPosition(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x05 };

			if (!checkId(id)) { badInput(); return 0; }

			return (gFloat)getFunction(command, 2, positionProcess, callback);
		}

		// Offset
		gFloat readOffset(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeOffset(uint8_t id, gFloat offset) { notSupport(); }

		// Deadband
		gFloat readDeadband(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2]{ 0b10100000 | id, 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			return (gFloat)getFunction(command, 2, 0, callback, { 8, 2 });
		}
		void writeDeadband(uint8_t id, gFloat deadband)
		{
			if (!checkId(id)) { badInput(); return; }

			if (!isRomDataAvailable(id)) { this->errorBits |= SystemError; return; }
			if (deadband < 0) { deadband = 0; }
			else if (deadband > 5) deadband = 5;

			// EEPROMデータを更新
			uint8_t* eepromData = getRomData(id);
			eepromData[8] = 0; eepromData[9] = deadband;

			uint8_t command[66];
			command[0] = (0b11000000 | id); command[1] = 0;
			memcpy(command + 2, eepromData, 64);
			getFunction(command, 66, 0, defaultWriteCallback);
		}

		// Target Time
		gFloat readTargetTime(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeTargetTime(uint8_t id, gFloat targetTime) { notSupport(); }

		// Accel Time
		gFloat readAccelTime(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeAccelTime(uint8_t id, gFloat accelTime) { notSupport(); }

		// P Gain
		uint32_t readPGain(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x01 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, 0, callback);
		}
		void writePGain(uint8_t id, uint32_t gain)
		{
			uint8_t command[3] = { (0b11000000 | id), 1, 0 };

			if (!checkId(id)) { badInput(); return; }

			if (gain < 1) gain = 1;
			else if (gain > 127) gain = 127;

			command[2] = gain;

			getFunction(command, 3, 0, defaultWriteCallback);
		}

		// I Gain
		uint32_t readIGain(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeIGain(uint8_t id, uint32_t gain) { notSupport(); }

		// D Gain
		uint32_t readDGain(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeDGain(uint8_t id, uint32_t gain) { notSupport(); }

		// Max Torque
		uint32_t readMaxTorque(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeMaxTorque(uint8_t id, uint32_t maxTorque) { notSupport(); }

		// Speed
		gFloat readSpeed(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2] = { (0b10100000 | id), 0x02 };

			if (!checkId(id)) { badInput(); return 0; }

			return (gFloat)getFunction(command, 2, 0, callback);
		}
		void writeSpeed(uint8_t id, gFloat speed)
		{
			uint8_t command[3] = { (0b11000000 | id), 2, 0 };

			if (!checkId(id)) { badInput(); return; }

			if (speed < 1) speed = 1;
			else if (speed > 127) speed = 127;

			command[2] = speed;

			getFunction(command, 3, 0, defaultWriteCallback);
		}

		// ID
		uint32_t readID(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[4] = { 0xFF, 0, 0, 0 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 4, 0, callback);
		}
		void writeID(uint8_t id, uint32_t newid)
		{
			uint8_t command[4]{ 0b11100000, 1, 1, 1 };

			if (!checkId(id)) { badInput(); return; }
			if (!checkId(newid)) { badInput(); return; }

			command[0] += newid;

			getFunction(command, 4, 0, 0);
		}

		// ROM
		void saveRom(uint8_t id) { notSupport(); }
		void loadRom(uint8_t id) { notSupport(); }
		void resetMemory(uint8_t id) { notSupport(); }

		// Baudrate
		uint32_t readBaudrate(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2]{ 0b10100000 | id, 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(command, 2, baudrateProcess, callback, { 26, 2 });
		}
		void writeBaudrate(uint8_t id, uint32_t baudrate)
		{
			if (!checkId(id)) { badInput(); return; }

			if (!isRomDataAvailable(id)) { this->errorBits |= SystemError; return; }

			uint8_t baudrateId = 0;
			switch (baudrate)
			{
			case 115200: baudrateId = 0x0A; break;
			case 625000: baudrateId = 0x01; break;
			case 1250000: baudrateId = 0x00; break;
			default: badInput(); return;
			}

			uint8_t* eepromData = getRomData(id);
			eepromData[27] = baudrateId;

			uint8_t command[66];
			command[0] = (0b11000000 | id); command[1] = 0;
			memcpy(command + 2, eepromData, 64);
			getFunction(command, 66, 0, defaultWriteCallback);
		}

		// CW Limit Position
		gFloat readLimitCWPosition(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2]{ 0b10100000 | id, 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			return (gFloat)getFunction(command, 2, positionProcess, callback, { 16, 4 });
		}
		void writeLimitCWPosition(uint8_t id, gFloat limitPosition)
		{
			if (!checkId(id)) { badInput(); return; }

			if (!isRomDataAvailable(id)) { this->errorBits |= SystemError; return; }

			int position = (7500 - limitPosition * 29.629);

			uint8_t* eepromData = getRomData(id);
			eepromData[16] = (position >> 12) & 0x0F;
			eepromData[17] = (position >> 8) & 0x0F;
			eepromData[18] = (position >> 4) & 0x0F;
			eepromData[19] = (position >> 0) & 0x0F;

			uint8_t command[66];
			command[0] = (0b11000000 | id); command[1] = 0;
			memcpy(command + 2, eepromData, 64);
			getFunction(command, 66, 0, defaultWriteCallback);
		}

		// CCW Limit Position
		gFloat readLimitCCWPosition(uint8_t id, CallbackType callback = 0)
		{
			uint8_t command[2]{ 0b10100000 | id, 0x00 };

			if (!checkId(id)) { badInput(); return 0; }

			return (gFloat)getFunction(command, 2, positionProcess, callback, { 20, 4 });
		}
		void writeLimitCCWPosition(uint8_t id, gFloat limitPosition)
		{
			if (!checkId(id)) { badInput(); return; }

			if (!isRomDataAvailable(id)) { this->errorBits |= SystemError; return; }

			int position = (7500 - limitPosition * 29.629);

			uint8_t* eepromData = getRomData(id);
			eepromData[20] = (position >> 12) & 0x0F;
			eepromData[21] = (position >> 8) & 0x0F;
			eepromData[22] = (position >> 4) & 0x0F;
			eepromData[23] = (position >> 0) & 0x0F;

			uint8_t command[66];
			command[0] = (0b11000000 | id); command[1] = 0;
			memcpy(command + 2, eepromData, 64);
			getFunction(command, 66, 0, defaultWriteCallback);
		}

		// Temperature Limit
		uint32_t readLimitTemperature(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeLimitTemperature(uint8_t id, uint32_t temperature)
		{
			uint8_t command[3] = { 0b11000000 | id, 2, 0 };

			uint8_t limit = ((100 - temperature) * 1.425 + 30);

			if (limit < 1) limit = 1;
			else if (limit > 127) limit = 127;

			command[2] = limit;

			getFunction(command, 3, 0, 0);
		}

		// Curent Limit
		uint32_t readLimitCurrent(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeLimitCurrent(uint8_t id, uint32_t current)
		{
			uint8_t command[3] = { 0b11000000 | id, 3, 0 };

			uint8_t limit = current / 100;

			if (limit < 1) limit = 1;
			else if (limit > 127) limit = 127;

			command[2] = limit;

			getFunction(command, 3, 0, 0);
		}

		// Drive Mode
		uint32_t readDriveMode(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeDriveMode(uint8_t id, uint32_t mode) { notSupport(); }

		// Burst Function
		void burstReadMemory(uint8_t* idList, uint8_t count, uint16_t address, uint8_t length, CallbackType callback) { notSupport(); }
		void burstWriteMemory(uint8_t* idList, uint32_t* dataList, uint8_t count, uint16_t address, uint8_t length) { notSupport(); }

		// Burst Function(Position)
		void burstReadPositions(uint8_t* idList, uint8_t count, CallbackType callback) { notSupport(); }
		void burstWriteTargetPositions(uint8_t* idList, gFloat* positionList, uint8_t count) { notSupport(); }
	};
}
