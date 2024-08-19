/*
* @file    gs2d_robotis.h
* @author
* @date    2021/01/26
* @brief
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gs2d_driver.h"
#include "gs2d_command.h"
#include "crc16.h"

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	template<class SerialClass, unsigned int bufferSize = 2, unsigned int commandSize = 50>
	class RobotisP20 : public CommandHandler<SerialClass, bufferSize, commandSize>, public Driver
	{
	private:
		// 受信データバッファ
		EventDataType responseData;
		Gs2dType<bool> isReceived;

		// コマンドリスト
		struct Instructions
		{
			static const uint8_t Ping = 0x01;
			static const uint8_t Read = 0x02;
			static const uint8_t Write = 0x03;
			static const uint8_t RegWrite = 0x04;
			static const uint8_t Action = 0x05;
			static const uint8_t FactoryReset = 0x06;
			static const uint8_t Reboot = 0x08;
			static const uint8_t SyncRead = 0x82;
			static const uint8_t SyncWrite = 0x83;
			static const uint8_t BulkRead = 0x92;
			static const uint8_t BulkWrite = 0x93;
		};

		// アドレスリスト
		struct Address
		{
			static const uint8_t Id = 7;
			static const uint8_t Baudrate = 8;
			static const uint8_t DriveMode = 10;
			static const uint8_t HomingOffset = 20;
			static const uint8_t TemperatureLimit = 31;
			static const uint8_t CurrentLimit = 38;
			static const uint8_t MaxPositionLimit = 48;
			static const uint8_t MinPositionLimit = 52;

			static const uint8_t TorqueEnable = 64;
			static const uint8_t PositionDGain = 80;
			static const uint8_t PositionIGain = 82;
			static const uint8_t PositionPGain = 84;
			static const uint8_t ProfileAcceleration = 108;
			static const uint8_t ProfileVelocity = 112;
			static const uint8_t GoalPosition = 116;
			static const uint8_t PresentCurrent = 126;
			static const uint8_t PresentVelocity = 128;
			static const uint8_t PresentPosition = 132;
			static const uint8_t PresentVoltage = 144;
			static const uint8_t PresentTemperature = 146;

			static const uint8_t GoalCurrent = 102;
			static const uint8_t OperatingMode = 11;
		};

		// 受信完了チェック関数
		bool isComplete(uint8_t* data, uint8_t length)
		{
			if (length < 6) return false;
			return (length >= data[5] + 7);
		}

		// ID不正チェック関数
		bool checkId(uint8_t id)
		{
			if (id < 0 || id > 254 || id == 253) return false;
			return true;
		}

		// 受信イベント関数
		void dataReceivedEvent(uint8_t* data, uint8_t length, uint8_t status)
		{
			uint32_t tmp = 0; // パラメータ用
			uint16_t paramLength = 0;

			// エラーステータスを更新
			this->errorBits |= status;

			do {
				// エラー状態チェック
				if (this->errorBits != 0) break;

				// 最低限の長さがあるか確認
				if (length < 9) { this->errorBits |= ResponseError; break; }

				// インストラクション値を確認
				if (data[7] != 0x55) { this->errorBits |= ResponseError; break; }

				// Lengthを取得して確認
				paramLength = (data[5] + (data[6] << 8));
				if (length != paramLength + 7) { this->errorBits |= ResponseError; break; }

				// Errorバイトを確認
				if (data[8] != 0) { this->errorBits |= ProtocolError; break; }

				// CheckSum検証
				uint16_t crc = data[length - 2] + (data[length - 1] << 8);
				if (crc != crc16::calculate(data, length - 2)) { this->errorBits |= ResponseError; break; }
			} while (false);

			// エラーがあれば終了
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

			// Parameterがあれば切り出し
			if (paramLength > 4) {
				for (int i = 0; i < paramLength - 4; i++) {
					tmp += (data[9 + i] << (i * 8));
				}
			}

			// データをコールバックか戻り値へ
			if (this->currentCommand.responseProcess) {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(data[4], this->errorBits, this->currentCommand.responseProcess(tmp));
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
					CallbackEventArgs e(data[4], this->errorBits, (int32_t)tmp);
					this->currentCommand.callback(e);
					if (!operatingMode) isReceived.set(true);
				}
				else {
					responseData.set((int32_t)tmp);
					isReceived.set(true);
				}
			}
		}

		// コマンドのパラメータ部生成
		template<typename T>
		uint8_t generateParameters(uint16_t address, T data, uint8_t length, uint8_t* buffer)
		{
			if (length > 4) length = 4;

			buffer[0] = (address & 0xFF);
			buffer[1] = ((address >> 8) & 0xFF);
			for (uint8_t i = 0; i < length; i++) {
				buffer[2 + i] = (((uint32_t)data >> (i * 8)) & 0xFF);
			}

			return length + 2;
		}

		EventDataType getFunction(uint8_t id, uint8_t instruction, uint8_t* param = 0, uint8_t length = 0, ResponseProcess responseProcess = 0, CallbackType callback = 0, uint8_t count = 1)
		{
			// コマンドの長さを取得
			uint8_t bufferLength = 4 + 1 + 2 + 1 + 2 + length;

			// コマンド領域を確保
			uint8_t* command = new uint8_t[bufferLength];

			// ヘッダー設定
			command[0] = 0xFF; command[1] = 0xFF; command[2] = 0xFD; command[3] = 0x00; command[4] = id;

			// Length設定
			command[5] = ((length + 3) & 0xFF);
			command[6] = (((length + 3) >> 8) & 0xFF);

			// Instruction設定
			command[7] = instruction;

			// Parameters
			if (length) memcpy(command + 8, param, length);

			// CheckSum設定
			uint16_t crc = crc16::calculate(command, bufferLength - 2);
			command[bufferLength - 2] = crc & 0xFF;
			command[bufferLength - 1] = (crc >> 8) & 0xFF;

			// Clear Error
			this->errorBits = 0;

			// バス待ち
			if (!operatingMode || callback == 0) {
				while (!this->isTrafficFree.get());
				this->isReceived.set(false);
			}

			// 送信
			this->addCommand(command, bufferLength, responseProcess, callback, count);
			delete[] command;

			// 不必要なら空データを返す
			if (operatingMode && callback != 0 || count == 0) return EventDataType((int32_t)0);

			// 同期モードの時はリスナを起動
			while (!isReceived.get())
			{
				if (!this->operatingMode) this->listener();
			}

			return responseData;
		}

		void spin() { this->listener(); }

		// ------------------------------------------------------------------------------------------
		static void defaultWriteCallback(CallbackEventArgs e) {  }

		// �P�ʕϊ��֐�
		static EventDataType pingProcess(int32_t data) { return EventDataType((int32_t)(data & 0xFFFF)); }
		static EventDataType readTorqueProcess(int32_t data) { return EventDataType((int32_t)((data) ? 1 : 0)); }
		static EventDataType currentProcess(int32_t data) { return EventDataType((int32_t)(data * 2.69)); }
		static EventDataType voltageProcess(int32_t data) { return EventDataType((gFloat)(data / 10.0)); }
		static EventDataType targetPositionProcess(int32_t data) { return EventDataType((gFloat)(data * 360.0 / 4096.0 - 180.0)); }
		static EventDataType offsetProcess(int32_t data) { return EventDataType((gFloat)(data * 0.088)); }
		static EventDataType targetTimeProcess(int32_t data) { return EventDataType((gFloat)(data / 1000.0)); }
		static EventDataType speedProcess(int32_t data) { return EventDataType((gFloat)(data * 0.229 * 6)); }
		static EventDataType baudrateProcess(int32_t data)
		{
			int32_t baudrateList[8]{ 9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000 };
			if (data > 7) return EventDataType((int32_t)0);
			return EventDataType(baudrateList[data]);
		}

		// ------------------------------------------------------------------------------------------
	public:
		RobotisP20() : responseData(EventDataType((int32_t)0)), isReceived(false) {}
		~RobotisP20() {}
		// ------------------------------------------------------------------------------------------
		// General
		uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, CallbackType callback)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t paramLength = generateParameters(address, length, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, paramLength, 0, callback);
		}
		void writeMemory(uint8_t id, uint16_t address, uint32_t data, uint8_t length)
		{
			if (!checkId(id)) { badInput(); return; }

			uint8_t param[6];
			uint8_t paramLength = generateParameters(address, data, length, param);

			getFunction(id, Instructions::Write, param, paramLength, 0, defaultWriteCallback);
		}

		// Ping
		uint16_t ping(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			return (int32_t)getFunction(id, Instructions::Ping, 0, 0, pingProcess, callback);
		}

		// Torque
		uint8_t readTorqueEnable(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::TorqueEnable, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, readTorqueProcess, callback);
		}

		void writeTorqueEnable(uint8_t id, uint8_t torque)
		{
			if (!checkId(id)) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::TorqueEnable, torque, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		uint8_t readOperatingMode(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::OperatingMode, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}

		void writeOperatingMode(uint8_t id, uint8_t mode)
		{
			if (!checkId(id)) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::OperatingMode, mode, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Temperature
		uint16_t readTemperature(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PresentTemperature, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}

		// Current
		int32_t readCurrent(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PresentCurrent, 2, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, currentProcess, callback);
		}

		// Voltage
		gFloat readVoltage(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PresentVoltage, 2, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, voltageProcess, callback);
		}

		// Target Position
		gFloat readTargetPosition(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::GoalPosition, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetPositionProcess, callback);
		}

		void writeTargetPosition(uint8_t id, gFloat position)
		{
			if (!checkId(id)) { badInput(); return; }

			if (position > 180.0) position = 180.0;
			else if (position < -180.0) position = -180.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::GoalPosition, (uint32_t)((position + 180.0) * 4096.0 / 360.0), 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		void writeGoalCurrent(uint8_t id, gFloat current)
		{
			if (!checkId(id)) { badInput(); return; }

			if (current < 0.0) current = 0.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::GoalCurrent, current, 2, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Current Position
		gFloat readCurrentPosition(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PresentPosition, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetPositionProcess, callback);
		}

		// Offset
		gFloat readOffset(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::HomingOffset, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, offsetProcess, callback);
		}
		void writeOffset(uint8_t id, gFloat offset)
		{
			if (!checkId(id)) { badInput(); return; }

			offset /= 0.088;

			if (offset > 1044479) offset = 1044479;
			else if (offset < -1044479) offset = -1044479;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::HomingOffset, (uint32_t)(offset), 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Deadband
		gFloat readDeadband(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeDeadband(uint8_t id, gFloat deadband) { notSupport(); }

		// Target Time
		gFloat readTargetTime(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }
			if (!(readDriveMode(id) && 0x04)) {
				this->invalidMode(); return 0;
				/*
				* DynamixelのDriveModeレジスタのBit2が1の場合のみ遷移時間指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}


			uint8_t param[6];
			uint8_t length = generateParameters(Address::ProfileVelocity, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetTimeProcess, callback);
		}
		void writeTargetTime(uint8_t id, gFloat targetTime)
		{
			if (!checkId(id)) { badInput(); return; }
			if (!(readDriveMode(id) && 0x04)) {
				this->invalidMode(); return;
				/*
				* DynamixelのDriveModeレジスタのBit2が1の場合のみ遷移時間指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}

			if (targetTime < 0) targetTime = 0;
			else if (targetTime > 32.737) targetTime = 32.737;

			targetTime *= 1000.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::ProfileVelocity, (uint32_t)targetTime, 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Accel Time
		gFloat readAccelTime(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			if (!(readDriveMode(id) && 0x04)) {
				this->invalidMode(); return 0;
				/*
				* DynamixelのDriveModeレジスタのBit2が1の場合のみ遷移時間指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}

			uint8_t param[6];
			uint8_t length = generateParameters(Address::ProfileAcceleration, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetTimeProcess, callback);
		}
		void writeAccelTime(uint8_t id, gFloat accelTime)
		{
			if (!checkId(id)) { badInput(); return; }
			if (!(readDriveMode(id) && 0x04)) {
				this->invalidMode(); return;
				/*
				* DynamixelのDriveModeレジスタのBit2が1の場合のみ遷移時間指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}

			if (accelTime < 0) accelTime = 0;
			else if (accelTime > 32.737) accelTime = 32.737;

			accelTime *= 1000.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::ProfileAcceleration, (uint32_t)accelTime, 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// P Gain
		uint32_t readPGain(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionPGain, 2, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writePGain(uint8_t id, uint32_t gain)
		{
			if (!checkId(id)) { badInput(); return; }

			if (gain < 0) gain = 0;
			else if (gain > 16383) gain = 16383;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionPGain, (uint32_t)gain, 2, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// I Gain
		uint32_t readIGain(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionIGain, 2, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writeIGain(uint8_t id, uint32_t gain)
		{
			if (!checkId(id)) { badInput(); return; }

			if (gain < 0) gain = 0;
			else if (gain > 16383) gain = 16383;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionIGain, (uint32_t)gain, 2, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// D Gain
		uint32_t readDGain(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionDGain, 2, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writeDGain(uint8_t id, uint32_t gain)
		{
			if (!checkId(id)) { badInput(); return; }

			if (gain < 0) gain = 0;
			else if (gain > 16383) gain = 16383;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PositionDGain, (uint32_t)gain, 2, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Max Torque
		uint32_t readMaxTorque(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeMaxTorque(uint8_t id, uint32_t maxTorque) { notSupport(); }

		// Speed
		gFloat readSpeed(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }
			if (readDriveMode(id) && 0x04) {
				this->invalidMode(); return 0;
				/*
				* DynamixelのDriveModeレジスタのBit2が0の場合のみ回転速度の指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile ( 0 : Speed )
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}

			uint8_t param[6];
			uint8_t length = generateParameters(Address::PresentVelocity, 4, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, speedProcess, callback);
		}
		void writeSpeed(uint8_t id, gFloat speed)
		{
			if (!checkId(id)) { badInput(); return; }
			if (readDriveMode(id) && 0x04) {
				this->invalidMode(); return;
				/*
				* DynamixelのDriveModeレジスタのBit2が0の場合のみ回転速度の指定が可能です。writeDriveModeで変更してください。
				*
				* Drive Mode :
				* 0b00000100
				*   |||||||+------- Reverse Mode
				*   ||||||+-------- Unused
				*   |||||+--------- Time-based Profile ( 0 : Speed )
				*   ||||+---------- Torque On by Goal-Update
				*   ++++----------- Unused
				*/
			}

			uint32_t rev_min = (int)(speed / 0.229 / 6.0);
			if (rev_min < 0) rev_min = 0;
			else if (rev_min > 32737) rev_min = 32737;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::ProfileVelocity, (uint32_t)rev_min, 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// ID
		uint32_t readID(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::Id, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writeID(uint8_t id, uint32_t newid)
		{
			if (!checkId(id)) { badInput(); return; }
			if (!checkId(newid)) { badInput(); return; }

			if (newid < 0) newid = 0;
			else if (newid > 252) newid = 252;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::Id, (uint32_t)newid, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// ROM
		void saveRom(uint8_t id) { notSupport(); }
		void loadRom(uint8_t id) { notSupport(); }
		void resetMemory(uint8_t id)
		{
			uint8_t param[1] = { 0x02 };
			getFunction(id, Instructions::FactoryReset, param, 1);
		}

		// Baudrate
		uint32_t readBaudrate(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::Baudrate, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, baudrateProcess, callback);
		}
		void writeBaudrate(uint8_t id, uint32_t baudrate)
		{
			if (!checkId(id)) { badInput(); return; }

			int32_t baudrateList[8]{ 9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000 };
			uint8_t baudrateIndex = 100;

			for (uint8_t i = 0; i < 8; i++) { if (baudrateList[i] == baudrate) { baudrateIndex = i; break; } }
			if (baudrateIndex == 100) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::Baudrate, (uint32_t)baudrateIndex, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// CW Limit Position
		gFloat readLimitCWPosition(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }
			if (readOperatingMode(id) != 3) {
				this->invalidMode(); return 0;
				/*
				* DynamixelのOperatingModeレジスタが3の時のみ有効です。writeOperatingModeで変更してください。
				*
				* Operating Mode :
				* 0 : Current Control Mode
				* 1 : Velocity Control Mode
				* 3(Default) : Position Control Mode
				* 4 : Extended Position Control Mode
				* 5 : Current-based Position Control Mode
				* 16 : PWM Control Mode
				*/
			}

			uint8_t param[6];
			uint8_t length = generateParameters(Address::MinPositionLimit, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetPositionProcess, callback);
		}
		void writeLimitCWPosition(uint8_t id, gFloat limitPosition)
		{
			if (!checkId(id)) { badInput(); return; }
			if (readOperatingMode(id) != 3) {
				this->invalidMode(); return;
				/*
				* DynamixelのOperatingModeレジスタが3の時のみ有効です。writeOperatingModeで変更してください。
				*
				* Operating Mode :
				* 0 : Current Control Mode
				* 1 : Velocity Control Mode
				* 3(Default) : Position Control Mode
				* 4 : Extended Position Control Mode
				* 5 : Current-based Position Control Mode
				* 16 : PWM Control Mode
				*/
			}

			if (limitPosition > 180.0) limitPosition = 180.0;
			else if (limitPosition < -180.0) limitPosition = -180.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::MinPositionLimit, (uint32_t)((limitPosition + 180.0) * 4096.0 / 360.0), 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// CCW Limit Position
		gFloat readLimitCCWPosition(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }
			if (readOperatingMode(id) != 3) {
				this->invalidMode(); return 0;
				/*
				* DynamixelのOperatingModeレジスタが3の時のみ有効です。writeOperatingModeで変更してください。
				*
				* Operating Mode :
				* 0 : Current Control Mode
				* 1 : Velocity Control Mode
				* 3(Default) : Position Control Mode
				* 4 : Extended Position Control Mode
				* 5 : Current-based Position Control Mode
				* 16 : PWM Control Mode
				*/
			}

			uint8_t param[6];
			uint8_t length = generateParameters(Address::MaxPositionLimit, 4, 2, param);

			return (gFloat)getFunction(id, Instructions::Read, param, length, targetPositionProcess, callback);
		}
		void writeLimitCCWPosition(uint8_t id, gFloat limitPosition)
		{
			if (!checkId(id)) { badInput(); return; }
			if (readOperatingMode(id) != 3) {
				this->invalidMode(); return;
				/*
				* DynamixelのOperatingModeレジスタが3の時のみ有効です。writeOperatingModeで変更してください。
				*
				* Operating Mode :
				* 0 : Current Control Mode
				* 1 : Velocity Control Mode
				* 3(Default) : Position Control Mode
				* 4 : Extended Position Control Mode
				* 5 : Current-based Position Control Mode
				* 16 : PWM Control Mode
				*/
			}

			if (limitPosition > 180.0) limitPosition = 180.0;
			else if (limitPosition < -180.0) limitPosition = -180.0;

			uint8_t param[6];
			uint8_t length = generateParameters(Address::MaxPositionLimit, (uint32_t)((limitPosition + 180.0) * 4096.0 / 360.0), 4, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Temperature Limit
		uint32_t readLimitTemperature(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::TemperatureLimit, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writeLimitTemperature(uint8_t id, uint32_t temperature)
		{
			if (!checkId(id)) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::TemperatureLimit, (uint32_t)temperature, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Curent Limit
		uint32_t readLimitCurrent(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::CurrentLimit, 2, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, currentProcess, callback);
		}
		void writeLimitCurrent(uint8_t id, uint32_t current)
		{
			if (!checkId(id)) { badInput(); return; }

			if (current < 0 || current > 3210) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::CurrentLimit, (uint32_t)(current / 2.69), 2, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Drive Mode
		uint32_t readDriveMode(uint8_t id, CallbackType callback = 0)
		{
			if (!checkId(id)) { badInput(); return 0; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::DriveMode, 1, 2, param);

			return (int32_t)getFunction(id, Instructions::Read, param, length, 0, callback);
		}
		void writeDriveMode(uint8_t id, uint32_t mode)
		{
			if (!checkId(id)) { badInput(); return; }

			if (mode > 5) { badInput(); return; }

			uint8_t param[6];
			uint8_t length = generateParameters(Address::DriveMode, (uint32_t)mode, 1, param);

			getFunction(id, Instructions::Write, param, length, 0, defaultWriteCallback);
		}

		// Burst Function
		void burstReadMemory(uint8_t* idList, uint8_t count, uint16_t address, uint8_t length, CallbackType callback)
		{
			uint8_t* param = new uint8_t[count + 4];
			param[0] = (address & 0xFF);
			param[1] = ((address & 0xFF) >> 8);
			param[2] = (length & 0xFF);
			param[3] = ((length & 0xFF) >> 8);
			for (uint8_t i = 0; i < count; i++) param[4 + i] = idList[i];

			getFunction(0xFE, Instructions::SyncRead, param, count + 4, 0, callback, count);
			delete[] param;
		}
		void burstWriteMemory(uint8_t* idList, uint32_t* dataList, uint8_t count, uint16_t address, uint8_t length)
		{
			uint8_t* param = new uint8_t[4 + count * (length + 1)];

			param[0] = (address & 0xFF);
			param[1] = ((address & 0xFF) >> 8);
			param[2] = (length & 0xFF);
			param[3] = ((length & 0xFF) >> 8);

			for (uint8_t i = 0; i < count; i++) {
				param[4 + i * (length + 1)] = idList[i];
				for (uint8_t k = 0; k < length; k++) {
					param[5 + i * (length + 1) + k] = ((dataList[i] >> (8 * k)) & 0xFF);
				}
			}

			getFunction(0xFE, Instructions::SyncWrite, param, 4 + count * (length + 1), 0, defaultWriteCallback, 0);
			delete[] param;
		}

		// Burst Function(Position)
		void burstReadPositions(uint8_t* idList, uint8_t count, CallbackType callback)
		{
			uint8_t* param = new uint8_t[count + 4];
			param[0] = (Address::PresentPosition & 0xFF);
			param[1] = ((Address::PresentPosition & 0xFF) >> 8);
			param[2] = 4;
			param[3] = 0;
			for (uint8_t i = 0; i < count; i++) param[4 + i] = idList[i];

			getFunction(0xFE, Instructions::SyncRead, param, count + 4, targetPositionProcess, callback, count);
			delete[] param;
		}
		void burstWriteTargetPositions(uint8_t* idList, gFloat* positionList, uint8_t count)
		{
			uint32_t* dataList = new uint32_t[count];

			for (uint8_t i = 0; i < count; i++) {
				dataList[i] = (positionList[i] + 180.0) * 4096.0 / 360.0;
			}

			uint8_t* param = new uint8_t[4 + count * 5];

			param[0] = (Address::GoalPosition & 0xFF);
			param[1] = ((Address::GoalPosition & 0xFF) >> 8);
			param[2] = 4;
			param[3] = 0;

			for (uint8_t i = 0; i < count; i++) {
				param[4 + i * 5] = idList[i];
				for (uint8_t k = 0; k < 4; k++) {
					param[5 + i * 5 + k] = ((dataList[i] >> (8 * k)) & 0xFF);
				}
			}

			getFunction(0xFE, Instructions::SyncWrite, param, 4 + count * 5, 0, defaultWriteCallback, 0);
			delete[] dataList;
			delete[] param;
		}
	};

}
