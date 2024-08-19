/*
* @file    gs2d_futaba.h
* @author
* @date    2021/01/30
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
    class Futaba : public CommandHandler<SerialClass, bufferSize, commandSize>, public Driver
    {
    private:
        // 受信データバッファ
        EventDataType responseData;
        Gs2dType<bool> isReceived;

        // Address
        struct Address
        {
            static const uint8_t ModelNumber = 0x00;
            static const uint8_t Id = 0x04;
            static const uint8_t Baudrate = 0x06;
            static const uint8_t CCWLimit = 0x0A;
            static const uint8_t CWLimit = 0x08;
            static const uint8_t TemperatureLimit = 0x0E;
            static const uint8_t TargetPosition = 0x1E;
            static const uint8_t CurrentPosition = 0x2A;
            static const uint8_t Current = 0x30;
            static const uint8_t Temperature = 0x32;
            static const uint8_t Voltage = 0x34;
            static const uint8_t TorqueEnable = 0x24;
            static const uint8_t TargetTime = 0x20;
            static const uint8_t CurrentSpeed = 0x2E;
            static const uint8_t PGain = 0x26;
            static const uint8_t MaxTorque = 0x23;
            static const uint8_t WriteFlashRom = 0xFF;
            static const uint8_t ResetMemory = 0xFF;
        };

        // 受信完了チェック関数
        bool isComplete(uint8_t* data, uint8_t length)
        {
            if (length < 6) return false;
            return (length >= data[5] + 8);
        }

        // ID不正チェック関数
        bool checkId(uint8_t id)
        {
            if (id < 1 || id > 127) return false;
            return true;
        }

        // CheckSum
        uint8_t calculateCheckSum(uint8_t* data, uint8_t length)
        {
            uint8_t sum = 0;
            for (uint8_t i = 2; i < length - 1; i++) sum ^= data[i];
            return (sum & 0xFF);
        }

        // 受信イベント関数
        void dataReceivedEvent(uint8_t* data, uint8_t length, uint8_t status)
        {
            uint32_t tmp = 0;

            // エラーステータスを更新
            this->errorBits |= status;

            do {
                // エラーがあれば受信を中止
                if (this->errorBits != 0) break;

                // チェックサム確認
                if (data[length - 1] != calculateCheckSum(data, length)) { this->errorBits |= ResponseError; break; }
            } while (false);

            // エラーがあれば強制的に完了処理
            if (this->errorBits != 0) {
                // コールバックがあれば起動
                if (this->currentCommand.callback) {
                    CallbackEventArgs e(this->errorBits);
                    this->currentCommand.callback(e);
                    if (operatingMode) return;
                }
                responseData.set((int32_t)0);
                isReceived.set(true);
                return;
            }

            // パラメータ抽出
            if (length > 8) {
                for (int i = 0; i < length - 8; i++) {
                    tmp += (data[7 + i] << (i * 8));
                }
            }

            // エラーがなければ完了処理
            if (this->currentCommand.responseProcess) {
                if (this->currentCommand.callback) {
                    CallbackEventArgs e(data[2], this->errorBits, this->currentCommand.responseProcess(tmp));
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
                    CallbackEventArgs e(data[2], this->errorBits, (int32_t)tmp);
                    this->currentCommand.callback(e);
                    if (!operatingMode) isReceived.set(true);
                }
                else {
                    responseData.set((int32_t)tmp);
                    isReceived.set(true);
                }
            }
        }

        void setFunction(uint8_t id, uint8_t address, uint32_t param = 0, uint8_t length = 0, ResponseProcess responseProcess = 0, CallbackType callback = 0, uint8_t flag = 0, uint8_t count = 1)
        {
            uint8_t bufferLength = 8 + length;
            uint8_t* command = new uint8_t[bufferLength];

            // ヘッダーを設定
            command[0] = 0xFA; command[1] = 0xAF; command[2] = id;
            command[3] = flag; command[4] = address; command[5] = length;
            command[6] = count;

            if (length != 0) {
                for (uint8_t i = 0; i < length; i++) command[7 + i] = (param >> (i * 8)) & 0xFF;
            }
            command[bufferLength - 1] = calculateCheckSum(command, bufferLength);

            // Clear Error
            this->errorBits = 0;

            // コマンド送信
            this->addCommand(command, bufferLength, responseProcess, callback, 0);
            delete[] command;
        }

        EventDataType getFunction(uint8_t id, uint8_t address, uint8_t flag, uint8_t length, ResponseProcess responseProcess = 0, CallbackType callback = 0)
        {
            uint8_t bufferLength = 8;
            uint8_t* command = new uint8_t[bufferLength];

            // ヘッダーをセット
            command[0] = 0xFA; command[1] = 0xAF; command[2] = id;
            command[3] = flag; command[4] = address; command[5] = length;
            command[6] = 0;

            command[bufferLength - 1] = calculateCheckSum(command, bufferLength);

            // コールバックが無い且つ同期モードの時のみバス待ち
            if (!operatingMode || callback == 0) {
                while (!this->isTrafficFree.get());
                this->isReceived.set(false);
            }

            // Clear Error
            this->errorBits = 0;

            // コマンドを送信
            this->addCommand(command, bufferLength, responseProcess, callback, 1);
            delete[] command;

            // 不必要なら空データを返す
            if (operatingMode && callback != 0) return EventDataType((int32_t)0);

            // 同期モードの時はリスナを起動
            while (!isReceived.get())
            {
                if (!this->operatingMode) this->listener();
            }

            return responseData;
        }

        void setFunctionBurstWrite(uint8_t id, uint8_t address, uint8_t flag, uint8_t length, uint8_t* idList, uint32_t* dataList, uint8_t count, ResponseProcess responseProcess = 0, CallbackType callback = 0)
        {
            uint8_t bufferLength = 8 + (length * count);
            uint8_t* command = new uint8_t[bufferLength];
            uint8_t pos = 0;

            // ヘッダーをセット
            command[pos++] = 0xFA; command[pos++] = 0xAF; command[pos++] = id;
            command[pos++] = flag; command[pos++] = address; command[pos++] = length;
            command[pos++] = count;

            for (uint8_t i = 0; i < count; i++) {
                command[pos++] = idList[i];
                for (uint8_t k = 0; k < length - 1; k++) {
                    command[pos++] = (dataList[i] >> (k * 8)) & 0xFF;
                }
            }
            command[pos] = calculateCheckSum(command, bufferLength);

            // コールバックが無い且つ同期モードの時のみバス待ち
            if (!operatingMode || callback == 0) {
                while (!this->isTrafficFree.get());
                this->isReceived.set(false);
            }

            // Clear Error
            this->errorBits = 0;

            // コマンドを送信
            this->addCommand(command, bufferLength, responseProcess, callback, 0);
            delete[] command;
        }
        void spin() { this->listener(); }

        // ------------------------------------------------------------------------------------------
        static EventDataType voltageProcess(int32_t data) { return EventDataType((gFloat)(data / 100.0)); }
        static EventDataType positionProcess(int32_t data) { return EventDataType((gFloat)(-(int16_t)data / 10.0)); }
        static EventDataType targetTimeProcess(int32_t data) { return EventDataType((gFloat)(data / 100.0)); }
        static EventDataType speedCallback(int32_t data) { return EventDataType((int32_t)(int16_t)data); }
        static EventDataType baudrateProcess(int32_t data)
        {
            int32_t baudrateList[10]{ 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400 };
            if (data > 9) return EventDataType((int32_t)0);
            return EventDataType(baudrateList[data]);
        }

        // ------------------------------------------------------------------------------------------
    public:
        Futaba() : responseData(EventDataType((int32_t)0)), isReceived(false) {}
        ~Futaba() {}

        // ------------------------------------------------------------------------------------------
        // General
        uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, CallbackType callback)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, address, 0x0F, length, 0, callback);
        }
        void writeMemory(uint8_t id, uint16_t address, uint32_t data, uint8_t length)
        {
            if (!checkId(id)) { badInput(); return; }

            setFunction(id, address, data, length, 0, 0);
        }

        // Ping
        uint16_t ping(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::ModelNumber, 0x0F, 2, 0, callback);
        }

        // Torque
        uint8_t readTorqueEnable(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::TorqueEnable, 0x0F, 1, 0, callback);
        }
        void writeTorqueEnable(uint8_t id, uint8_t torque)
        {
            if (!checkId(id)) { badInput(); return; }

            setFunction(id, Address::TorqueEnable, torque, 1);
        }

        // Temperature
        uint16_t readTemperature(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::Temperature, 0x0F, 2, 0, callback);
        }

        // Current
        int32_t readCurrent(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::Current, 0x0F, 2, 0, callback);
        }

        // Voltage
        gFloat readVoltage(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::Voltage, 0x0F, 2, voltageProcess, callback);
        }

        // Target Position
        gFloat readTargetPosition(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::TargetPosition, 0x0F, 2, positionProcess, callback);
        }
        void writeTargetPosition(uint8_t id, gFloat position)
        {
            if (!checkId(id)) { badInput(); return; }

            if (position < -150) position = -150;
            else if (position > 150) position = 150;

            uint16_t positionInt = (int16_t)(position * -10);

            setFunction(id, Address::TargetPosition, positionInt, 2);
        }

        // Current Position
        gFloat readCurrentPosition(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::CurrentPosition, 0x0F, 2, positionProcess, callback);
        }

        // Offset
        gFloat readOffset(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeOffset(uint8_t id, gFloat offset) { notSupport(); }

        // Deadband
        gFloat readDeadband(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeDeadband(uint8_t id, gFloat deadband) { notSupport(); }

        // Target Time
        gFloat readTargetTime(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::TargetTime, 0x0F, 2, targetTimeProcess, callback);
        }
        void writeTargetTime(uint8_t id, gFloat targetTime)
        {
            if (!checkId(id)) { badInput(); return; }

            if (targetTime < 0) targetTime = 0;
            else if (targetTime > 163.83) targetTime = 163.83;

            uint16_t targetTimeInt = targetTime * 100;

            setFunction(id, Address::TargetTime, targetTimeInt, 2);
        }

        // Accel Time
        gFloat readAccelTime(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeAccelTime(uint8_t id, gFloat accelTime) { notSupport(); }

        // P Gain
        uint32_t readPGain(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::PGain, 0x0F, 1, 0, callback);
        }
        void writePGain(uint8_t id, uint32_t gain)
        {
            if (!checkId(id)) { badInput(); return; }

            if (gain < 1) gain = 1;
            else if (gain > 255) gain = 255;

            setFunction(id, Address::PGain, gain, 1);
        }

        // I Gain
        uint32_t readIGain(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeIGain(uint8_t id, uint32_t gain) { notSupport(); }

        // D Gain
        uint32_t readDGain(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeDGain(uint8_t id, uint32_t gain) { notSupport(); }

        // Max Torque
        uint32_t readMaxTorque(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::MaxTorque, 0x0F, 1, 0, callback);
        }
        void writeMaxTorque(uint8_t id, uint32_t maxTorque)
        {
            if (!checkId(id)) { badInput(); return; }

            if (maxTorque < 1) maxTorque = 1;
            else if (maxTorque > 100) maxTorque = 100;

            setFunction(id, Address::MaxTorque, maxTorque, 1);
        }

        // Speed
        gFloat readSpeed(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::CurrentSpeed, 0x0F, 2, speedCallback, callback);
        }
        void writeSpeed(uint8_t id, gFloat speed) { notSupport(); }

        // ID
        uint32_t readID(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::Id, 0x0F, 1, 0, callback);
        }
        void writeID(uint8_t id, uint32_t newid)
        {
            if (!checkId(id)) { badInput(); return; }
            if (!checkId(newid)) { badInput(); return; }

            setFunction(id, Address::Id, newid, 1);
        }

        // ROM
        void saveRom(uint8_t id)
        {
            if (!checkId(id)) { badInput(); return; }

            setFunction(id, Address::WriteFlashRom, 0, 0, 0, 0, 0x40, 0);
        }
        void loadRom(uint8_t id) { notSupport(); }
        void resetMemory(uint8_t id)
        {
            if (!checkId(id)) { badInput(); return; }

            setFunction(id, Address::ResetMemory, 0, 0, 0, 0, 0x10, 0);
        }

        // Baudrate
        uint32_t readBaudrate(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::Baudrate, 0x0F, 1, baudrateProcess, callback);
        }
        void writeBaudrate(uint8_t id, uint32_t baudrate)
        {
            if (!checkId(id)) { badInput(); return; }

            int32_t baudrateList[10]{ 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400 };
            uint8_t baudrateIndex = 100;

            for (uint8_t i = 0; i < 10; i++) {
                if (baudrateList[i] == baudrate) { baudrateIndex = i; }
            }
            if (baudrateIndex == 100) { badInput(); return; }

            setFunction(id, Address::Baudrate, baudrateIndex, 1);
        }

        // CW Limit Position
        gFloat readLimitCWPosition(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::CWLimit, 0x0F, 2, positionProcess, callback);
        }
        void writeLimitCWPosition(uint8_t id, gFloat limitPosition)
        {
            if (!checkId(id)) { badInput(); return; }

            if (limitPosition < -150 || limitPosition > 0) { badInput(); return; }

            uint16_t limitShort = limitPosition * -10;

            setFunction(id, Address::CWLimit, limitShort, 2);
        }

        // CCW Limit Position
        gFloat readLimitCCWPosition(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (gFloat)getFunction(id, Address::CCWLimit, 0x0F, 2, positionProcess, callback);
        }
        void writeLimitCCWPosition(uint8_t id, gFloat limitPosition)
        {
            if (!checkId(id)) { badInput(); return; }

            if (limitPosition > 150 || limitPosition < 0) { badInput(); return; }

            uint16_t limitShort = (int16_t)(limitPosition * -10);

            setFunction(id, Address::CCWLimit, limitShort, 2);
        }

        // Temperature Limit
        uint32_t readLimitTemperature(uint8_t id, CallbackType callback = 0)
        {
            if (!checkId(id)) { badInput(); return 0; }

            return (int32_t)getFunction(id, Address::TemperatureLimit, 0x0F, 2, 0, callback);
        }
        void writeLimitTemperature(uint8_t id, uint32_t temperature) { notSupport(); }

        // Curent Limit
        uint32_t readLimitCurrent(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeLimitCurrent(uint8_t id, uint32_t current) { notSupport(); }

        // Drive Mode
        uint32_t readDriveMode(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
        void writeDriveMode(uint8_t id, uint32_t mode) { notSupport(); }

        // Burst Function
        void burstReadMemory(uint8_t* idList, uint8_t count, uint16_t address, uint8_t length, CallbackType callback) { notSupport(); }
        void burstWriteMemory(uint8_t* idList, uint32_t* dataList, uint8_t count, uint16_t address, uint8_t length)
        {
            setFunctionBurstWrite(0, address, 0, length, idList, dataList, count, 0, 0);
        }

        // Burst Function(Position)
        void burstReadPositions(uint8_t* idList, uint8_t count, CallbackType callback) { notSupport(); }
        void burstWriteTargetPositions(uint8_t* idList, gFloat* positionList, uint8_t count)
        {
            uint32_t* positionListInt = new uint32_t[count];
            for (uint8_t i = 0; i < count; i++)
            {
                checkId(idList[i]);

                int16_t position = positionList[i] * -10;

                if (position < -1500) position = -1500;
                else if (position > 1500) position = 1500;

                positionListInt[i] = (uint16_t)position;
            }

            setFunctionBurstWrite(0, Address::TargetPosition, 0, 3, idList, positionListInt, count, 0, 0);

            delete[] positionListInt;
        }
    };
}
