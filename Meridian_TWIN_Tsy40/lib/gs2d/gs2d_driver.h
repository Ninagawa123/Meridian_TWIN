/*
* @file    gs2d_driver.h
* @author
* @date    2021/01/26
* @brief
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <inttypes.h>
#include <string.h>
#include "gs2d_type.h"

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	class Driver
	{
	protected:
		// エラーコード保存用（例外発生はしない）
		uint8_t errorBits = 0;
		void notSupport(void) { errorBits |= NotSupportError; }
		void badInput(void) {
			errorBits |= BadInputError;
		}

		// 同期/非同期モードのフラグ
		bool operatingMode = false;

	public:

		Driver() {}
		virtual ~Driver() {}

		// System
		// Multi Thread 
		void changeOperatingMode(bool threading) { operatingMode = threading; }
		virtual void spin(void) = 0;

		// Error
		uint8_t getErrorCode(void) { return errorBits; }
		void clearErrorCode(void) { errorBits = 0; }

		// ------------------------------------------------------------------------------------------
		// General
		virtual uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, CallbackType callback) = 0;
		virtual void writeMemory(uint8_t id, uint16_t address, uint32_t data, uint8_t length) = 0;

		// Ping
		virtual uint16_t ping(uint8_t id, CallbackType callback = 0) = 0;

		// Torque
		virtual uint8_t readTorqueEnable(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeTorqueEnable(uint8_t id, uint8_t torque) = 0;

		// Temperature
		virtual uint16_t readTemperature(uint8_t id, CallbackType callback = 0) = 0;

		// Current
		virtual int32_t readCurrent(uint8_t id, CallbackType callback = 0) = 0;

		// Voltage
		virtual gFloat readVoltage(uint8_t id, CallbackType callback = 0) = 0;

		// Target Position
		virtual gFloat readTargetPosition(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeTargetPosition(uint8_t id, gFloat position) = 0;

		// Current Position
		virtual gFloat readCurrentPosition(uint8_t id, CallbackType callback = 0) = 0;

		// Offset
		virtual gFloat readOffset(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeOffset(uint8_t id, gFloat offset) = 0;

		// Deadband
		virtual gFloat readDeadband(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeDeadband(uint8_t id, gFloat deadband) = 0;

		// Target Time
		virtual gFloat readTargetTime(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeTargetTime(uint8_t id, gFloat targetTime) = 0;

		// Accel Time
		virtual gFloat readAccelTime(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeAccelTime(uint8_t id, gFloat accelTime) = 0;

		// P Gain
		virtual uint32_t readPGain(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writePGain(uint8_t id, uint32_t gain) = 0;

		// I Gain
		virtual uint32_t readIGain(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeIGain(uint8_t id, uint32_t gain) = 0;

		// D Gain
		virtual uint32_t readDGain(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeDGain(uint8_t id, uint32_t gain) = 0;

		// Max Torque
		virtual uint32_t readMaxTorque(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeMaxTorque(uint8_t id, uint32_t maxTorque) = 0;

		// Speed
		virtual gFloat readSpeed(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeSpeed(uint8_t id, gFloat speed) = 0;

		// ID
		virtual uint32_t readID(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeID(uint8_t id, uint32_t newid) = 0;

		// ROM
		virtual void saveRom(uint8_t id) = 0;
		virtual void loadRom(uint8_t id) = 0;
		virtual void resetMemory(uint8_t id) = 0;

		// Baudrate
		virtual uint32_t readBaudrate(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeBaudrate(uint8_t id, uint32_t baudrate) = 0;

		// CW Limit Position
		virtual gFloat readLimitCWPosition(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeLimitCWPosition(uint8_t id, gFloat limitPosition) = 0;

		// CCW Limit Position
		virtual gFloat readLimitCCWPosition(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeLimitCCWPosition(uint8_t id, gFloat limitPosition) = 0;

		// Temperature Limit
		virtual uint32_t readLimitTemperature(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeLimitTemperature(uint8_t id, uint32_t temperature) = 0;

		// Curent Limit
		virtual uint32_t readLimitCurrent(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeLimitCurrent(uint8_t id, uint32_t current) = 0;

		// Drive Mode
		virtual uint32_t readDriveMode(uint8_t id, CallbackType callback = 0) = 0;
		virtual void writeDriveMode(uint8_t id, uint32_t mode) = 0;

		// Burst Function
		virtual void burstReadMemory(uint8_t* idList, uint8_t count, uint16_t address, uint8_t length, CallbackType callback) = 0;
		virtual void burstWriteMemory(uint8_t* idList, uint32_t* dataList, uint8_t count, uint16_t address, uint8_t length) = 0;

		// Burst Function(Position)
		virtual void burstReadPositions(uint8_t* idList, uint8_t count, CallbackType callback) = 0;
		virtual void burstWriteTargetPositions(uint8_t* idList, gFloat* positionList, uint8_t count) = 0;
	};
}
