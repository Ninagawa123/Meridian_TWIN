#pragma once

namespace crc16
{
	unsigned short calculate(unsigned char* data, unsigned short length)
	{
		unsigned short crc = 0;
		for (unsigned short i = 0; i < length; i++) {
			crc ^= (data[i] << 8);
			for (unsigned char j = 0; j < 8; j++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x8005;
				}
				else {
					crc <<= 1;
				}
			}
		}

		return crc;
	}
}
