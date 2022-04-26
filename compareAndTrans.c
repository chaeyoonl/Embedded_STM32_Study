/*
 * rb.c
 *
 * Created: 2018-04-14 ���� 3:12:06
 *  Author: dauera
 */

#include "calculateFunction.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief  ring buffer init.
 * @param  ptRB
 size
 * @retval unsigned int 0 ����
 */

bool compare(unsigned char asc, int compareTempData) {

	unsigned char compareData[99] = { (unsigned char) 0xa5,
			(unsigned char) 0x01, (unsigned char) 0x00, (unsigned char) 0x59,
			(unsigned char) 0x7e };

	if (asc == compareData[compareTempData]) {
		return true;
	}else {
		return false;
	}

}

const char * returnTransmitData(int select) {

	unsigned char compareData[600] = {
				(unsigned char) 0xa5, (unsigned char) 0x02, (unsigned char) 0x80,
				(unsigned char) 0x01, (unsigned char) 0xd7, (unsigned char) 0x7e,

				(unsigned char) 0xa5,
				(unsigned char) 0x19, (unsigned char) 0xf2, (unsigned char) 0x56,
				(unsigned char) 0x65, (unsigned char) 0x72, (unsigned char) 0x31,
				(unsigned char) 0x2e, (unsigned char) 0x30, (unsigned char) 0x30,
				(unsigned char) 0x2c, (unsigned char) 0x32, (unsigned char) 0x30,
				(unsigned char) 0x32, (unsigned char) 0x32, (unsigned char) 0x2d,
				(unsigned char) 0x30, (unsigned char) 0x34, (unsigned char) 0x2d,
				(unsigned char) 0x31, (unsigned char) 0x33, (unsigned char) 0x20,
				(unsigned char) 0x31, (unsigned char) 0x30, (unsigned char) 0x3a,
				(unsigned char) 0x30, (unsigned char) 0x30, (unsigned char) 0x34,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x07, (unsigned char) 0xd3,
				(unsigned char) 0x00, (unsigned char) 0x08, (unsigned char) 0x40,
				(unsigned char) 0x8e, (unsigned char) 0x0b, (unsigned char) 0x00,
				(unsigned char) 0x9f, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x04, (unsigned char) 0xd0,
				(unsigned char) 0x0c, (unsigned char) 0x00, (unsigned char) 0x00,
				(unsigned char) 0x7a, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x05, (unsigned char) 0xdc,
				(unsigned char) 0x07, (unsigned char) 0xe3, (unsigned char) 0x01,
				(unsigned char) 0x01, (unsigned char) 0x8d, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x05, (unsigned char) 0xd9,
				(unsigned char) 0x5a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x01, (unsigned char) 0x0d, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x09, (unsigned char) 0xd4,
				(unsigned char) 0x00, (unsigned char) 0x1a, (unsigned char) 0x00,
				(unsigned char) 0x80, (unsigned char) 0x22, (unsigned char) 0x2e,
				(unsigned char) 0x01, (unsigned char) 0x00, (unsigned char) 0x92,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x02, (unsigned char) 0xd8,
				(unsigned char) 0x02, (unsigned char) 0x7e, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x18, (unsigned char) 0xd1,
				(unsigned char) 0x00, (unsigned char) 0x00, (unsigned char) 0x08,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x08,
				(unsigned char) 0x14, (unsigned char) 0x03, (unsigned char) 0x0c,
				(unsigned char) 0x03, (unsigned char) 0x01, (unsigned char) 0x0c,
				(unsigned char) 0x02, (unsigned char) 0x0d, (unsigned char) 0x0c,
				(unsigned char) 0x04, (unsigned char) 0x0b, (unsigned char) 0x0e,
				(unsigned char) 0x08, (unsigned char) 0x08, (unsigned char) 0x01,
				(unsigned char) 0x03, (unsigned char) 0x00, (unsigned char) 0xce,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x18, (unsigned char) 0xd1,
				(unsigned char) 0x00, (unsigned char) 0x00, (unsigned char) 0x08,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x08,
				(unsigned char) 0x14, (unsigned char) 0x03, (unsigned char) 0x0c,
				(unsigned char) 0x03, (unsigned char) 0x01, (unsigned char) 0x0c,
				(unsigned char) 0x02, (unsigned char) 0x0d, (unsigned char) 0x0c,
				(unsigned char) 0x04, (unsigned char) 0x0b, (unsigned char) 0x0e,
				(unsigned char) 0x08, (unsigned char) 0x08, (unsigned char) 0x01,
				(unsigned char) 0x03, (unsigned char) 0x00, (unsigned char) 0xce,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x03, (unsigned char) 0xc7,
				(unsigned char) 0x00, (unsigned char) 0x00, (unsigned char) 0x90,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0d, (unsigned char) 0xec,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0xe1, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x02, (unsigned char) 0x8a,
				(unsigned char) 0x00, (unsigned char) 0xce, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0d, (unsigned char) 0xec,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0x20, (unsigned char) 0x20, (unsigned char) 0x20,
				(unsigned char) 0xe1, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x19, (unsigned char) 0xf2,
				(unsigned char) 0x56, (unsigned char) 0x65, (unsigned char) 0x72,
				(unsigned char) 0x31, (unsigned char) 0x2e, (unsigned char) 0x30,
				(unsigned char) 0x30, (unsigned char) 0x2c, (unsigned char) 0x32,
				(unsigned char) 0x30, (unsigned char) 0x32, (unsigned char) 0x32,
				(unsigned char) 0x2d, (unsigned char) 0x30, (unsigned char) 0x34,
				(unsigned char) 0x2d, (unsigned char) 0x31, (unsigned char) 0x33,
				(unsigned char) 0x20, (unsigned char) 0x31, (unsigned char) 0x30,
				(unsigned char) 0x3a, (unsigned char) 0x30, (unsigned char) 0x30,
				(unsigned char) 0x34, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x04, (unsigned char) 0xd0,
				(unsigned char) 0x0c, (unsigned char) 0x00, (unsigned char) 0x00,
				(unsigned char) 0x7a, (unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x06, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x21,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x07, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x20,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x08, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x1f,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x09, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x1e,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0a, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x0d,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0b, (unsigned char) 0x14, (unsigned char) 0x00,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x00, (unsigned char) 0x1b,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0c, (unsigned char) 0x0c, (unsigned char) 0x00,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x00,
				(unsigned char) 0x00, (unsigned char) 0x00, (unsigned char) 0x32,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x1a,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0e, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x19,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x0f, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x18,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x10, (unsigned char) 0x0c, (unsigned char) 0x00,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x00, (unsigned char) 0x1e,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x11, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x16,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x12, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x15,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x13, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x14,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x14, (unsigned char) 0x0c, (unsigned char) 0x00,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x00,
				(unsigned char) 0x00, (unsigned char) 0x00, (unsigned char) 0x2a,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x15, (unsigned char) 0x0c, (unsigned char) 0x00,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x0a,
				(unsigned char) 0x0a, (unsigned char) 0x0a, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x00, (unsigned char) 0x19,
				(unsigned char) 0x7e,

				(unsigned char) 0xa5, (unsigned char) 0x0c, (unsigned char) 0xd2,
				(unsigned char) 0x16, (unsigned char) 0x0c, (unsigned char) 0x02,
				(unsigned char) 0x0d, (unsigned char) 0x0c, (unsigned char) 0x04,
				(unsigned char) 0x0b, (unsigned char) 0x0e, (unsigned char) 0x08,
				(unsigned char) 0x08, (unsigned char) 0x01, (unsigned char) 0x11,
				(unsigned char) 0x7e







	};

	return compareData[select];



}



const char * returnTransmitData_Test() {
	return "A5 19 F2 56 65 72 31 2E 30 30 2C 32 30 32 32 2D 30 34 2D 31 33 20 31 30 3A 30 30 34 7E A5 07 D3 00 08 40 8E 0B 00 9F 7E A5 04 D0 0C 00 00 7A 7E A5 05 DC 07 E3 01 01 8D 7E A5 05 D9 5A 0A 0A 01 0D 7E A5 09 D4 00 1A 00 80 22 2E 01 00 92 7E A5 02 D8 02 7E 7E A5 18 D1 00 00 08 0A 0A 08 14 03 0C 03 01 0C 02 0D 0C 04 0B 0E 08 08 01 03 00 CE 7E A5 0C D2 00 0C 02 0D 0C 04 0B 0E 08 08 01 27 7E A5 03 C7 00 00 90 7E A5 0D EC 20 20 20 20 20 20 20 20 20 20 20 20 E1 7E A5 02 8A 00 CE 7E A5 0D EC 20 20 20 20 20 20 20 20 20 20 20 20 E1 7E A5 19 F2 56 65 72 31 2E 30 30 2C 32 30 32 32 2D 30 34 2D 31 33 20 31 30 3A 30 30 34 7E A5 04 D0 0C 00 00 7A 7E A5 0C D2 06 0C 02 0D 0C 04 0B 0E 08 08 01 21 7E A5 0C D2 07 0C 02 0D 0C 04 0B 0E 08 08 01 20 7E A5 0C D2 08 0C 02 0D 0C 04 0B 0E 08 08 01 1F 7E A5 0C D2 09 0C 02 0D 0C 04 0B 0E 08 08 01 1E 7E A5 0C D2 0A 0C 02 0D 0C 04 0B 0E 08 08 01 1D 7E A5 0C D2 0B 14 00 0A 0A 0A 0A 0A 08 08 00 1B 7E A5 0C D2 0C 0C 00 0A 0A 0A 0A 0A 00 00 00 32 7E A5 0C D2 0D 0C 02 0D 0C 04 0B 0E 08 08 01 1A 7E A5 0C D2 0E 0C 02 0D 0C 04 0B 0E 08 08 01 19 7E A5 0C D2 0F 0C 02 0D 0C 04 0B 0E 08 08 01 18 7E A5 0C D2 10 0C 00 0A 0A 0A 0A 0A 08 08 00 1E 7E A5 0C D2 11 0C 02 0D 0C 04 0B 0E 08 08 01 16 7E A5 0C D2 12 0C 02 0D 0C 04 0B 0E 08 08 01 15 7E A5 0C D2 13 0C 02 0D 0C 04 0B 0E 08 08 01 14 7E A5 0C D2 14 0C 00 0A 0A 0A 0A 0A 00 00 00 2A 7E A5 0C D2 15 0C 00 0A 0A 0A 0A 0A 08 08 00 19 7E A5 0C D2 16 0C 02 0D 0C 04 0B 0E 08 08 01 11 7E";

}
