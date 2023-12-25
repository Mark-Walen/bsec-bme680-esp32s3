#include "crc16.h"

static inline uint16_t crc16_update(uint8_t byte, uint16_t crc) {
	int i;
	int xor_flag;

	/* For each bit in the data byte, starting from the leftmost bit */
	for (i = 7; i >= 0; i--) {
		/* If leftmost bit of the CRC is 1, we will XOR with
		 * the polynomial later */
		xor_flag = crc & 0x8000;

		/* Shift the CRC, and append the next bit of the
		 * message to the rightmost side of the CRC */
		crc <<= 1;
		crc |= (byte & (1 << i)) ? 1 : 0;

		/* Perform the XOR with the polynomial */
		if (xor_flag)
			crc ^= CRC16_POLY;
	}

	return crc;
}

static inline uint16_t crc16_finalize(uint16_t crc) {
	int i;

	/* Augment 16 zero-bits */
	for (i = 0; i < 2; i++) {
		crc = crc16_update(0, crc);
	}

	return crc;
}

uint16_t crc16_calculate(const void* data, size_t size)
{
    uint8_t *buffer = (uint8_t *) data;
    uint16_t crc = CRC16_INITIAL_VALUE;

    for (size_t i = 0; i < size; i++)
    {
        crc = crc16_update(buffer[i], crc);
    }
    crc = crc16_finalize(crc);
    return crc;
}
