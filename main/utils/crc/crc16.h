#ifndef __CRC16_H__
#define __CRC16_H__

#include <stdint.h>
#include <stddef.h>

#define CRC16_POLY 0xAC9A
#define CRC16_INITIAL_VALUE 0xFFFF

uint16_t crc16_calculate(const void* data, size_t size);

#endif
