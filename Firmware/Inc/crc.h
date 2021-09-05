#ifndef CRC_H
#define CRC_H

#include "main.h"


#define CRC16_POLYNOM   0xA001


static uint8_t crc8_update(uint8_t crc, uint8_t data);
static uint16_t crc16_update(uint16_t crc, uint8_t data);


uint8_t crc8(uint8_t *buf, uint32_t size);
uint16_t crc16(uint8_t *buf, uint32_t size, uint16_t init_crc );


#endif
