#ifndef _NOR_FLASH_H_
#define _NOR_FLASH_H_
#include <stdint.h>
#include "tx_api.h"
#include "Driver_SPI.h"

int32_t nor_read(uint32_t flash_address, uint8_t *dest, uint32_t size);
int32_t nor_write(uint32_t flash_address, const uint8_t *source, uint32_t size);
int32_t nor_write_deepseek(uint32_t flash_address, const uint8_t *source, uint32_t size);
int32_t nor_block_erase(uint32_t flash_address, uint32_t erase_size);
int32_t nor_block_erase_deepseek(uint32_t flash_address, uint32_t erase_size);
int32_t nor_flash_init(ARM_DRIVER_SPI* pdrv, TX_BYTE_POOL *pmem);
#endif