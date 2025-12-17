#ifndef __SD_FLASH_H__
#define __SD_FLASH_H__
#include <stdint.h>
#include "tx_api.h"
#define FILE_COUNT	(6)

int32_t  sd_flash_init(TX_BYTE_POOL *pmem);

int32_t sd_block_erase(uint32_t block_addr, uint32_t erase_blk_size);

int32_t  sd_read(uint32_t block_addr, uint8_t *dest, uint32_t size);

int32_t  sd_write(uint32_t block_addr, const uint8_t *source, uint32_t size);

int fatFsFileOpen(char *name, int adcId);

int fatFsFileWrite(int adcId, char *dataBuf, int dataLen);

#endif