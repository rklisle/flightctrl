/*
 * interface_flash.c
 *  Created on: 2022年8月7日
 *      Author: 成宏璟
 */
#include "interface_flash.h"
#include "../../nor_flash.h"
#include    "SPI_STM32H7xx.h"

unsigned char FlashInit()
{
    nor_flash_init( &Driver_SPI1 ,NULL);
    return 0;
}

unsigned char QspiFlashProgram(unsigned int memoryAddress, unsigned int flashAddress, unsigned int byteLength)
{
    nor_block_erase(flashAddress, byteLength);
    unsigned char *p;
    p = (unsigned char *)memoryAddress;
    HAL_Delay(20);
    int writeSuccessLen = nor_write(flashAddress, (const unsigned char*)p, byteLength);
	return 0;
}

unsigned char QspiFlashRead( unsigned int flashAddress, unsigned int memoryAddress, unsigned int byteLength)
{
    // uint8_t rd_mem[64] = {0};
    //nor_read(0x00, rd_mem, sizeof(rd_mem));
    unsigned char *p = (unsigned char *)memoryAddress;
    int readSuccessLen = nor_read(flashAddress, p, byteLength);
	return 0;
}