/*
 * interface_power.h
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */


#include "../support/os_types.h"

#ifndef INTERFACE_FLASH_H_
#define INTERFACE_FLASH_H_

unsigned char FlashInit();
unsigned char QspiFlashRead( unsigned int flashAddress, unsigned int memoryAddress, unsigned int byteLength);
unsigned char QspiFlashProgram(unsigned int memoryAddress, unsigned int flashAddress, unsigned int byteLength);
#endif /* INTERFACE_PWM_H_ */
