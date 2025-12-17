/*
 * interface_uart.h
 *
 *  Created on: 2022年8月4日
 *      Author: 成宏璟
 */

#include    "UART_STM32H7xx.h"
#include "../support/os_types.h"
#include <string.h>

#ifndef INTERFACE_UART_H_

#define INTERFACE_UART_H_

#define RECV_422_MAX_LEN (1025)


extern int UART_Setting(int channel, unsigned int Baud, char Par, int stopLen);
int UART_PutBuff(int channel, unsigned char * TxBuff, unsigned short Length);
int UART_GetFrame(int channel, unsigned char * RxBuff, unsigned short MaxLength);

#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
