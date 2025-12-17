#ifndef __UART_AGENT_H__
#define __UART_AGENT_H__
#include <stdint.h>

int32_t fcs_uart_recv(int32_t fd, uint8_t* pdata, uint32_t len);

int32_t fcs_uart_send(int32_t fd, const uint8_t* pdata, uint32_t len);

int32_t fcs_uart_init(TX_BYTE_POOL* pool, ARM_DRIVER_USART* pdrv, uint32_t baudrate, uint32_t parity, uint32_t stopbits);

extern void ReConnectUart();

#endif
