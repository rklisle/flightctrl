
#ifndef __APP_CAN_H__
#define __APP_CAN_H__
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "tx_api.h"
#include "stm32h7xx_hal.h"

extern int fdCan[2];

typedef void (*pcan_recv_cb_t)(uint32_t id, bool ext_id, const uint8_t* pdata, uint32_t datalen);


int32_t app_can_send(int32_t fd, uint32_t id, bool ext_id, const uint8_t* pdata, uint32_t datalen);


int32_t app_can_init(TX_BYTE_POOL *pmem, FDCAN_HandleTypeDef* phdl,pcan_recv_cb_t pcb );

#endif