/*
 * interface_power.h
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */


#include "../support/os_types.h"

#ifndef INTERFACE_CAN_H_
#define INTERFACE_CAN_H_

/* 标准帧发送（11位ID） */
int SendCanFrame(OS_U8 canIndex, OS_U16 id, OS_U8 length, OS_U8 * TxData);

/* 扩展帧发送（29位ID） */
int SendCanFrameExt(OS_U8 canIndex, OS_U32 id, OS_U8 length, OS_U8 * TxData);

#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
