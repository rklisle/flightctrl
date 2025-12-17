/*
 * interface_power.h
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */


#include "../support/os_types.h"

#ifndef INTERFACE_CAN_H_
#define INTERFACE_CAN_H_
extern int SendCanFrame(OS_U8 canIndex, OS_U16 id, OS_U8 length, OS_U8 * TxData);

#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
