/*
 * modOnceBattery.h
 *
 *  Created on: 2024Äê6ÔÂ16ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODONCEBATTERY_H_
#define SRC_MODULES_MODONCEBATTERY_H_


#include "../support/os_framework.h"
#include <stdbool.h>
extern OS_U8 OnceBatteryInit();
extern OS_U8 SaveOnceBattery();
extern OS_U8 MsgToOnceBattery();

extern OS_U8 battPowerCmd;
extern OS_U8 batt24VCmd;
extern OS_U8 OnceBatteryStatusUpdate();
extern void CanRtBattHandler(long unsigned int id, bool ext_id, const OS_U8* pdata, long unsigned int datalen);
#pragma pack(1)
typedef struct
{
	OS_U16 mcu28V;
	OS_U16 mcu28A;
	OS_U8 mcu24V;
	OS_U8 mcu24A;
	OS_S16 battTemp;
	OS_S16 mcuTemp;
}
OnceBattStru;
#pragma pack(0)
extern OnceBattStru onceBattStru;
#endif /* SRC_MODULES_MODONCEBATTERY_H_ */
