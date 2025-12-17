/*
 * interface_power.h
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */


#include "../support/os_types.h"

#ifndef INTERFACE_POWER_H_
#define INTERFACE_POWER_H_

typedef enum
{
	DEVICE_MAIN_BATT = 1,
	DEVICE_IMU_28V,
	DEVICE_FUSE_1_E28V,
	DEVICE_FUSE_2_ISO28V,
    DEVICE_FUSE_5V,
    DEVICE_BATT_ENGINE,
    DEVICE_BATT_SRV,
    DEVICE_BATT_BATT2,
}POWER_DEVICE;

extern OS_BOOL powerState[8];

extern OS_S8 PowerOn(POWER_DEVICE dev);

extern OS_S8 PowerOff(POWER_DEVICE dev);

extern OS_S8 SeqOn(OS_U8 channel);

extern OS_S8 SeqOff(OS_U8 channel);

extern OS_U8 InitPwrSeq();
#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
