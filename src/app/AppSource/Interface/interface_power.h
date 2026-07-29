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
    DEVICE_MBAT = 0,   // mbat
    DEVICE_SCOUT_E28V, // scout
    DEVICE_FUSE28V,    // fuse deto
    DEVICE_FUSE_ISO5V, // fuse
    DEVICE_SRV_PWR28V, // srv
    DEVICE_NUM,        //
}POWER_DEVICE;

OS_S8 PowerOn(POWER_DEVICE dev);

OS_S8 PowerOff(POWER_DEVICE dev);

OS_S8 SeqOn(OS_U8 channel);

OS_S8 SeqOff(OS_U8 channel);

// extern OS_U8 InitPwrSeq();
#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
