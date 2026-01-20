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
    DEVICE_MBAT = 0,   //主电*************
    DEVICE_SCOUT_E28V, //导引头***********1导引头
    DEVICE_FUSE28V,    //引信（可控电源）**2引信可控电源
    DEVICE_FUSE_ISO5V, //引信*************3引信5V信号
    DEVICE_SRV_PWR28V, //舵机电源*********4舵机
    DEVICE_NUM,        //外设数量
}POWER_DEVICE;

OS_S8 PowerOn(POWER_DEVICE dev);

OS_S8 PowerOff(POWER_DEVICE dev);

OS_S8 SeqOn(OS_U8 channel);

OS_S8 SeqOff(OS_U8 channel);

// extern OS_U8 InitPwrSeq();
#endif /* SRC_UCAS_SERVICE_INTERFACE_H_ */
