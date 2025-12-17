/*
 * modBC.h
 *
 *  Created on: 2022年3月15日
 *      Author: Lenovo
 */

#ifndef SRC_MODULES_MODBC_H_
#define SRC_MODULES_MODBC_H_

#include "../support/os_framework.h"
extern OS_U8 SelfStatusUpdata();
extern OS_U8 SelfFrameOut();

#pragma pack(1)
typedef struct
{
	/***************
	 * 入遥测部分
	 * *************/
	FUNC_DOMAIN workStage;
		//各单机通信状态，初始设置为200tick，每个tick调用-1，每次收到数据恢复200.保证在通讯中断1秒内能够反馈到遥测
	OS_U8 gpsCountDown;
}DeviceState;
#pragma pack()

extern OS_U8 SendNavQiuStart;
extern OS_U8 checkState;
extern OS_U8 navState;
extern OS_U16 ReloadAppInfoFromFlash();
extern OS_U8 StartAirSpdCalibration();
extern OS_U8 DoAirSpdCalibration();
extern void ADC3_Init(void);
#endif /* SRC_MODULES_MODBC_H_ */
