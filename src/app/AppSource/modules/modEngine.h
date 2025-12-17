/*
 * modEngineEle.h
 *
 *  Created on: 2024Äê4ÔÂ19ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODENGINEELE_H_
#define SRC_MODULES_MODENGINEELE_H_

#include "../support/os_framework.h"

extern OS_U8 EngineInit();
extern OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 EngineRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U16 ChkEngineFrame(OS_MEM* pmData);
extern OS_U8 AutoDriveEnginePwm();

extern OS_U32 CurEngineRpm;
extern OS_U32 EngineRpmCmd;
extern OS_U8 EngineStartStatus;

extern int SendEngineRpm();
extern int StartEngine();
extern int StopEngine();

#pragma pack(1)
typedef struct
{
	OS_U8 head;	//0xFF
    OS_U8 head1;// 0x0
    OS_U8 runningStatus;
    OS_U8 error;
    OS_U32 id;
    OS_U32 settingRpm;
    OS_U32 curRpm;
    OS_FLOAT temp;
    OS_U32 runningSecond;
    OS_FLOAT battV;
    OS_FLOAT battA;
    OS_FLOAT pumpV;
    OS_FLOAT Pa;
    OS_U16 pumpRpm;
    OS_U16 fuelConsum;
    OS_FLOAT ouputV;
    OS_U32 outputW;
}STRU_RUNNING_INFO;

typedef struct
{
	OS_U8 head;	
    OS_U8 head1;
    OS_U16 slowRpm;
    OS_U16 stopRpm;
    OS_U16 coldRpm;
    OS_U16 accRecord;
    OS_U16 deAccRecord;
    OS_FLOAT pumpMaxV;
    OS_FLOAT battLowV;
    OS_U16 maxTemp;
    OS_U16 pumpSpd;
    OS_U16 flutCliab;
    OS_U16 totalMinite;
    OS_FLOAT version;
    OS_U32 maxRpm;
    OS_U32 startCount;
}STRU_RUNNING_PARAM_INFO;

typedef struct
{
	OS_U8 head;
    OS_U8 pwmSlaver;
    OS_U8 pwmMaster;
    OS_U8 startDelay;
    OS_FLOAT pumpV;
    OS_U16 startFireRpm;
    OS_U16 hotEngineRpm;
    OS_U16 engineDetachRpm;
    OS_U16 dropPower;
    OS_FLOAT startSpeed;
    OS_FLOAT fireV;
    OS_U8 isAccHot;
    OS_U8 eleStableParam;
    OS_U8 fireSecond;
    
}STRU_START_PARAM_INFO;
#pragma pack()

#endif /* SRC_MODULES_MODENGINEELE_H_ */
