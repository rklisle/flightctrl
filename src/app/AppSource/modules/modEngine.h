/*
 * modEngineEle.h
 *
 *  Created on: 2024年4月19日
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODENGINEELE_H_
#define SRC_MODULES_MODENGINEELE_H_

#include "../support/os_framework.h"
#include "modECUqueue.h"

extern OS_U8 EngineInit();

extern OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 EngineRtHandler(STRU_422_MSG_INFO * frame);
void EngineHandler();
extern OS_U16 ChkEngineFrame(OS_MEM* pmData);
extern OS_U8 AutoDriveEnginePwm();

extern OS_U32 CurEngineRpm;
extern OS_U32 EngineRpmCmd;
extern OS_U8 EngineStartStatus;

extern int SendEngineRpm();
extern int StartEngine();
extern int StopEngine();
void SetEngineThrot(float percent);

#pragma pack(1)
typedef struct
{
	OS_U8 head;	            // 0xFF
    OS_U8 head1;            // 0x00
    OS_U8 runningStatus;    // 运行状态     **280程序有用**         **014 状态机的状态**
    OS_U8 error;            // 错误码       **280程序没用**
    OS_U32 id;              // 设备ID       **280程序没用**
    OS_U32 settingRpm;      // 设定转速     **280程序没用**         **014 cmd64**
    OS_U32 curRpm;          // 当前转速     **280程序有用**         **014 cmd69**
    OS_FLOAT temp;          // 温度         **280程序有用**         **014 cmd6 **
    OS_U32 runningSecond;   // 运行秒数     **280程序没用**
    OS_FLOAT battV;         // 电池电压     **280程序没用**
    OS_FLOAT battA;         // 电池电流     **280程序没用**
    OS_FLOAT pumpV;         // 油泵电压     **280程序没用**
    OS_FLOAT Pa;            // 压力         **280程序没用**         **014 cmd8 **
    OS_U16 pumpRpm;         // 油泵转速     **280程序没用**
    OS_U16 fuelConsum;      // 油量消耗     **280程序有用**         **014 //TODO 协议里没有，给0**
    OS_FLOAT ouputV;        // 发电电压     **280程序没用**
    OS_U32 outputW;         // 发电功率     **280程序没用**
}STRU_RUNNING_INFO; // 0x30

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
}STRU_RUNNING_PARAM_INFO;   // 0x32

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
    
}STRU_START_PARAM_INFO; // 0x31
#pragma pack()

#endif /* SRC_MODULES_MODENGINEELE_H_ */
