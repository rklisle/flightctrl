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

void EngineInit();
void StartEngine();
void StopEngine();
void SetEngineThrot(float percent);

/** 处理数据链传来的命令，也就是地面的遥控指令
 * 控制转速指令：存数据池、控发动机转速
 * 获取启动参数：发送相应命令
 * 获取运行参数：
 */
OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame);//数据链 ——> 飞控
// extern OS_U32 EngineRtHandler(STRU_422_MSG_INFO * frame);
void EngineHandler();//解析数据：发动机 ——> 飞控
// extern OS_U16 ChkEngineFrame(OS_MEM* pmData);
OS_U8 AutoDriveEnginePwm();//每100ms进来控制一次engine油门

extern OS_U32 CurEngineRpm;    //014 油门百分比*10 取值[0~1000]；  280 转速
// extern OS_U32 EngineRpmCmd;
// extern OS_U8 EngineStartStatus;

// extern int SendEngineRpm();

#pragma pack(1)
typedef struct
{
	OS_U8 head;	            // 0xFF
    OS_U8 head1;            // 0x00
    OS_U8 runningStatus;    // 运行状态         // CurState
    OS_U8 error;            // 错误码           //  
    OS_U32 fuel_pressure;   // 实际油压         // fuel_pressure 
    OS_U32 jet1_duty;       // 实际喷油1脉宽    // jet1_duty *         **014 cmd64**
    OS_U32 curRpm;          // 实际转速         // rpm      **014 cmd69**
    OS_FLOAT ambient_temp;  // 温度             // ambient_temp      **014 cmd6 **
    OS_U32 jet2_duty;       // 实际喷油2脉宽    // jet2_duty *   **014 cmd119 **
    OS_FLOAT battV;         // 电池电压         // battV *014 cmd96 **
    OS_FLOAT battA;         // 电池电流         // battA *014 cmd91 **
    OS_FLOAT actual_throttle;// 风门开度97       // actual_throttle 
    OS_FLOAT ch1_temp;       // 通道1实际温度    // ch1_temp
    OS_U16 ch2_temp;         // 通道2实际温度    // ch2_temp
    OS_U16 ch3_temp;         // 通道3实际温度    // ch3_temp
    OS_FLOAT ch4_temp;       // 通道4实际温度    // ch4_temp
    OS_U32 outputW;          // 发电功率         //  
}STRU_RUNNING_INFO; // HACK: TEST ECU    param30 0x30

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
    OS_U16 maxTemp;         // **014 cmd59 **
    OS_U16 pumpSpd;
    OS_U16 flutCliab;
    OS_U16 totalMinite;     // **014 cmd117 **
    OS_FLOAT version;       // **014 cmd100 **
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
