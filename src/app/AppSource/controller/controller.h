/*
 * modBC.h
 *
 *  Created on: 2022年3月15日
 *      Author: Lenovo
 */

#ifndef SRC_MODULES_MODBC_H_
#define SRC_MODULES_MODBC_H_

#include "../support/os_framework.h"

//#define USE_NAV_BOARD
//#define USE_NAV_IMU
extern OS_BOOL use_nav_board;



extern OS_U8 ControllerStatusUpdata();
extern OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 Ignition();
extern OS_U8 DoIgnition();
extern OS_U16 BatteryActived;
extern OS_U8 IgnitionMark;
extern void ADC3_Init(void) ;
#define CAN1_NODE_COUNT	(6)
#define CAN2_NODE_COUNT	(1)
#pragma pack(1)
typedef struct
{
	OS_U8 engineState;	//0:不操作  1：起动  2：停止
	OS_U8 umOpen;//	开伞指令0:不操作， 1:开伞
	//OS_U8 wingTouched;
	OS_U8 steableFlight;
	OS_U8 luanched;	// 起飞tick>100,置1； 否则为0
	OS_U8 destory;//自毁 0:不操作 1:自毁
}FLIGHT_SEQ;
#pragma pack()

extern OS_DOUBLE curPress;
extern OS_DOUBLE curAirSpdPa;
// extern OS_DOUBLE curAirSpd;	// MML 未使用，注释
extern OS_U8 InitReportParam();
extern OS_U8 SeqCalc();
extern OS_U8 OnFireCmdSend();
extern OS_U8 AutoLuanchProcess();
extern FLIGHT_SEQ flightSeq;
#endif /* SRC_MODULES_MODBC_H_ */
