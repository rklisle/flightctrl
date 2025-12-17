/*
 * BusInteract.h
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */

#ifndef SRC_CORE_BUSINTERACT_H_
#define SRC_CORE_BUSINTERACT_H_

#include "DataPool.h"
#include "Telecontrol.h"

#define MODULE_COUNT	(3)

#define RT_FLYCTRL		(0)	//智能控制器
#define RT_GPS_1		(1)	//GPS
#define RT_GPS_2		(2)	//备用


typedef struct RT
{
	OS_U8 ckIndex;
	OS_U8 chIndex;
	OS_U8 devID;
	OS_U32	devBuad;
	OS_BOOL oddCheckEnable;
	OS_U32 (*ptr_RtHandler)(STRU_422_MSG_INFO * frame);
	OS_U16 (*ptr_ChkFrameSum)(OS_MEM* pmData);
	OS_U8 (*ptr_Init)();
}RT;

typedef struct RT_CAN
{
	OS_U32 (*ptr_RtHandler)(STRU_CAN_MSG * msg);
	OS_U8 (*ptr_Init)();
}RT_CAN;

extern RT rtList[MODULE_COUNT];
extern void InitRts();
extern void InitCanRts();
extern void BusDataHandle();
extern OS_U32 BCMsgHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 MsgToDevice(OS_U8 rtIndex, OS_U8 msgID, OS_U16 msgLen, OS_U8* buf);
extern OS_U8 PrintDebug(char *str);
#endif /* SRC_CORE_BUSINTERACT_H_ */
