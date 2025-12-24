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

#define MODULE_COUNT	(9)
#define CAN_COUNT		(2)

#define RT_SCOUT    	(0)	//
#define RT_FUSE			(1)	//
#define RT_IMU		    (2)	//
#define RT_DATA_LINK	(3)	//
// #define RT_SRV          (4)
#define RT_ENGINE       (5)
#define RT_HIL          (6)

#define RT_NAV  		(7)//组合导航
#define RT_P900  		(8)//P900

// #define CAN_RT_BATT	 	(0)
#define CAN_RT_SRV	 	(0)//CAN控制的4个舵机
#define CAN_RT_POWERSEQ	(1)

typedef struct RT
{
	OS_U8 ckIndex;
	OS_U8 chIndex;
	OS_U8 devID;
	OS_U32	devBuad;
	OS_BOOL oddCheckEnable;
    OS_BOOL evenCheckEnable;
    OS_U8 devStopLen;
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
