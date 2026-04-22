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

#define MODULE_COUNT	(8)// 总共多少个串口设备
#define CAN_COUNT		(2)

#define RT_SCOUT    	(0)	//014 RT_SCOUT_CTL
// #define RT_SCOUT_CTL    	(0)	// 新增
#define RT_SCOUT_ATTITUDE  	(1)	// 新增
#define RT_DATA_LINK	(2)
#define RT_FUSE			(3)
// #define NOT_USED		(4)
#define PRINTF_UART_CHANNEL		(4)
#define RT_HIL          (5)
#define RT_ENGINE       (6)
#define RT_NAV  		(7)//	UART8
// #define RT_IMU		    (2)	//280用的
// #define RT_SRV          (4)
// #define RT_P900  		(8)//P900
// #define CAN_RT_BATT	 	(0)
#define CAN_RT_SRV	 	(0)//CAN控制的4个舵机
#define CAN_RT_POWERSEQ	(1)

typedef struct RT
{
	OS_U8 ckIndex;	// 可能是 飞机编号。280没用到
	OS_U8 chIndex;	// 串口号 = 此值 + 1
	OS_U8 devID;	// 可能是设备编号，原来外设会传过来设备号，需要解析，280没用到
	OS_U8 flags;	// flags for sepcial poll handler
	OS_BOOL oddCheckEnable;	// 奇校验
    OS_BOOL evenCheckEnable;// 偶校验
    OS_U8 devStopLen;		// 停止位
	OS_U32	devBuad;		// 外设波特率
	OS_U32 (*ptr_RtHandler)(STRU_422_MSG_INFO * frame);	// 中断回调
	OS_U16 (*ptr_ChkFrameSum)(OS_MEM* pmData);			// 校验和
	OS_U8 (*ptr_Init)();	// 初始化
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
