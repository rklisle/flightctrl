/*
 * fuse.h
 *
 *  Created on: 2025年4月19日
 *      Author: lenovo
 */

#ifndef SRC_PAYLOAD_FUSE_H_
#define SRC_PAYLOAD_FUSE_H_
#include "../support/os_framework.h"


#pragma pack(1)
// typedef struct
// {
// 	OS_U8 fuseMode;
// 	OS_U8 power24VMode;
//     OS_U8 activeStatus;
//     OS_U8 bitStatus;
//     OS_U16 chechsum1;
//     OS_U16 ms20tick;
//     OS_S16 az;
//     OS_S16 ay;
//     OS_S16 ax;
//     OS_U16 temp;
//     OS_U16 g;
//     OS_U16 power5V;
//     OS_U16 fuseuf;
//     OS_U8 keep;
//     OS_U16 chechsum;
// }FUSE_LONG_STATUS;
typedef enum
{
    ARM_I   = 0,   // EB 90 FC 00 55 00 00 00
    ARM_II  = 1,   // EB 90 FC 00 00 55 00 00
    ARM_III = 2,   // EB 90 FC 00 00 00 55 00
    DETO    = 3    // EB 90 FC 00 00 00 00 55
} FuzeCmdType;
typedef struct
{
	OS_U8  feedbk;    //反馈与控制信号
	OS_U8  task;     //任务执行状态
    OS_U16 firV;  // *0.0004     //发火电压 
    OS_U16 prxA;   // *0.00122  //近炸电流
    OS_U16 ic12V;  // *0.003        //集成电路电源状态
    OS_U16 detV;  // *0.0004     //起爆电路电源状态
    OS_U16 dcfA;   // *0.000065  //电磁阀电流
    OS_U16 c1Stat;   // *0.0023   //电容 1 充电状态
    OS_U16 unitNo;   //             //单元编号
    OS_U8  impSt;             //撞击 / 近炸状态
}FUSE_RCV_FRAME;    //17 Bytes

#pragma pack(0)
extern OS_U8 InitFuse();
extern OS_U32 FuseRtHandler(STRU_422_MSG_INFO *data);
extern OS_U16 ChkFuseStandardFrame(OS_MEM* pmData);
extern OS_U8 FuseSend(FuzeCmdType cmd);

#endif /* SRC_PAYLOAD_FUSE_H_ */
