/*
 * PwrSeqCtl.h
 *
 *  Created on: 2021年10月16日
 *      Author: QL
 */

#ifndef SRC_PWRSEQCTL_H_
#define SRC_PWRSEQCTL_H_

#include "../support/os_framework.h"
#include <stdbool.h>

#define POWER_ON				(0xFBD1)
#define POWER_OFF				(0x0)

#define SEQ_COUNT				(5)
#define SEQ_GROUP_COUNT			(2)
#define SEQ_GROUP_ITEM_COUNT	(5)


typedef struct PWR_SEQ
{
	OS_U8 seqID;
	OS_U32 seqChannel;
	OS_U16 seqTick;
	OS_U8 openCount;
	OS_U8 closeCount;
}PWR_SEQ;

typedef struct PWR_SEQ_GROUP
{
	OS_U8 groupID;
	PWR_SEQ seqList[SEQ_GROUP_ITEM_COUNT];
	OS_U16 seqStartTick[SEQ_GROUP_ITEM_COUNT];
	OS_U16 groupTick;
}PWR_SEQ_GROUP;

enum seqGroupID
{
	AIR_SAC = 1,
    CHUTE_OPEN,
};

enum seqID
{
	CHUTE = 1,
	CYLINDER,
	AIR_SAC_1,
	AIR_SAC_2,
	TEST_1,
	TEST_2,
	TEST_3,
	TEST_4,
	TEST_5,
	TEST_6,
	TEST_7,
	TEST_8,
	TEST_9,
	TEST_10,
};

#pragma pack(1)
typedef struct CAN_RECV_VA
{
	OS_U16 battV; 
    OS_U16 groundV;
    OS_U16 engineV;
    OS_U16 mainV;
    OS_U16 fireV;	//MML 协议中是8位
    OS_U16 battA;	//MML 协议中是8位
    OS_U16 groundA;	//MML 协议中是8位
    OS_U16 engineA;	//MML 协议中是8位
    OS_U16 mainA;	//MML 协议中是8位
    OS_U8 fireA; 
    OS_U32 mcuTemp;	//MML 协议中是16位
}CAN_RECV_VA;
#pragma pack(0)

extern OS_U8 BookingMode;
extern OS_U8 PowerOffCount;
extern OS_U8 TrigerSeq(int seqIndex);
extern OS_U8 TrigerSeqWithWidth(int seqIndex, unsigned short width);
extern OS_U8 TrigerGroupSeq(int groupIndex);
extern OS_U8 SeqHandle();
extern OS_U8 GeneratePwrBuf(int channel, OS_U8* buf);
extern OS_U32 PwrCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 PwrRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 StartCrashMearuare();
extern void SavePwrSeq();

extern void CanRtPwrSeqHandler(long unsigned int id, bool ext_id, const OS_U8* pdata, long unsigned int datalen);
#endif /* SRC_PWRSEQCTL_H_ */
