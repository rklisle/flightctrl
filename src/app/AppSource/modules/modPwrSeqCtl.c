/*
 * PwrSeqCtl.c
 *
 *  Created on: 2021年10月16日
 *      Author: QL
 */

#include "modPwrSeqCtl.h"
#include "modSrvCtl.h"
#include "../core/BusInteract.h"
#include "../interface/interface_power.h"
#include "../interface/interface_gpio.h"

//#include "../flight/os_flight_data.h"
OS_U8 PowerOffCount = 0;
static int PWRTimSeq(int channelNo, int isEnalbe);

#define PWRCH(x) (0x1 << (x - 1))

PWR_SEQ pwrSeqs[SEQ_COUNT + 1]={
	//  ID 			 		channel     				tick	retryCount
		{TEST_1, 			PWRCH(1),					40,			3,3},
		{TEST_2,			PWRCH(2),					40,			3,3},
		{TEST_3, 			PWRCH(3),					40,			3,3},
		{TEST_4, 			PWRCH(4),					40,			3,3},
		{TEST_5, 			PWRCH(5),					40,			3,3},
		{0}
};

PWR_SEQ_GROUP seqGroups[] = {
	//groupID		seqID												seqStartTick		groupTick
	{AIR_SAC,
			{
					{AIR_SAC_1,	    PWRCH(3),			20,		2,3},
					{AIR_SAC_2,    	PWRCH(4),			20,	    2,3},
					{0},
			},
			{0, 24}, 0},
    {CHUTE_OPEN,
			{
					{CHUTE,	        PWRCH(1),			20,		2,3},
					{CYLINDER,    	PWRCH(2),			20,	    2,3},
					{0},
			},
			{0, 24}, 0},
	{0}
};
/*********************************************
 * 函数功能: TrigerSeq
 * 函数功能: 根据给定的时序编号，设定当前时序，由状态机判断状态并发送
 * *******************************************/
#define MAX_SIMULTANEOUS_SEQ_COUNT	(5)
static PWR_SEQ curSeqList[MAX_SIMULTANEOUS_SEQ_COUNT] = {0};
static PWR_SEQ_GROUP curGroupSeq = {0};

OS_U8 TrigerSeq(int seqIndex)
{
	for(int i=0;i<SEQ_COUNT;i++)
	{
		if(pwrSeqs[i].seqID == seqIndex)
		{
			int curIndex = 0;
			while(curSeqList[curIndex].seqID != 0)
			{
				curIndex++;
				if(curIndex>=MAX_SIMULTANEOUS_SEQ_COUNT)
					return -1;
			}
			memcpy(&curSeqList[curIndex], &pwrSeqs[i], sizeof(PWR_SEQ));
			break;
		}
	}
	return 0;
}

OS_U8 TrigerSeqWithWidth(int seqIndex, unsigned short width)
{
	for(int i=0;i<SEQ_COUNT;i++)
	{
		if(pwrSeqs[i].seqID == seqIndex)
		{
			pwrSeqs[i].seqTick = width / 5;
		}
	}

	for(int i=0;i<SEQ_COUNT;i++)
	{
		if(pwrSeqs[i].seqID == seqIndex)
		{
			int curIndex = 0;
			while(curSeqList[curIndex].seqID != 0)
			{
				curIndex++;
				if(curIndex>=MAX_SIMULTANEOUS_SEQ_COUNT)
					return -1;
			}
			memcpy(&curSeqList[curIndex], &pwrSeqs[i], sizeof(PWR_SEQ));
			break;
		}
	}
	return 0;
}

static int PWRTimSeq(int channelNo, int isEnalbe)
{
	for(int i=0; i<SEQ_COUNT; i++)
	{
		if((channelNo & (1<<i)) != 0)
		{
			if(isEnalbe == TRUE)
			{
				SeqOn(i);
			}
			if(isEnalbe == FALSE)
			{
				SeqOff(i);
			}
		}
	}

	//SETDATA(pDataPoolSelf, "BJms", channelNo, OS_U16);//
	return 0;
}


/*********************************************
 * 函数功能: TrigerGroupSeq
 * 函数功能: 根据给定的组时序编号，设定当前时序组，由状态机判断状态并发送
 * *******************************************/
OS_U8 TrigerGroupSeq(int groupIndex)
{
	if((g_DeviceState.workStage &DOM_TRIGGERON) != DOM_TRIGGERON)
			return -1;
	for(int i=0;i<SEQ_GROUP_COUNT;i++)
	{
		if(seqGroups[i].groupID == groupIndex)
		{
			if(curGroupSeq.groupID != 0)
				return -1;
			memcpy(&curGroupSeq, &seqGroups[i], sizeof(PWR_SEQ_GROUP));
			break;
		}
	}
	return 0;
}

/*********************************************
 * 函数功能: SeqHandle
 * 函数功能: 负责发送时序信号及关闭信号发送，当时序结束后，设定当前时序为空
 * *******************************************/
OS_U8 SeqGroupHandle();
OS_U8 SeqHandle()
{
	if(curGroupSeq.groupID != 0)
	{
		SeqGroupHandle();
		return 0;
	}
	OS_U32 openChannel = 0;
	OS_U32 closeChannel = 0;
	for(int i=0;i<MAX_SIMULTANEOUS_SEQ_COUNT;i++)
	{
		if(curSeqList[i].seqID != 0)
		{
			if(curSeqList[i].seqTick > 0 && curSeqList[i].openCount > 0)
			{
				openChannel |= curSeqList[i].seqChannel;
				curSeqList[i].openCount--;
			}
			if(curSeqList[i].seqTick == 0 && curSeqList[i].closeCount > 0)
			{
				closeChannel |= curSeqList[i].seqChannel;
				curSeqList[i].closeCount--;
			}
			if(curSeqList[i].closeCount == 0)
			{
				memset(&curSeqList[i], 0, sizeof(PWR_SEQ));
				return 0;
			}
			if(curSeqList[i].seqTick>0)
				curSeqList[i].seqTick--;
		}
	}
	if(openChannel)
	{
		PWRTimSeq(openChannel, TRUE);
	}
	if(closeChannel)
	{
		PWRTimSeq(closeChannel, FALSE);
	}

	return 0;
}
/*********************************************
 * 函数名称:SeqGroupHandle()
 * 函数功能:根据组号码，发送连续的时序信息
 * *******************************************/
OS_U8 SeqGroupHandle()
{
	OS_U32 closeChannel = 0;
	for(int i=0;i<SEQ_GROUP_ITEM_COUNT;i++)
	{
		PWR_SEQ* seq = &(curGroupSeq.seqList[i]);
		if(seq->seqID == 0)
			continue;
		OS_U16 start = curGroupSeq.seqStartTick[i];
		if(start<=curGroupSeq.groupTick && seq->seqTick > 0 && seq->openCount > 0)
		{
			PWRTimSeq(seq->seqChannel, TRUE);
			seq->openCount--;
		}
		if(start<=curGroupSeq.groupTick && seq->seqTick == 0 && seq->closeCount > 0)
		{
			closeChannel |= seq->seqChannel;
			seq->closeCount--;
		}

		if(seq->closeCount == 0)
		{
			memset(seq, 0, sizeof(PWR_SEQ));
			continue;
		}
		if(start <= curGroupSeq.groupTick && seq->seqTick > 0)
		{
			seq->seqTick--;
		}
	}
	if(closeChannel)
	{
		PWRTimSeq(closeChannel, FALSE);
	}

	OS_U8 groupEndTag = 0;
	for(int i=0;i<SEQ_GROUP_ITEM_COUNT;i++)
	{
		if(curGroupSeq.seqList[i].seqID > 0)
			groupEndTag++;
	}
	if(groupEndTag == 0)
	{
		memset(&curGroupSeq, 0, sizeof(PWR_SEQ_GROUP));
		return 0;
	}
	curGroupSeq.groupTick++;
	return 0;
}

/*********************************************
 *
 * 函数功能:为单机配电指令生成发往配电设备的数据区
 * *******************************************/
OS_U8 GeneratePwrBuf(int channel, OS_U8* buf)
{
	BOOL open = FALSE;
	if(channel < 0x10)
	{
		open = TRUE;
	}
	else
	{
		open = FALSE;
		channel = channel >> 4;
	}
	OS_U32 pos;
	pos = 0b1111;
	pos = pos<< ((channel - 1) * 4);
	memcpy(buf, &pos, 4);
	OS_U16 isOpen = open? POWER_ON : POWER_OFF;
	memcpy(buf + 4 + (channel - 1)*2, &isOpen, 2);

	return 0;
}

/*************************************************
 * 函数名称 :PwrCmdHandler()
 * 函数功能:智能控制器在接收到无线配电指令时，认为是针对control版的配电指令
 * 		    因为给时序配电器的无线配电指令已直接发往时序配电器。
 * 		    由智能控制器转发至时序配电器的功能为单机配电功能
 * ************************************************/
OS_U8 powerNeedRsp[3] = {0};
extern OS_U16 AutoZeroCount;
OS_U32 PwrCmdHandler(STRU_422_MSG_INFO * frame)
{

	OS_U8 msgId = frame->u8MsgID;
	switch(msgId)
	{
		case CMD_POWER_REQ://单机配电
		{
			OS_U8 pwrByte = frame->au8Data[0];
			if(pwrByte > 0x0F)
			{
				PowerOff(((pwrByte >> 4) & 0xF));
			}
			else
			{
				PowerOn((pwrByte & 0xF));
			}
		}
		break;
		case CMD_SEQ_POWER_REQ://时序测试
		{
			OS_U8 testChannel = frame->au8Data[0];
			OS_U16 width;
			memcpy(&width, frame->au8Data + 1, 2);
			if(testChannel != 31)
				TrigerSeqWithWidth(TEST_1 + testChannel, width);
            
           // TrigerGroupSeq(AIR_SAC);
		}
		break;
	}
	return 0;
}
//#define POWER_TEST

CAN_RECV_VA canRecvVa;

void SavePwrSeq()
{
    SETDATA(pDataPoolPwr, "battV", canRecvVa.battV,	OS_U16);
    SETDATA(pDataPoolPwr, "groundV", canRecvVa.groundV,	OS_U16);
    SETDATA(pDataPoolPwr, "engineV", canRecvVa.engineV,	OS_U16);
    SETDATA(pDataPoolPwr, "VCombin", canRecvVa.mainV,	OS_U16);
    SETDATA(pDataPoolPwr, "VFire", canRecvVa.fireV,	OS_U16);
    SETDATA(pDataPoolPwr, "battA", canRecvVa.battA,	OS_U16);
    SETDATA(pDataPoolPwr, "groundA", canRecvVa.groundA,	OS_U16);
    SETDATA(pDataPoolPwr, "engineA", canRecvVa.engineA,	OS_U16);
    SETDATA(pDataPoolPwr, "ACombin", canRecvVa.mainA,	OS_U16);
    SETDATA(pDataPoolPwr, "Afire", canRecvVa.fireA,	OS_U16);
    SETDATA(pDataPoolPwr, "pwrTemp", canRecvVa.mcuTemp,	OS_U16);
}

void CanRtPwrSeqHandler(long unsigned int id, bool ext_id, const OS_U8* pdata, long unsigned int datalen)
{
    unsigned long long data;
    memcpy(&data, pdata, 8);
    OS_U16 mcuTempLow = 0, mcuTempHigh = 0;
    if(id == 0x183)//V
    {
        canRecvVa.battV = (OS_U16)((data >> 0 ) & 0xFFF);  //0.1
        canRecvVa.groundV = (OS_U16)((data >> 12 ) & 0xFFF);//0.1
        canRecvVa.engineV = (OS_U16)((data >> 24 ) & 0xFFF);//0.1
        canRecvVa.mainV = (OS_U16)((data >> 36 ) & 0xFFF);//0.1
        canRecvVa.fireV = (OS_U16)((data >> 48 ) & 0xFFF);//0.1
        mcuTempLow = ((data >> 60 ) & 0xF); //1
        
    }
    else if(id == 0x184)//A
    {
        canRecvVa.battA = (OS_U16)((data >> 0 ) & 0xFFF);  //0.1
        canRecvVa.groundA = (OS_U16)((data >> 12 ) & 0xFFF);//0.1
        canRecvVa.engineA = (OS_U16)((data >> 24 ) & 0xFFF);//0.1
        canRecvVa.mainA = (OS_U16)((data >> 36 ) & 0xFFF);//0.1
        canRecvVa.fireA = (OS_U8)((data >> 48 ) & 0xFF);//0.1
        mcuTempHigh = ((data >> 56 ) & 0xFF); //1
       
    }
    //SavePwrSeq();
    canRecvVa.mcuTemp = ((mcuTempHigh << 4) & 0xFF0) + (mcuTempLow & 0xF);
    g_DeviceState.powerCountDown = 200;
    
}


