/*
 * PwrSeqCtl.c
 *
 *  Created on: 2021��10��16��
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
 * ��������: TrigerSeq
 * ��������: ���ݸ�����ʱ���ţ��趨��ǰʱ����״̬���ж�״̬������
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
 * ��������: TrigerGroupSeq
 * ��������: ���ݸ�������ʱ���ţ��趨��ǰʱ���飬��״̬���ж�״̬������
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
 * ��������: SeqHandle
 * ��������: ������ʱ���źż��ر��źŷ��ͣ���ʱ��������趨��ǰʱ��Ϊ��
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
 * ��������:SeqGroupHandle()
 * ��������:��������룬����������ʱ����Ϣ
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
 * ��������:Ϊ�������ָ�����ɷ�������豸��������
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
 * �������� :PwrCmdHandler()
 * ��������:���ܿ������ڽ��յ��������ָ��ʱ����Ϊ�����control������ָ��
 * 		    ��Ϊ��ʱ����������������ָ����ֱ�ӷ���ʱ���������
 * 		    �����ܿ�����ת����ʱ��������Ĺ���Ϊ������繦��
 * ************************************************/
OS_U8 powerNeedRsp[3] = {0};
extern OS_U16 AutoZeroCount;
OS_U32 PwrCmdHandler(STRU_422_MSG_INFO * frame)
{

	OS_U8 msgId = frame->u8MsgID;
	switch(msgId)
	{
		case CMD_POWER_REQ://�������
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
		case CMD_SEQ_POWER_REQ://ʱ�����
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
    SETDATA(pDataPoolPwr, "Batt28V", canRecvVa.battV,	OS_U16);
    SETDATA(pDataPoolPwr, "groundV", canRecvVa.groundV,	OS_U16);
    //SETDATA(pDataPoolPwr, "engineV", canRecvVa.engineV,	OS_U16);
	SETDATA(pDataPoolPwr, "engineV", canRecvVa.engineV,	OS_U32);
    SETDATA(pDataPoolPwr, "VCombin", canRecvVa.mainV,	OS_U16);
    SETDATA(pDataPoolPwr, "Batt28A", canRecvVa.battA,	OS_U16);
    SETDATA(pDataPoolPwr, "groundA", canRecvVa.groundA,	OS_U16);
    //SETDATA(pDataPoolPwr, "engineA", canRecvVa.engineA,	OS_U16);
	SETDATA(pDataPoolPwr, "engineA", canRecvVa.engineA,	OS_U32);
    SETDATA(pDataPoolPwr, "ACombin", canRecvVa.mainA,	OS_U16);
    SETDATA(pDataPoolPwr, "pwrTemp", canRecvVa.mcuTemp,	OS_U16);
}

OS_U8 CanRtPwrSeqHandler(OS_U32 id, OS_BOOL ext_id, const OS_U8* pdata, OS_U8 datalen)
{
    if(id == 0x183)//V	// Э�����
    {
		//e.g. unsigned long long data = 0x0001D63AAF164000ULL;(ʵ��CANץ����������С������������ڴ棬����һ���ֽ���)
		//�õ�
		// pdata[0] = 0x00  // Byte1
		// pdata[1] = 0x40  // Byte2
		// pdata[2] = 0x16  // Byte3
		// pdata[3] = 0xAF  // Byte4
		// pdata[4] = 0x3A  // Byte5
		// pdata[5] = 0xD6  // Byte6
		// pdata[6] = 0x01  // Byte7
		// pdata[7] = 0x00  // Byte8
		canRecvVa.battV = (pdata[0] << 4) | (pdata[1] >> 4);      //0.01	// �����������У������õ�0x004
		canRecvVa.groundV = ((pdata[1] & 0x0F) << 8) | pdata[2];  //0.01	// �����������У������õ�0x016
		canRecvVa.engineV = (pdata[3] << 4) | (pdata[4] >> 4);    //0.01	// �����������У������õ�0xAF3
		canRecvVa.mainV = ((pdata[4] & 0x0F) << 8) | pdata[5];    //0.01	// �����������У������õ�0xAD6
    }
    else if(id == 0x184)//A	// Э�����
    {
        canRecvVa.battA = pdata[0];  	//0.1
        canRecvVa.groundA = pdata[1];	//0.1
        canRecvVa.engineA = pdata[2];	//0.1
        canRecvVa.mainA = pdata[3];		//0.1
		canRecvVa.mcuTemp = (pdata[7] << 8) | pdata[6]; //0.1
    }
	else{
		return -1;
	}
    g_DeviceState.powerCountDown = 200;	// �������200*5ms��û�����ô�ֵ��˵��CAN2��Ҳ��������һֱû���ϱ�����
    return 0;
}


