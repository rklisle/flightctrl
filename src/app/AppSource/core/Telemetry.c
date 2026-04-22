/*
 * Telemetry.c
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */
#include "TelemetryDef.h"
#include "BusInteract.h"
#include "../modules/modSD.h"

#define TELE_PARAM_GROUPID_POS		(0)
#define TELE_PARAM_CURTIME_POS		(1)
#define TELE_PARAM_FIRSTDATA_POS	(5)


void PushDefInDataPool()
{
	for(int i=0;i<TELEMETRY_GROUP_COUNT;i++)
	{
		for(int j=0;j<tmGroups[i].telemetryParaCount;j++)
		{
			INITDATA(tmGroups[i].telemetryParamList[j].pPool,
					tmGroups[i].telemetryParamList[j].paramCode,
					tmGroups[i].telemetryParamList[j].byteCount);
		}
	}
	for(int j=0;j<tmFlight.telemetryParaCount;j++)
	{
		INITDATA(tmFlight.telemetryParamList[j].pPool,
				tmFlight.telemetryParamList[j].paramCode,
				tmFlight.telemetryParamList[j].byteCount);
	}
}

/***********************************************************
 * 函数名称:InitTelemetry()
 * 函数功能:按照任务需要，定义帧号、下传周期、参数个数
 * 作者:	成宏璟
 ***********************************************************/
void InitTelemetry()
{
	SETDATA(pDataPoolSelf, "nullBtye", 0,	OS_U8);
    //控制电量遥测参数
	tmFlight.telemetryTickInterval = 1;
	tmFlight.telemetryGroupID = 0;
	tmFlight.telemetryParamList = (telemetryParam*)&tm_flight;
	tmFlight.telemetryParaCount = sizeof(tm_flight)/sizeof(telemetryParam);;
	tmFlight.telemetryPktBytes = TM_PKT_BYTE_FLIGHT;

	int groupIndex = 0;
	//数据链遥测遥测，5ms下传一次
	tmGroups[groupIndex].telemetryTickInterval = 1;
	tmGroups[groupIndex].telemetryGroupID = 0;
	tmGroups[groupIndex].telemetryParamList = (telemetryParam*)&tm_zh;
	tmGroups[groupIndex].telemetryParaCount = sizeof(tm_zh)/sizeof(telemetryParam);
	tmGroups[groupIndex].telemetryPktBytes = TM_PKT_BYTE_PAYLOAD;
	groupIndex++;
	//电气电量遥测参数
	tmGroups[groupIndex].telemetryTickInterval = 1;
	tmGroups[groupIndex].telemetryGroupID = 0;
	tmGroups[groupIndex].telemetryParamList = (telemetryParam*)&tm_200hz;
	tmGroups[groupIndex].telemetryParaCount = sizeof(tm_200hz)/sizeof(telemetryParam);
	tmGroups[groupIndex].telemetryPktBytes = TM_PKT_BYTE_200HZ;
	groupIndex++;

	int _40hzCount = 0;//sizeof(tm_40hz_pack)/sizeof(tm_40hz_pack[0]);

	//每25ms需要下传的遥测，分包下传
	for(int pktIndex=0;pktIndex<TEMEMETRY_GROUP_COUNT_40HZ;pktIndex++)
	{
		tmGroups[groupIndex].telemetryTickInterval = 5;
		tmGroups[groupIndex].telemetryGroupID = pktIndex;
		tmGroups[groupIndex].telemetryParaCount = 0;
		tmGroups[groupIndex].telemetryPktBytes = TM_PKT_BYTE_40HZ;
		if(pktIndex < _40hzCount)
		{
			tmGroups[groupIndex].telemetryParamList = (telemetryParam *)&tm_40hz_pack[pktIndex];
			for(int i=0;i<TM_PKT_PARAM_COUNT_40hz_10hz;i++)
			{
				if(tm_40hz_pack[pktIndex][i].pPool != NULL)
					tmGroups[groupIndex].telemetryParaCount++;
			}
		}
		groupIndex++;
	}

	//每100ms需要下传的遥测，分包下传
	int _10hzCount = 0;//sizeof(tm_10hz_pack)/sizeof(tm_10hz_pack[0]);;//
	for(int pktIndex=0;pktIndex<TEMEMETRY_GROUP_COUNT_10HZ;pktIndex++)
	{
		tmGroups[groupIndex].telemetryTickInterval = 20;
		tmGroups[groupIndex].telemetryGroupID = pktIndex;
		tmGroups[groupIndex].telemetryParaCount = 0;
		tmGroups[groupIndex].telemetryPktBytes = TM_PKT_BYTE_10HZ;
		if(pktIndex < _10hzCount)
		{
			tmGroups[groupIndex].telemetryParamList = (telemetryParam*)&tm_10hz_pack[pktIndex];
			for(int i=0;i<TM_PKT_PARAM_COUNT_40hz_10hz;i++)
			{
				if(tm_10hz_pack[pktIndex][i].pPool != NULL)
					tmGroups[groupIndex].telemetryParaCount++;
			}
		}
		groupIndex++;
	}

	PushDefInDataPool();
}

/***********************************************************
 * 函数名称:TelemetryCombine()
 * 输入参数:OS_U8 : groupIndex 本次需要组织的遥测帧编号，类似vcid
 * 		 uchar * : buf 传入的存储遥测数据的数据区。调用者需要保证buf足够大。
 * 输出参数:OS_U16* : len 本帧组成后的总长度，返回后用于计算多帧时的下一帧偏移。
 *
 * 函数功能:根据输入的帧号，从数据池查找帧内所需参数的数值，并按要求长度组帧
 * 作者:	成宏璟
 ***********************************************************/
static void TelemetryCombine(OS_U8 groupIndex, unsigned char *buf, OS_U16 *len)
{
	telemetry *te;
	if( groupIndex >= TELEMETRY_GROUP_COUNT)
	{
		te = &tmFlight;
	}
	else
	{
		te = &tmGroups[groupIndex];
	}
	memset(buf, 0, te->telemetryPktBytes);
	unsigned char *p = buf;
	*len = 0;
	for(int i=0;i<te->telemetryParaCount;i++)
	{
		DataPoolKey key;
		DataPoolValue value;
		DataPoolType type;
		p_DataPool pDataPool = te->telemetryParamList[i].pPool;
		if(pDataPool == NULL)
			break;;
		
        memcpy(&key, te->telemetryParamList[i].paramCode, PARAM_CODE_MAXLEN);
       
		pDataPool->ptr_GetData(pDataPool, key, &value, &type);
					memcpy(p, &value, type);
					p+= type;
					*len += type;
	}
	*len = te->telemetryPktBytes;
	return;
}

/***********************************************************
 * 函数名称:GetTelemetryByTick()
 * 输入参数:OS_U32 : tick 时钟计数器，每次调用本函数递增1，每5ms调用本函数一次
 * 		 uchar * : buf 传入的存储遥测数据的数据区。调用者需要保证buf足够大。
 * 返回值  : OS_U16 : 本次帧的长度
 * 函数功能:根据输入的计数器，查找预定义的遥测下传列表，选取在当前时刻需要下传的遥测
 * 		   帧，将所有需要下传的帧合并为一帧并返回。
 * 		   关于groupID的计算，不同帧的groupID是按比特位错开的，如果时间片需要同时
 * 		   下传多帧时，groupId通过或关系，标记为多帧合并
 * 作者:	成宏璟
 ***********************************************************/
static OS_U16 GetTelemetryByTick(OS_U32 tick, OS_U8* buf)
{
	/*******************************
	 * 遥测帧格式定义见文件:
	 * 《TXII-Y1电量遥测参数数据处理要求》
	 * ************************/
//	OS_U8 groupId = tick % 20;
	OS_U16 frameLen = 0;
	unsigned char *p = buf + TELE_PARAM_FIRSTDATA_POS;	//数据区第7字节开始为第一个参数
	for(int i=0;i<TELEMETRY_GROUP_COUNT;i++)
	{
		if(tick % tmGroups[i].telemetryTickInterval == tmGroups[i].telemetryGroupID)
		{
			OS_U16 len;
			TelemetryCombine(i, p, &len);
			p = p + len;
			frameLen += len;
		}
	}
	//数据区第0字节为group编号
	*(OS_U8*)(buf + TELE_PARAM_GROUPID_POS) = 0x81;//groupId + 0x81;
	//数据区第1~4字节为时间
	*(OS_U32*)(buf + TELE_PARAM_CURTIME_POS) = g_DeviceState.currTime * 1e4;
	return frameLen + TELE_PARAM_FIRSTDATA_POS;
}
/***********************************************************
 * 函数名称:GetFlightTelemetryByTick()
 * 输入参数:OS_U32 : tick 时钟计数器，每次调用本函数递增1，每5ms调用本函数一次
 * 		 uchar * : buf 传入的存储遥测数据的数据区。调用者需要保证buf足够大。
 * 返回值  : OS_U16 : 本次帧的长度
 * 函数功能: 将飞控专用遥测数据组成字节序，准备发往中心程序器
 * 作者:	成宏璟
 ***********************************************************/
static OS_U16 GetFlightTelemetryByTick(OS_U32 tick, OS_U8* buf)
{
	//OS_U8 groupId = tick % 20;
	unsigned char *p = buf + TELE_PARAM_FIRSTDATA_POS;	//数据区第5字节开始为第一个参数
	OS_U16 len;
	TelemetryCombine(TELEMETRY_GROUP_COUNT, p, &len);

	//数据区第0字节为group编号
	*(OS_U8*)(buf + TELE_PARAM_GROUPID_POS) = 0x82;
	//数据区第1~4字节为时间
// BUG：此处用g_DeviceState.currTime是否合适，因为这个值起飞时会清0，如果是系统时间，应该用curTime（全局变量）
	*(OS_U32*)(buf + TELE_PARAM_CURTIME_POS) = g_DeviceState.currTime * 1e4;
	return len + TELE_PARAM_FIRSTDATA_POS;
}
/***********************************************************
 * 函数名称:TelemetryFrameOut()
 * 输入参数:OS_U32 : tick 时钟计数器，本函数由状态机驱动每5ms调用一次，tick+1
 * 函数功能:获取遥测数据区后，添加遥测头，发往中心程序器。据本次修改意见，发往中心程序器的程序
 * 分为飞控相关及其它，分别走中心程序器的XDI（飞控）,XCC（其它）端口。
 * 作者:	成宏璟
 ***********************************************************/
extern OS_BOOL FlashProgramming;
void TelemetryFrameOut()	// 遥测数据发送给数据链 5ms运行一次
{
	OS_U32 tick = g_DeviceState.CurrTick;

	OS_U8 buf[500] = {0};
	OS_U8 bufflight[500] = {0};
	OS_U16 msgLen = GetTelemetryByTick(tick, buf);//电气电量参数
	OS_U16 flightLen = GetFlightTelemetryByTick(tick, bufflight);//控制电量遥测参数参数

    if(FlashProgramming == TRUE)
    {
        return;
    }
    
	if((g_DeviceState.CurrTick) % 40 == 0)
	{
		MsgToDevice(RT_DATA_LINK, TM_OTHER, msgLen, buf);//
       // MsgToDevice(RT_P900, TM_OTHER, msgLen, buf);//
        
	}
	else if((g_DeviceState.CurrTick + 20) % 40 == 0)
	{
		MsgToDevice(RT_DATA_LINK, TM_FLIGHT, flightLen, bufflight);//
        //MsgToDevice(RT_P900, TM_FLIGHT, flightLen, bufflight);//
	}
    
 	MsgToDevice(RT_HIL, TM_FLIGHT, flightLen, bufflight);//仅把控制数据发送至半实物仿真
    
	OS_U8 sdBuf[500];
	memcpy(sdBuf, bufflight, flightLen);
	memcpy(sdBuf + flightLen, buf + 5, msgLen);
    WriteToSD(0, sdBuf, (OS_U32)(flightLen + msgLen - 5));//所有数据合并写入SD卡
	return;

}

