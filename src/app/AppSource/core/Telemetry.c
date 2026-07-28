/*
 * Telemetry.c
 *
 *  Created on: 2022??1??21??
 *      Author: ChengHongjing
 */
#include "TelemetryDef.h"
#include "BusInteract.h"
#include "../modules/modSD.h"
#include "../modules/modHil.h"
#include "../flightNew/port/flightPort.h"
#include <string.h>

#define TELE_PARAM_GROUPID_POS		(0)
#define TELE_PARAM_CURTIME_POS		(1)
#define TELE_PARAM_FIRSTDATA_POS	(5)

/* Match tm_flight paramCode (max 8 chars, may be not NUL-terminated). */
static OS_BOOL TmCodeEq(const char *code, const char *name)
{
	return (strncmp(code, name, PARAM_CODE_MAXLEN) == 0);
}

/*
 * Pack g_CtrltoDL_tel fields directly into 0x82 buffer.
 * SETDATA(pDataPoolFly, ...) often fails to land in the pool before TelemetryCombine;
 * tarLon/tarLat already used this path — extend to all FlightTM control fields.
 * Scales/types must match FlightTMOutputHandle() + tm_flight[] byteCount.
 */
static OS_BOOL TelemetryPackFlightDirectParam(const char *code, OS_U8 byteCount, unsigned char *p)
{
	if(code == NULL || p == NULL)
		return FALSE;

	/* ---- 1 byte ---- */
	if(byteCount == 1)
	{
		if(TmCodeEq(code, "curPtNo"))
		{
			OS_U8 v = (OS_U8)g_CtrltoDL_tel.curPtNo;
			memcpy(p, &v, 1);
			return TRUE;
		}
		return FALSE;
	}

	/* ---- 2 bytes ---- */
	if(byteCount == 2)
	{
		OS_S16 s16;
		OS_U16 u16;

		if(TmCodeEq(code, "tarAlt"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.curTargetAlt; memcpy(p, &s16, 2); return TRUE; }

		/* HIL DD4 序号原样回传；占用控制包 2 字节预留 res1，不改变包长。 */
		if(TmCodeEq(code, "res1"))
		{ u16 = g_hilEchoSequence; memcpy(p, &u16, 2); return TRUE; }

		if(TmCodeEq(code, "rollCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.rudderRollCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "pitchCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.rudderPitchCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "yawCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.rudderYawCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "gamaCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.gamaCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "varthCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.varthetaCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "nycCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.nycCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "highCmd"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.heightCmd; memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "vyCmd"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.vyCmd; memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "ac_dZ"))
		{ u16 = (OS_U16)(g_CtrltoDL_tel.ac_dZ * 100.0); memcpy(p, &u16, 2); return TRUE; }

		if(TmCodeEq(code, "thrusCmd"))
		{ u16 = (OS_U16)(g_CtrltoDL_tel.thrustCmd * 10.0); memcpy(p, &u16, 2); return TRUE; }
		if(TmCodeEq(code, "rpmState"))
		{ u16 = (OS_U16)g_CtrltoDL_tel.rpmState; memcpy(p, &u16, 2); return TRUE; }

		if(TmCodeEq(code, "ac_Vx"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.ac_Vx * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "ac_Vy"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.ac_Vy * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "ac_Vz"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.ac_Vz * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "ac_dR"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.ac_dR * 10.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "azimuth"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.cur_azimuth * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "thetav"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.cur_thetav * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "psicv"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.cur_psicv * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "Vcmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.Vcmd * 10.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "nyCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.nyCmd_Guidance * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "nzCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.nzCmd_Guidance * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "wyCmd"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.wyCmd * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "pitch_nT"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.pitch_rate_nT_filterOut * 1000.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "yaw_nT"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.yaw_rate_nT_filterOut * 1000.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "dRn"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.dRn; memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "dRu"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.dRu; memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "dRe"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.dRe; memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "PitchPre"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.Pitch_Preset_Angle * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "YawPre"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.Yaw_Preset_Angle * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "gamacCom"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.gamac_compensate * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "uz_gamac"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.uz_gamac * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "mx_ESO"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.mx_ESO * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "ADRC"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.fduox_ADRC * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "alphaIns"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.alpha_ins * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "betaIns"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.beta_ins * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "nyflt"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.nyflt * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "nzflt"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.nzflt * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "mass_cal"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.mass_calc * 10.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "ugfZetac"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.ugf_zetac * 100.0); memcpy(p, &s16, 2); return TRUE; }
		if(TmCodeEq(code, "uqkf"))
		{ s16 = (OS_S16)(g_CtrltoDL_tel.uqkf * 100.0); memcpy(p, &s16, 2); return TRUE; }

		if(TmCodeEq(code, "curAlt"))
		{ s16 = (OS_S16)g_CtrltoDL_tel.curAlt; memcpy(p, &s16, 2); return TRUE; }

		return FALSE;
	}

	/* ---- 4 bytes ---- */
	if(byteCount == 4)
	{
		OS_S32 s32;
		OS_U32 u32;
		OS_FLOAT f32;

		if(TmCodeEq(code, "tarLon"))
		{ s32 = (OS_S32)(g_CtrltoDL_tel.curTargetLon * 1e7); memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "tarLat"))
		{ s32 = (OS_S32)(g_CtrltoDL_tel.curTargetLat * 1e7); memcpy(p, &s32, 4); return TRUE; }

		if(TmCodeEq(code, "fcstate"))
		{ u32 = (OS_U32)g_CtrltoDL_tel.flight_control_state; memcpy(p, &u32, 4); return TRUE; }

		/* 与 FlightTMOutputHandle：ac_dL*100 → U32；Excel 系数 0.01 */
		if(TmCodeEq(code, "ac_dL"))
		{ u32 = (OS_U32)(g_CtrltoDL_tel.ac_dL * 100.0); memcpy(p, &u32, 4); return TRUE; }

		if(TmCodeEq(code, "deltaR"))
		{ u32 = (OS_U32)(g_CtrltoDL_tel.deltaR * 100.0); memcpy(p, &u32, 4); return TRUE; }

		/* 飞控帧计数：5ms 节拍 CurrTick */
		if(TmCodeEq(code, "fliCount"))
		{ u32 = (OS_U32)g_DeviceState.CurrTick; memcpy(p, &u32, 4); return TRUE; }

		if(TmCodeEq(code, "DusState"))
		{ s32 = (OS_S32)g_CtrltoDL_tel.Dubins_stage; memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "DusType1"))
		{ s32 = (OS_S32)g_CtrltoDL_tel.dubins_type1; memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "DusType2"))
		{ s32 = (OS_S32)g_CtrltoDL_tel.dubins_type2; memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "DusType3"))
		{ s32 = (OS_S32)g_CtrltoDL_tel.dubins_type3; memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "DbsLen"))
		{ f32 = (OS_FLOAT)g_CtrltoDL_tel.Dubins_length; memcpy(p, &f32, 4); return TRUE; }

		if(TmCodeEq(code, "Qv"))
		{ u32 = (OS_U32)(g_CtrltoDL_tel.Qv * 100.0); memcpy(p, &u32, 4); return TRUE; }
		if(TmCodeEq(code, "cnt_alti"))
		{ u32 = (OS_U32)g_CtrltoDL_tel.count_altitude_change; memcpy(p, &u32, 4); return TRUE; }

		if(TmCodeEq(code, "curLon"))
		{ s32 = (OS_S32)(g_CtrltoDL_tel.curLon * 1e7); memcpy(p, &s32, 4); return TRUE; }
		if(TmCodeEq(code, "curLat"))
		{ s32 = (OS_S32)(g_CtrltoDL_tel.curLat * 1e7); memcpy(p, &s32, 4); return TRUE; }

		return FALSE;
	}

	return FALSE;
}


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
	OS_BOOL isFlightTm = (groupIndex >= TELEMETRY_GROUP_COUNT);
	for(int i=0;i<te->telemetryParaCount;i++)
	{
		DataPoolKey key;
		DataPoolValue value;
		DataPoolType type;
		OS_U8 byteCount = te->telemetryParamList[i].byteCount;
		p_DataPool pDataPool = te->telemetryParamList[i].pPool;
		if(pDataPool == NULL)
			break;;

		if(isFlightTm && TelemetryPackFlightDirectParam(te->telemetryParamList[i].paramCode, byteCount, p))
		{
			p += byteCount;
			*len += byteCount;
			continue;
		}

        memcpy(&key, te->telemetryParamList[i].paramCode, PARAM_CODE_MAXLEN);
       
		if(pDataPool->ptr_GetData(pDataPool, key, &value, &type) != 0)
		{
			memset(&value, 0, sizeof(value));
			type = byteCount;
		}
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
    
	if((g_DeviceState.CurrTick) % 80 == 0)//40
	{
		MsgToDevice(RT_DATA_LINK, TM_OTHER, msgLen, buf);//
       // MsgToDevice(RT_P900, TM_OTHER, msgLen, buf);//
        
	}
	else if((g_DeviceState.CurrTick + 20) % 80 == 0)//40
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

