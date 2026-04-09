/*
 * scout.c
 *
 *  Created on: 2023年5月19日
 *      Author: lenovo
 */
#include "fuse.h"
#include "../support/os_basic.h"
#include "../interface/interface_uart.h"
#include "MsnTime.h"
#include "../FlightSupport.h"
#include <math.h>
#include "../core/BusInteract.h"
#include "../support/common.h"
#include "../support/os_bufferLoop.h"

FUSE_LONG_STATUS fuseStatus = {0};
OS_U8 FuseSend(OS_U8 msgID);
OS_U8 InitFuse()
{
	buffLoop[RT_FUSE].syncHead_A = 0xAA;
	buffLoop[RT_FUSE].syncHead_B = 0x55;
	buffLoop[RT_FUSE].lenExtern = 0;	// 数据帧中表示长度字节，之外还有lenExtern个字节，总共构成一帧数据	//普通422消息，头部6字节，校验和2字节不算入长度字段
	buffLoop[RT_FUSE].head = 0;
	buffLoop[RT_FUSE].tail = 0;
	buffLoop[RT_FUSE].lenPos = 2;	//0;	//除了Head_A和Head_B之外再偏移多少字节才到长度字节	//同步头后第n个字节为长度
	buffLoop[RT_FUSE].fixedLen = 15;	//0：取HeadA、HeadB后面的两个字节作为长度；-1：暂不清楚；具体数值：固定长度
	buffLoop[RT_FUSE].inited = TRUE;
	return 0;
}

OS_U16 ChkFuseStandardFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	// if(pmData[1] != 0xAA || pmData[2] != 0x55)
	if(pmData[1] != buffLoop[RT_FUSE].syncHead_A || pmData[2] != buffLoop[RT_FUSE].syncHead_B)
	{
		return 0;
	}
    
	// pmData[15] * 0x100 + pmData[14]
    OS_U16 checkSumRecv = pmData[buffLoop[RT_FUSE].fixedLen] * 0x100 + pmData[buffLoop[RT_FUSE].fixedLen - 1];	//（小端序） 下标30这个字节左移8位 + 下标29的字节
    OS_U16 checkSumCalc = CheckSum16_8(pmData + 1, buffLoop[RT_FUSE].fixedLen - 2);
    
   if(checkSumRecv != checkSumCalc)
	{//校验和不通过
		static int errorCount32 = 0;
        errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "sctChkEr", errorCount,OS_U8);
		return 0;
	}

	/** 重新组帧 */
	pmData[3] = pmData[5];
	pmData[4] = 0;	// pmData[3];
	pmData[5] = 0;	// pmData[4];
	pmData[6] = 0x11;
	pmData[7] = pmData[9];
	pmData[9] = pmData[12];
	pmData[11] = pmData[10];
	pmData[12] = pmData[11];
	pmData[10] = pmData[13];
	pmData[13] = pmData[14];
	pmData[14] = pmData[15];
	return pmData[3] - 1 + 5;

	// /** 原来280程序 */
    // memmove(pmData + 7, pmData + 6, pmData[3]);	// BUG 此处是否会指针踩了别的区域？
    // memcpy(data,pmData, 100);
	// pmData[4] = 0;
	// pmData[5] = 0;
	// pmData[6] = 0x11;
	// return pmData[3] + 5;
}

OS_U32 FuseRtHandler(STRU_422_MSG_INFO *data)
{
    if(data->u16Len != 0x0F)
    {
        return -1;
    }
    // if(data->u16Len == 0x0B)
    // {
    //     //change to 30byte frame
    //     FuseSend(0x55);
    // }
	//SCOUT_STATUS scoStatus;
	memcpy(&fuseStatus, data->au8Data, sizeof(FUSE_LONG_STATUS));

    float ax = fuseStatus.ax * 0.00122 / 0.18 / 0.7 * 9.8;
    float ay = fuseStatus.ay * 0.00122 / 0.18 / 0.7 * 9.8;
    float az = fuseStatus.az * 0.00122 / 0.18 / 0.7 * 9.8;
    float g = fuseStatus.g * 0.00122;
    float power5V = fuseStatus.power5V * 0.002;
    
    SETDATA(pDataPoolSelf, "fuseMode", fuseStatus.fuseMode,	OS_U8);
	SETDATA(pDataPoolSelf, "fuse24V", fuseStatus.power24VMode,	OS_U8);
	SETDATA(pDataPoolSelf, "fuseActv", fuseStatus.activeStatus,	OS_U8);
	SETDATA(pDataPoolSelf, "fuseBIT", fuseStatus.bitStatus,	OS_U8);
	SETDATA(pDataPoolSelf, "fuse5V", fuseStatus.power5V,    OS_U16);
	SETDATA(pDataPoolSelf, "fuseuf", fuseStatus.fuseuf,	    OS_U16);
    
    SETDATA(pDataPoolSelf, "fuseax", ax,	    OS_FLOAT);
    SETDATA(pDataPoolSelf, "fuseay", ay,	    OS_FLOAT);
    SETDATA(pDataPoolSelf, "fuseaz", az,	    OS_FLOAT);
    SETDATA(pDataPoolSelf, "fuseg", g,	    OS_FLOAT);
    SETDATA(pDataPoolSelf, "fuseTemp", fuseStatus.temp,	    OS_U16);
    
	g_DeviceState.fuseCountDown = 200;
	return 0;
}

static OS_U8 GenFuseBuf(OS_U8 *buf, OS_U8 msgID)
{
    OS_U8 data[10];
    data[0] = 0x55;
    data[1] = 0xAA;
    data[2] = 0x09;
	switch (msgID)
	{
	case 0x3E:
		/* code */
		data[3] = 0x01;
		break;
	default:
		data[3] = 0x03;
		break;
	}
    data[4] = 0x11;
    data[5] = 0x21;
	data[6] = msgID;

	OS_U16 checksum = CheckSum16_8(data, 7);
    data[7] = (checksum & 0xFF);
    data[8] = ((checksum >> 8) & 0xFF);
    // memcpy(buf, data, 9);
	
	for (int i = 0; i < 10; i++)
	{
		memcpy(buf + i * 9, data, 9);
	}
    return 90;
}

static OS_U8 MsgToFuse(OS_U8 *buf, OS_U16 len)
{
	RT rt = rtList[RT_FUSE];
    OS_U8 data[100];
    memcpy(data, buf, len);
	UART_PutBuff(rt.chIndex, buf, len);
	return 0;
}

OS_U8 FuseSend(OS_U8 msgID)
{
	OS_U8 inData[200] = {0};
	OS_U8 inLen = GenFuseBuf(inData, msgID);
	MsgToFuse(inData, inLen);
	return 0;
}
