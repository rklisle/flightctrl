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
	buffLoop[RT_FUSE].lenExtern = 4;	//普通422消息，头部6字节，校验和2字节不算入长度字段
	buffLoop[RT_FUSE].head = 0;
	buffLoop[RT_FUSE].tail = 0;
	buffLoop[RT_FUSE].lenPos = 0;	//同步头后第n个字节为长度
	buffLoop[RT_FUSE].fixedLen = 50;
	buffLoop[RT_FUSE].inited = TRUE;
	return 0;
}

OS_U16 ChkFuseStandardFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != 0xAA || pmData[2] != 0x55)
	{
		return 0;
	}
    
    OS_U16 checkSumRecv = pmData[pmData[3]] * 0x100 + pmData[pmData[3] - 1];
    OS_U16 checkSumCalc = CheckSum16_8(pmData + 1, pmData[3] - 2);
    
   if(checkSumRecv != checkSumCalc)
	{//校验和不通过
		static int errorCount32 = 0;
        errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "sctChkEr", errorCount,OS_U8);
		return 0;
	}
   
    memmove(pmData + 7, pmData + 6, pmData[3]);
    memcpy(data,pmData, 100);
	pmData[4] = 0;
	pmData[5] = 0;
	pmData[6] = 0x11;
	return pmData[3] + 5;
}

OS_U32 FuseRtHandler(STRU_422_MSG_INFO *data)
{
    if(data->u16Len == 0x0B)
    {
        //change to 30byte frame
        FuseSend(0x55);
    }
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
    data[3] = msgID;//0x55; //msgid
    data[4] = 0x11;
    data[5] = 0x21;
    if(msgID == 0x55)
    {
        data[6] = 0x1A;//long frame
    }
    OS_U16 checksum = CheckSum16_8(data, 7);
    data[7] = (checksum & 0xFF);
    data[8] = ((checksum >> 8) & 0xFF);
    memcpy(buf, data, 9);
    return 9;
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
