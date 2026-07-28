/*
 * scout.c
 *
 *  Created on: 2023��5��19��
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
#include "log_ctrl.h"

#define SWAP_U16(x)  ((OS_U16)(((x) >> 8) | ((x) << 8)))

FUSE_RCV_FRAME fuseFrame = {0};

OS_U8 InitFuse()
{
	buffLoop[RT_FUSE].syncHead_A = 0xEB;
	buffLoop[RT_FUSE].syncHead_B = 0x90;
	buffLoop[RT_FUSE].lenExtern = 0;	// 数据帧中表示长度字节，之外还有lenExtern个字节，总共构成一帧数据	//普通422消息，头部6字节，校验和2字节不算入长度字段
	buffLoop[RT_FUSE].head = 0;
	buffLoop[RT_FUSE].tail = 0;
	buffLoop[RT_FUSE].lenPos = 0;	//0;	//除了Head_A和Head_B之外再偏移多少字节才到长度字节	//同步头后第n个字节为长度
	buffLoop[RT_FUSE].fixedLen = 27;	//0：取HeadA、HeadB后面的两个字节作为长度；-1：暂不清楚；具体数值：固定长度
	buffLoop[RT_FUSE].inited = TRUE;
	return 0;
}

OS_U16 ChkFuseStandardFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != buffLoop[RT_FUSE].syncHead_A || pmData[2] != buffLoop[RT_FUSE].syncHead_B || pmData[3] != 0xFC)
	{
		return 0;
	}
    
    static uint32_t last_check = 0;
    static uint32_t s_error_cnt = 0;
    static uint32_t s_total_cnt = 0;
    s_total_cnt++;
    uint32_t cur_check = pmData[4]*0x10000 + pmData[5]*0x100 + pmData[6];
    if((last_check + 1) != cur_check)
    {
        s_error_cnt++;
		LOG_VAL("s_error_cnt = %d\n", s_error_cnt);
    }
    last_check = cur_check;

	// pmData[26] * 0x100 + pmData[27]
    OS_U16 checkSumRecv = pmData[buffLoop[RT_FUSE].fixedLen - 1] * 0x100 + pmData[buffLoop[RT_FUSE].fixedLen];	// Big-endian
    OS_U16 checkSumCalc = crc16_xmodem(pmData + 1, buffLoop[RT_FUSE].fixedLen - 2);
    
   if(checkSumRecv != checkSumCalc)
	{//Verification failed
		static int errorCount32 = 0;
        errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "sctChkEr", errorCount,OS_U8);
		return 0;
	}

	/** Assemble into a standard frame */
	pmData[3] = 17;		//au8Data lenth
	pmData[4] = 0;		// 
	pmData[5] = 0;		// u8Seq
	pmData[6] = 0x11;	// u8MsgID  // accoding to 280
	memcpy(pmData+7, pmData+9, 17);// au8Data 
	return pmData[3] + 7 + 2;
}

OS_U32 FuseRtHandler(STRU_422_MSG_INFO *data)
{
    if(data->u16Len != 17)
    {
        return -1;
    }

	memcpy(&fuseFrame, data->au8Data, sizeof(FUSE_RCV_FRAME));

	// SD_CMD = (fuseFrame.feedback >> 0) & 0x01;
	// SF_OK  = (fuseFrame.feedback >> 1) & 0x01;

    SETDATA(pDataPoolSelf, "fzFeedbk", fuseFrame.feedbk,OS_U8 );
    SETDATA(pDataPoolSelf, "fzTask",   fuseFrame.task,	OS_U8 );
    SETDATA(pDataPoolSelf, "fzFirV",   (SWAP_U16(fuseFrame.firV)),	OS_U16);
    SETDATA(pDataPoolSelf, "fzPrxA",   (SWAP_U16(fuseFrame.prxA)),	OS_U16);
    SETDATA(pDataPoolSelf, "fzIC12V",  (SWAP_U16(fuseFrame.ic12V)),	OS_U16);
    SETDATA(pDataPoolSelf, "fzDetV",   (SWAP_U16(fuseFrame.detV)),	OS_U16);
    SETDATA(pDataPoolSelf, "fzDcfA",   (SWAP_U16(fuseFrame.dcfA)),	OS_U16);
    SETDATA(pDataPoolSelf, "fzC1Stat", (SWAP_U16(fuseFrame.c1Stat)),OS_U16);
    SETDATA(pDataPoolSelf, "fzUnitNo", (SWAP_U16(fuseFrame.unitNo)),OS_U16);
    SETDATA(pDataPoolSelf, "fzImpSt",  fuseFrame.impSt,	OS_U8 );


/** 280 
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
*/    
	g_DeviceState.fuseCountDown = 200;
	return 0;
}

static OS_U8 GenFuseBuf(OS_U8 *buf, FuzeCmdType cmd)
{
    OS_U8 command[10] = {0};

    command[0] = 0xEB;
    command[1] = 0x90;
    command[2] = 0xFC;

    switch (cmd)
    {
    case ARM_I:
        command[4] = 0x55;  // EB 90 FC 00 55 00 00 00      69 35
        break;
    case ARM_II:
        command[5] = 0x55;  // EB 90 FC 00 00 55 00 00      15 75
        break;
    case ARM_III:
        command[6] = 0x55;  // EB 90 FC 00 00 00 55 00      51 01
        break;
    case DETO:
        command[7] = 0x55;  // EB 90 FC 00 00 00 00 55      AA 1B
        break;
    default:
        return 0;  // 无效命令
    }

    // 计算 CRC16（前8字节）
    OS_U16 checksum = crc16_xmodem(command, 8);
    command[8] = ((checksum >> 8) & 0xFF);
    command[9] = (checksum & 0xFF);

    // 重复10次，共100字节
    for (int i = 0; i < 10; i++)
    {
        memcpy(buf + i * 10, command, 10);
    }

    return 100;
}

static OS_U8 MsgToFuse(OS_U8 *buf, OS_U16 len)
{
	RT rt = rtList[RT_FUSE];
	UART_PutBuff(rt.chIndex, buf, len);
	return 0;
}

OS_U8 FuseSend(FuzeCmdType cmd)
{
	if(cmd < ARM_I || cmd > DETO) return -1;

	OS_U8 inData[128] = {0};
	OS_U8 inLen = GenFuseBuf(inData, cmd);
	MsgToFuse(inData, inLen);
	return 0;
}
