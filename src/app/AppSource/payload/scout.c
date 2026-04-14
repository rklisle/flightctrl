/*
 * scout.c
 *
 *  Created on: 2023年5月19日
 *      Author: lenovo
 */
#include "scout.h"
#include "../support/os_basic.h"
#include "../interface/interface_uart.h"
#include "MsnTime.h"
#include "../FlightSupport.h"
#include "../flight/os_flight_io.h"
#include <math.h>
#include "../core/BusInteract.h"
#include "../support/common.h"
#include "../support/os_bufferLoop.h"

SCOUT_STATUS scoStatus = {0};
OS_U8 IsLoadImage = 0;
OS_U8 workMode = 0;
OS_U16 userSetX = 0;
OS_U16 userSetY = 0;
OS_U32 userSetFrame = 0;
OS_U8 isUserSetNew = 0;

OS_U8 ScoutCheckByte(OS_U8 *buf, OS_U8 len)
{
	return XorSum8(buf, len);
}

OS_U8 InitScout()
{
    SETDATA(pDataPoolFly,	"sctLock",	   0,		OS_U8);
    
	SETDATA(pDataPoolFly,	"sctPitch",	   0,		OS_S16);
	SETDATA(pDataPoolFly,	"sctYaw",	   0,		OS_S16);


	SETDATA(pDataPoolFly, 	"viewPitc",    0 , 	OS_S16);//s16 90/32767
	SETDATA(pDataPoolFly, 	"viewYaw",     0 , 	OS_S16);//u16 	360/65535

	SETDATA(pDataPoolFly, 	"vPitchSp",     0/0.002,		OS_S16);//s16   0.01
	SETDATA(pDataPoolFly, 	"vYawSp", 	    0/0.002,		OS_S16);//s16 	0.01

	buffLoop[RT_SCOUT].syncHead_A = 0x77;
	buffLoop[RT_SCOUT].syncHead_B = 0xAB;
	buffLoop[RT_SCOUT].lenExtern = 4;	//普通422消息，头部6字节，校验和2字节不算入长度字段
	buffLoop[RT_SCOUT].head = 0;
	buffLoop[RT_SCOUT].tail = 0;
	buffLoop[RT_SCOUT].lenPos = 0;	//同步头后第n个字节为长度
	buffLoop[RT_SCOUT].fixedLen = 50;
	buffLoop[RT_SCOUT].inited = TRUE;

    
	return 0;
}

OS_U16 ChkScoutStandardFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != 0x77 || pmData[2] != 0xAB)
	{
		return 0;
	}
    OS_U8 temp_crc[2];
    temp_crc[0] = pmData[49];
    temp_crc[1] = pmData[50];
   
    OS_U16 crcValue;
    memcpy(&crcValue, temp_crc, 2);
    
    
   if(Chk16CRC_U8(pmData + 1, 50 - 2, crcValue) == OS_FALSE)
	{//校验和不通过
		static int errorCount32 = 0;
        errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "sctChkEr", errorCount,OS_U8);
		return 0;
	}
   
    memmove(pmData + 7, pmData + 6, 50 - 7);
    memcpy(data,pmData, 100);
	pmData[4] = 0;
	pmData[5] = 0;
	pmData[6] = 0x11;
	return pmData[3] + 5;
}

OS_U32 ScoutRtHandler(STRU_422_MSG_INFO *data)
{
		//SCOUT_STATUS scoStatus;

	OS_U8 msgID = data->u8MsgID;
	switch(msgID)
	{
	case SCOUT_BFRAME:
		{
			OS_U8 ctrlrebackID = data->au8Data[0];
			OS_U8 bookrebackID = data->au8Data[3];
		
			if(ctrlrebackID == 0xB6 || bookrebackID == 0X01 || bookrebackID == 0X02 || bookrebackID == 0X03 || bookrebackID == 0XA3)
			{
				MsgToDevice(RT_DATA_LINK,msgID,data->u16Len, data->au8Data);
			}
			else if( ctrlrebackID == 0xD6 || ctrlrebackID == 0xC6 ||  bookrebackID == 0xA1 || bookrebackID == 0xB1 || bookrebackID == 0xA2 || bookrebackID == 0xB2 || bookrebackID == 0xA1)
			{
				IsLoadImage = 0;
				MsgToDevice(RT_DATA_LINK,msgID,data->u16Len, data->au8Data);
			}
			else
			{
				memcpy(&scoStatus, data->au8Data, sizeof(SCOUT_STATUS));

				OS_U8 locked = (scoStatus.lightState == 3 || scoStatus.lightState == 4)?1:0;
				SETDATA(pDataPoolFly,	"sctLock",	   locked,		OS_U8);
				SETDATA(pDataPoolFly,	"sctCheck",	   scoStatus.selfcheck,		OS_U8);
				
				SETDATA(pDataPoolFly,	"sctPitch",	   scoStatus.scoutPitch,		OS_S16);
				SETDATA(pDataPoolFly,	"sctYaw",	   scoStatus.scoutYaw,		OS_S16);

				SETDATA(pDataPoolFly, 	"viewPitc",    0 , 	OS_S16);//s16 90/32767
				SETDATA(pDataPoolFly, 	"viewYaw",     0 , 	OS_S16);//u16 	360/65535

				SETDATA(pDataPoolFly, 	"vPitchSp",     scoStatus.viewPitchSpd,	OS_S16);//s16   0.01
				SETDATA(pDataPoolFly, 	"vYawSp", 	    scoStatus.viewYawSpd,		OS_S16);//s16 	0.01
			}
		}
		break;
	case IMAGE_REBACK:
	case SOFT_REBACKE:
		{
			MsgToDevice(RT_DATA_LINK,msgID,data->u16Len, data->au8Data);
		}
		break;
  	}
	
		g_DeviceState.scoutCountDown = 200;
		return 0;
}
void ScoutGroundHandler(STRU_422_MSG_INFO *data)
{
	OS_U8 msgID = data->u8MsgID;
	switch(msgID)
	{
	case SCOUT_AFRAME:
	{
		OS_U8 bookID = data->au8Data[4];
		if(bookID == 0x01 || bookID == 0x02 || data->au8Data[0] == 0xB6)
		{
			IsLoadImage = 1;
			MsgToDevice(RT_SCOUT,msgID,data->u16Len, data->au8Data);
		}
		else
		{
			MsgToDevice(RT_SCOUT,msgID,data->u16Len, data->au8Data);
		}
		
	}
	   break;
	case TEMPLATE_TXT:
	case IMAGE_INFO:
	case SOFT_UPDATE:
	{
		MsgToDevice(RT_SCOUT,msgID,data->u16Len, data->au8Data);
	}
	   break;
    case CMD_USER_SETTARGET:
    {
        isUserSetNew = 20;
        memcpy(&userSetX, data->au8Data + 0, 2);
        memcpy(&userSetY, data->au8Data + 2, 2);
        memcpy(&userSetFrame, data->au8Data + 4, 4);
    }
        break;
		case CMD_SET_IMAGEMODE:
			workMode = data->au8Data[0];
			break;
  }
}
static OS_U8 GenScoutBuf(OS_U8 *inData)
{
	//OS_U8 inData[200] = {0};
	static OS_U16 cmdframeCount = 0;
	static OS_U8 updateMark[3] = {0};

	//	static OS_U32 tick = 0;
	SCOUT_CMD cmd = {0};
	cmd.ctrlCmd = 6;// conn test;
  cmd.workMode = workMode; 
    if(isUserSetNew > 0)
    {
        cmd.ctrlCmd = 9;
				updateMark[0] = 0;
        isUserSetNew--;
    }
	
	cmd.lowlighlCtrl = 0;
	cmd.modelbookCmd = 0;

	OS_S32 temps32;
	OS_S16 temps16;
	OS_U16 tempu16;
	OS_FLOAT tempfloat;
	//弹目距离（可不准确）
	GetDataFast(pDataPoolFly, "ac_dL",	&tempu16);
	cmd.targetDis = tempu16 * 1 * 1;
	//导弹速度
	GetDataFast(pDataPoolSelf, "GrdSpd",	&temps16);
	cmd.spd = temps16 * 0.1 * 10;
	//弹体俯仰角（+-90)
	GetDataFast(pDataPoolImu, "navPitch",	&temps16);
	cmd.pitch = temps16 * 0.01 * 100;
	//弹体方位角(+-180)
	GetDataFast(pDataPoolImu, "navDir",	&temps16);
	cmd.dir = temps16 * 0.01;
	if(cmd.dir > 180)
	cmd.dir -= 360;
	cmd.dir = cmd.dir * 100;
	//弹体滚转角(+-180)
	GetDataFast(pDataPoolImu, "navRoll",	&temps16);
	cmd.roll = temps16 * 0.01 * 100;
	//弹体俯仰角速度
	GetDataFast(pDataPoolImu, "imuWz",	&tempfloat);
	cmd.pitchSpd = tempfloat * 100;
	//弹体方位角速度
	GetDataFast(pDataPoolImu, "imuWy",	&tempfloat);
	cmd.dirSpd = tempfloat * 100;
	//弹体滚转角速度
	GetDataFast(pDataPoolImu, "imuWx",	&tempfloat);
	cmd.rollSpd = tempfloat * 100;
	//弹体经度
	GetDataFast(pDataPoolImu, "navLon",	&temps32);
	cmd.lon = temps32 * 1e-7 * 1e6;
	//弹体纬度
	GetDataFast(pDataPoolImu, "navLat",	&temps32);
	cmd.lat = temps32 * 1e-7 * 1e6;
	//弹体高度
	GetDataFast(pDataPoolImu, "navHigh",&tempfloat);
	cmd.high = tempfloat * 1;
	//目标经度（可不准确）
	GetDataFast(pDataPoolMsn, "tarLon",	&temps32);
	cmd.targetlon = temps32 * 1e-7 * 1e6;
	//目标纬度（可不准确）
	GetDataFast(pDataPoolMsn, "tarLat",	&temps32);
	cmd.targetlat = temps32 * 1e-7 * 1e6;
	//目标高度（可不准确）
	GetDataFast(pDataPoolMsn, "tarAlt",&temps16);
	cmd.targetHigh = temps16 * 1;
/* *********************************************** MML 20260413*****************************************************

	cmd.scoutPitchSet = pOutput->Pitch_Preset_Angle / 0.002;//预装俯仰框架角
	cmd.scoutyawSet = pOutput->Yaw_Preset_Angle / 0.002;//预装偏航框架角
*/
	cmd.offsety = userSetY;//图像俯仰方向（上下）像素值，手选攻击目标时使用（左上0，0）
	cmd.offsetx = userSetX;//图像偏航方向（左右）像素值，手选攻击目标时使用（左上0，0）
	cmd.useOffsetFrameCount = userSetFrame;//上面两个参数的来源对应的帧号，需要一个不采用时的值

	if(scoStatus.ctrlback == cmd.ctrlCmd || updateMark[0] == 1 )
	{
        cmd.ctrlCmd = 0;
        updateMark[0] = 1;
	}
	if(scoStatus.lowlightback == cmd.lowlighlCtrl || updateMark[1] == 1 )
	{
        cmd.lowlighlCtrl = 0;
        updateMark[1] = 1;
	}
	if(scoStatus.imageback == cmd.modelbookCmd || updateMark[2] == 1 )
	{
        cmd.modelbookCmd = 0;
        updateMark[2] = 1;
	}

	memcpy(inData, &cmd, sizeof(SCOUT_CMD));

	//Insert16CRC_U8(buf, sizeof(SCOUT_CMD) - 2, (OS_U16*)(buf + sizeof(SCOUT_CMD) - 2));

	cmdframeCount++;
	return sizeof(SCOUT_CMD);
}
OS_U8 ScoutAutoSend()
{
    if(g_DeviceState.CurrTick % 4 != 0)
    {
        return 0;
    }
	OS_U8 inData[200] = {0};
	OS_U8 inLen = GenScoutBuf(inData);
	MsgToDevice(RT_SCOUT,SCOUT_AFRAME,inLen,inData);
	return 0;
}
