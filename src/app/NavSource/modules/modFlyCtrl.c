/*
 * modIMU.c
 *
 *  Created on: 2021年10月16日
 *      Author: QL
 */
#include "../StateMachine.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../core/Telecontrol.h"
#include "../interface/interface_uart.h"
#include "modFlash.h"
#include "modGps.h"
#include <math.h>

#include "../support/common.h"
#include "./modImu.h"
#include "./modFlyCtrl.h"
#include "../navSupport.h"
#define PI (3.1415926)

#define IMU_X_POSITIVE	1
#define IMU_X_NEGATIVE	-1
#define IMU_Y_POSITIVE	2
#define IMU_Y_NEGATIVE  -2
#define IMU_Z_POSITIVE 3
#define IMU_Z_NEGATIVE	-3
OS_S8 installMode[3] = {IMU_X_POSITIVE, IMU_Z_POSITIVE, IMU_Y_NEGATIVE, };
NavInitStr initData = {0};
OS_DOUBLE StartFocusTime = 0;
OS_U8 flightMode = 0;//正式飞行模式
OS_U8 MsgToFlyCtrl()
{
	STRU_NAV_INFO navInfo;

	MsgToDevice(RT_FLYCTRL, 0, sizeof(navInfo), (OS_U8*)&navInfo);

	return 0;
}

static OS_U8 SetNavInstallMode(int x, int y, int z)
{
	installMode[0] = x;
	installMode[1] = y;
	installMode[2] = z;
	navState = 0x45;
	return 0;
}

/***********************************************************
 * 函数名称:ToNavModel()
 * 函数功能: 转导航功能，在执行该功能前需要惯组先完成水平计算。水平计算获取俯仰的不水平度
 * 		       加90即为俯仰角，偏航角的不水平度即为偏航角。
 ***********************************************************/
OS_U8 ToNavModel()
{
	return 0;
}

OS_U8 EphData[0xA000];
OS_U16 EphLen;
OS_U8 EncpEphSet(void *data)
{
	/*if(ephRecvReadyFlag == TRUE && data->EphemerisEncpState == 0)
	{
		Starteph = 0x4A4B;
		MsgToIMU1(BUS_IMU_INFO_SEND_EPH);
		Tick = 100;
	}
	if(data->EphemerisEncpState == 1)
	{
		ephRecvReadyFlag = FALSE;

		if(Tick == 100)
		{
			UART_PutBuff((int)rtList[RT_MEMS].chIndex, EphData, EphLen);
		}
		Tick--;
		if(Tick == 0)
		{
			Starteph = 0x4C4D;
			MsgToIMU1(BUS_IMU_INFO_SEND_EPH);
			Tick = 2000;
		}
	}
	if(data->EphemerisEncpState == 2)
	{
		;
	}
	*/
	return 0;
}

/***********************************************************
 * 函数名称:ImuRtHandler()
 * 函数功能: 惯组总线处理函数，本型号智能控制器接收惯组发出的指令:
 * 			1.惯组定时发送帧 0x93		:(1)存数据池 (2)依据falsh数据进行数据处理 (3)水平计算 (4)再存数据池
 * 参考资料: <TXII-Y1 422箭上通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 FlyctrlRtHandler(STRU_422_MSG_INFO * frame)
{

	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case BUS_NAV_FLY_MODE:
		flightMode = frame->au8Data[0];
		break;
	case BUS_NAV_INIT_DATA:	//安装模式设置（含初始诸元)
		{
			OS_S8 x,y,z;
			memcpy(&x,&frame->au8Data[0],1);
			memcpy(&y,&frame->au8Data[1],1);
			memcpy(&z,&frame->au8Data[2],1);
			SetNavInstallMode(x, y, z);

			memcpy(&initData.lon, frame->au8Data +3, sizeof(OS_DOUBLE));
			memcpy(&initData.lat, frame->au8Data +11, sizeof(OS_DOUBLE));
			memcpy(&initData.alt, frame->au8Data +19, sizeof(OS_DOUBLE));
			memcpy(&initData.fai0, frame->au8Data +27, sizeof(OS_DOUBLE));
            
            
			NavInit(&initData);
		}
		break;
	case BUS_NAV_FOCUS:		//对准指令
		{
			nav_input_imu.cmd = 1;
			navState = 0x20;
			StartFocusTime = GetCurTime() * 1000;
		}
		break;
	case BUS_NAV_START_NAV:	//转导航指令
		{
			if(nav_input_imu.cmd != 3)
				nav_input_imu.cmd = 3;
			else
				nav_input_imu.cmd = 2;
		}
		break;
	case BUS_NAV_IGNATION:
		nav_input_imu.cmd = 4;
		break;

    /*
	case BUS_NAV_IMUDATA:
		{
			useSimuData = OS_TRUE;
			//按右前下转换
			navInputImu2.DATA_YACCEL1ms = *(float *)(&frame->au8Data[0]);
			navInputImu2.DATA_ZACCEL1ms = *(float *)(&frame->au8Data[4]);
			navInputImu2.DATA_XACCEL1ms = *(float *)(&frame->au8Data[8]);
			navInputImu2.DATA_YGYRO1ms = *(float *)(&frame->au8Data[12]);
			navInputImu2.DATA_ZGYRO1ms = *(float *)(&frame->au8Data[16]);
			navInputImu2.DATA_XGYRO1ms = *(float *)(&frame->au8Data[20]);

			GNSS_INPUT_Data2.Lon = *(double *)(&frame->au8Data[24]);
			GNSS_INPUT_Data2.Lat = *(double *)(&frame->au8Data[32]);
			GNSS_INPUT_Data2.Height = *(float *)(&frame->au8Data[40]);
			GNSS_INPUT_Data2.Vel_north = *(float *)(&frame->au8Data[44]);
			GNSS_INPUT_Data2.Vel_UP =  *(float *)(&frame->au8Data[48]);
			GNSS_INPUT_Data2.Vel_east =  *(float *)(&frame->au8Data[52]);

			GNSS_INPUT_Data2.Status1 = frame->au8Data[56];
		}
		break;
        */
	default:
		break;
	}
	return 0;
}

OS_U8 StartEncpEphToNav(OS_U8 *data, OS_U16 len)
{
	//ephRecvReadyFlag = TRUE;

	//memcpy(EphData, data, len);
	//EphLen = len;
	return 0;
}
