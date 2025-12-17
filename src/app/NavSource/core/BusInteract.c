/*
 * BusInteract.c
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 *
 * 本模块作为箭上总线功能的所有处理集合。
 * 总线通信为箭上各单机设备之间的联系方式，常见方式包括1553b,can,spacewire,LVDS,422,1394,以太网等
 * 本型号火箭选用422接口
 * 无论采用何总线接口，均需要通过本模块进行消息的处理分发
 */
#include "BusInteract.h"
#include "../controller/controller.h"
#include "../support/common.h"
#include "../interface/interface_uart.h"
#include "../modules/modFlyCtrl.h"
#include "../modules/modGps.h"
#include <string.h>

RT rtList[MODULE_COUNT];
RT_CAN rtCan;

/***********************************************************
 * 函数名称:InitRts()
 * 函数功能: 本函数在main函数初始化硬件设备前调用，其功能包括:
 * 		1.为各422设备接口定义参数
 * 		2.为各设备定义处理函数
 * 		3.定义本机处理函数
 * 作者:	成宏璟
 ***********************************************************/
void InitRts()
{
	for(int i=0;i<MODULE_COUNT;i++)
	{
		rtList[i].ptr_ChkFrameSum = ChkStandardFrame;//所有rt先初始化为标准校验函数
		rtList[i].ptr_Init = PTR_NULL;//所有rt先定义为“无特殊初始化要求”
	}


	rtList[RT_FLYCTRL].ckIndex = 0;//0		//智能控制器
	rtList[RT_FLYCTRL].chIndex = 0;//0
	rtList[RT_FLYCTRL].devID = 0x01;
	rtList[RT_FLYCTRL].devBuad = 460800;
	rtList[RT_FLYCTRL].oddCheckEnable = FALSE;
	rtList[RT_FLYCTRL].ptr_RtHandler = FlyctrlRtHandler;

	rtList[RT_GPS_1].ckIndex = 0;//0		//GPS
	rtList[RT_GPS_1].chIndex = 1;//0
	rtList[RT_GPS_1].devID = 0x01;
	rtList[RT_GPS_1].devBuad = 115200;
	rtList[RT_GPS_1].oddCheckEnable = FALSE;
	rtList[RT_GPS_1].ptr_RtHandler = GpsRtHandler;
	rtList[RT_GPS_1].ptr_ChkFrameSum = ChkGpsFrame;

	rtList[RT_GPS_2].ckIndex = 0;//0		//备用
	rtList[RT_GPS_2].chIndex = 2;//1
	rtList[RT_GPS_2].devID = 0x01;
	rtList[RT_GPS_2].devBuad = 115200;
	rtList[RT_GPS_2].oddCheckEnable = FALSE;
	rtList[RT_GPS_2].ptr_RtHandler = GpsRtHandler;
	rtList[RT_GPS_2].ptr_ChkFrameSum = ChkGpsFrame;

	for(int i=0;i<MODULE_COUNT;i++)
	{
		if(rtList[i].ptr_Init != PTR_NULL)
		{
			rtList[i].ptr_Init();//执行特殊协议初始化
		}
	}
}

/***********************************************************
 * 函数名称:BusDataHandle()
 * 函数功能: 本函数由状态机每5ms调用一次，每次调用时会从422接口接收本周期内的数据
 * 			所有的连接设备的422口都会被遍历到。来自地面的指令会进入到遥控处理模块
 * 			其它信息会进入到对应的各设备处理模块。
 * 			对消息来源的判断依据不再依赖设备号，而是通过接收消息的串口号识别。
 * 作者:	成宏璟
 ***********************************************************/
void BusDataHandle()
{
	STRU_STANDARD_FRAME * frmPtr = NULL;
	//每一次进到本函数，都需要处理目前所有422接口收到的所有数据
	while ((frmPtr = (STRU_STANDARD_FRAME *)PeekStandardMessage()))//数据已经过校验
	{
		//frmPtr->pStand422Data.u8Dev = rtList[frmPtr->u8RtIndex].devID;
		if(rtList[frmPtr->u8RtIndex].ptr_RtHandler != NULL)
			rtList[frmPtr->u8RtIndex].ptr_RtHandler((STRU_422_MSG_INFO *)&(frmPtr->pStand422Data));
	}
}

/***********************************************************
 * 函数名称:PrintDebug()
 * 函数功能: 打印调试函数，在任意处调用本函数，可在磨砂卡中查看打印内容.
 * 使用示例:
 *  OS_U16 count = *(OS_U16 *)(frame->au8Data + 2);
	char info[50] = {0};
	sprintf(info,"\nrecv flash info, msgId is %d, current is:%d",frame->u8MsgID, count);
	PrintDebug(info);
 * 作者:	成宏璟
 ***********************************************************/
OS_U8 PrintDebug(char *str)
{
	OS_U16 strLen = strlen(str);
	UART_PutBuff((int)rtList[RT_GPS_2].chIndex, (OS_U8*)str, strLen);
	return 0;
}

/***********************************************************
 * 函数名称: MsgToDevice()
 * 函数功能: 经智能控制器处理后，需要发送给设备的指令，包括转发地面请求，飞控对舵机的操作等
 * 作者:	成宏璟
 ***********************************************************/
OS_U8 MsgToDevice(OS_U8 rtIndex, OS_U8 msgID, OS_U16 msgLen, OS_U8* buf)
{
	static OS_U8 u8Seq[MODULE_COUNT] = {0};
	STRU_422_MSG_INFO msg = { 0 };
	msg.u8Seq = u8Seq[rtIndex]++;
	msg.u8MsgID = msgID;
	msg.u16Len = msgLen;
	memcpy(msg.au8Data, buf, msgLen);
	SendCRCed422Message(rtList[rtIndex].ckIndex,
					rtList[rtIndex].chIndex,
					(OS_MEM*)&msg,
					_422_FRAME_SYNCCHAR_LEN,	//计算校验和的起点，不包括0x55AA
					msg.u16Len + _422_FRAME_HEADER_LEN + _422_FRAME_FOOTER_LEN);
	return 0;
}


