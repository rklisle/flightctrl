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
#include "../support/common.h"
#include "../support/os_bufferLoop.h"
#include "../interface/interface_uart.h"
#include "../modules/modHil.h"
#include "../modules/modNav.h"
#include "../modules/modSrvCtl.h"
#include "../modules/modPwrSeqCtl.h"
#include "../modules/modOnceBattery.h"
#include "../modules/modMEMS.h"
#include "../modules/modEngine.h"
#include "../payload/scout.h"
#include "../payload/fuse.h"
#include "../../app_can.h"
#include "Telecontrol.h"
//#include "../support/os_bufferLoop.h"
#include <string.h>

RT rtList[MODULE_COUNT];
RT_CAN rtCan[CAN_COUNT];

/***********************************************************
 * 函数名称:InitCanRts()
 * 函数功能: 本函数在main函数初始化硬件设备前调用，其功能包括:
 * 		1.为各CAN设备接口定义参数
 * 		2.为各设备定义处理函数
 * 		3.定义本机处理函数
 * 作者:	成宏璟
 ***********************************************************/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
void InitCanRts()
{
    hfdcan1.Instance = FDCAN1;
    hfdcan2.Instance = FDCAN2;
    // fdCan[0] = app_can_init(NULL, &hfdcan1, CanRtBattHandler);	//MML舵机
	fdCan[0] = app_can_init(NULL, &hfdcan1, CanRtServoHandler);  // 改为舵机处理函数
    fdCan[1] = app_can_init(NULL, &hfdcan2, CanRtPwrSeqHandler);	//MML配电板

}
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
	/*************************** 
	 * 快速遥测接2号口，序号1
	 * 半实物接1号口，序号0
	 * 箭地接6号口，序号5
	 * 数据链接3号口，序号2
	 * 组合导航接5号口，序号4
	 * 	
	 * *************************************  280
	 * 导引头 UART1
	 ***** 火控/引信 UART2
	 * IMU UART3********************
	 ***** 数据链 UART4
	 * 舵机 UART5
	 ***** 发动机 UART6
	 ***** 半实物 UART7
	 * 导航版上的NAV UART8***********
	 ********************** */
	/***************************************** 014
	 * 导引头——UART1/2，chIndex0/1
	 ***** 数据链——UART3，chIndex2
	 ***** 引信——UART4，chIndex3
	 ***** 仿真口——UART6 / 网口，chIndex5
	 ***** 发动机——UART7，chIndex6
	 */
	rtList[RT_SCOUT].ckIndex = 0;//0		//载荷1  任务机
	rtList[RT_SCOUT].chIndex = 0;//	导引头 UART1 /***************** 目前不用了 ************/
	rtList[RT_SCOUT].devID = 0x01;
	rtList[RT_SCOUT].devBuad = 230400;
    rtList[RT_SCOUT].devStopLen = 1;
	rtList[RT_SCOUT].oddCheckEnable = FALSE;
    rtList[RT_SCOUT].evenCheckEnable = FALSE;
	rtList[RT_SCOUT].ptr_RtHandler = ScoutRtHandler;
    //rtList[RT_SCOUT].ptr_Init = InitScout;
    //rtList[RT_SCOUT].ptr_ChkFrameSum = ChkScoutStandardFrame;

	// rtList[RT_SCOUT_CTL].ckIndex = 0;
	// rtList[RT_SCOUT_CTL].chIndex = 0;// 导引头 控制信息	UART1
	// rtList[RT_SCOUT_CTL].devID = 0x01;
	// rtList[RT_SCOUT_CTL].devBuad = 230400;
	// rtList[RT_SCOUT_CTL].devStopLen = 1;
	// rtList[RT_SCOUT_CTL].oddCheckEnable = FALSE;
    // rtList[RT_SCOUT_CTL].evenCheckEnable = FALSE;
    // rtList[RT_SCOUT_CTL].ptr_ChkFrameSum = NULL;
	// rtList[RT_SCOUT_CTL].ptr_RtHandler = ScoutRtHandler;

	rtList[RT_SCOUT_ATTITUDE].ckIndex = 0;
	rtList[RT_SCOUT_ATTITUDE].chIndex = 1;// 导引头 航姿信息	UART2
	rtList[RT_SCOUT_ATTITUDE].devID = 0x01;
	rtList[RT_SCOUT_ATTITUDE].devBuad = 230400;
	rtList[RT_SCOUT_ATTITUDE].devStopLen = 1;
	rtList[RT_SCOUT_ATTITUDE].oddCheckEnable = FALSE;
    rtList[RT_SCOUT_ATTITUDE].evenCheckEnable = FALSE;
    rtList[RT_SCOUT_ATTITUDE].ptr_ChkFrameSum = NULL;
	rtList[RT_SCOUT_ATTITUDE].ptr_RtHandler = ScoutRtHandler;

	rtList[RT_DATA_LINK].ckIndex = 0;//0
	rtList[RT_DATA_LINK].chIndex = 2;// 数据链	UART3
	rtList[RT_DATA_LINK].devID = 0x05;
	rtList[RT_DATA_LINK].devBuad = 921600;	/**   280:  921600;*/
	rtList[RT_DATA_LINK].devStopLen = 1;
	rtList[RT_DATA_LINK].oddCheckEnable = FALSE;
    rtList[RT_DATA_LINK].evenCheckEnable = FALSE;
    rtList[RT_DATA_LINK].ptr_ChkFrameSum = ChkDataLinkFrame;
	rtList[RT_DATA_LINK].ptr_RtHandler = CmdHandler;

	rtList[RT_FUSE].ckIndex = 0;//0
	rtList[RT_FUSE].chIndex = 3;//引信——UART4
	rtList[RT_FUSE].devID = 0x04;
	rtList[RT_FUSE].devBuad = 115200;
	rtList[RT_FUSE].devStopLen = 1;
	rtList[RT_FUSE].oddCheckEnable = FALSE;
    rtList[RT_FUSE].evenCheckEnable = FALSE;
	rtList[RT_FUSE].ptr_RtHandler = FuseRtHandler;
    rtList[RT_FUSE].ptr_Init = InitFuse;
    rtList[RT_FUSE].ptr_ChkFrameSum = ChkFuseStandardFrame;

    rtList[RT_HIL].ckIndex = 0;//0
	rtList[RT_HIL].chIndex = 5;//仿真口——UART6
	rtList[RT_HIL].devID = 0x04;
	rtList[RT_HIL].devBuad = 230400;	/** 	280:	921600; */  
	rtList[RT_HIL].devStopLen = 1;
	rtList[RT_HIL].oddCheckEnable = FALSE;
    rtList[RT_HIL].evenCheckEnable = FALSE;
	rtList[RT_HIL].ptr_RtHandler = HilRtHandler;

    rtList[RT_ENGINE].ckIndex = 0;//0		//链路  射后透传
	rtList[RT_ENGINE].chIndex = 6;/********** ***表明fd = 6************ *ECU——UART7 */
	rtList[RT_ENGINE].devID = 0x01;
	rtList[RT_ENGINE].flags = 0x01;
	rtList[RT_ENGINE].devBuad = 115200;
	rtList[RT_ENGINE].devStopLen = 1;
	rtList[RT_ENGINE].oddCheckEnable = FALSE;
    rtList[RT_ENGINE].evenCheckEnable = FALSE;
	rtList[RT_ENGINE].ptr_RtHandler = EngineHandler;//解析数据：发动机 ——> 飞控
    rtList[RT_ENGINE].ptr_Init = EngineInit;
    rtList[RT_ENGINE].ptr_ChkFrameSum = NULL;//解析数据：发动机 ——> 飞控，查看校验CRC

	rtList[RT_NAV].ckIndex = 0;//0		//链路  射后透传
	rtList[RT_NAV].chIndex = 7;//导航板——UART8
	rtList[RT_NAV].devID = 0x01;
	rtList[RT_NAV].devBuad = 460800;
	rtList[RT_NAV].devStopLen = 1;
	rtList[RT_NAV].oddCheckEnable = FALSE;
    rtList[RT_NAV].evenCheckEnable = FALSE;
	rtList[RT_NAV].ptr_RtHandler = NavRtHandler;
/** ******************************************************************************************* */

// rtList[RT_IMU].ckIndex = 0;//0		//链路  射后透传
	// rtList[RT_IMU].chIndex = 2;//2
	// rtList[RT_IMU].devID = 0x01;
	// rtList[RT_IMU].devBuad = 921600;
	// rtList[RT_IMU].devStopLen = 1;
	// rtList[RT_IMU].oddCheckEnable = FALSE;
    // rtList[RT_IMU].evenCheckEnable = FALSE;
	// rtList[RT_IMU].ptr_RtHandler = ImuRtHandler;
    // rtList[RT_IMU].ptr_Init = ImuInit;
    // rtList[RT_IMU].ptr_ChkFrameSum = ChkImuFrame;
    
    // rtList[RT_SRV].ckIndex = 0;//0		//链路  射后透传	//MML舵机
	// rtList[RT_SRV].chIndex = 4;//4
	// rtList[RT_SRV].devID = 0x01;
	// rtList[RT_SRV].devBuad = 115200;
	// rtList[RT_SRV].devStopLen = 1;
	// rtList[RT_SRV].oddCheckEnable = FALSE;
    // rtList[RT_SRV].evenCheckEnable = FALSE;
	// rtList[RT_SRV].ptr_RtHandler = SrvRtHandler;
    // rtList[RT_SRV].ptr_Init = InitSrv;
	// rtList[RT_SRV].ptr_ChkFrameSum = ChkSrvFrame;


    // rtList[RT_P900].ckIndex = 0;//0		//链路  射后透传	MML目前没用
	// rtList[RT_P900].chIndex = 8;//2
	// rtList[RT_P900].devID = 0x01;
	// rtList[RT_P900].devBuad = 115200;
	// rtList[RT_P900].devStopLen = 1;
	// rtList[RT_P900].oddCheckEnable = FALSE;
    // rtList[RT_P900].evenCheckEnable = FALSE;
	// rtList[RT_P900].ptr_RtHandler = CmdHandler;
    // rtList[RT_P900].ptr_ChkFrameSum = ChkEngineFrame;
    
	for(int i=0;i<MODULE_COUNT;i++)
	{

		if(rtList[i].ptr_Init != PTR_NULL)
		{
			rtList[i].ptr_Init();//执行特殊协议初始化
		}
		else
		{
			InitBuffLoop(i);
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
void BusDataHandle()	// 1ms调用一次
{
	STRU_STANDARD_FRAME * frmPtr = NULL;
	//每一次进到本函数，都需要处理目前所有422接口收到的所有数据
	while ((frmPtr = (STRU_STANDARD_FRAME *)PeekStandardMessage()))//数据已经过校验
	{
		if(rtList[frmPtr->u8RtIndex].ptr_RtHandler != NULL)
			rtList[frmPtr->u8RtIndex].ptr_RtHandler((STRU_422_MSG_INFO *)&(frmPtr->pStand422Data));
	}

	// check if regular poll required
	for(int k=0;k<sizeof(rtList)/sizeof(rtList[0]);k++)
	{
		if((rtList[k].flags != 0)&&(rtList[k].ptr_RtHandler != NULL))
		{
			rtList[k].ptr_RtHandler(NULL);
		}
	}
/*
	STRU_CAN_MSG *msg = NULL;
	while ((msg = (STRU_CAN_MSG *)PeekCanMessage()))//数据已经过校验
	{
		if(rtCan[msg->CanIndex].ptr_RtHandler != NULL)
			rtCan[msg->CanIndex].ptr_RtHandler(msg);
	}*/
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
	UART_PutBuff((int)rtList[RT_HIL].chIndex, (OS_U8*)str, strLen);
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


