/*
 * modSrvCtl.c
 *
 *  Created on: 2021年10月16日
 *      Author: QL
 */

#include "modSrvCtl.h"
#include "modPwrSeqCtl.h"
#include "../FlightSupport.h"
#include "./modEngine.h"
#include "../interface/interface_power.h"
#include "../interface/interface_gpio.h"
#include "../interface/interface_can.h"
#include "../interface/interface_uart.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../flight/os_flight_io.h"
#include "../support/os_bufferLoop.h"

#include <math.h>
#include <string.h>
#ifndef PI
#define PI		(3.141592653589793)           //3.1415926535
#endif
#define d2r		(57.29577951308402)

int cycleCount;
double toRad(double deg)
{
	return deg / d2r;
}

double toDeg(double rad)
{
	return rad * d2r;
}
STRU_SERVO_MinLoopTest_REQUEST g_stServoMinLoopTestReq={0};

static FREQ_SCAN fs = {
		0.005,	// tickStep
		0.25,	// freqs
		{1,			1,		1,		1,1,1,1,1},	//eable
		5, // zeroAngle
		10 // ampAngle
	};

OS_U8 AutoZero();
    
OS_U8 InitSrv()	//TODO: MML舵机
{
    buffLoop[RT_SRV].syncHead_A = 0xEE;
	buffLoop[RT_SRV].syncHead_B = 0x00;
	buffLoop[RT_SRV].lenExtern = 6;	
	buffLoop[RT_SRV].head = 0;
	buffLoop[RT_SRV].tail = 0;
	buffLoop[RT_SRV].lenPos = 1;	
	buffLoop[RT_SRV].fixedLen = 48;
	buffLoop[RT_SRV].inited = TRUE;
    return 0;
}

OS_U16 ChkSrvFrame(OS_MEM* pmData)	//TODO: MML舵机
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != 0xEE || pmData[2] != 0x00|| pmData[3] != 0xAA )
	{
		return 0;
	}
    OS_U8 temp_crc[2];
    temp_crc[0] = pmData[48];
    temp_crc[1] = pmData[47];
   
    OS_U16 crcValue;
    memcpy(&crcValue, temp_crc, 2);
   
    OS_U16 t = crc16_ccitt(pmData + 1, 46);
    if(t != crcValue)
    {
        return 0;
    }
	pmData[3] = pmData[5];
	pmData[4] = 0;
	pmData[5] = 0;
	pmData[6] = 0x11;
	return pmData[3] + 8;
}

/***********************************************************
 * 函数名称: MsgToSrv()
 * 函数功能: 伺服额外要求指令计数及发送帧计数，因此剥离一层控制数据，再发送
 * 作者:	未知
 ***********************************************************/
OS_U8 MsgToSrv(OS_DOUBLE ctrlDeg[6], OS_U8 ctrlMode/*control = 0x02, 0x44=setZero*/)	//TODO: MML舵机
{
    SETDATA(pDataPoolSrv, "Sr1Cmd", ctrlDeg[0] * 100, OS_S16); 
	SETDATA(pDataPoolSrv, "Sr2Cmd", ctrlDeg[1] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr3Cmd", ctrlDeg[2] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr4Cmd", ctrlDeg[3] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr5Cmd", ctrlDeg[4] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr6Cmd", ctrlDeg[5] * 100, OS_S16);
    OS_U8 sendData[21];
    sendData[0] = 0xAA;
    sendData[1] = 0x00;
    sendData[2] = 0xEE;
    sendData[3] = 0x00;
    sendData[4] = 0x0E;
    sendData[5] = 0x01;
    for(int i=0;i<6;i++)
    {
        OS_S16 tempDeg = ctrlDeg[i] * 1000;
        memcpy(sendData + 6 + i*2, &tempDeg, 2);
    }
    sendData[18] = ctrlMode;
    OS_U16 t = crc16_ccitt(sendData, 19);
    sendData[19] = (t >> 8) & 0xFF;
    sendData[20] = t & 0xFF;
    UART_PutBuff(rtList[RT_SRV].chIndex, sendData, sizeof(sendData));
	return 0;
}

/***********************************************************
 * 函数名称: ServoCtlOnce_3Rudder()
 * 函数功能: 单次计算及伺服控制，由飞控代码调用，传入为3个舵的偏转弧度
 * 作者:	未知
 ***********************************************************/

OS_U8 ServoCtlOnce_6Rudder(double actDeg_1, double actDeg_2, double actDeg_3, 
    double actDeg_4, double actDeg_5, double actDeg_6)	//TODO: MML舵机
{
    double deg[6];
    deg[0] = actDeg_1; 
    deg[1] = actDeg_2; 
    deg[2] = actDeg_3; 
    deg[3] = actDeg_4; 
    deg[4] = actDeg_5; 
    deg[5] = actDeg_6; 
	MsgToSrv(deg, 0x02 );
	return 0;
}

/***********************************************************
 * 函数名称: InsertServoTestData()
 * 函数功能: 伺服小回路测试专用函数。小回路测试前，地面会通过小回路测试指令上传扫频相关参数，见FREQ_SCAN结构
 * 			并修改状态机状态为伺服小回路模式。在该模式下，飞控代码会每5ms（由状态机触发的定时器调用飞控代码）
 * 			调用本函数，计算出每次的线控舵偏长度，通过指针返回。
 * 作者:	成宏璟
 ***********************************************************/
//该函数5ms调用一次，实现扫频
int InsertServoTestData(
	double* actuator_I,
	double* actuator_II,
	double* actuator_III,double* actuator_IV,double* actuator_V,double* actuator_VI)
{
	static unsigned int tickOffset=0;
	//static unsigned int freqIndex = 0;
	/*算法公式计算过程，屏蔽公式推导过程
	double k = freq * 2*PI;
	double x = tickOffset*fs.tickStep;
	double b = 0;
	double x0 = kx;
	double maxX = 2*PI;		//正弦一轮是2pi
	 */
	//扫频频率最低0.1，低于0.1不处理。最大15
	if(fs.freq>0.049 && fs.freq < 15.001 && tickOffset*fs.tickStep *fs.freq <= cycleCount)
			//&& tickOffset*fs.tickStep*fs.freqs[freqIndex] <= fs.scanCnt[freqIndex])
	{

		*actuator_I 	 = fs.enable[0]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_II	 = fs.enable[1]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_III    = fs.enable[2]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
        *actuator_IV 	 = fs.enable[3]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_V	     = fs.enable[4]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_VI     = fs.enable[5]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		++tickOffset;
	}
	else
	{
		tickOffset=0;
		//测试结束，关闭状态机
		g_DeviceState.workStage &=  ~((unsigned int)DOM_SIMSRVDAT);
		return -1;
	}
	return 0;
}

OS_U16 waitForStart = 0;
OS_U8 MiniLoopSimulation()	//TODO: MML舵机
{
	if((g_DeviceState.workStage & DOM_SIMSRVDAT) != DOM_SIMSRVDAT)
		return -1;
	if(waitForStart > 0)
	{
		waitForStart--;
		return 0;
	}
	double a1=0,a2=0,a3=0,a4=0,a5=0,a6=0;
	//short mm1,mm2,mm3,mm4;,&mm1,&mm2,&mm3,&mm4
	InsertServoTestData(&a1, &a2, &a3, &a4, &a5, &a6);

	ServoCtlOnce_6Rudder(a1,a2,a3,a4,a5,a6);
	return 0;
}

OS_U8 SaveSrvInDataPool(STRU_SRV_INFO *data)	//TODO: MML舵机
{
    SETDATA(pDataPoolSrv, "Sr1Read", data->srv1Read / 10,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr2Read", data->srv2Read / 10,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr3Read", data->srv3Read / 10,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr4Read", data->srv4Read / 10,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr5Read", data->srv5Read / 10,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr6Read", data->srv6Read / 10,	OS_S16);
    
    SETDATA(pDataPoolSrv, "Sr1A", data->srv1A,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr2A", data->srv2A,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr3A", data->srv3A,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr4A", data->srv4A,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr5A", data->srv5A,	OS_S16);
	SETDATA(pDataPoolSrv, "Sr6A", data->srv6A,	OS_S16);
    return 0;
}


/***********************************************************
 * 函数名称: ServoRtHandler()
 * 函数功能: 伺服总线处理函数，伺服的设定是不会定时给智能控制器发送内容的，仅在收到智能控制器的指令后回复。测发控阶段，均需下传地面
 * 			本型号智能控制器接收伺服发出的指令:
 * 			1.伺服自检回复0x11		: 地面已取消自检命令，因此也收不到这个回复了，协议中规定的格式与控制回复相同。
 * 			2.伺服控制回复0x22		: 存数据池，伺服的绝大多数内容来自控制回复，包括地面控制指令、小回路指令、发射后的控制指令。
 * 			3.伺服调零回复0x23		: 存数据池
 * 参考资料: <TXII-Y1 422箭上通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 SrvRtHandler(STRU_422_MSG_INFO * srvMsg)	//TODO: MML舵机
{
	//首先判断是什么类型的指令
	//对数据区前四字节进行判断
	RECV_CMD_ID recvCmdId = srvMsg->u8MsgID;
	//根据反馈指令做处理
	switch(recvCmdId)
	{
		case 0x11:
            SaveSrvInDataPool((STRU_SRV_INFO *)srvMsg->au8Data);
			break;
	
		default:
			break;
	}
    g_DeviceState.srvCountDown = 200;
	return 0;
}

OS_U8 StartMiniLoop(float freq, float amp, float zero, OS_U8 enable[6])
{
	fs.freq = freq;
	fs.ampAngle = amp;
	fs.zeroAngle = zero;
	memcpy(fs.enable, enable, 6*sizeof(OS_U8));
	//开启扫频
	g_DeviceState.workStage |= DOM_SIMSRVDAT;
	return 0;
}
/***********************************************************
 * 函数名称: ServoCmdHandler()
 * 函数功能: 伺服指令处理函数，本型号智能控制器接收地面发出的指令:
 * 			1.伺服测量请求0x42: 对伺服进行控制，指定伺服移动固定位置，直接转发至伺服驱动器。回复地面不由本函数控制
 * 			2.伺服零位装订0x40: 对伺服进行调零，接转发至伺服驱动器。回复地面不由本函数控制
 * 			3.小回路测试0x44	: 小回路测试有专门的状态机状态。接收指令后开始小回路测试，并立即返回（执行中0x11)，当测试完成后再
 * 							: 由其它函数返回（执行完成0x22）
 * 参考资料: <TXII-Y1 箭地通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ServoCmdHandler(STRU_422_MSG_INFO * frame)	//TODO: MML舵机
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case CMD_SRV_CTRL_REQ://0x42 伺服控制请求
	{
		OS_S16 ctrlDegi[6];
        OS_DOUBLE ctrlDeg[6];
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * 6);
		for(int i=0;i<6;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x02);
	}
		break;
    case CMD_SRV_BOOKMODE:
    {
		OS_S16 ctrlDegi[6];
        OS_DOUBLE ctrlDeg[6];
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * 6);
		for(int i=0;i<6;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x44);
	}
		break;
	case CMD_SRV_MINLOOP_REQ://0x44 伺服小回路
		{
			STRU_SERVO_MinLoopTest_REQUEST* srvPtr = (STRU_SERVO_MinLoopTest_REQUEST*)frame->au8Data;
			//设置扫频参数
			OS_U8 enable[6];
			for(int i=0;i<6;i++)
			{
				enable[i] = ((srvPtr->LevelOneAcuator_Enb >> i) & 1);
			}
			waitForStart = 0;
			cycleCount = srvPtr->count;
			StartMiniLoop(
					srvPtr->Freq * 0.01,
					srvPtr->Amp * 0.1f,
					0,
					enable
					);
			//开启伺服小回路模式后，由飞控软件计算并向伺服发送数据，此处不再转发伺服
		}
		break;
	}
	return 0;
}

OS_U8 SrvStatusUpdata()	//TODO: MML舵机
{
    
	MiniLoopSimulation();	//伺服小回路
	//伺服位置查询，每轮循环只查询一个伺服，以尽量避免can冲突
	/*int srvIndex = g_DeviceState.CurrTick % 4;
	//if(g_DeviceState.CurrTick % 100 == 0)
	MsgToSrv(srvIndex + 1, CMD_GET_CTRL , 0);
	//
	//DoSrvProtect();
	//伺服零位装订检测
	if(SrvEncp > 0)
	{
		SrvEncp--;
		if(SrvEncp <4*10 + 1)
		{
			for(int i=1;i<=4;i++)
			{
				if(SrvEncp / 10 == (i) && SrvEncp % 10 == 0)
				MsgToSrv(i, CMD_SET_ZERO, 0);
			}
		}
	}
	if(g_DeviceState.CurrTick % 10 == 0)
	{
		AutoZero();
	}

	
    */
	return 0;
}

OS_U8 DoSrvProtect()	//TODO: MML舵机
{
	if(SrvProtect > 0)
	{
		 double deg[6] = {0};
		MsgToSrv(deg, 0x02);
	}
	return 0;
}

OS_U16 AutoZeroCount = 0;

OS_U8 AutoZero()	//TODO: MML舵机
{
	if(BookingMode == 1)
		return 1;
	if(AutoZeroCount > 0)
	{
        double deg[6] = {0};
		MsgToSrv(deg, 0x02);
		AutoZeroCount--;
	}
	return 0;
}

