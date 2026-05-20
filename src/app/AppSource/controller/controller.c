/*
 * modBC.c
 *
 *  Created on: 2022年3月15日
 *      Author: Lenovo
 */
#include "stm32h7xx_hal_adc.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../StateMachine.h"
#include "../core/Telecontrol.h"
#include "../stmToZynq.h"
#include "../../pressure_sensor.h"
#include "../modules/modMEMS.h"
#include "../modules/modFlash.h"
#include "../modules/modEngine.h"
#include "../modules/modSD.h"
#include "./controller.h"
//#include "../flight/os_flight_io.h"
#include "../FlightSupport.h"
#include "../modules/modNav.h"
#include "../interface/interface_gpio.h"
#include "../support/common.h"
//#include "../comm/CommHandler.h"
#include "../payload/MsnTime.h"
#include "../interface/interface_power.h"
#include <math.h>
#include "fuse.h"

// /******************** Test umbrella Servo & ECU(PWM)***************************/
// #include "interface_timer.h"

static OS_U8 SelfCheckCollpse();
OS_U8 LunchDetective();
float Read_CPU_Temperature(void) ;
extern long calcTimeCpu0;
extern int sd_card_fault;
OS_U8 EngineStartCmd = 0;    //来自地面的控制参数， 1：启动发动机；0：停止发动机
float System_GetCoreTemperature();
extern void ReConnectUart();
OS_DOUBLE AirSpdHistory[150];
extern void *g_pControl;

OS_DOUBLE Average(OS_DOUBLE array[], int len)
{
	OS_DOUBLE all = 0;
	for(int i=0;i<len;i++)
	{
		all+=array[i];
	}

	return all/len;
}

OS_U8 CalcAirSpd()
{
    pressure_status_t pressData;
    pressure_get_status(&pressData);
	g_baro_data.static_pressure = pressData.abs_pressure;
	g_baro_data.total_pressure = pressData.abs_pressure + pressData.diff_pressure;

    float curAirSpdPa = fabs(fabs(pressData.diff_pressure));// - calibrationValue);
    float curPress = pressData.abs_pressure;
    float ru;
	float press_height= 44306.60f * (1.0f - powf(curPress*0.001 / 101.325f, (0.19026f)));
    uav_density(press_height, &ru);

	int airHistoryCount = sizeof(AirSpdHistory)/sizeof(OS_DOUBLE);
	for(int i = airHistoryCount - 1; i > 0; i--)
	{
		AirSpdHistory[i] = AirSpdHistory[i-1];
	}
	AirSpdHistory[0] = sqrt(1.225f / ru) * sqrt(2 * curAirSpdPa/ 1.225);
	float curAirSpd = Average(AirSpdHistory, airHistoryCount);
    
    SETDATA(pDataPoolSelf,	"AirPress",	curPress *0.1,	OS_U16);
    SETDATA(pDataPoolSelf,	"AirHigh",	press_height,	OS_S16);
    SETDATA(pDataPoolSelf,	"AirSpd",	curAirSpd * 10, OS_S16);
    
    return 0;
}

/***********************************************************
 * 函数名称:AutoLuanchProcess()
 * 函数功能: 设备上电后自动开始进入上电、自检、对准、准备发射等流程
 *			 流程完成后等待最终发射，其中过程无需任何人为干预
 * 作者:	成宏璟
 ***********************************************************/
OS_U8 AutoLuanchProcess()  // 5ms运行一次
{
	static int AutoStep = 0;//0
	if(AutoStep == 14)
	return 0;
	// if((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC)
	// 		return 0;    
	//EngineStartCmd = 1;
	//IgnitionMark = TRUE;
	//1.对各设备上电
	//5s时对惯组、导引头上电
	if(g_DeviceState.currTime > 5.0 && AutoStep == 0)
	{
		//判配电板合路供电电压，小于20伏时发送开启命令
		if(g_DeviceState.imuCountDown == 0)
		{
			static int powerSend = 0;
			if(powerSend == 0)
			{
				PowerOn(DEVICE_SCOUT_E28V);
				powerSend = 1;
			}
			else
			{
				AutoStep = 1;
			}
		}
		else
		{
			AutoStep = 1;
		}
	}
	//7s时对引信上电
	if(g_DeviceState.currTime > 7.0 && AutoStep == 1)
	{
		//引信供电无采集点，判断引信通信状态
		if(g_DeviceState.fuseCountDown == 0)
		{
			static int sendFlag = 0;
			if(sendFlag == 0)
			{
				// PowerOn(DEVICE_FUSE28V);	// 改在modNav.c Line 536
				PowerOn(DEVICE_FUSE_ISO5V);
				sendFlag = 1;
			}
			else{
				AutoStep = 2;
			}
		}
		else
		{
			AutoStep = 2;
		}
	}
	//9s时对伺服上电
	if(g_DeviceState.currTime > 9.0 && AutoStep == 2)
	{
		//伺服供电无采集点，判断伺服通信状态
		if(g_DeviceState.srvCountDown == 0)
		{
			static int powerSend1 = 0;
			if(powerSend1 == 0)
			{
					PowerOn(DEVICE_SRV_PWR28V);

// /******************** Test Servo (CAN & PWM)***************************/
// double angleStep = 10.0;

// for(double angle = -30.0; angle <= 30.0; angle += angleStep) {
// 	ServoCtlOnce_6Rudder(angle, angle, angle, angle, angle, angle);
// 	tx_thread_sleep(1000);
// }
// for(double angle = 30.0; angle >= -30.0; angle -= angleStep) {
// 	ServoCtlOnce_6Rudder(angle, angle, angle, angle, angle, angle);
// 	tx_thread_sleep(1000);
// }
// /******************** Test umbrella Servo (PWM)***************************/
// AngleServo_SetAngle(SERVO_PWM7, 50.0f);
// /******************** Test ECU(PWM)***************************/
// PulseServo_Init(ECU_PWM8, 1.0f);
// PulseServo_SetPulseWidth(ECU_PWM8, 2.0f);
// /*****************************************************/

				powerSend1 = 1;
			}
			else
			{
					AutoStep = 3;
			}
		}
		else
		{
			AutoStep = 3;
		}
	}
	//11s时对发动机上电
	if(g_DeviceState.currTime > 11.0 && AutoStep == 3)
	{
		//不判断供电，直接判断发动机通信
		if(g_DeviceState.ecuCountDown == 0)
		{
			static int powerSend2 = 0;
			if(powerSend2 == 0)
			{
					// PowerOn(DEVICE_BATT_ENGINE);	// 发动机
					// PowerOn(DEVICE_BATT_BATT2);
					powerSend2 = 1;
			}
			else
			{
					AutoStep = 4;
			}
		}
		else
		{
			AutoStep = 4;
		}
	}
    //2.等待地面加载任务信息
	if(AutoStep == 4)
	{
		OS_U8 msnID;
		GetDataFast(pDataPoolMsn, "msnDevID", &msnID);

		if(g_DeviceStatus.msgFromGCS != 0x00)
		{
			InitSD();
		}

		if(msnID != 0xFF)
		{
			// InitPwrSeq();	//MML 开火工品4
			// InitSD();
			AutoStep = 5;
		}
	}
	//3.惯组对准
	if(AutoStep == 5)
	{
		//启动对准
		STRU_422_MSG_INFO msg;
		msg.u8MsgID = CMD_NAV_INIT;
		NavCmdHandler(&msg);
		msg.u8MsgID = CMD_HOR_CALC_REQ;
		NavCmdHandler(&msg);
		ImuCmdHandler(&msg); //TODO: IMU 014这个应该是没有了。
		AutoStep = 6;
	}
	//4.判断对准完成，完成后转导航
	if(AutoStep == 6)
	{
    //AutoStep = 7;
		//对准完成
		//ToImu();
		OS_U8 imuStatus, navStatus;
		GetDataFast(pDataPoolImu, "navState", &imuStatus);
		GetDataFast(pDataPoolNav, "navState", &navStatus);
		//中科导控自己转导航，所以判断导航状态
		//if((imuStatus == 0x3F)&&(navStatus == 0x3F))
		// if(imuStatus == 0x3F)
		// {
		// 	STRU_422_MSG_INFO msg;
		// 	msg.u8MsgID = CMD_TO_NAV_REQ;
		// 	//NavCmdHandler(&msg);
		// 	ImuCmdHandler(&msg);
		// 	AutoStep = 7;
		// }
		if(navStatus == 0x3F)
		{
			// 自动转导航
			STRU_422_MSG_INFO msg;
			msg.u8MsgID = CMD_TO_NAV_REQ;
			NavCmdHandler(&msg);
			AutoStep = 7;
		}
	}
	//5.星历装订(空缺)/ 自动转射后
	if(AutoStep == 7)
	{
		// AutoStep = 8;
		OS_U8 navStatus;
		GetDataFast(pDataPoolNav, "navState", &navStatus);
		if(navStatus == 0x64)
		{
			// 自动转射后
			STRU_422_MSG_INFO msg;
			msg.u8MsgID = CMD_TO_AFTER_LUANCH;
			NavCmdHandler(&msg);
			AutoStep = 8;
		}
	}
	//6.等待发动机启动指令发出
	if(AutoStep == 8)
	{
		// 初始化控制代码
		g_pControl = ControlInitial();

		if(EngineStartCmd == 1)
		{
			AutoStep = 9;
		}
	}
	//6.发动机启动
	if(AutoStep == 9)
	{
		//AutoStep = 10; //test
		//return 0;
		if(EngineStartCmd == 0)
		{
			//用户手动又发送了停机指令
			StopEngine();
			AutoStep = 8;
		}
		else
		{
			if(g_DeviceState.CurrTick % 4 == 0)// 每20ms，进if
			{
				//先判断发动机是否已进入运行状态
				// struct EngineStatus engineStatus = {0};
				// modECU_GetEngineStatus(&engineStatus);
				// switch(engineStatus.CntState)//0停机，1启动中，2散热 3故障 4脱机 5运行（这肯定是协议里的）
				OS_U8 cntState;
				GetDataFast(pDataPoolSelf, "ecuState", &cntState);
				switch(cntState)//0停机，1启动中，2散热 3故障 4脱机 5运行（这肯定是协议里的）
				{
					case ENGINE_STOPED://发送启动指令
							StartEngine();//应该是启动流程
							break;
					case ENGINE_WARMUP://等待启动
							break;
					case ENGINE_SHUTTING_DOWN:
							break;
					case ENGINE_RUNNING:
							AutoStep = 10;
							break;
				}
			}
		}
	}
	//7.自动进入预发射
	if(AutoStep == 10)
	{
    g_DeviceState.luanchStart = 1;
		SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);//预发射完成，等待发射解锁指令
		AutoStep = 11;
	}
	//8.预发射完成，等待地面解锁指令
	if(AutoStep == 11)
	{
		if(EngineStartCmd == 0)
		{
			//用户手动又发送了停机指令         
			StopEngine();
			AutoStep = 8;
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);
		}
		if(IgnitionMark == TRUE)//地面发送全部解锁指令
		{
			AutoStep = 12;
		}
	}
	//9.地面解锁已经完成，等待火箭激发
	if(AutoStep == 12)
	{
		if(EngineStartCmd == 0)
		{
			//用户手动又发送了停机指令
			StopEngine();
			AutoStep = 8;
			IgnitionMark = FALSE;
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);
		}
		OS_U8 startFly = 0;
		GetDataFast(pDataPoolSelf,	"startFly",	&startFly);
		if(startFly == 1)//DoIgnition函数将startFly置1，地面 发控首页 - 起飞
		{
			FuseSend(ARM_I);	//已经起飞了，发送一级解保指令（条件2g过载）
			// 判断第8字节为0x40
			OS_U8 fzFeedbk;
			GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
			if(fzFeedbk & 0x40)
			{
				AutoStep = 13;
			}
		}
	}    
	//10.起飞之后，飞出2km后给引信的引爆电源上电
	if(AutoStep == 13)
	{
		static uint8_t FzOnFlag = 0;
		if (0 == FzOnFlag)
		{
			OS_S32 launchLon,launchLat,curLon,curLat;
			OS_S16 launchHigh;
			OS_FLOAT curHigh;
		
			GetDataFast(pDataPoolFly, "DataLon", &launchLon);//起飞位置的经纬高
			GetDataFast(pDataPoolFly, "DataLat", &launchLat);//
			GetDataFast(pDataPoolFly, "DataHigh", &launchHigh);//

			launchLon = launchLon * 1e-7;
			launchLat = launchLat * 1e-7;

			GetDataFast(pDataPoolImu, "navLon", &curLon);//当前位置的经纬高
			GetDataFast(pDataPoolImu, "navLat", &curLat);//
			GetDataFast(pDataPoolImu, "navHigh", &curHigh);//

			curLon = curLon * 1e-7;
			curLat = curLat * 1e-7;

double lat_m = (curLat - launchLat) * 111320.0;

// 经度方向：需要乘以 cos(纬度)
double mid_lat_rad = (launchLat + curLat) / 2.0 * 3.1415926535 / 180.0;
double lon_m = (curLon - launchLon) * 111320.0 * cos(mid_lat_rad);

double h_m = curHigh - launchHigh;

double dist = sqrt(lat_m*lat_m + lon_m*lon_m);

			if((dist > 2000)&&(h_m > 200))
			{
				FuseSend(ARM_II);// 发送二级解保

				OS_U8 fzFeedbk;
				GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
				if(fzFeedbk & 0xC0)
				{
					PowerOn(DEVICE_FUSE28V);
					g_DeviceStatus.FzOnStamp_s = GetCurTime();	// tx_time_get();
					FzOnFlag = 1;
				}
			}
		}
		else
		{
			// 引信-引爆电源已经上电
			// 延时2s
			if((GetCurTime() - g_DeviceStatus.FzOnStamp_s) > 2)
			{
				// 判断距离目标是否<2km	
				OS_DOUBLE deltaR;
				GetDataFast(pDataPoolFly, "deltaR", &deltaR);//弹目距离
				if(deltaR != 0)
				{
					if(deltaR < 2000)
					{
						FuseSend(ARM_III);//发送三级解保

						// 判断第9字节为0x80
						OS_U8 fzTask;
						GetDataFast(pDataPoolSelf, "fzTask", &fzTask);
						if(fzTask & 0x80)
						{
							AutoStep = 14;
						}
					}
				}
			}
		}
	}
	SETDATA(pDataPoolMsn,	"autoStep",	AutoStep,	OS_U8);	
	//判断导引头是否连接，如果连接则类型为0b11 = 3，如果未连接类型为0b10 = 2
	if(g_DeviceState.scoutCountDown != 0)
	{
		SETDATA(pDataPoolMsn,	"paylodtp",	3,	OS_U8);
	}
	else
	{
		SETDATA(pDataPoolMsn,	"paylodtp",	2,	OS_U8);
	}
  return 0;
}



/***********************************************************
 * 函数名称:ControllerStatusUpdata()
 * 函数功能: 智能控制器5ms定时器，所有需要智能控制器按时间触发的行为均在此执行。包括：
 * 	1.自检数据收集，并存数据池入遥测
 * 	2.发射前给地面发送配电信息报告
 * 作者:	成宏璟
 ***********************************************************/
OS_U8 ControllerStatusUpdata()	// 5ms运行一次
{
	//判断起飞条件，条件满足就起飞；检查各外设通信状态
	SelfCheckCollpse();		//自检配置和回报	
	CalcAirSpd();		// 计算空速
	ReConnectUart();	// 重连串口
	return 0;
}


/***********************************************************
 * 函数名称:SelfCheckCollpse()
 * 函数功能: 智能控制器各设备状态收集，如果有地面请求自检信息，则组织自检信息下传
 * 作者:	成宏璟
 ***********************************************************/
static OS_U8 SelfCheckCollpse()
{
    LunchDetective();
	//北京时间
	//SETDATA(pDataPoolSelf,	"BJTime",	g_DeviceState.BJTimeSecond,	OS_U32);

	//核心温度
	float t = System_GetCoreTemperature();
	g_DeviceState.temperature = t;

	//收集各设备通信状态
	g_DeviceState.hilCountDown = g_DeviceState.hilCountDown > 0?g_DeviceState.hilCountDown-1:0;
	g_DeviceState.ecuCountDown  = g_DeviceState.ecuCountDown > 0?g_DeviceState.ecuCountDown-1:0;
	g_DeviceState.battCountDown  = g_DeviceState.battCountDown > 0?g_DeviceState.battCountDown-1:0;
    g_DeviceState.powerCountDown  = g_DeviceState.powerCountDown > 0?g_DeviceState.powerCountDown-1:0;
	g_DeviceState.navCountDown  = g_DeviceState.navCountDown > 0?g_DeviceState.navCountDown - 1:0;
	g_DeviceState.srvCountDown  = g_DeviceState.srvCountDown > 0?g_DeviceState.srvCountDown-1:0;
    //g_DeviceState.fuseCountDown = g_DeviceState.fuseCountDown > 0?g_DeviceState.fuseCountDown-1:0;
	g_DeviceState.imuCountDown = g_DeviceState.imuCountDown > 0?g_DeviceState.imuCountDown-1:0;
	g_DeviceState.scoutCountDown = g_DeviceState.scoutCountDown >0? g_DeviceState.scoutCountDown-1:0;

	SETDATA(pDataPoolSelf,	"commHil",	g_DeviceState.hilCountDown,		OS_U8);
	SETDATA(pDataPoolSelf,	"commBatt",	g_DeviceState.battCountDown,	OS_U8);
  SETDATA(pDataPoolSelf,	"commPwr",	g_DeviceState.powerCountDown,	OS_U8);
	SETDATA(pDataPoolSelf,	"commEcu",	g_DeviceState.ecuCountDown,		OS_U8);
	SETDATA(pDataPoolSelf,	"commNav",	g_DeviceState.navCountDown,		OS_U8);

	SETDATA(pDataPoolSelf,	"commSrv",	g_DeviceState.srvCountDown,		OS_U8);
	SETDATA(pDataPoolSelf,	"commImu",	g_DeviceState.imuCountDown,		OS_U8);
  SETDATA(pDataPoolSelf,	"commFuse",	g_DeviceState.fuseCountDown,	OS_U8);
	SETDATA(pDataPoolSelf,	"commScot",	g_DeviceState.scoutCountDown,	OS_U8);

	SETDATA(pDataPoolSelf,	"cpuTemp",	g_DeviceState.temperature * 100,		OS_S16);
	SETDATA(pDataPoolSelf,	"selfMode",	g_DeviceState.workStage,		OS_U8);

	OS_U8 sdState;
	if(SD_Enable == FALSE)
	{
			sdState = 0xFF;
	}
	else
	{
			sdState = sd_card_fault==0?1:0xEE;
	}

	SETDATA(pDataPoolSelf,	"sdState",	sdState,	OS_U8);

	return 0;
}

OS_U8 InitReportParam()
{
	SETDATA(pDataPoolNav,	"imuFocus",	 0,		OS_U8);
	SETDATA(pDataPoolSelf,	"tcCmd",	 0xAA,		OS_U8);
    
    SETDATA(pDataPoolSelf,	"flyError",	 0xFF,		OS_U8);

	return 0;
}

/***********************************************************
 * 函数名称:BCCmdHandler()
 * 函数功能: 需要智能控制器处理的指令内容
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame)	// 数据链或仿真过来的指令
{
	OS_U8 msgId = frame->u8MsgID;
	switch(msgId)
	{
	case CMD_BJTIME_SET:
	{
		OS_U16 year = *(OS_U16*)(frame->au8Data);
		OS_U8 month = *(frame->au8Data + 2);
		OS_U8 day = *(frame->au8Data + 3);
		OS_U8 hour = *(frame->au8Data + 4);
		OS_U8 minite = *(frame->au8Data + 5);
		OS_U8 second = *(frame->au8Data + 6);
		OS_U16 microSecond = *(OS_U16*)(frame->au8Data + 7);
		g_DeviceState.BJTimeSecond = SetSecondByDate(year, month, day, hour, minite, second);
		g_DeviceState.BJTimeMS = microSecond;
		break;
	}
	/*
	case CMD_DATA_SET:
	{
		//发送给飞控
		FLIGHT_CMD cmd;
		cmd.cmdType = 0;//参数上传
		cmd.paramID = frame->au8Data[0];
		cmd.paramValue = *(OS_FLOAT *)(&frame->au8Data[1]);
		SETDATA(pDataPoolSelf, "tcCmd", cmd.paramID,	OS_U8);
		FlightControlCmd(cmd);
		break;
	}*/
	case CMD_URGENT_LAND:	// 0x22 摄像头视频 - 紧急伞降
	{
		SETDATA(pDataPoolSelf,  "flyError", 1,	OS_U8);	//地面让紧急伞降
		SETDATA(pDataPoolSelf, "tcCmd", 0xC0,	OS_U8);
		DoOpenUm();	//紧急伞降开伞
		break;
	}
	case CMD_URGENT_RETURN:	// 0x23 摄像头视频 - 紧急返航
	{
		SETDATA(pDataPoolSelf,  "flyError", 2,	OS_U8);	//地面让紧急返航
		SETDATA(pDataPoolSelf, "tcCmd", 0xC1,	OS_U8);
		DoReturnHomeward();
		break;
	}
	case CMD_ENGINE_START:	// 0xF6 首页 - 发动机启动
	{
		EngineStartCmd = 1;
		break;
	}
	case CMD_ENGINE_STOP:	// 0xF7 首页 - 发动机停机
	{
		EngineStartCmd = 0;
		break;
	}
	case CMD_FORE_LAUNCH_REQ:	//预发射（未使用）
	{
		if(0x11==frame->au8Data[0])
		{
			g_DeviceState.luanchStart = 1;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);
		}
		else
		{
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0,	OS_U8);
			IgnitionMark = FALSE;//发射后拖插未分离前也可以退出预发射以中止发射流程
		}
	}
		break;
	case CMD_LAUNCH_REQ:	//0xFA 首页 - 全部解锁
	{
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			if(g_DeviceState.luanchStart == 1)
				Ignition();
		}
	}
		break;
	case CMD_LUANCH_FORCE:	// 0xFB 首页 - 起飞
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
			{
				MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
				SETDATA(pDataPoolSelf,	"luanMode",	2,	OS_U8); //起飞模式地面点击起飞的方式 0 单发 1 连发 2 齐发
				IgnitionMark = FALSE;
				DoIgnition();
			}
		}
		break;
	}
	return 0;
}

FLIGHT_SEQ flightSeq = {0};
OS_U8 SeqCalc()
{
	OS_U32 flightTick = g_DeviceState.CurrTick;
	OS_DOUBLE flightTime = flightTick * 0.005;
	if((DOM_AUTOMATIC & g_DeviceState.workStage) != DOM_AUTOMATIC)
	{
		return -1;
	}
	if(flightTime >= 0.5)// ****************MML： 时间 > 500ms ****************
	{
		flightSeq.luanched = 1;
	}
	else
	{
		flightSeq.luanched = 0;
	}
	return 0;
}


OS_U8 DoIgnition()// 1、地面软件点击“起飞”（测试用）/ 2、过载（主条件）起飞/ 3、速度起飞（副条件）/ 4、仿真起飞（测试用）
{
	MsgToNAV(BUS_NAV_IGNATION,PTR_NULL,0);

	g_DeviceState.workStage &= ~((unsigned int)DOM_INTERACTIVE);
	g_DeviceState.workStage |= DOM_AUTOMATIC;
	g_DeviceState.flightStartTime = GetCurTime();
	g_DeviceState.CurrTick = 0;

	SETDATA(pDataPoolSelf,	"startFly",	1,	OS_U8);
	SETDATA(pDataPoolSelf,	"luncTime",	g_DeviceState.BJTimeSecond,	OS_U32);
	SETDATA(pDataPoolSelf,  "flyError", 0,	OS_U8);//正常飞行   
	return 0;
}

OS_U8 IgnitionMark = FALSE;  // 地面全部解锁按钮点击标志

/***********************************************************
 * 函数名称:Ignition()
 * 函数功能: 发射指令，可被数据链、地面调用。接收到指令后设置整箭状态，重新初始化飞控算法
 * 输入，清除舵控积累数据
 * 作者:	成宏璟
 ***********************************************************/
OS_U8 Ignition()
{
	IgnitionMark = TRUE;
	SETDATA(pDataPoolSelf,	"RecvLunc",	1,	OS_U8);
	g_DeviceState.luanchStart = 1;

	MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
	//测试阶段，直接发射
	//DoIgnition();
	return 0;
}

OS_U8 LunchDetective()	// 判断起飞条件，起飞
{
	if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
	{
		OS_FLOAT fAx;
		GetDataFast(pDataPoolImu, "imuAx", &fAx);//
        
		OS_S16 vn,ve;
		GetDataFast(pDataPoolImu, "navVe", &ve);//
		GetDataFast(pDataPoolImu, "navVn", &vn);//
		OS_U8 navState;
		GetDataFast(pDataPoolImu, "navState", &navState);//
		float fvn,fve;
		fvn = vn * 0.01;
		fve = ve * 0.01;
		float v = sqrt(pow(fvn,2) + pow(fve,2));
        
		static OS_U8 detachCount = 1;
		if(fabs(fAx) > 30.0)
		{
			if(detachCount == 5)
			{
				DoIgnition();
				SETDATA(pDataPoolSelf,	"luanMode",	1,	OS_U8); //起飞模式1 加速度大于30判断的起飞方式
				IgnitionMark = FALSE;
			}
			else
			{
				detachCount++;
			}
		}
		else
		{
			if(v > 10.0 && navState == 0x60)
			{
				if(detachCount == 5)
				{
						DoIgnition();
						SETDATA(pDataPoolSelf,	"luanMode",	3,	OS_U8); //起飞模式组合导航状态和速度大于10
						IgnitionMark = FALSE;
				}
				else
				{
						detachCount++;
				}
			}
			else
			{
					detachCount = 1;
			}
		}
	}
	return 0;
}

ADC_HandleTypeDef hadc3;
extern void Error_Handler(void);
void ADC3_Init(void)
{
    /* USER CODE BEGIN ADC3_Init 0 */

    /* USER CODE END ADC3_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC3_Init 1 */

    /* USER CODE END ADC3_Init 1 */

    /** Common config
    */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode = DISABLE;
    hadc3.Init.Oversampling.Ratio = 1;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }
    hadc3.Init.Resolution = ADC_RESOLUTION_16B;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
		sConfig.OffsetRightShift = DISABLE;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

float System_GetCoreTemperature()
{
    float temperature; 
    uint16_t ts_cal1 =  *(__IO uint16_t *)(0x1FF1E820UL);
    uint16_t ts_cal2 =  *(__IO uint16_t *)(0x1FF1E840UL);	
    
    HAL_ADC_Start(&hadc3); 

    HAL_ADC_PollForConversion(&hadc3, 10);  

    uint32_t adc_value = HAL_ADC_GetValue(&hadc3);
    int a = adc_value - ts_cal1;
    int b = ts_cal2 - ts_cal1;
    
    temperature =(((a)*80.0) / (b)) + 30.0f;
    
    return temperature;
}