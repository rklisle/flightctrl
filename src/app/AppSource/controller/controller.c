/*
 * modBC.c
 *
 *  Created on: 2022锟斤拷3锟斤拷15锟斤拷
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
#include "../modules/modHil.h"
#include "../interface/interface_gpio.h"
#include "../Interface/interface_timer.h"
#include "../support/common.h"
//#include "../comm/CommHandler.h"
#include "../payload/MsnTime.h"
#include "flightPort.h"
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
OS_U8 EngineStartCmd = 0;    //鏉ヨ嚜鍦伴潰鐨勬帶鍒跺弬鏁帮紝 1锛氬惎鍔ㄥ彂鍔ㄦ満锛?0锛氬仠姝㈠彂鍔ㄦ満
float System_GetCoreTemperature();
extern void ReConnectUart();
extern void *g_pControl;
OS_DOUBLE AirSpdHistory[150];

#define LAUNCH_AUTO_STEP_DONE        16
#define LAUNCH_RPM_START_MIN         2300u
#define LAUNCH_RPM_THROTTLE_MIN      3400u
#define LAUNCH_RPM_START_HOLD_MS     10000u
#define LAUNCH_RPM_THROTTLE_HOLD_MS  5000u
#define LAUNCH_PRE_THROTTLE_PCT      30.0f

OS_DOUBLE Average(OS_DOUBLE array[], int len)
{
	OS_DOUBLE all = 0;
	for(int i=0;i<len;i++)
	{
		all+=array[i];
	}

	return all/len;
}
float static_pressure,total_pressure;
OS_U8 CalcAirSpd()
{
    pressure_status_t pressData;
    pressure_get_status(&pressData);
	// g_baro_data.static_pressure = pressData.abs_pressure;
	// g_baro_data.total_pressure = pressData.abs_pressure + pressData.diff_pressure;
	static_pressure = pressData.abs_pressure;
	total_pressure = pressData.abs_pressure + pressData.diff_pressure;

    float curAirSpdPa = fabs(pressData.diff_pressure);
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
    
    SETDATA(pDataPoolSelf,	"AirPress",	curPress*0.1,	OS_U16);//?锟斤拷
    SETDATA(pDataPoolSelf,	"AirHigh",	press_height*10,	OS_S16);//锟斤拷?锟???锟??
    SETDATA(pDataPoolSelf,	"AirSpd",	curAirSpd*10, OS_S16);//锟斤拷?锟斤拷锟斤拷
    
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
	static int AutoStep = 0;
	static int powerSend = 0;
	static int sendFlag = 0;
	static int powerSend1 = 0;
	static int powerSend2 = 0;
	OS_U8 msnID;
	STRU_422_MSG_INFO msg;
	OS_U8 navStatus;
	OS_U8 curState;
	uint16_t rpm;
	static bool ThrFlag = true;
	static uint32_t startStamp = 0;
	OS_U8 startFly = 0;
	OS_U8 fzFeedbk;
	OS_U8 fzTask;
	static uint8_t FzOnFlag = 0;
	OS_S32 launchLon,launchLat,curLon,curLat;
	OS_S16 launchHigh;
	OS_FLOAT curHigh;
	double dist;
	double h_m;
	const char* msg_log;

	if(AutoStep == 14)
	return 0;
	//1.对各设备上电
	//5s时对惯组、导引头上电
	if(g_DeviceState.currTime > 5.0 && AutoStep == 0)
	{
		//判配电板合路供电电压，小于20伏时发送开启命令
		if(g_DeviceState.imuCountDown == 0)
		{
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
			if(sendFlag == 0)
			{
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
			if(powerSend1 == 0)
			{
				PowerOn(DEVICE_SRV_PWR28V);
				powerSend1 = 1;
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
			if(powerSend2 == 0)
			{
				// PowerOn(DEVICE_BATT_ENGINE);	// 014发动机不受配电板控制上下电
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
		GetDataFast(pDataPoolMsn, "msnDevID", &msnID);

		if(g_DeviceStatus.msgFromGCS != 0x00)
		{
			InitSD();
		}

		if(g_DeviceState.CurrTick % 200 == 0)
		{
			msg_log = "waiting for CMD_MSN_UPDATE\n";
			fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
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
		msg.u8MsgID = CMD_NAV_INIT;
		NavCmdHandler(&msg);
		msg.u8MsgID = CMD_HOR_CALC_REQ;
		NavCmdHandler(&msg);
		AutoStep = 6;
	}
	//4.判断对准完成，完成后转导航
	if(AutoStep == 6)
	{
		GetDataFast(pDataPoolNav, "navState", &navStatus);

		if(navStatus == 0x20)
		{
			if(g_DeviceState.CurrTick % 200 == 0)
			{
				msg_log = "FOCUSING\n";
				fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
			}
		}

		if(navStatus == 0x3F)
		{
			// 自动转导航
			msg.u8MsgID = CMD_TO_NAV_REQ;
			NavCmdHandler(&msg);

			if(g_DeviceState.CurrTick % 200 == 0)
			{
				msg_log = "FOCUS DONE\n";
				fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
			}
		}
		
		if(navStatus == 0x64)
		{
			// 自动转射后
			msg.u8MsgID = CMD_TO_AFTER_LUANCH;
			NavCmdHandler(&msg);
			// 初始化控制代码
			if((DOM_HILSMODE & g_DeviceState.workStage) != DOM_HILSMODE)
			{
				g_pControl = ControlInitial();
			}
			AutoStep = 7;
		}
	}
	//5.星历装订(空缺)
	if(AutoStep == 7)
	{
		AutoStep = 8;
	}
	//6.等待发动机启动指令发出
	if(AutoStep == 8)
	{
        if(g_DeviceState.CurrTick % 200 == 0)
        {
            msg_log = "waiting for ECU Start\n";
            fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
        }

		if(EngineStartCmd == 1)
		{
			AutoStep = 9;
		}
	}
	//6.发动机启动
	if(AutoStep == 9)
	{
		if(EngineStartCmd == 0)
		{
			//用户手动又发送了停机指令
			StopEngine();
			AutoStep = 8;
		}
		else
		{
			if(g_DeviceState.CurrTick % 20 == 0)// 每100ms，进if
			{
				//先判断发动机是否已进入运行状态
				GetDataFast(pDataPoolSelf, "ecuState", &curState);
				GetDataFast(pDataPoolSelf, "ecuGetRp", &rpm);
				switch(curState)//0停机，1启动中，2散热 3故障 4脱机 5运行（这肯定是协议里的）
				{
					case ENGINE_STOPED://发送启动指令
							StartEngine();
							break;
					case ENGINE_WARMUP://等待启动
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								msg_log = "ENGINE_WARMUP\n";
								fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
							}
							break;
					case ENGINE_SHUTTING_DOWN:
							break;
					case ENGINE_ERROR:
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								msg_log = "ENGINE_ERROR\n";
								fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
							}
							break;
					case ENGINE_RUNNING:
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								msg_log = "ENGINE_RUNNING\n";
								fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log));    
							}
							/* 运行到这里表示：
							1、启动成功*/
							/* 2、转速稳定在2300以上超过10s
							3、发了一个30%油门
							4、转速稳定在3400以上超过5s*/
							SetEngineThrot(30.0f);
							if(rpm > 3400)
							{
								if(ThrFlag)
								{
									startStamp = tx_time_get();
									ThrFlag = false;
								}
								else
								{
									// 判断时间是否大于5s
									if((tx_time_get() - startStamp) > 5000)
									{
										ThrFlag = true;
										AutoStep = 10;
									}
								}
							}
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
		GetDataFast(pDataPoolSelf,	"startFly",	&startFly);
		if(startFly == 1)//DoIgnition函数将startFly置1，地面 发控首页 - 起飞/过载/速度/hil
		{
			FuseSend(ARM_I);	//已经起飞了，发送一级解保指令（条件2g过载）
			// 判断第8字节为0x40
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
		if (0 == FzOnFlag)
		{
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

			dist = haversine_distance(launchLat, launchLon, curLat, curLon);
			h_m = curHigh - launchHigh;

			if((dist > 2000)&&(h_m > 200))
			{
				FuseSend(ARM_II);// 发送二级解保

				GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
				if(fzFeedbk & 0xC0)
				{
					PowerOn(DEVICE_FUSE28V);
					g_DeviceStatus.FzOnStamp_s = tx_time_get();
					FzOnFlag = 1;
				}
			}
		}
		else
		{
			// 引信-引爆电源已经上电
			// 延时2s
			if((tx_time_get() - g_DeviceStatus.FzOnStamp_s) > 2000)
			{
				//开始进入攻击点时，给引信发送三级解保
				//方法一：根据控制输出标志位判断
				if(g_controller_to_switch.flag_fuze_unlock == 1)
				// //方法二：根据控制输出当前航点号判断
				// OS_U8 curPtNo;
				// GetDataFast(pDataPoolMsn, "WP_cur", &curPtNo);
				// if(Arp[curPtNo].w == MSN_CMD_ATTACK)
				{
					FuseSend(ARM_III);//发送三级解保
					
					// 判断第9字节为0x80
					GetDataFast(pDataPoolSelf, "fzTask", &fzTask);
					if(fzTask & 0x80)
					{
						AutoStep = 14;
					}
				}
			}
		}
	}
	SETDATA(pDataPoolMsn,	"autoStep",	AutoStep,	OS_U8);	
    if(g_DeviceState.CurrTick % 200 == 0)
    {
		char info[50] = {0};
		sprintf(info,"AutoStep = %d, NAVnavState = 0x%x, ecuState = %d\n", 
						AutoStep, 			navStatus,		curState);
        fcs_uart_send(RT_LOG, (const uint8_t*)info, strlen(info));
    }

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
 * 锟斤拷锟斤拷锟斤拷锟斤拷:ControllerStatusUpdata()
 * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷锟???锟斤拷锟斤拷锟??5ms锟斤拷?锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷锟???锟斤拷锟斤拷锟斤拷锟???锟???锟斤拷锟斤拷锟斤拷?锟斤拷锟???锟???锟???锟斤拷锟斤拷锟斤拷锟??
 * 	1.锟???锟斤拷锟斤拷锟斤拷?锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷锟???锟斤拷
 * 	2.锟斤拷锟斤拷?锟斤拷锟斤拷锟???锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷锟斤拷
 * 锟斤拷锟斤拷:	锟???锟絑
 ***********************************************************/
OS_U8 ControllerStatusUpdata()	// 5ms锟斤拷锟斤拷?锟斤拷
{
	//锟???锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟???锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷??
	SelfCheckCollpse();//锟斤拷锟斤拷?锟斤拷锟???锟??????锟斤拷SD锟斤拷锟?????

	/* Full sim (useNav==0): air data from simulator; otherwise onboard sensor */
	if((g_DeviceState.workStage & DOM_HILSMODE) != DOM_HILSMODE)
	{
		CalcAirSpd();
	}
	ReConnectUart();	// 锟斤拷锟斤拷锟斤拷锟斤拷
	return 0;
}


/***********************************************************
 * 锟斤拷锟斤拷锟斤拷锟斤拷:SelfCheckCollpse()
 * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷锟???锟斤拷锟斤拷锟斤拷锟斤拷???锟???锟斤拷锟斤拷锟斤拷锟???锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷锟斤拷?锟斤拷锟斤拷锟斤拷?锟???锟斤拷锟???锟斤拷锟??
 * 锟斤拷锟斤拷:	锟???锟絑
 ***********************************************************/
static OS_U8 SelfCheckCollpse()
{
	//锟斤拷锟斤拷?锟??????
    LunchDetective();
	
	//锟斤拷锟斤拷?锟斤拷
	//SETDATA(pDataPoolSelf,	"BJTime",	g_DeviceState.BJTimeSecond,	OS_U32);

	//锟斤拷锟斤拷锟斤拷锟??
	float t = System_GetCoreTemperature();
	g_DeviceState.temperature = t;

	//锟???锟斤拷锟斤拷??锟斤拷??
	g_DeviceState.hilCountDown = g_DeviceState.hilCountDown > 0?g_DeviceState.hilCountDown-1:0;
	g_DeviceState.ecuCountDown  = g_DeviceState.ecuCountDown > 0?g_DeviceState.ecuCountDown-1:0;
	g_DeviceState.battCountDown  = g_DeviceState.battCountDown > 0?g_DeviceState.battCountDown-1:0;
    g_DeviceState.powerCountDown  = g_DeviceState.powerCountDown > 0?g_DeviceState.powerCountDown-1:0;
	g_DeviceState.navCountDown  = g_DeviceState.navCountDown > 0?g_DeviceState.navCountDown - 1:0;
	g_DeviceState.srvCountDown  = g_DeviceState.srvCountDown > 0?g_DeviceState.srvCountDown-1:0;
    //g_DeviceState.fuseCountDown = g_DeviceState.fuseCountDown > 0?g_DeviceState.fuseCountDown-1:0;
	g_DeviceState.imuCountDown = g_DeviceState.imuCountDown > 0?g_DeviceState.imuCountDown-1:0;
	g_DeviceState.scoutCountDown = g_DeviceState.scoutCountDown >0? g_DeviceState.scoutCountDown-1:0;

	SETDATA(pDataPoolSelf,	"commHil",	g_DeviceState.hilCountDown,	OS_U8);//锟斤拷锟斤拷
	SETDATA(pDataPoolSelf,	"commBatt",	g_DeviceState.battCountDown,	OS_U8);//锟斤拷锟??????
	SETDATA(pDataPoolSelf,	"commPwr",	g_DeviceState.powerCountDown,	OS_U8);//锟斤拷锟斤拷
	SETDATA(pDataPoolSelf,	"commEcu",	g_DeviceState.ecuCountDown,	OS_U8);//锟斤拷锟斤拷锟斤拷
	SETDATA(pDataPoolSelf,	"commNav",	g_DeviceState.navCountDown,	OS_U8);//锟斤拷锟斤拷

	SETDATA(pDataPoolSelf,	"commSrv",	g_DeviceState.srvCountDown,		OS_U8);//锟???锟??
	SETDATA(pDataPoolSelf,	"commImu",	g_DeviceState.imuCountDown,		OS_U8);//锟?????
	SETDATA(pDataPoolSelf,	"commFuse",	g_DeviceState.fuseCountDown,	OS_U8);//锟斤拷锟斤拷
	SETDATA(pDataPoolSelf,	"commScot",	g_DeviceState.scoutCountDown,	OS_U8);//锟斤拷锟斤拷???

	SETDATA(pDataPoolSelf,	"cpuTemp",	g_DeviceState.temperature * 100,		OS_S16);//锟???锟斤拷锟斤拷
	SETDATA(pDataPoolSelf,	"selfMode",	g_DeviceState.workStage,		OS_U8);//锟斤拷锟斤拷锟???锟??

	OS_U8 sdState;
	if(SD_Enable == FALSE)
	{
			sdState = 0xFF;
	}
	else
	{
			sdState = sd_card_fault==0?1:0xEE;
	}

	SETDATA(pDataPoolSelf,	"sdState",	sdState,	OS_U8);//锟???SD锟斤拷??

	return 0;
}

OS_U8 InitReportParam()
{
	SETDATA(pDataPoolNav,	"imuFocus",	 0,		OS_U8);
	
	SETDATA(pDataPoolSelf,	"tcCmd",	 0xAA,		OS_U8);//锟斤拷锟斤拷??锟斤拷
    SETDATA(pDataPoolSelf,	"flyError",	 0xFF,		OS_U8);

	return 0;
}

/***********************************************************
 * 锟斤拷锟斤拷锟斤拷锟斤拷:BCCmdHandler()
 * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷?锟斤拷锟???锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟???锟斤拷锟斤拷锟斤拷
 * 锟斤拷锟斤拷:	锟???锟絑
 ***********************************************************/
OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame)	// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷?锟斤拷
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
		//锟斤拷锟???锟斤拷?锟??
		FLIGHT_CMD cmd;
		cmd.cmdType = 0;//锟斤拷锟斤拷锟???锟??
		cmd.paramID = frame->au8Data[0];
		cmd.paramValue = *(OS_FLOAT *)(&frame->au8Data[1]);
		SETDATA(pDataPoolSelf, "tcCmd", cmd.paramID,	OS_U8);
		FlightControlCmd(cmd);
		break;
	}*/
	case CMD_URGENT_LAND:	/* 0x22 emergency parachute */
	{
		/* flyError: 0 normal, 1 urgent land, 2 urgent return, ... */
		SETDATA(pDataPoolSelf,  "flyError", 1,	OS_U8);
		SETDATA(pDataPoolSelf, "tcCmd", 0xC0,	OS_U8);
		DoOpenUm();
		break;
	}
	case CMD_URGENT_RETURN:	/* 0x23 emergency return */
	{
		SETDATA(pDataPoolSelf,  "flyError", 2,	OS_U8);
		SETDATA(pDataPoolSelf, "tcCmd", 0xC1,	OS_U8);
		DoReturnHomeward();
		break;
	}
	case CMD_ENGINE_START:	/* 0xF6 engine start command */
	{
		EngineStartCmd = 1;
		break;
	}
	case CMD_ENGINE_STOP:	/* 0xF7 engine stop command */
	{
		EngineStartCmd = 0;
		break;
	}
	case CMD_FORE_LAUNCH_REQ:	/* 0xF8 pre-launch enable/disable */
	{
		/* data[0]==0x11: arm launch; otherwise disarm and clear ignition mark */
		if(0x11==frame->au8Data[0])
		{
			g_DeviceState.luanchStart = 1;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);
		}
		else
		{
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0,	OS_U8);
			IgnitionMark = FALSE;
		}
	}
		break;
	case CMD_LAUNCH_REQ:	/* 0xFA launch command: arm ignition detect (needs pre-launch) */
	{
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			if(g_DeviceState.luanchStart == 1)
			{
				SETDATA(pDataPoolSelf,	"RecvLunc",	1,	OS_U8);
				IgnitionMark = TRUE;
				/* Do not call DoIgnition() here; wait for force / detach detect */
			}
		}
	}
		break;
	case CMD_LUANCH_FORCE:	/* 0xFB force launch */
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
			{
				/* luanMode: 0 none, 1 overload, 2 force, 3 ground-speed */
				SETDATA(pDataPoolSelf, "luanMode",	2,	OS_U8);
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
	/* Mark launched after T+0.5 s (MML: flight time > 500 ms) */
	if(flightTime >= 0.5)
	{
		flightSeq.luanched = 1;
	}
	else
	{
		flightSeq.luanched = 0;
	}
	return 0;
}


/* Enter automatic flight: notify NAV, clear Interactive, reset flight clock */
OS_U8 DoIgnition()
{
	MsgToNAV(BUS_NAV_IGNATION,PTR_NULL,0);
	
	g_DeviceState.workStage &= ~((unsigned int)DOM_INTERACTIVE);
	g_DeviceState.workStage |= DOM_AUTOMATIC;
	g_DeviceState.flightStartTime = GetCurTime();
	g_DeviceState.CurrTick = 0;

	SETDATA(pDataPoolSelf, "startFly",	1,	OS_U8);	/* takeoff flag */
	SETDATA(pDataPoolSelf, "luncTime",	g_DeviceState.BJTimeSecond,	OS_U32);	/* BJ launch time */
	SETDATA(pDataPoolSelf, "flyError", 0,	OS_U8);	/* clear flight fault */
	return 0;
}

OS_U8 IgnitionMark = FALSE;  /* TRUE after CMD_LAUNCH_REQ; armed for detach / force launch */

/* Notify NAV ignition only (DoIgnition path is handled elsewhere) */
OS_U8 Ignition()
{
	MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
	/* DoIgnition(); */
	return 0;
}

/* Launch detach detect: overload (imuAx) or NAV ground speed (need IgnitionMark) */
OS_U8 LunchDetective()
{
	if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
	{
		OS_FLOAT fAx;
		GetDataFast(pDataPoolImu, "imuAx", &fAx);	/* IMU axial accel, m/s^2 */

		OS_S16 vn,ve;
		GetDataFast(pDataPoolImu, "navVe", &ve);
		GetDataFast(pDataPoolImu, "navVn", &vn);
		OS_U8 navState;
		GetDataFast(pDataPoolImu, "navState", &navState);
		float fvn,fve;
		fvn = vn * 0.01;
		fve = ve * 0.01;
		float v = sqrt(pow(fvn,2) + pow(fve,2));	/* horizontal ground speed m/s */

		static OS_U8 detachCount = 1;
		if(fabs(fAx) > 30.0)
		{
			/* Mode 1: |Ax| > 30 for ~5 cycles (~25 ms) */
			if(detachCount == 5)
			{
				DoIgnition();
				SETDATA(pDataPoolSelf,	"luanMode",	1,	OS_U8);	/* overload launch */
				IgnitionMark = FALSE;
			}
			else
			{
				detachCount++;
			}
		}
		else
		{
			/* Mode 3: Vg > 10 m/s and navState==0x60 for ~5 cycles */
			if(v > 10.0 && navState == 0x60)
			{
				if(detachCount == 5)
				{
					DoIgnition();
					SETDATA(pDataPoolSelf,	"luanMode",	3,	OS_U8);	/* speed launch */
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