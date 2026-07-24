/*
 * modBC.c
 *
 *  Created on: 2022��3��15��
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
OS_U8 EngineStartCmd = 0;    //来自地面的控制参数， 1：启动发动机；0：停止发动机
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

static OS_U16 LaunchGetEngineRpm(void)
{
	OS_U16 rpm = 0;

	GetDataFast(pDataPoolSelf, "ecuGetRp", &rpm);
	return rpm;
}

static OS_U8 LaunchHoldRpmAbove(OS_U16 threshold, OS_U32 holdMs, OS_U32 *pElapsedMs)
{
	if (LaunchGetEngineRpm() >= threshold) {
		*pElapsedMs += 5U;
		if (*pElapsedMs >= holdMs) {
			*pElapsedMs = 0U;
			return 1U;
		}
	} else {
		*pElapsedMs = 0U;
	}
	return 0U;
}

static void LaunchApplyPreLaunchThrottle(void)
{
	SETDATA(pDataPoolFly, "EngineRp", (OS_U16)300, OS_U16);
	SetEngineThrot(LAUNCH_PRE_THROTTLE_PCT);
}

static void LaunchApplyStartHoldThrottle(void)
{
	SETDATA(pDataPoolFly, "EngineRp", (OS_U16)250, OS_U16);
	SetEngineThrot(25.0f);
}

static void LaunchAbortEngineStart(int *pAutoStep, OS_U32 *pStartHoldMs, OS_U32 *pThrottleHoldMs)
{
	StopEngine();
	SetEngineThrot(0.0f);
	*pStartHoldMs = 0U;
	*pThrottleHoldMs = 0U;
	*pAutoStep = 7;
	g_DeviceState.luanchStart = 0;
	IgnitionMark = FALSE;
	SETDATA(pDataPoolSelf, "RecvLunc", 0x00, OS_U8);
}

static void EnsureLaunchControl(void)
{
	static OS_U8 controlReady = 0;

	if(controlReady != 0)
		return;

	if(g_pControl != NULL)
	{
		deleteCMathControlMain(g_pControl);
		g_pControl = NULL;
	}
	g_pControl = ControlInitial();
	if(g_pControl != NULL)
		controlReady = 1;
}

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
    
    SETDATA(pDataPoolSelf,	"AirPress",	curPress*0.1,	OS_U16);//?��
    SETDATA(pDataPoolSelf,	"AirHigh",	press_height*10,	OS_S16);//��?�??�?
    SETDATA(pDataPoolSelf,	"AirSpd",	curAirSpd*10, OS_S16);//��?����
    
    return 0;
}

/***********************************************************
 * ��������:AutoLuanchProcess()
 * ��������: �??�??���??���??�����??�?????�???��?��?�����������?????
 *			 ������?�??����?��?���??��������?���??��?
 * ����:	�??�Z
 ***********************************************************/
OS_U8 AutoLuanchProcess()  /* 5 ms period */
{
	int testCmd=0;
	if(testCmd==1)
	{
		PowerOn(DEVICE_FUSE28V);
		PowerOn(DEVICE_FUSE_ISO5V);
	}
	static int AutoStep = 0;
	static OS_U32 launchStartHoldMs = 0;
	static OS_U32 launchThrottleHoldMs = 0;

	if(AutoStep == LAUNCH_AUTO_STEP_DONE)
	return 0;

	/* AutoStep launch sequence (takeoff PPT v2):
	 *   0-3  Power-on sequence (scout / fuse / servo / ECU)
	 *   4    Wait mission upload (msnDevID != 0xFF)
	 *   5    Nav alignment start (CMD_NAV_INIT + CMD_HOR_CALC_REQ)
	 *   6    Wait nav alignment done (navState == 0x3F), then
	 *        CMD_TO_NAV_REQ + CMD_TO_AFTER_LUANCH (转导航提前, 紧随转射后)
	 *   7    Wait engine start command (CMD_ENGINE_START)
	 *   8    ECU crank until ENGINE_RUNNING
	 *   9    Start complete: RPM >= 2300 for 10 s at 25% throttle
	 *  10    Pre-launch 30% throttle, RPM >= 3400 for 5 s
	 *  11    Wait takeoff unlock (CMD_FORE_LAUNCH_REQ, data[0]==0x11)
	 *  12    Wait INU ready (navState == 0x64), enter standby-for-launch
	 *  13    Wait launch command (CMD_LAUNCH_REQ -> IgnitionMark)
	 *  14    Fuse arm stage I after startFly
	 *  15    Fuse arm stage II/III in flight
	 *  16    Launch sequence complete
	 */

	/* Step 0: at T+5 s, power on scout head (DEVICE_SCOUT_E28V) */
	if(g_DeviceState.currTime > 5.0 && AutoStep == 0)
	{
		// Skip scout power-on if IMU link is already alive
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
	/* Step 1: at T+7 s, power on fuse isolation (DEVICE_FUSE_ISO5V) */
	if(g_DeviceState.currTime > 7.0 && AutoStep == 1)
	{
		if(g_DeviceState.fuseCountDown == 0)
		{
			static int sendFlag = 0;
			if(sendFlag == 0)
			{
				// PowerOn(DEVICE_FUSE28V);	
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
	/* Step 2: at T+9 s, power on servo bus (DEVICE_SRV_PWR28V) */
	if(g_DeviceState.currTime > 9.0 && AutoStep == 2)
	{		
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
	/* Step 3: at T+11 s, ECU power sequence (placeholder) */
	if(g_DeviceState.currTime > 11.0 && AutoStep == 3)
	{
		// ECU battery power-on is optional here
		if(g_DeviceState.ecuCountDown == 0)
		{
			static int powerSend2 = 0;
			if(powerSend2 == 0)
			{
					// PowerOn(DEVICE_BATT_ENGINE);	// ������
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
	/* Step 4: wait mission upload from ground (PPT: mission load) */
	if(AutoStep == 4)
	{
		OS_U8 msnID;
		GetDataFast(pDataPoolMsn, "msnDevID", &msnID);

		//
		if(g_DeviceStatus.msgFromGCS != 0x00)
		{
			InitSD();
		}

		if(msnID != 0xFF)
		{
			// InitPwrSeq();	//MML ����?4
			// InitSD();
			AutoStep = 5;
		}
	}
	/* Step 5: start nav alignment (PPT: nav alignment) */
	if(AutoStep == 5)
	{
		STRU_422_MSG_INFO msg;
		msg.u8MsgID = CMD_NAV_INIT;
		NavCmdHandler(&msg);
		msg.u8MsgID = CMD_HOR_CALC_REQ; /* horizontal alignment request */
		NavCmdHandler(&msg);
		ImuCmdHandler(&msg); /* TODO: IMU alignment for 014 variant */
		AutoStep = 6;
	}
	/* Step 6: wait nav alignment complete, then switch-to-nav + post-launch nav */
	if(AutoStep == 6)
	{
		OS_U8 imuStatus, navStatus;
		STRU_422_MSG_INFO msg;
		GetDataFast(pDataPoolImu, "navState", &imuStatus);
		GetDataFast(pDataPoolNav, "navState", &navStatus);

		if(navStatus == 0x3F)
		{
			/* PPT v2: 杞?瀵艰埅 after alignment, before engine start */
			msg.u8MsgID = CMD_TO_NAV_REQ;
			NavCmdHandler(&msg);
		}
		
		if(navStatus == 0x64)
		{
			/* Immediately send 杞?灏勫悗 */
			msg.u8MsgID = CMD_TO_AFTER_LUANCH;
			NavCmdHandler(&msg);
			AutoStep = 7;
		}
	}
	/* Step 7: wait engine start command from ground (PPT: engine start) */
	if(AutoStep == 7)
	{
		EnsureLaunchControl();

		if(EngineStartCmd == 1)
		{
			AutoStep = 8;
		}
	}
	/* Step 8: crank ECU until ENGINE_RUNNING */
	if(AutoStep == 8)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}
		else if(g_DeviceState.CurrTick % 4 == 0)
		{
			OS_U8 cntState;

			GetDataFast(pDataPoolSelf, "ecuState", &cntState);
			switch(cntState)
			{
			case ENGINE_STOPED:
				StartEngine();
				break;
			case ENGINE_WARMUP:
			case ENGINE_SHUTTING_DOWN:
				break;
			case ENGINE_RUNNING:
				launchStartHoldMs = 0U;
				AutoStep = 9;
				break;
			default:
				break;
			}
		}
	}
	/* Step 9: start complete - hold RPM >= 2300 for 10 s*/
	if(AutoStep == 9)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}
		else
		{
			//LaunchApplyStartHoldThrottle();
			if(LaunchHoldRpmAbove(LAUNCH_RPM_START_MIN, LAUNCH_RPM_START_HOLD_MS, &launchStartHoldMs))
			{
				launchThrottleHoldMs = 0U;
				AutoStep = 10;
			}
		}
	}
	/* Step 10: pre-launch 30% throttle, hold RPM >= 3400 for 5 s (PPT: throttle stable) */
	if(AutoStep == 10)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}
		else
		{
			LaunchApplyPreLaunchThrottle();
			if(LaunchHoldRpmAbove(LAUNCH_RPM_THROTTLE_MIN, LAUNCH_RPM_THROTTLE_HOLD_MS, &launchThrottleHoldMs))
			{
				AutoStep = 11;
			}
		}
	}
	/* Step 11: wait takeoff unlock from ground (PPT: takeoff unlock) */
	if(AutoStep == 11)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}
		else if(g_DeviceState.luanchStart == 1)
		{
			AutoStep = 12;
		}
	}
	/* Step 12: wait INU ready (navState == 0x64), then enter standby-for-launch */
	if(AutoStep == 12)
	{
		OS_U8 navStatus;

		GetDataFast(pDataPoolNav, "navState", &navStatus);
		if(navStatus == 0x64)
		{
			SETDATA(pDataPoolSelf, "RecvLunc", 0xCC, OS_U8);
			AutoStep = 13;
		}
	}
	/* Step 13: wait launch command (PPT: standby for launch, CMD_LAUNCH_REQ) */
	if(AutoStep == 13)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}
		else if(IgnitionMark == TRUE)
		{
			AutoStep = 14;
		}
	}
	/* Step 14: fuse arm stage I after flight has started */
	if(AutoStep == 14)
	{
		if(EngineStartCmd == 0)
		{
			LaunchAbortEngineStart(&AutoStep, &launchStartHoldMs, &launchThrottleHoldMs);
		}

		OS_U8 startFly = 0;
		GetDataFast(pDataPoolSelf, "startFly", &startFly);
		if(startFly == 1)
		{
			FuseSend(ARM_I);
			OS_U8 fzFeedbk;
			GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
			if(fzFeedbk & 0x40)
			{
				AutoStep = 15;
			}
		}
	}
	/* Step 15: fuse arm stage II/III when range/altitude criteria met in flight */
	if(AutoStep == 15)
	{
		static uint8_t FzOnFlag = 0;
		if (0 == FzOnFlag)
		{
			OS_S32 launchLon,launchLat,curLon,curLat;
			OS_S16 launchHigh;
			OS_FLOAT curHigh;
			GetDataFast(pDataPoolFly, "DataLon", &launchLon);
			GetDataFast(pDataPoolFly, "DataLat", &launchLat);
			GetDataFast(pDataPoolFly, "DataHigh", &launchHigh);
			launchLon = launchLon * 1e-7;
			launchLat = launchLat * 1e-7;

			GetDataFast(pDataPoolImu, "navLon", &curLon);
			GetDataFast(pDataPoolImu, "navLat", &curLat);
			GetDataFast(pDataPoolImu, "navHigh", &curHigh);
			curLon = curLon * 1e-7;
			curLat = curLat * 1e-7;

			double dist = haversine_distance(launchLat, launchLon, curLat, curLon);
			double h_m = curHigh - launchHigh;

			/* Arm fuse stage II when horizontal range > 2 km and altitude > 200 m */
			if((dist > 2000)&&(h_m > 200))
			{
				FuseSend(ARM_II);
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
			/* After fuse 28 V on, wait 2 s then arm stage III if deltaR < 2 km */
			if((GetCurTime() - g_DeviceStatus.FzOnStamp_s) > 2)
			{
				//开始进入攻击点时，给引信发送三级解保
				//方法1：根据控制输出标志位判断
				if(g_controller_to_switch.flag_fuze_unlock == 1)
				// //方法2：根据控制输出当前航点号判断
				// OS_U8 curPtNo;
				// GetDataFast(pDataPoolMsn, "WP_cur", &curPtNo);
				// if(Arp[curPtNo].w == MSN_CMD_ATTACK)
				// //方法3：根据控制输出当前航点号判断
				// OS_FLOAT deltaR;
				// GetDataFast(pDataPoolFly, "deltaR", &deltaR);
				// if(deltaR != 0)
				// {
				// 	if(deltaR < 2000)

				{
					FuseSend(ARM_III);//发送三级解保
					
					// 判断第9字节为0x80
					OS_U8 fzTask;
					GetDataFast(pDataPoolSelf, "fzTask", &fzTask);
					if(fzTask & 0x80)
					{
						AutoStep = LAUNCH_AUTO_STEP_DONE;
					}
				}
			}
		}
	}
	/* Publish autoStep for telemetry */
	SETDATA(pDataPoolMsn,	"autoStep",	AutoStep, OS_U8);
	
	/* Payload type: 3 = scout link alive, 2 = scout link down */
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
 * ��������:ControllerStatusUpdata()
 * ��������: ���??�����?5ms��?����������?���??�������??�??������?���??�??�??�������?
 * 	1.�??������?����������?���??��
 * 	2.����?�����??��������?����
 * ����:	�??�Z
 ***********************************************************/
OS_U8 ControllerStatusUpdata()	// 5ms����?��
{
	//�??���������������������??����������?��??
	SelfCheckCollpse();//����?���??�?????��SD���????

	/* Full sim (useNav==0): air data from simulator; otherwise onboard sensor */
	if (((g_DeviceState.workStage & DOM_HILSMODE) == 0) || hilInput.useNav != 0) {
		CalcAirSpd();
	}
	
	ReConnectUart();	// ��������
	return 0;
}


/***********************************************************
 * ��������:SelfCheckCollpse()
 * ��������: ���??��������???�??�������??��������?����?������?�??���??���?
 * ����:	�??�Z
 ***********************************************************/
static OS_U8 SelfCheckCollpse()
{
	//����?�?????
    LunchDetective();
	
	//����?��
	//SETDATA(pDataPoolSelf,	"BJTime",	g_DeviceState.BJTimeSecond,	OS_U32);

	//�������?
	float t = System_GetCoreTemperature();
	g_DeviceState.temperature = t;

	//�??����??��??
	g_DeviceState.hilCountDown = g_DeviceState.hilCountDown > 0?g_DeviceState.hilCountDown-1:0;
	g_DeviceState.ecuCountDown  = g_DeviceState.ecuCountDown > 0?g_DeviceState.ecuCountDown-1:0;
	g_DeviceState.battCountDown  = g_DeviceState.battCountDown > 0?g_DeviceState.battCountDown-1:0;
    g_DeviceState.powerCountDown  = g_DeviceState.powerCountDown > 0?g_DeviceState.powerCountDown-1:0;
	g_DeviceState.navCountDown  = g_DeviceState.navCountDown > 0?g_DeviceState.navCountDown - 1:0;
	g_DeviceState.srvCountDown  = g_DeviceState.srvCountDown > 0?g_DeviceState.srvCountDown-1:0;
    //g_DeviceState.fuseCountDown = g_DeviceState.fuseCountDown > 0?g_DeviceState.fuseCountDown-1:0;
	g_DeviceState.imuCountDown = g_DeviceState.imuCountDown > 0?g_DeviceState.imuCountDown-1:0;
	g_DeviceState.scoutCountDown = g_DeviceState.scoutCountDown >0? g_DeviceState.scoutCountDown-1:0;

	SETDATA(pDataPoolSelf,	"commHil",	g_DeviceState.hilCountDown,	OS_U8);//����
	SETDATA(pDataPoolSelf,	"commBatt",	g_DeviceState.battCountDown,	OS_U8);//���?????
	SETDATA(pDataPoolSelf,	"commPwr",	g_DeviceState.powerCountDown,	OS_U8);//����
	SETDATA(pDataPoolSelf,	"commEcu",	g_DeviceState.ecuCountDown,	OS_U8);//������
	SETDATA(pDataPoolSelf,	"commNav",	g_DeviceState.navCountDown,	OS_U8);//����

	SETDATA(pDataPoolSelf,	"commSrv",	g_DeviceState.srvCountDown,		OS_U8);//�??�?
	SETDATA(pDataPoolSelf,	"commImu",	g_DeviceState.imuCountDown,		OS_U8);//�????
	SETDATA(pDataPoolSelf,	"commFuse",	g_DeviceState.fuseCountDown,	OS_U8);//����
	SETDATA(pDataPoolSelf,	"commScot",	g_DeviceState.scoutCountDown,	OS_U8);//����???

	SETDATA(pDataPoolSelf,	"cpuTemp",	g_DeviceState.temperature * 100,		OS_S16);//�??����
	SETDATA(pDataPoolSelf,	"selfMode",	g_DeviceState.workStage,		OS_U8);//�����??�?

	OS_U8 sdState;
	if(SD_Enable == FALSE)
	{
			sdState = 0xFF;
	}
	else
	{
			sdState = sd_card_fault==0?1:0xEE;
	}

	SETDATA(pDataPoolSelf,	"sdState",	sdState,	OS_U8);//�??SD��??

	return 0;
}

OS_U8 InitReportParam()
{
	SETDATA(pDataPoolNav,	"imuFocus",	 0,		OS_U8);
	
	SETDATA(pDataPoolSelf,	"tcCmd",	 0xAA,		OS_U8);//����??��
    SETDATA(pDataPoolSelf,	"flyError",	 0xFF,		OS_U8);

	return 0;
}

/***********************************************************
 * ��������:BCCmdHandler()
 * ��������: ��?���??�����������??������
 * ����:	�??�Z
 ***********************************************************/
OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame)	// ����������������?��
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
		//���??��?�?
		FLIGHT_CMD cmd;
		cmd.cmdType = 0;//�����??�?
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