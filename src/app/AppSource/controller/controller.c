/*
 * modBC.c
 *
 *  Created on: 2022ï¿½ï¿½3ï¿½ï¿½15ï¿½ï¿½
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
#include "log_ctrl.h"

// /******************** Test umbrella Servo & ECU(PWM)***************************/
// #include "interface_timer.h"

static OS_U8 SelfCheckCollpse();
OS_U8 LunchDetective();
float Read_CPU_Temperature(void) ;
extern long calcTimeCpu0;
extern int sd_card_fault;
OS_U8 EngineStartCmd = 0;    //À´×ÔµØÃæµÄ¿ØÖÆ²ÎÊý£¬ 1£ºÆô¶¯·¢¶¯»ú£»0£ºÍ£Ö¹·¢¶¯»ú
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
    
    SETDATA(pDataPoolSelf,	"AirPress",	curPress*0.1,	OS_U16);//?ï¿½ï¿½
    SETDATA(pDataPoolSelf,	"AirHigh",	press_height*10,	OS_S16);//ï¿½ï¿½?ï¿???ï¿??
    SETDATA(pDataPoolSelf,	"AirSpd",	curAirSpd*10, OS_S16);//ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½
    
    return 0;
}

/***********************************************************
 * º¯ÊýÃû³Æ:AutoLuanchProcess()
 * º¯Êý¹¦ÄÜ: Éè±¸ÉÏµçºó×Ô¶¯¿ªÊ¼½øÈëÉÏµç¡¢×Ô¼ì¡¢¶Ô×¼¡¢×¼±¸·¢ÉäµÈÁ÷³Ì
 *			 Á÷³ÌÍê³ÉºóµÈ´ý×îÖÕ·¢Éä£¬ÆäÖÐ¹ý³ÌÎÞÐèÈÎºÎÈËÎª¸ÉÔ¤
 * ×÷Õß:	³Éºê­Z
 ***********************************************************/
OS_U8 AutoLuanchProcess()  // 5msÔËÐÐÒ»´Î
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

	if(AutoStep == 14)
	return 0;
	//1.¶Ô¸÷Éè±¸ÉÏµç
	//5sÊ±¶Ô¹ß×é¡¢µ¼ÒýÍ·ÉÏµç
	if(g_DeviceState.currTime > 5.0 && AutoStep == 0)
	{
		//ÅÐÅäµç°åºÏÂ·¹©µçµçÑ¹£¬Ð¡ÓÚ20·üÊ±·¢ËÍ¿ªÆôÃüÁî
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
	//7sÊ±¶ÔÒýÐÅÉÏµç
	if(g_DeviceState.currTime > 7.0 && AutoStep == 1)
	{
		//ÒýÐÅ¹©µçÎÞ²É¼¯µã£¬ÅÐ¶ÏÒýÐÅÍ¨ÐÅ×´Ì¬
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
	//9sÊ±¶ÔËÅ·þÉÏµç
	if(g_DeviceState.currTime > 9.0 && AutoStep == 2)
	{
		//ËÅ·þ¹©µçÎÞ²É¼¯µã£¬ÅÐ¶ÏËÅ·þÍ¨ÐÅ×´Ì¬
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
	//11sÊ±¶Ô·¢¶¯»úÉÏµç
	if(g_DeviceState.currTime > 11.0 && AutoStep == 3)
	{
		//²»ÅÐ¶Ï¹©µç£¬Ö±½ÓÅÐ¶Ï·¢¶¯»úÍ¨ÐÅ
		if(g_DeviceState.ecuCountDown == 0)
		{
			if(powerSend2 == 0)
			{
				// PowerOn(DEVICE_BATT_ENGINE);	// 014·¢¶¯»ú²»ÊÜÅäµç°å¿ØÖÆÉÏÏÂµç
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
    //2.µÈ´ýµØÃæ¼ÓÔØÈÎÎñÐÅÏ¢
	if(AutoStep == 4)
	{
		GetDataFast(pDataPoolMsn, "msnDevID", &msnID);

		if(g_DeviceStatus.msgFromGCS != 0x00)
		{
			InitSD();
		}

		if(g_DeviceState.CurrTick % 200 == 0)
		{
			LOG_STR("waiting for CMD_MSN_UPDATE\n");
		}

		if(msnID != 0xFF)
		{
			// InitPwrSeq();	//MML ¿ª»ð¹¤Æ·4
			// InitSD();
			AutoStep = 5;
		}
	}
	//3.¹ß×é¶Ô×¼
	if(AutoStep == 5)
	{
		//Æô¶¯¶Ô×¼
		msg.u8MsgID = CMD_NAV_INIT;
		NavCmdHandler(&msg);
		msg.u8MsgID = CMD_HOR_CALC_REQ;
		NavCmdHandler(&msg);
		AutoStep = 6;
	}
	//4.ÅÐ¶Ï¶Ô×¼Íê³É£¬Íê³Éºó×ªµ¼º½
	if(AutoStep == 6)
	{
		GetDataFast(pDataPoolNav, "navState", &navStatus);

		if(navStatus == 0x20)
		{
			if(g_DeviceState.CurrTick % 200 == 0)
			{
				LOG_STR("Focusing...\n");
			}
		}

		if(navStatus == 0x3F)
		{
			// ×Ô¶¯×ªµ¼º½
			msg.u8MsgID = CMD_TO_NAV_REQ;
			NavCmdHandler(&msg);

			if(g_DeviceState.CurrTick % 200 == 0)
			{
				LOG_STR("Focus Done\n");
			}
		}
		
		if(navStatus == 0x64)
		{
			// ×Ô¶¯×ªÉäºó
			msg.u8MsgID = CMD_TO_AFTER_LUANCH;
			NavCmdHandler(&msg);
			// ³õÊ¼»¯¿ØÖÆ´úÂë
			if((DOM_HILSMODE & g_DeviceState.workStage) != DOM_HILSMODE)
			{
				g_pControl = ControlInitial();
			}
			AutoStep = 7;
		}
	}
	//5.ÐÇÀú×°¶©(¿ÕÈ±)
	if(AutoStep == 7)
	{
		AutoStep = 8;
	}
	//6.µÈ´ý·¢¶¯»úÆô¶¯Ö¸Áî·¢³ö
	if(AutoStep == 8)
	{
        if(g_DeviceState.CurrTick % 200 == 0)
        {
			LOG_STR("waiting for ECU Start\n");
        }

		if(EngineStartCmd == 1)
		{
			AutoStep = 9;
		}
	}
	//6.·¢¶¯»úÆô¶¯
	if(AutoStep == 9)
	{
		if(EngineStartCmd == 0)
		{
			//ÓÃ»§ÊÖ¶¯ÓÖ·¢ËÍÁËÍ£»úÖ¸Áî
			StopEngine();
			AutoStep = 8;
		}
		else
		{
			if(g_DeviceState.CurrTick % 20 == 0)// Ã¿100ms£¬½øif
			{
				//ÏÈÅÐ¶Ï·¢¶¯»úÊÇ·ñÒÑ½øÈëÔËÐÐ×´Ì¬
				GetDataFast(pDataPoolSelf, "ecuState", &curState);
				GetDataFast(pDataPoolSelf, "ecuGetRp", &rpm);
				switch(curState)//0Í£»ú£¬1Æô¶¯ÖÐ£¬2É¢ÈÈ 3¹ÊÕÏ 4ÍÑ»ú 5ÔËÐÐ£¨Õâ¿Ï¶¨ÊÇÐ­ÒéÀïµÄ£©
				{
					case ENGINE_STOPED://·¢ËÍÆô¶¯Ö¸Áî
							StartEngine();
							break;
					case ENGINE_WARMUP://µÈ´ýÆô¶¯
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								LOG_STR("ENGINE_WARMUP...\n");
							}
							break;
					case ENGINE_SHUTTING_DOWN:
							break;
					case ENGINE_ERROR:
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								LOG_STR("ENGINE_ERROR\n");
							}
							break;
					case ENGINE_RUNNING:
							if(g_DeviceState.CurrTick % 200 == 0)
							{
								LOG_STR("ENGINE_RUNNING\n");
							}
							/* ÔËÐÐµ½ÕâÀï±íÊ¾£º
							1¡¢Æô¶¯³É¹¦*/
							/* 2¡¢×ªËÙÎÈ¶¨ÔÚ2300ÒÔÉÏ³¬¹ý10s
							3¡¢·¢ÁËÒ»¸ö30%ÓÍÃÅ
							4¡¢×ªËÙÎÈ¶¨ÔÚ3400ÒÔÉÏ³¬¹ý5s*/
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
									// ÅÐ¶ÏÊ±¼äÊÇ·ñ´óÓÚ5s
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
	//7.×Ô¶¯½øÈëÔ¤·¢Éä
	if(AutoStep == 10)
	{
		g_DeviceState.luanchStart = 1;
		SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);//Ô¤·¢ÉäÍê³É£¬µÈ´ý·¢Éä½âËøÖ¸Áî
		AutoStep = 11;
	}
	//8.Ô¤·¢ÉäÍê³É£¬µÈ´ýµØÃæ½âËøÖ¸Áî
	if(AutoStep == 11)
	{
		if(g_DeviceState.CurrTick % 200 == 0)
		{
			LOG_STR("waiting for unlock...\n");
		}
		if(EngineStartCmd == 0)
		{
			//ÓÃ»§ÊÖ¶¯ÓÖ·¢ËÍÁËÍ£»úÖ¸Áî         
			StopEngine();
			AutoStep = 8;
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);
		}
		if(IgnitionMark == TRUE)//µØÃæ·¢ËÍÈ«²¿½âËøÖ¸Áî
		{
			AutoStep = 12;
		}
	}
	//9.µØÃæ½âËøÒÑ¾­Íê³É£¬µÈ´ý»ð¼ý¼¤·¢
	if(AutoStep == 12)
	{
		if(g_DeviceState.CurrTick % 200 == 0)
		{
			LOG_STR("waiting for lauch...\n");
		}
		if(EngineStartCmd == 0)
		{
			//ÓÃ»§ÊÖ¶¯ÓÖ·¢ËÍÁËÍ£»úÖ¸Áî
			StopEngine();
			AutoStep = 8;
			IgnitionMark = FALSE;
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);
		}
		GetDataFast(pDataPoolSelf,	"startFly",	&startFly);
		if(startFly == 1)//DoIgnitionº¯Êý½«startFlyÖÃ1£¬µØÃæ ·¢¿ØÊ×Ò³ - Æð·É/¹ýÔØ/ËÙ¶È/hil
		{
			FuseSend(ARM_I);	//ÒÑ¾­Æð·ÉÁË£¬·¢ËÍÒ»¼¶½â±£Ö¸Áî£¨Ìõ¼þ2g¹ýÔØ£©
			// ÅÐ¶ÏµÚ8×Ö½ÚÎª0x40
			GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
			if(fzFeedbk & 0x40)
			{
				AutoStep = 13;
			}
		}
	}    
	//10.Æð·ÉÖ®ºó£¬·É³ö2kmºó¸øÒýÐÅµÄÒý±¬µçÔ´ÉÏµç
	if(AutoStep == 13)
	{
		if (0 == FzOnFlag)
		{
			GetDataFast(pDataPoolFly, "DataLon", &launchLon);//Æð·ÉÎ»ÖÃµÄ¾­Î³¸ß
			GetDataFast(pDataPoolFly, "DataLat", &launchLat);//
			GetDataFast(pDataPoolFly, "DataHigh", &launchHigh);//

			launchLon = launchLon * 1e-7;
			launchLat = launchLat * 1e-7;

			GetDataFast(pDataPoolImu, "navLon", &curLon);//µ±Ç°Î»ÖÃµÄ¾­Î³¸ß
			GetDataFast(pDataPoolImu, "navLat", &curLat);//
			GetDataFast(pDataPoolImu, "navHigh", &curHigh);//

			curLon = curLon * 1e-7;
			curLat = curLat * 1e-7;

			dist = haversine_distance(launchLat, launchLon, curLat, curLon);
			h_m = curHigh - launchHigh;

			if((dist > 2000)&&(h_m > 200))
			{
				FuseSend(ARM_II);// ·¢ËÍ¶þ¼¶½â±£

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
			// ÒýÐÅ-Òý±¬µçÔ´ÒÑ¾­ÉÏµç
			// ÑÓÊ±2s
			if((tx_time_get() - g_DeviceStatus.FzOnStamp_s) > 2000)
			{
				//¿ªÊ¼½øÈë¹¥»÷µãÊ±£¬¸øÒýÐÅ·¢ËÍÈý¼¶½â±£
				//·½·¨Ò»£º¸ù¾Ý¿ØÖÆÊä³ö±êÖ¾Î»ÅÐ¶Ï
				if(g_controller_to_switch.flag_fuze_unlock == 1)
				// //·½·¨¶þ£º¸ù¾Ý¿ØÖÆÊä³öµ±Ç°º½µãºÅÅÐ¶Ï
				// OS_U8 curPtNo;
				// GetDataFast(pDataPoolMsn, "WP_cur", &curPtNo);
				// if(Arp[curPtNo].w == MSN_CMD_ATTACK)
				{
					FuseSend(ARM_III);//·¢ËÍÈý¼¶½â±£
					
					// ÅÐ¶ÏµÚ9×Ö½ÚÎª0x80
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
		LOG_VAL("AutoStep = %d, NAVnavState = 0x%x, ecuState = %d\n", 
            AutoStep, navStatus, curState);
    }

	//ÅÐ¶Ïµ¼ÒýÍ·ÊÇ·ñÁ¬½Ó£¬Èç¹ûÁ¬½ÓÔòÀàÐÍÎª0b11 = 3£¬Èç¹ûÎ´Á¬½ÓÀàÐÍÎª0b10 = 2
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
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½:ControllerStatusUpdata()
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿??5msï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿???ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿???ï¿???ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿??
 * 	1.ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿???ï¿½ï¿½
 * 	2.ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½
 * ï¿½ï¿½ï¿½ï¿½:	ï¿???ï¿½Z
 ***********************************************************/
OS_U8 ControllerStatusUpdata()	// 5msï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½
{
	//ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½??
	SelfCheckCollpse();//ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿???ï¿??????ï¿½ï¿½SDï¿½ï¿½ï¿?????

	/* Full sim (useNav==0): air data from simulator; otherwise onboard sensor */
	if((g_DeviceState.workStage & DOM_HILSMODE) != DOM_HILSMODE)
	{
		CalcAirSpd();
	}
	ReConnectUart();	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	return 0;
}


/***********************************************************
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½:SelfCheckCollpse()
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½???ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿???ï¿½ï¿½ï¿???ï¿½ï¿½ï¿??
 * ï¿½ï¿½ï¿½ï¿½:	ï¿???ï¿½Z
 ***********************************************************/
static OS_U8 SelfCheckCollpse()
{
	//ï¿½ï¿½ï¿½ï¿½?ï¿??????
    LunchDetective();
	
	//ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½
	//SETDATA(pDataPoolSelf,	"BJTime",	g_DeviceState.BJTimeSecond,	OS_U32);

	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿??
	float t = System_GetCoreTemperature();
	g_DeviceState.temperature = t;

	//ï¿???ï¿½ï¿½ï¿½ï¿½??ï¿½ï¿½??
	g_DeviceState.hilCountDown = g_DeviceState.hilCountDown > 0?g_DeviceState.hilCountDown-1:0;
	g_DeviceState.ecuCountDown  = g_DeviceState.ecuCountDown > 0?g_DeviceState.ecuCountDown-1:0;
	g_DeviceState.battCountDown  = g_DeviceState.battCountDown > 0?g_DeviceState.battCountDown-1:0;
    g_DeviceState.powerCountDown  = g_DeviceState.powerCountDown > 0?g_DeviceState.powerCountDown-1:0;
	g_DeviceState.navCountDown  = g_DeviceState.navCountDown > 0?g_DeviceState.navCountDown - 1:0;
	g_DeviceState.srvCountDown  = g_DeviceState.srvCountDown > 0?g_DeviceState.srvCountDown-1:0;
    //g_DeviceState.fuseCountDown = g_DeviceState.fuseCountDown > 0?g_DeviceState.fuseCountDown-1:0;
	g_DeviceState.imuCountDown = g_DeviceState.imuCountDown > 0?g_DeviceState.imuCountDown-1:0;
	g_DeviceState.scoutCountDown = g_DeviceState.scoutCountDown >0? g_DeviceState.scoutCountDown-1:0;

	SETDATA(pDataPoolSelf,	"commHil",	g_DeviceState.hilCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½
	SETDATA(pDataPoolSelf,	"commBatt",	g_DeviceState.battCountDown,	OS_U8);//ï¿½ï¿½ï¿??????
	SETDATA(pDataPoolSelf,	"commPwr",	g_DeviceState.powerCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½
	SETDATA(pDataPoolSelf,	"commEcu",	g_DeviceState.ecuCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	SETDATA(pDataPoolSelf,	"commNav",	g_DeviceState.navCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½

	SETDATA(pDataPoolSelf,	"commSrv",	g_DeviceState.srvCountDown,		OS_U8);//ï¿???ï¿??
	SETDATA(pDataPoolSelf,	"commImu",	g_DeviceState.imuCountDown,		OS_U8);//ï¿?????
	SETDATA(pDataPoolSelf,	"commFuse",	g_DeviceState.fuseCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½
	SETDATA(pDataPoolSelf,	"commScot",	g_DeviceState.scoutCountDown,	OS_U8);//ï¿½ï¿½ï¿½ï¿½???

	SETDATA(pDataPoolSelf,	"cpuTemp",	g_DeviceState.temperature * 100,		OS_S16);//ï¿???ï¿½ï¿½ï¿½ï¿½
	SETDATA(pDataPoolSelf,	"selfMode",	g_DeviceState.workStage,		OS_U8);//ï¿½ï¿½ï¿½ï¿½ï¿???ï¿??

	OS_U8 sdState;
	if(SD_Enable == FALSE)
	{
			sdState = 0xFF;
	}
	else
	{
			sdState = sd_card_fault==0?1:0xEE;
	}

	SETDATA(pDataPoolSelf,	"sdState",	sdState,	OS_U8);//ï¿???SDï¿½ï¿½??

	return 0;
}

OS_U8 InitReportParam()
{
	SETDATA(pDataPoolNav,	"imuFocus",	 0,		OS_U8);
	
	SETDATA(pDataPoolSelf,	"tcCmd",	 0xAA,		OS_U8);//ï¿½ï¿½ï¿½ï¿½??ï¿½ï¿½
    SETDATA(pDataPoolSelf,	"flyError",	 0xFF,		OS_U8);

	return 0;
}

/***********************************************************
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½:BCCmdHandler()
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: ï¿½ï¿½?ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿???ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * ï¿½ï¿½ï¿½ï¿½:	ï¿???ï¿½Z
 ***********************************************************/
OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame)	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½ï¿½
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
		//ï¿½ï¿½ï¿???ï¿½ï¿½?ï¿??
		FLIGHT_CMD cmd;
		cmd.cmdType = 0;//ï¿½ï¿½ï¿½ï¿½ï¿???ï¿??
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