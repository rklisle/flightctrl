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
#include "../interface/interface_gpio.h"
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
OS_U8 EngineStartCmd = 0;    //���Ե���Ŀ��Ʋ�����???? 1��������������0��ֹͣ������
float System_GetCoreTemperature();
extern void ReConnectUart();
extern void *g_pControl;
OS_DOUBLE AirSpdHistory[150];

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
    
    SETDATA(pDataPoolSelf,	"AirPress",	curPress *0.1,	OS_U16);//ѹ��
    SETDATA(pDataPoolSelf,	"AirHigh",	press_height,	OS_S16);//��ѹ�߶�
    SETDATA(pDataPoolSelf,	"AirSpd",	curAirSpd * 10, OS_S16);//��ѹ����
    
    return 0;
}

/***********************************************************
 * ��������:AutoLuanchProcess()
 * ��������: �豸�ϵ���Զ���ʼ�����ϵ�????�Լ졢��׼��׼�����������????
 *			 ������ɺ�ȴ����շ��䣬���й��������κ���Ϊ��Ԥ
 * ����:	�ɺ�Z
 ***********************************************************/
OS_U8 AutoLuanchProcess()  // 5ms����һ��
{
	int testCmd=0;
	if(testCmd==1)
	{
		PowerOn(DEVICE_FUSE28V);
		PowerOn(DEVICE_FUSE_ISO5V);
	}
	//���̵��Ȳ��裬��ʼΪ0
	static int AutoStep = 0;
	if(AutoStep == 14)
	return 0;

	//��ɺ�????��ִ�����̵���
	// if((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC)
	// 		return 0;    
	//EngineStartCmd = 1; //����������ָ��
	//IgnitionMark = TRUE;//���������ָ��????
	
	///1.�Ը��豸�ϵ�
	//�ɿ������в���ʼ�����ȴ�5s�󣬶Թ��顢����ͷ�ϵ�
	if(g_DeviceState.currTime > 5.0 && AutoStep == 0)
	{
		//�������·�����ѹ��С��20��ʱ���Ϳ�������
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
	//11sʱ�Է������ϵ�
	if(g_DeviceState.currTime > 11.0 && AutoStep == 3)
	{
		//���жϹ��磬ֱ���жϷ�����ͨ��
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
    ///2.�ȴ���������������?��SD����ʼ��
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
			// InitPwrSeq();	//MML ����Ʒ4
			// InitSD();
			AutoStep = 5;
		}
	}
	///3.������?
	if(AutoStep == 5)
	{
		//������׼
		STRU_422_MSG_INFO msg;
		msg.u8MsgID = CMD_NAV_INIT;
		NavCmdHandler(&msg);
		msg.u8MsgID = CMD_HOR_CALC_REQ;//���������յ���ʼ��Ԫ����Ҫ������׼����
		NavCmdHandler(&msg);
		ImuCmdHandler(&msg); //TODO: IMU 014���Ӧ����û���ˡ�????
		AutoStep = 6;
	}
	///4.�ж϶�׼��ɣ���ɺ�ת����
	if(AutoStep == 6)
	{
    	//AutoStep = 7;
		//��׼���????
		//ToImu();
		OS_U8 imuStatus, navStatus;
		GetDataFast(pDataPoolImu, "navState", &imuStatus);
		GetDataFast(pDataPoolNav, "navState", &navStatus);
		//�пƵ����Լ�ת�����������жϵ���״̬
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
			// �Զ�ת����
			STRU_422_MSG_INFO msg;
			msg.u8MsgID = CMD_TO_NAV_REQ;
			NavCmdHandler(&msg);
			AutoStep = 7;
		}
	}
	///5.����װ��(��ȱ)/ �Զ�ת���????
	if(AutoStep == 7)
	{
		// AutoStep = 8;
		OS_U8 navStatus;
		GetDataFast(pDataPoolNav, "navState", &navStatus);
		if(navStatus == 0x64)
		{
			// �Զ�ת���????
			STRU_422_MSG_INFO msg;
			msg.u8MsgID = CMD_TO_AFTER_LUANCH;
			NavCmdHandler(&msg);
			AutoStep = 8;
		}
	}
	//6.�ȴ�����������ָ���
	if(AutoStep == 8)
	{
		// ��ʼ�����ƴ���
		EnsureLaunchControl();

		//�յ�����վ������������ָ�
		if(EngineStartCmd == 1)
		{
			AutoStep = 9;
		}
	}
	//6.����������
	if(AutoStep == 9)
	{
		//AutoStep = 10; //test
		//return 0;

		//����������������
		//�յ��˵���վ��ȡ������ָ�������ֹͣ���˻ص����ȴ�������������
		if(EngineStartCmd == 0)
		{
			//�û��ֶ��ַ�����ͣ��ָ��
			StopEngine();
			AutoStep = 8;
		}
		else
		{
			if(g_DeviceState.CurrTick % 4 == 0)// ÿ20ms����if
			{
				//���жϷ������Ƿ��ѽ�������״̬
				// struct EngineStatus engineStatus = {0};
				// modECU_GetEngineStatus(&engineStatus);
				// switch(engineStatus.CntState)//0ͣ����1�����У�2ɢ�� 3���� 4�ѻ� 5���У���϶����?����ģ�????
				OS_U8 cntState;
				GetDataFast(pDataPoolSelf, "ecuState", &cntState);
				switch(cntState)//0ͣ����1�����У�2ɢ�� 3���� 4�ѻ� 5���У���϶����?����ģ�????
				{
					case ENGINE_STOPED://��������ָ��
							StartEngine();//Ӧ������������
							break;
					case ENGINE_WARMUP://�ȴ�����
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
	//7.�Զ�����Ԥ����
	if(AutoStep == 10)
	{
    	g_DeviceState.luanchStart = 1;//��λ����׼���ñ�ʶ
		SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);//Ԥ������ɣ��ȴ��������ָ��
		AutoStep = 11;
	}
	//8.Ԥ������ɣ��ȴ��������ָ��
	if(AutoStep == 11)
	{
		//�յ��˵���վ��ȡ������ָ�������ֹͣ���˻ص����ȴ�������������
		if(EngineStartCmd == 0)
		{
			//�û��ֶ��ַ�����ͣ��ָ��         
			StopEngine();
			AutoStep = 8;
			
			g_DeviceState.luanchStart = 0;//�������׼���ñ��?
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);//���Ԥ�������
		}

		//�յ�����վȫ������ָ��
		if(IgnitionMark == TRUE)
		{
			AutoStep = 12;
		}
	}
	
	//9.��������Ѿ���ɣ��ȴ����������????
	//���ݵ���ָ�ȡ������/������ֹͣ��;
	//�����������������һ��������????
	if(AutoStep == 12)
	{
		//�յ��˵���վ��ȡ������ָ�������ֹͣ���˻ص����ȴ�������������
		if(EngineStartCmd == 0)
		{
			//�û��ֶ��ַ�����ͣ��ָ��
			StopEngine();
			AutoStep = 8;
			IgnitionMark = FALSE;
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0x00,	OS_U8);//���Ԥ�������
		}

		//DoIgnition() �ڲ���"startFly"����1�������????
		OS_U8 startFly = 0;
		GetDataFast(pDataPoolSelf,	"startFly",	&startFly);
		if(startFly == 1)
		{
			FuseSend(ARM_I);	//�Ѿ�����ˣ�����һ���Ᵽָ��????����2g���أ�
			// �жϵ�8�ֽ�Ϊ0x40
			OS_U8 fzFeedbk;
			GetDataFast(pDataPoolSelf, "fzFeedbk", &fzFeedbk);
			if(fzFeedbk & 0x40)
			{
				AutoStep = 13;
			}
		}
	}    
	//10.����?�󣬷ɳ�2km����Ը߶ȴ���????2m�󣬸����ŵ�������Դ�ϵ�
	if(AutoStep == 13)
	{
		//���Ź��缰�����Ᵽ�������ϵ�
		static uint8_t FzOnFlag = 0;
		if (0 == FzOnFlag)
		{
			OS_S32 launchLon,launchLat,curLon,curLat;
			OS_S16 launchHigh;
			OS_FLOAT curHigh;
			GetDataFast(pDataPoolFly, "DataLon", &launchLon);//���λ�õľ�γ��????
			GetDataFast(pDataPoolFly, "DataLat", &launchLat);//
			GetDataFast(pDataPoolFly, "DataHigh", &launchHigh);//
			launchLon = launchLon * 1e-7;
			launchLat = launchLat * 1e-7;

			GetDataFast(pDataPoolImu, "navLon", &curLon);//��ǰλ�õľ�γ��
			GetDataFast(pDataPoolImu, "navLat", &curLat);//
			GetDataFast(pDataPoolImu, "navHigh", &curHigh);//
			curLon = curLon * 1e-7;
			curLat = curLat * 1e-7;

			double lat_m = (curLat - launchLat) * 111320.0;//γ�Ȳ� ��Ӧ��ˮƽ����
			// ���ȷ�����Ҫ���� cos(γ��)
			double mid_lat_rad = (launchLat + curLat) / 2.0 * 3.1415926535 / 180.0;
			double lon_m = (curLon - launchLon) * 111320.0 * cos(mid_lat_rad);//���Ȳ� ��Ӧ��ˮƽ����
			
			//�߶Ȳ�
			double h_m = curHigh - launchHigh;
			//�������????
			double dist = sqrt(lat_m*lat_m + lon_m*lon_m);

			if((dist > 2000)&&(h_m > 200))
			{
				//���Ͷ����Ᵽ����ָ���ȡִ��״̬���Ᵽ����ʱ�����ʵ��ϵ�
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
		//������Դ�ϵ���ӳ�????2s��ʼ�жϣ���Ŀ����С��2km�������Ᵽָ��
		else
		{
			// ����-������Դ�Ѿ��ϵ�
			// ��ʱ2s
			if((GetCurTime() - g_DeviceStatus.FzOnStamp_s) > 2)
			{
				// �жϾ���Ŀ���Ƿ�<2km	
				OS_FLOAT deltaR;
				GetDataFast(pDataPoolFly, "deltaR", &deltaR);//��Ŀ����
				if(deltaR != 0)
				{
					//���������Ᵽ
					if(deltaR < 2000)
					{
						FuseSend(ARM_III);

						//�жϵ�9�ֽ�Ϊ0x80
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
	//���̵��Ȳ��裬����
	SETDATA(pDataPoolMsn,	"autoStep",	AutoStep, OS_U8);	//�������̲��裨�������Ŵ�����
	
	//�жϵ���ͷ�Ƿ����ӣ���������������?0b11 = 3�����δ���������?0b10 = 2
	if(g_DeviceState.scoutCountDown != 0)
	{
		SETDATA(pDataPoolMsn,	"paylodtp",	3,	OS_U8);//����ͷ����
	}
	else
	{
		SETDATA(pDataPoolMsn,	"paylodtp",	2,	OS_U8);//����ͷδ����
	}
	
	return 0;
}



/***********************************************************
 * ��������:ControllerStatusUpdata()
 * ��������: ���ܿ�����5ms��ʱ����������Ҫ���ܿ�������ʱ�䴥������Ϊ���ڴ�ִ�С�������
 * 	1.�Լ������ռ����������ݳ���ң��
 * 	2.����ǰ�����淢��������?����
 * ����:	�ɺ�Z
 ***********************************************************/
OS_U8 ControllerStatusUpdata()	// 5ms����һ��
{
	//�ж���������������������ɣ����������?��״̬
	SelfCheckCollpse();//���·ɿ��¶ȡ�ͨѶ״̬��SD���洢״̬
	
	CalcAirSpd();		// �ɼ���ѹ��������ѹ�߶ȡ�����
	
	ReConnectUart();	// ��������
	return 0;
}


/***********************************************************
 * ��������:SelfCheckCollpse()
 * ��������: ���ܿ��������豸״̬�ռ�������е��������Լ����?������֯�Լ���Ϣ�´�
 * ����:	�ɺ�Z
 ***********************************************************/
static OS_U8 SelfCheckCollpse()
{
	//����ж�????
    LunchDetective();
	
	//����ʱ��
	//SETDATA(pDataPoolSelf,	"BJTime",	g_DeviceState.BJTimeSecond,	OS_U32);

	//�����¶�
	float t = System_GetCoreTemperature();
	g_DeviceState.temperature = t;

	//�ռ����豸ͨ��״̬
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
	SETDATA(pDataPoolSelf,	"commBatt",	g_DeviceState.battCountDown,	OS_U8);//���????
	SETDATA(pDataPoolSelf,	"commPwr",	g_DeviceState.powerCountDown,	OS_U8);//����
	SETDATA(pDataPoolSelf,	"commEcu",	g_DeviceState.ecuCountDown,	OS_U8);//������
	SETDATA(pDataPoolSelf,	"commNav",	g_DeviceState.navCountDown,	OS_U8);//����

	SETDATA(pDataPoolSelf,	"commSrv",	g_DeviceState.srvCountDown,		OS_U8);//�ŷ�
	SETDATA(pDataPoolSelf,	"commImu",	g_DeviceState.imuCountDown,		OS_U8);//�߲൥Ԫ
	SETDATA(pDataPoolSelf,	"commFuse",	g_DeviceState.fuseCountDown,	OS_U8);//����
	SETDATA(pDataPoolSelf,	"commScot",	g_DeviceState.scoutCountDown,	OS_U8);//����ͷͨѶ

	SETDATA(pDataPoolSelf,	"cpuTemp",	g_DeviceState.temperature * 100,		OS_S16);//�ɿ��¶�
	SETDATA(pDataPoolSelf,	"selfMode",	g_DeviceState.workStage,		OS_U8);//�����׶�

	OS_U8 sdState;
	if(SD_Enable == FALSE)
	{
			sdState = 0xFF;
	}
	else
	{
			sdState = sd_card_fault==0?1:0xEE;
	}

	SETDATA(pDataPoolSelf,	"sdState",	sdState,	OS_U8);//�洢SD��״̬

	return 0;
}

OS_U8 InitReportParam()
{
	SETDATA(pDataPoolNav,	"imuFocus",	 0,		OS_U8);
	
	SETDATA(pDataPoolSelf,	"tcCmd",	 0xAA,		OS_U8);//����վָ��
    SETDATA(pDataPoolSelf,	"flyError",	 0xFF,		OS_U8);

	return 0;
}

/***********************************************************
 * ��������:BCCmdHandler()
 * ��������: ��Ҫ���ܿ�����������ָ������
 * ����:	�ɺ�Z
 ***********************************************************/
OS_U32 ControllerCmdHandler(STRU_422_MSG_INFO * frame)	// ����������������ָ��
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
		//���͸��ɿ�
		FLIGHT_CMD cmd;
		cmd.cmdType = 0;//�����ϴ�
		cmd.paramID = frame->au8Data[0];
		cmd.paramValue = *(OS_FLOAT *)(&frame->au8Data[1]);
		SETDATA(pDataPoolSelf, "tcCmd", cmd.paramID,	OS_U8);
		FlightControlCmd(cmd);
		break;
	}*/
	case CMD_URGENT_LAND:	// 0x22 ����ͷ��Ƶ - ����ɡ��
	{
		SETDATA(pDataPoolSelf,  "flyError", 1,	OS_U8);	//�����ý���ɡ��
		SETDATA(pDataPoolSelf, "tcCmd", 0xC0,	OS_U8);
		DoOpenUm();	//����ɡ����ɡ
		break;
	}
	case CMD_URGENT_RETURN:	// 0x23 ����ͷ��Ƶ - ��������
	{
		SETDATA(pDataPoolSelf,  "flyError", 2,	OS_U8);	//�����ý�������
		SETDATA(pDataPoolSelf, "tcCmd", 0xC1,	OS_U8);
		DoReturnHomeward();
		break;
	}
	case CMD_ENGINE_START:	// 0xF6 ��ҳ - ����������
	{
		EngineStartCmd = 1;//����������
		break;
	}
	case CMD_ENGINE_STOP:	// 0xF7 ��ҳ - ������ͣ��
	{
		EngineStartCmd = 0;//�������ػ�
		break;
	}
	case CMD_FORE_LAUNCH_REQ://ǿ�Ʒ���׼�����???? �� ȡ������(׼��δ���????)
	{
		//�յ�����ָ�ǰ�÷���׼�����????
		if(0x11==frame->au8Data[0])
		{
			g_DeviceState.luanchStart = 1;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xCC,	OS_U8);
		}
		//���������ǰ��ȡ������????
		else
		{
			g_DeviceState.luanchStart = 0;
			SETDATA(pDataPoolSelf,	"RecvLunc",	0,	OS_U8);
			//������ϲ�δ����ǰҲ�����˳�Ԥ��������ֹ��������????
			IgnitionMark = FALSE;
		}
	}
		break;
	case CMD_LAUNCH_REQ:	//0xFA ��ҳ - ȫ�����������һ��������׼�������������ǿ�Ʒ���׼����ɣ�????
	{
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			//�������׼������ˣ����????
			if(g_DeviceState.luanchStart == 1)
			{
				//g_DeviceState.luanchStart = 1;//��Ч
				SETDATA(pDataPoolSelf,	"RecvLunc",	1,	OS_U8);
				IgnitionMark = TRUE;

				//�ظ�����
				//Ignition();
				//�ڲ������ˣ�IgnitionMark = TRUE;
				//�ڲ������ˣ��򵼺����ͣ����ָ��????
			}
		}
	}
		break;
	case CMD_LUANCH_FORCE:	// 0xFB ��ҳ - ���????
		if( 0xAA == frame->au8Data[0]
		 && 0xBB == frame->au8Data[1]
		 && 0xCC == frame->au8Data[2]
		 && 0xDD == frame->au8Data[3]
		 && 0xEE == frame->au8Data[4])
		{
			//
			if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
			{
				SETDATA(pDataPoolSelf, "luanMode",	2,	OS_U8); //���ģʽ��������ɵķ�ʽ 0 ���� 1 ���ٶ����� 2 ���Է��䣨�ۺϲ��Ի����ָ��????3 �ٶ�����
				
				IgnitionMark = FALSE;
				DoIgnition();
				//�ڲ������� MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
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
	//����ǰδִ��
	if((DOM_AUTOMATIC & g_DeviceState.workStage) != DOM_AUTOMATIC)
	{
		return -1;
	}
	//������0.5s����Ϊ�ɿ�����
	if(flightTime >= 0.5)// ****************MML�� ʱ�� > 500ms ****************
	{
		flightSeq.luanched = 1;
	}
	else
	{
		flightSeq.luanched = 0;
	}
	return 0;
}


OS_U8 DoIgnition()// 1�����������������ɡ��������ã�/ 2�����أ������������????/ 3���ٶ���ɣ���������????/ 4��������ɣ������ã�????
{
	MsgToNAV(BUS_NAV_IGNATION,PTR_NULL,0);//�������ͣ��������������ָ��????
	
	g_DeviceState.workStage &= ~((unsigned int)DOM_INTERACTIVE);
	g_DeviceState.workStage |= DOM_AUTOMATIC;
	g_DeviceState.flightStartTime = GetCurTime();
	g_DeviceState.CurrTick = 0;

	SETDATA(pDataPoolSelf, "startFly",	1,	OS_U8);//��ɱ��?
	SETDATA(pDataPoolSelf, "luncTime",	g_DeviceState.BJTimeSecond,	OS_U32);//��ϵ���ʱ��????�����ա�ʱ����
	SETDATA(pDataPoolSelf, "flyError", 0,	OS_U8);//��������   
	return 0;
}

OS_U8 IgnitionMark = FALSE;  // ����ȫ��������ť�����־����ɺ������????

/***********************************************************
 * ��������:Ignition()
 * ��������: ����ָ��ɱ���������������á����յ�ָ�����������״̬�����³�ʼ���ɿ��㷨
 * ���룬�����ػ�������
 * ����:	�ɺ�Z
 ***********************************************************/
OS_U8 Ignition()
{
	//����ϵ�������???? ���????
	MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
	
	//���Խ׶Σ�ֱ�ӷ���
	//DoIgnition();
	return 0;
}

OS_U8 LunchDetective()	// �ж�������������
{
	//��������������ִ��
	if(IgnitionMark == TRUE && ((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC))
	{
		OS_FLOAT fAx;
		GetDataFast(pDataPoolImu, "imuAx", &fAx);//������ٶ�????
        
		OS_S16 vn,ve;
		GetDataFast(pDataPoolImu, "navVe", &ve);
		GetDataFast(pDataPoolImu, "navVn", &vn);
		OS_U8 navState;
		GetDataFast(pDataPoolImu, "navState", &navState);//����״̬
		float fvn,fve;
		fvn = vn * 0.01;
		fve = ve * 0.01;
		float v = sqrt(pow(fvn,2) + pow(fve,2));	//ˮƽ�ٶ�
        
		static OS_U8 detachCount = 1;
		if(fabs(fAx) > 30.0)
		{
			//���ģ�?1�������������ٶȴ���30��ÿ�뷽
			if(detachCount == 5)
			{
				DoIgnition();
				SETDATA(pDataPoolSelf,	"luanMode",	1,	OS_U8); //����ģʽ
				IgnitionMark = FALSE;
			}
			else
			{
				detachCount++;
			}
		}
		else
		{
			//���ģ�?3����ϵ���״�?���ٶȴ���10m/s
			if(v > 10.0 && navState == 0x60)
			{
				if(detachCount == 5)
				{
					DoIgnition();
					SETDATA(pDataPoolSelf,	"luanMode",	3,	OS_U8);
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