/*
 * modEngineEle.c
 *
 *  Created on: 2024��4��19��
 *      Author: lenovo
 */

#include "modEngine.h"
#include "modOnceBattery.h"
#include "../core/DataPool.h"
#include "../core/BusInteract.h"
#include "../StateMachine.h"
#include "../controller/controller.h"
#include "../Interface/interface_uart.h"

OS_U32 CurEngineRpm = 0;    //014 ���Űٷֱ�*10 ȡֵ[0~1000]��  280 ת��
OS_U32 EngineRpmCmd = 0;
OS_U8 EngineStartStatus = 0;
OS_U8 SyncToGround = 0;     //������Ƶģ������Ƿ�����洫������

void EngineInit()
{
    modECU_EngineInit();
}

/** ��������
 * ��Σ����Űٷֱ�? ȡֵ0~100.0
 */
void SetEngineThrot(float percent)
{
    // unsigned int rpm = percent * 10.0f;//Ϊ������ң��������ǽ����������?��һ��
    // SETDATA(pDataPoolSelf,	"engSetRp",	rpm,  OS_U16);
    modECU_setThrottle_percent(percent);
}

/** ���������� */
void StartEngine()
{
    modECU_startEngine();
}

/** ������ͣ�� */
void StopEngine()
{
    modECU_stopEngine();
}

/** �������������������Ҳ���ǵ����ң��ָ��?
 * ����ת��ָ������ݳء��ط�����ת��
 * ��ȡ����������������Ӧ����
 * ��ȡ���в�����
 */
OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame)
{
	switch(frame->u8MsgID)
	{
        case CMD_ECU_RPM_SETTING:
        {
            OS_U32 thro;   //014 ���Űٷֱ�*10 ȡֵ[0~1000]��  280 ת��
            memcpy(&thro, frame->au8Data, 4);
            if(thro < 0) {thro = 0;}
            if(thro > 1000) {thro = 1000;}
            SETDATA(pDataPoolFly, "EngineRp", thro,  OS_U16);
			SetEngineThrot(thro / 10.0f);
            break;
        }
        case CMD_GET_START_PARAM: // �ͱ�ֹͣ
        {
            modECU_pumpOff();
            break;
        }
        case CMD_GET_RUNNING_PARAM: // �ͱÿ���
        {
            modECU_pumpOn();
            break;
        }
        case CMD_GET_RUNNING_INFO://������Ƶģ������Ƿ�����洫������
        {
            if(frame->au8Data[0] == 0x11)
            {
                SyncToGround = 1;
            }
            if(frame->au8Data[0] == 0x22)
            {
                SyncToGround = 0;
            }
            break;
        }
        case CMD_START_STOP_ENGINE://���淢��������������
        {
            if(frame->au8Data[0] == 0x11)
            {
                StartEngine();
            }
            if(frame->au8Data[0] == 0x22)
            {
                StopEngine();
            }
            break;
        }
        default:
            break;
	}

	return 0;
}

/** �������ݣ������� ����> �ɿ�  �洢���� & ���������� */
void EngineHandler()   // HACK: TEST ECU restore & display
{
/** ****************014 ����******************* */
    static STRU_RUNNING_INFO param30 = {0};
    // static STRU_START_PARAM_INFO param31 = {0};
    // static STRU_RUNNING_PARAM_INFO param32 = {0};
    struct EngineStatus engineStatus = {0};

    if((g_DeviceState.CurrTick) % 20 == 0)  // ÿ100ms����һ������
    {
        modECU_GetEngineStatus(&engineStatus);
    /** **************** �����ݳ� ******************* */
        SETDATA(pDataPoolSelf, "engSetRp",	engineStatus.fuel_pressure,	OS_U16);
        // SETDATA(pDataPoolSelf, "ecuSetRp",	engineStatus.actual_throttle,	OS_U16);//97
        SETDATA(pDataPoolSelf, "ecuSetRp",	engineStatus.expect_throttle,	OS_U16);//86

        SETDATA(pDataPoolSelf, "ecuGetRp",  engineStatus.rpm,              OS_U16);
        SETDATA(pDataPoolSelf, "ecu24V", engineStatus.jet1_duty,	OS_S16);
        SETDATA(pDataPoolSelf, "ecu24A", engineStatus.jet2_duty,	OS_S16);

        SETDATA(pDataPoolSelf, "scout1", engineStatus.ch1_temp,	OS_S16);
        SETDATA(pDataPoolSelf, "scout2", engineStatus.ch2_temp,	OS_S16);
        SETDATA(pDataPoolSelf, "scout3", engineStatus.ch3_temp,	OS_S16);
        SETDATA(pDataPoolSelf, "scout4", engineStatus.ch4_temp,	OS_S16);

        SETDATA(pDataPoolSelf, "ecuState",	engineStatus.CurState,	OS_U8);
        SETDATA(pDataPoolSelf, "ecuError",	engineStatus.ecuError,	OS_U8);

    /** **************** ������������ʾ ******************* */
        param30.runningStatus = engineStatus.CurState;         //当前发动机的状�?
        param30.error = engineStatus.ecuError;                 // 发动机错�?�?
        param30.fuel_pressure = engineStatus.fuel_pressure;    //实际油压
        param30.jet1_duty = engineStatus.jet1_duty;            // 实际喷油1脉�??
        param30.curRpm = engineStatus.rpm;                     // 实际�?�?
        param30.ambient_temp = engineStatus.ambient_temp;      // �?境温�?
        param30.jet2_duty = engineStatus.jet2_duty;            // 实际喷油2脉�??
        param30.battV = engineStatus.battV * 0.01;             // 系统输入电压
        param30.battA = engineStatus.battA * 0.1;              // 系统输入电流
        // param30.actual_throttle = engineStatus.actual_throttle;//97
        param30.actual_throttle = engineStatus.expect_throttle;// 期望的油门位�?
        param30.ch1_temp = engineStatus.ch1_temp;
        param30.ch2_temp = engineStatus.ch2_temp;
        param30.ch3_temp = engineStatus.ch3_temp;
        param30.ch4_temp = engineStatus.ch4_temp;
        param30.outputW = 0;

        if(SyncToGround)
        {
            MsgToDevice(RT_DATA_LINK, 0x30, sizeof(STRU_RUNNING_INFO), (OS_U8 *)&param30);
    //         MsgToDevice(RT_DATA_LINK, 0x31, sizeof(STRU_START_PARAM_INFO), (OS_U8 *)&param31);
    //         MsgToDevice(RT_DATA_LINK, 0x32, sizeof(STRU_RUNNING_PARAM_INFO), (OS_U8 *)&param32);
        }
    }
}

OS_U8 AutoDriveEnginePwm()//ÿ100ms��������һ��engine����
{
    if(g_DeviceState.CurrTick % 20 == 0)
    {
        // GetCurEngineRpm();

    }
    if((g_DeviceState.CurrTick + 10) % 20 == 0)//ÿ5ms * 20 = 100ms����һ��if
    {
        if(((DOM_AUTOMATIC & g_DeviceState.workStage) == DOM_AUTOMATIC) && flightSeq.umOpen != 1)
        {
            GetDataFast(pDataPoolFly, "EngineRp", &CurEngineRpm);
			SetEngineThrot(CurEngineRpm *0.1f);
        }
    }
	return 0;
}

