/*
 * modEngineEle.c
 *
 *  Created on: 2024年4月19日
 *      Author: lenovo
 */

#include "modEngine.h"
#include "modOnceBattery.h"
#include "../core/DataPool.h"
#include "../core/BusInteract.h"
#include "../StateMachine.h"
#include "../controller/controller.h"
#include "../Interface/interface_uart.h"

OS_U32 CurEngineRpm = 0;    //014 油门百分比*10 取值[0~1000]；  280 转速
OS_U32 EngineRpmCmd = 0;
OS_U8 EngineStartStatus = 0;
OS_U8 SyncToGround = 0;     //地面控制的：控制是否给地面传输数据

void EngineInit()
{
    modECU_EngineInit();
}

/** 设置油门
 * 入参：油门百分比 取值0~100.0
 */
void SetEngineThrot(float percent)
{
    // unsigned int rpm = percent * 10.0f;//为了用上遥测表，我们将数据类型转化一下
    // SETDATA(pDataPoolSelf,	"engSetRp",	rpm,  OS_U16);
    modECU_setThrottle_percent(percent);
}

/** 启动发动机 */
void StartEngine()
{
    modECU_startEngine();
}

/** 发动机停机 */
void StopEngine()
{
    modECU_stopEngine();
}

/** 处理数据链传来的命令，也就是地面的遥控指令
 * 控制转速指令：存数据池、控发动机转速
 * 获取启动参数：发送相应命令
 * 获取运行参数：
 */
OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame)
{
	switch(frame->u8MsgID)
	{
        case CMD_ECU_RPM_SETTING:
        {
            OS_U32 thro;   //014 油门百分比*10 取值[0~1000]；  280 转速
            memcpy(&thro, frame->au8Data, 4);
            if(thro < 0) {thro = 0;}
            if(thro > 1000) {thro = 1000;}
            SETDATA(pDataPoolFly, "EngineRp", thro,  OS_U16);
			SetEngineThrot(thro / 10.0f);
            break;
        }
        case CMD_GET_START_PARAM: // 油泵停止
        {
            modECU_pumpOff();
            break;
        }
        case CMD_GET_RUNNING_PARAM: // 油泵开启
        {
            modECU_pumpOn();
            break;
        }
        case CMD_GET_RUNNING_INFO://地面控制的：控制是否给地面传输数据
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
        case CMD_START_STOP_ENGINE://地面发过来的启动命令
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

/** 解析数据：发动机 ——> 飞控  存储数据 & 发给数据链 */
void EngineHandler()   // HACK: TEST ECU restore & display
{
/** ****************014 新增******************* */
    static STRU_RUNNING_INFO param30 = {0};
    // static STRU_START_PARAM_INFO param31 = {0};
    // static STRU_RUNNING_PARAM_INFO param32 = {0};
    struct EngineStatus engineStatus = {0};

    if((g_DeviceState.CurrTick) % 20 == 0)  // 每100ms保存一次数据
    {
        modECU_GetEngineStatus(&engineStatus);
        /** **************** save ******************* */
        /* virables where engine PC software displayed */
        SETDATA(pDataPoolSelf, "ecuMPwr",   engineStatus.UMainPwr,                  OS_U16);
        SETDATA(pDataPoolSelf, "ecuIIgn1",  engineStatus.IIgnition1,                OS_U16);
        SETDATA(pDataPoolSelf, "ecuIIgn2",  engineStatus.IIgnition2,                OS_U16);
        SETDATA(pDataPoolSelf, "ecuFuPre",  engineStatus.Actual_fuel_pressure,      OS_U16);
        SETDATA(pDataPoolSelf, "ecuFuDut",  engineStatus.Fuel_pump_duty_cycle,      OS_U16);
        SETDATA(pDataPoolSelf, "ecuJet1",   engineStatus.Actual_jet1_duty_cycle,    OS_U16);
        SETDATA(pDataPoolSelf, "ecuJet2",   engineStatus.Actual_jet2_duty_cycle,    OS_U16);
        SETDATA(pDataPoolSelf, "ecuAirPr",  engineStatus.Air_pressure,              OS_U16);
        SETDATA(pDataPoolSelf, "ecuATemp",  engineStatus.Ambient_temperatur,        OS_U16);
        SETDATA(pDataPoolSelf, "ecuTemp1",  engineStatus.CH_Temperature1,           OS_U16);
        SETDATA(pDataPoolSelf, "ecuTemp2",  engineStatus.CH_Temperature2,           OS_U16);
        SETDATA(pDataPoolSelf, "ecuTemp3",  engineStatus.CH_Temperature3,           OS_U16);
        SETDATA(pDataPoolSelf, "ecuTemp4",  engineStatus.CH_Temperature4,           OS_U16);
        SETDATA(pDataPoolSelf, "ecuIgn1F",  engineStatus.Ignition1_on_off_flag,     OS_U16);
        SETDATA(pDataPoolSelf, "ecuIgn2F",  engineStatus.Ignition2_on_off_flag,     OS_U16);
        SETDATA(pDataPoolSelf, "ecuPumpF",  engineStatus.Pump_on_off_flag,          OS_U16);
        SETDATA(pDataPoolSelf, "ecuModeF",  engineStatus.RPM_Regulator_on_off_flag, OS_U16);
        SETDATA(pDataPoolSelf, "ecuChokF",  engineStatus.Choke_on_off_flag,         OS_U16);
        SETDATA(pDataPoolSelf, "ecuRPM",    engineStatus.Actual_RPM,                OS_U16);
        SETDATA(pDataPoolSelf, "ecuRPM2",   engineStatus.The_2nd_RPM_value,         OS_U16);
        /* warmup & coolingDown judgement */
        SETDATA(pDataPoolSelf, "ecuFbTho",	engineStatus.feedback_throttle, OS_U16);//86
        SETDATA(pDataPoolSelf, "ecuThoF",	engineStatus.throttle_state, OS_U16);//35
        /* ecu status */
        SETDATA(pDataPoolSelf, "ecuState",	engineStatus.CurState,          OS_U8);
        SETDATA(pDataPoolSelf, "ecuError",	engineStatus.ecuError,          OS_U8);

        /** **************** display ******************* */
        param30.runningStatus = engineStatus.CurState;         //当前发动机的状�?
        param30.error = engineStatus.ecuError;                 // 发动机错�?�?
        param30.fuel_pressure = engineStatus.Actual_fuel_pressure;    //实际油压
        param30.jet1_duty = engineStatus.Actual_jet1_duty_cycle;            // 实际喷油1脉�??
        param30.curRpm = engineStatus.Actual_RPM;                     // 实际�?�?
        param30.ambient_temp = engineStatus.Ambient_temperatur;      // �?境温�?
        param30.jet2_duty = engineStatus.Actual_jet2_duty_cycle;            // 实际喷油2脉�??
        param30.battV = engineStatus.UMainPwr * 0.01;             // 系统输入电压
        param30.battA = engineStatus.battA * 0.1;              // 系统输入电流
        param30.actual_throttle = engineStatus.feedback_throttle;// 期望的油门位�?
        param30.ch1_temp = engineStatus.CH_Temperature1;
        param30.ch2_temp = engineStatus.CH_Temperature2;
        param30.ch3_temp = engineStatus.CH_Temperature3;
        param30.ch4_temp = engineStatus.CH_Temperature4;
        param30.outputW = 0;

        if(SyncToGround)
        {
            MsgToDevice(RT_DATA_LINK, 0x30, sizeof(STRU_RUNNING_INFO), (OS_U8 *)&param30);
    //         MsgToDevice(RT_DATA_LINK, 0x31, sizeof(STRU_START_PARAM_INFO), (OS_U8 *)&param31);
    //         MsgToDevice(RT_DATA_LINK, 0x32, sizeof(STRU_RUNNING_PARAM_INFO), (OS_U8 *)&param32);
        }
    }
}

OS_U8 AutoDriveEnginePwm()//每100ms进来控制一次engine油门
{
    if(g_DeviceState.CurrTick % 20 == 0)
    {
        // GetCurEngineRpm();

    }
    if((g_DeviceState.CurrTick + 10) % 20 == 0)//每5ms * 20 = 100ms，进一次if
    {
        if(((DOM_AUTOMATIC & g_DeviceState.workStage) == DOM_AUTOMATIC) && flightSeq.umOpen != 1)
        {
            // 状态：正常飞行，且未收到伞降命令。        控制发动机的油门
            GetDataFast(pDataPoolFly, "EngineRp", &CurEngineRpm);
			SetEngineThrot(CurEngineRpm *0.1f);// 全局变量中存的应该是控制给出的油门开度 * 10
        }
    }
	return 0;
}

