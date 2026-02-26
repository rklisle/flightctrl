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
    g_DeviceState.ecuCountDown = 200;
/** ****************014 新增******************* */
    static STRU_RUNNING_INFO param30 = {0};
    // static STRU_START_PARAM_INFO param31 = {0};
    // static STRU_RUNNING_PARAM_INFO param32 = {0};
    struct EngineStatus engineStatus = {0};

    if((g_DeviceState.CurrTick) % 20 == 0)  // 每100ms保存一次数据
    {
        modECU_GetEngineStatus(&engineStatus);
    /** **************** 存数据池 ******************* */
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

        SETDATA(pDataPoolSelf, "ecuState",	engineStatus.CntState,	OS_U8);
        SETDATA(pDataPoolSelf, "ecuError",	engineStatus.ecuError,	OS_U8);

    /** **************** 发数据链，显示 ******************* */
        param30.runningStatus = engineStatus.CntState;
        param30.fuel_pressure = engineStatus.fuel_pressure;
        param30.jet1_duty = engineStatus.jet1_duty;
        param30.curRpm = engineStatus.rpm;
        param30.ambient_temp = engineStatus.ambient_temp;
        param30.jet2_duty = engineStatus.jet2_duty;
        param30.battV = engineStatus.battV * 0.01;
        param30.battA = engineStatus.battA * 0.1;
        // param30.actual_throttle = engineStatus.actual_throttle;//97
        param30.actual_throttle = engineStatus.expect_throttle;//86
        param30.ch1_temp = engineStatus.ch1_temp;
        param30.ch2_temp = engineStatus.ch2_temp;
        param30.ch3_temp = engineStatus.ch3_temp;
        param30.ch4_temp = engineStatus.ch4_temp;
        param30.error = engineStatus.ecuError;

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

