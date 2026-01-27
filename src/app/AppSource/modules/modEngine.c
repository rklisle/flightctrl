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

/** 设置油门 */
void SetEngineThrot(float percent)
{
    unsigned int rpm = percent * 10.0f;//为了用上遥测表，我们将数据类型转化一下
    SETDATA(pDataPoolSelf,	"engSetRp",	rpm,  OS_U16);
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
            OS_U32 rpm;   //014 油门百分比*10 取值[0~1000]；  280 转速
            memcpy(&rpm, frame->au8Data, 4);
            SETDATA(pDataPoolFly, "EngineRp", rpm,  OS_U16);
			SetEngineThrot(rpm / 10.0f);// TODO: 油门设置需要再调整，目前是写软件时的临时设置
            break;
        }
        case CMD_GET_START_PARAM: // 280发动机协议，获取发动机启动参数
        {
            break;
        }
        case CMD_GET_RUNNING_PARAM: // 280发动机协议，获取发动机运行参数
        {
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

// /** 解析数据：发动机 ——> 飞控  存储数据 & 发给数据链 */
// OS_U32 EngineRtHandler(STRU_422_MSG_INFO * frame)   // MML发动机UART7 保存运行参数
// {
//     // switch(frame->u8MsgID)
//     // {
//     //     case 0x30:
//     //         SaveRunningInfo((STRU_RUNNING_INFO *)frame->au8Data);
//     //         break;
//     //     case 0x31:
//     //         SaveStartParam((STRU_START_PARAM_INFO *)frame->au8Data);
//     //         break;
//     //     case 0x32:
//     //         SaveRunningParam((STRU_RUNNING_PARAM_INFO *)frame->au8Data);
//     //         break;
//     // }
//     // g_DeviceState.ecuCountDown = 200;
//     // return 0;
// }

/** 解析数据：发动机 ——> 飞控  存储数据 & 发给数据链 */
void EngineHandler()   // MML发动机UART7 保存运行参数
{
    g_DeviceState.ecuCountDown = 200;
/** ****************014 新增******************* */
    static STRU_RUNNING_INFO param30 = {0};
    static STRU_START_PARAM_INFO param31 = {0};
    static STRU_RUNNING_PARAM_INFO param32 = {0};
    struct EngineStatus engineStatus = {0};
    modECU_GetEngineStatus(&engineStatus);
/** **************** 原先处理0x30需要用的参数 ******************* */
    param30.runningStatus = engineStatus.CntState;
    // param30.settingRpm = engineStatus.expect_rpm;
    param30.curRpm = engineStatus.rpm;
    param30.temp = engineStatus.ambient_temp;
    param30.Pa = engineStatus.air_pressure * 100;
    param30.runningSecond = engineStatus.runningMinite * 60;
    param30.battV = engineStatus.battV;
    param30.battA = engineStatus.battA;

    SETDATA(pDataPoolSelf, "ecuSetRp",	engineStatus.expect_rpm,	OS_U16);
    SETDATA(pDataPoolSelf, "ecuGetRp",  engineStatus.rpm,              OS_U16);
    SETDATA(pDataPoolSelf, "ecuTemp",   engineStatus.ambient_temp * 10,OS_U16);
    SETDATA(pDataPoolSelf, "ecuState",	engineStatus.CntState,	OS_U8);
    SETDATA(pDataPoolSelf, "ecuError",	0,	OS_U8);
    SETDATA(pDataPoolSelf, "fuelRate",	0,	OS_U16);
    SETDATA(pDataPoolSelf, "ecu24V", engineStatus.battV,	OS_S16);
    SETDATA(pDataPoolSelf, "ecu24A", engineStatus.battA,	OS_S16);
    
    param32.maxTemp = engineStatus.maxTemp;
    param32.totalMinite = engineStatus.totalMinite;
    param32.version = engineStatus.version;

    if(SyncToGround)
    {
        MsgToDevice(RT_DATA_LINK, 0x30, sizeof(STRU_RUNNING_INFO), (OS_U8 *)&param30);
/** **************** 原先处理0x31需要用的参数 ******************* */
        MsgToDevice(RT_DATA_LINK, 0x31, sizeof(STRU_START_PARAM_INFO), (OS_U8 *)&param31);
/** **************** 原先处理0x32需要用的参数 ******************* */
        MsgToDevice(RT_DATA_LINK, 0x32, sizeof(STRU_RUNNING_PARAM_INFO), (OS_U8 *)&param32);
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

