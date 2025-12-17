/*
 * modEngineEle.c
 *
 *  Created on: 2024Äê4ÔÂ19ÈÕ
 *      Author: lenovo
 */

#include "modEngine.h"
#include "modOnceBattery.h"
#include "../core/DataPool.h"
#include "../core/BusInteract.h"
#include "../Interface/interface_uart.h"

OS_U32 CurEngineRpm = 0;
OS_U32 EngineRpmCmd = 0;
OS_U8 EngineStartStatus = 0;
OS_U8 SyncToGround = 0;
OS_U8 EngineInit()
{
	return 0;
}

OS_U16 ChkEngineFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != 0xFF && pmData[1] != 0xFE && pmData[1] != 0xFD )
	{
		return 0;
	}
    int crcPos = 0;
    if(pmData[1] == 0xFF)
    {
        crcPos = 53;       
    }
    if(pmData[1] == 0xFE)
    {
        crcPos = 29;
    }
    if(pmData[1] == 0xFD)
    {
        crcPos = 41;
    }
    OS_U8 read_crc = pmData[crcPos];
    OS_U8 calc_crc = CalCRC8(pmData + 1, crcPos - 1);
    if(read_crc != calc_crc)
    {
        return 0;
    }
    memmove(pmData + 7, pmData + 1, crcPos);
	pmData[3] = crcPos - 1;
	pmData[4] = 0;
	pmData[5] = 0;
	if(pmData[1] == 0xFF)
    {
        pmData[6] = 0x30;    
    }
    if(pmData[1] == 0xFE)
    {
        pmData[6] = 0x31;   
    }
    if(pmData[1] == 0xFD)
    {
       pmData[6] = 0x32;   
    }
	return crcPos + 9;
}

int SendEngineRpm(unsigned int rpm)
{
    SETDATA(pDataPoolSelf,	"engSetRp",	rpm,  OS_U16);
		OS_U8 EngineRpmData[8] = {0x48, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F};
    memcpy(EngineRpmData + 2, &rpm, 4);
    EngineRpmData[6] = CalCRC8(EngineRpmData, 6);
    UART_PutBuff(rtList[RT_ENGINE].chIndex, EngineRpmData, 8);
	return 0;
}

int StartEngine()
{
    OS_U8 StartData[4] = {0x48, 0x02, 0x03, 0x2F};
    {
        UART_PutBuff(rtList[RT_ENGINE].chIndex, StartData, 4);
        return 0;
    }
}
    
int StopEngine()
{
    OS_U8 StopData[4] = {0x48, 0x04, 0x05, 0x2F};
    {
        UART_PutBuff(rtList[RT_ENGINE].chIndex, StopData, 4);
        return 0;
    }
}

int GetCurEngineRpm()
{
    OS_U8 QueryData[4] = {0x50, 0x31, 0x30, 0x2F};
    UART_PutBuff(rtList[RT_ENGINE].chIndex, QueryData, 4);
    return 0;
}

OS_U32 EngineCmdHandler(STRU_422_MSG_INFO * frame)
{
	switch(frame->u8MsgID)
	{
        case CMD_ECU_RPM_SETTING:
        {
            OS_U32 rpm;
            memcpy(&rpm, frame->au8Data, 4);
            if(rpm > 60000)
                rpm = 60000;
            SETDATA(pDataPoolFly, "EnginePw", rpm,  OS_U16);
            SendEngineRpm(rpm);
            break;
        }
        case CMD_GET_START_PARAM:
        {
            OS_U8 data[4] = {0x50, 0x31, 0x31, 0x2F};
            UART_PutBuff(rtList[RT_ENGINE].chIndex, data, 4);
            break;
        }
        case CMD_GET_RUNNING_PARAM:
        {
            OS_U8 data[4] = {0x50, 0x31, 0x32, 0x2F};
            UART_PutBuff(rtList[RT_ENGINE].chIndex, data, 4);
            break;
        }
        case CMD_GET_RUNNING_INFO:
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
        case CMD_START_STOP_ENGINE:
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

OS_U8 SaveRunningInfo(STRU_RUNNING_INFO *data)
{
    if(data->runningStatus == 1)
    {
        EngineStartStatus = 1;
    }
    else
    {
        EngineStartStatus = 0;
    }
    
    SETDATA(pDataPoolSelf,	"ecuSetRp",	data->settingRpm,	OS_U16);
    SETDATA(pDataPoolSelf,	"ecuGetRp",	data->curRpm,	OS_U16);
    SETDATA(pDataPoolSelf,	"ecuTemp",	data->temp * 10,	OS_U16);
    SETDATA(pDataPoolSelf,	"ecuState",	data->runningStatus,	OS_U8);
    SETDATA(pDataPoolSelf,	"ecuError",	data->error,	OS_U8);
    SETDATA(pDataPoolSelf,	"fuelRate",	data->fuelConsum,	OS_U16);
    
    SETDATA(pDataPoolSelf, "ecu24V", data->battV * 10,	OS_S16);
		SETDATA(pDataPoolSelf, "ecu24A", data->battA * 10,	OS_S16);
    
    if(SyncToGround)
    {
        MsgToDevice(RT_DATA_LINK, 0x30, sizeof(STRU_RUNNING_INFO), (OS_U8 *)data);
//        MsgToDevice(RT_HIL, 0x30, sizeof(STRU_RUNNING_INFO), (OS_U8 *)data);
    }
    return 0;
}

OS_U8 SaveStartParam(STRU_START_PARAM_INFO *data)
{
    MsgToDevice(RT_DATA_LINK, 0x31, sizeof(STRU_START_PARAM_INFO), (OS_U8 *)data);
    //MsgToDevice(RT_HIL, 0x31, sizeof(STRU_START_PARAM_INFO), (OS_U8 *)data);
    return 0;
    
}

OS_U8 SaveRunningParam(STRU_RUNNING_PARAM_INFO *data)
{
    MsgToDevice(RT_DATA_LINK, 0x32, sizeof(STRU_RUNNING_PARAM_INFO), (OS_U8 *)data);
   // MsgToDevice(RT_HIL, 0x32, sizeof(STRU_RUNNING_PARAM_INFO), (OS_U8 *)data);
    return 0;
}

OS_U32 EngineRtHandler(STRU_422_MSG_INFO * frame)
{
    switch(frame->u8MsgID)
    {
        case 0x30:
            SaveRunningInfo((STRU_RUNNING_INFO *)frame->au8Data);
            break;
        case 0x31:
            SaveStartParam((STRU_START_PARAM_INFO *)frame->au8Data);
            break;
        case 0x32:
            SaveRunningParam((STRU_RUNNING_PARAM_INFO *)frame->au8Data);
            break;
    }
    g_DeviceState.ecuCountDown = 200;
    return 0;
}

OS_U8 AutoDriveEnginePwm()
{
    if(g_DeviceState.CurrTick % 20 == 0)
    {
        GetCurEngineRpm();
    }
    if((g_DeviceState.CurrTick + 10) % 20 == 0)
    {
        if(((DOM_AUTOMATIC & g_DeviceState.workStage) == DOM_AUTOMATIC) && flightSeq.umOpen != 1)
        {
            SendEngineRpm(CurEngineRpm);
        }
    }
	return 0;
}

