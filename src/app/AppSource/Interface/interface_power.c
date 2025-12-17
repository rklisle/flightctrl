/*
 * interface_power.c
 *  Created on: 2022年8月7日
 *      Author: 成宏璟
 */
#include "interface_power.h"
#include "interface_can.h"
#include "../modules/modOnceBattery.h"
#include "../core/BusInteract.h"
#include "tx_api.h"

OS_U8 InitPwrSeq()
{
    OS_U32 openSeqCmd = 0x00000400;
    //tx_thread_sleep(20);
    SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&openSeqCmd);//open seq switch
    return 0;
}

OS_BOOL powerState[8];
OS_BOOL seqState[6];
OS_S8 PowerOn(POWER_DEVICE dev)
{
	powerState[dev-1] = TRUE;
    OS_U32 powerCmd = 0;
    OS_U8 battCmd = 0;
	switch(dev)
	{
	case DEVICE_MAIN_BATT://主电池
		powerCmd = (1<<0);
		break;
	case DEVICE_IMU_28V:	//对外供电
        powerCmd = (1<<2);
		break;
	case DEVICE_FUSE_1_E28V:	//引信
        powerCmd = (1<<4);
		break;
	case DEVICE_FUSE_2_ISO28V://引信点火电路
		powerCmd = (1<<6);
		break;
    case DEVICE_FUSE_5V://引信5V信号
		powerCmd = (1<<8);
		break;
	case DEVICE_BATT_ENGINE:	//发动机
		battCmd = 0x10;
		break;
    case DEVICE_BATT_SRV:	//舵机
		battCmd = 0x40;
		break;
    case DEVICE_BATT_BATT2:
        battCmd = 0x04;
        break;
	}
    if(powerCmd != 0)
    {
        SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&powerCmd);
    }
    if(battCmd != 0)
    {
        SendCanFrame(CAN_RT_BATT, 0x241, 1, (OS_U8 *)&battCmd);
    }
    OS_U8 bytePowerState = 0;
    for(int i=0;i<8;i++)
    {
        bytePowerState |= (powerState[i] << i);
    }
    SETDATA(pDataPoolPwr, "PwrCmd", bytePowerState,	OS_U8);
	return 0;
}

OS_S8 PowerOff(POWER_DEVICE dev)
{
	powerState[dev-1] = FALSE;
	OS_U32 powerCmd = 0;
    OS_U8 battCmd = 0;
	switch(dev)
	{
	case DEVICE_MAIN_BATT://主电池
		powerCmd = (1<<1);
		break;
	case DEVICE_IMU_28V:	
        powerCmd = (1<<3);
		break;
	case DEVICE_FUSE_1_E28V:	//引信
        powerCmd = (1<<5);
		break;
	case DEVICE_FUSE_2_ISO28V://引信点火电路
		powerCmd = (1<<7);
		break;
    case DEVICE_FUSE_5V://引信5V信号
		powerCmd = (1<<9);
		break;
    case DEVICE_BATT_ENGINE:	//发动机
		battCmd = 0x20;
		break;
	case DEVICE_BATT_SRV:	//舵机
		battCmd = 0x80;
		break;
    case DEVICE_BATT_BATT2:
        battCmd = 0x08;
        break;
	}
    if(powerCmd != 0)
    {
        SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&powerCmd);
    }
    if(battCmd != 0)
    {
        SendCanFrame(CAN_RT_BATT, 0x241, 1, (OS_U8 *)&battCmd);
    }
    OS_U8 bytePowerState = 0;
    for(int i=0;i<8;i++)
    {
        bytePowerState |= (powerState[i] << i);
    }
    SETDATA(pDataPoolPwr, "PwrCmd", bytePowerState,	OS_U8);
	return 0;
}

OS_U8 SeqCmd = 0;
OS_S8 SeqOn(OS_U8 channel)
{
	SeqCmd |= (1<<channel);
    SETDATA(pDataPoolPwr, "FireCmd", SeqCmd,	OS_U8);
	//Drv_SetSeqPulse(channel, TRUE);
    
    //Send seq to can2, which is the power board;
    switch(channel)
    {
    case 0:
        SETDATA(pDataPoolSelf, "seqDrop", 1,	OS_U8);
        break;
    case 1:
        SETDATA(pDataPoolSelf, "seqUmb", 1,	OS_U8);
        break;
    case 2:
        SETDATA(pDataPoolSelf, "seqSac1", 1,	OS_U8);
        break;
    case 3:
        SETDATA(pDataPoolSelf, "seqSac2", 1,	OS_U8);
        break;
    }
        
    OS_U32 cmd = 1 << (channel*2 + 12);
    SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&cmd);
	return 0;
}

OS_S8 SeqOff(OS_U8 channel)
{
	SeqCmd &= (~(1<<channel));
    SETDATA(pDataPoolPwr, "FireCmd", SeqCmd,	OS_U8);
	
    /*
    switch(channel)
    {
    case 0:
        SETDATA(pDataPoolSelf, "seqDrop", 0,	OS_U8);
        break;
    case 1:
        SETDATA(pDataPoolSelf, "seqUmb", 0,	OS_U8);
        break;
    case 2:
        SETDATA(pDataPoolSelf, "seqSac1", 0,	OS_U8);
        break;
    case 3:
        SETDATA(pDataPoolSelf, "seqSac2", 0,	OS_U8);
        break;
    }
    */
    OS_U32 cmd = 1 << (channel*2 + 1 + 12);
    SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&cmd);
	return 0;
}
