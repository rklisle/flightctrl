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

// OS_U8 InitPwrSeq()
// {
//     OS_U32 openSeqCmd = 0x00000400;
//     //tx_thread_sleep(20);
//     SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&openSeqCmd);//open seq switch//MML 开火工品4
//     return 0;
// }

OS_BOOL powerState[8];
OS_BOOL seqState[6];
OS_S8 PowerOn(POWER_DEVICE dev)
{
	powerState[dev-1] = TRUE;
    OS_U32 powerCmd = 0;
    OS_U8 battCmd = 0;
	switch(dev)
	{
    //MML 014协议
    case DEVICE_IMU_28V:        //导引头
        powerCmd = 0x10000000;
        break;
    case DEVICE_FUSE_2_ISO28V:	//引信可控电源
        powerCmd = 0x40000000;
        break;
    case DEVICE_FUSE_5V:        //引信5V信号
        powerCmd = 0x00010000;
        break;
    case DEVICE_BATT_SRV:	    //舵机
        powerCmd = 0x04000000;
        break;
	}
    if(powerCmd != 0)
    {
        SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&powerCmd);
    }
    if(battCmd != 0)
    {
        // SendCanFrame(CAN_RT_BATT, 0x241, 1, (OS_U8 *)&battCmd);
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
    //MML 014协议
    case DEVICE_MAIN_BATT:	    //主电池
        powerCmd = (0x01000000 << 1);
        break;
    case DEVICE_IMU_28V:        //导引头
        powerCmd = (0x10000000 << 1);
        break;
    case DEVICE_FUSE_2_ISO28V:	//引信可控电源
        powerCmd = (0x40000000 << 1);
        break;
    case DEVICE_FUSE_5V:        //引信5V信号
        powerCmd = (0x00010000 << 1);
        break;
    case DEVICE_BATT_SRV:	    //舵机
        powerCmd = (0x04000000 << 1);
        break;
	}
    if(powerCmd != 0)
    {
        SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&powerCmd);
    }
    if(battCmd != 0)
    {
        // SendCanFrame(CAN_RT_BATT, 0x241, 1, (OS_U8 *)&battCmd);
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
        SETDATA(pDataPoolSelf, "seqDrop", 1,	OS_U8); //MML 时序抛伞
        break;
    case 1:
        SETDATA(pDataPoolSelf, "seqUmb", 1,	OS_U8);     //MML 时序开伞
        break;
    case 2:
        SETDATA(pDataPoolSelf, "seqSac1", 1,	OS_U8); //MML 时序前气囊
        break;
    case 3:
        SETDATA(pDataPoolSelf, "seqSac2", 1,	OS_U8); //MML 时序后气囊
        break;
    }

    // 开某一路火工品的开关
    // OS_U32 cmd = 1 << (channel*2 + 12);
    // SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&cmd);
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
    // 关某一路火工品的开关
    // OS_U32 cmd = 1 << (channel*2 + 1 + 12);
    // SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&cmd);
	return 0;
}
