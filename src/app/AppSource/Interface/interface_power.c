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

OS_U8 currentPowerState = 0;

OS_S8 PowerOn(POWER_DEVICE dev)
{
    OS_U32 CANCmd_power = 0;
	switch(dev)
	{
    // 014 protocol
    case DEVICE_SCOUT_E28V:        // scout
        CANCmd_power = 0x10000000;
        break;
    case DEVICE_FUSE28V:	      // fuse deto
        CANCmd_power = 0x40000000;
        break;
    case DEVICE_FUSE_ISO5V:       // fuse
        CANCmd_power = 0x00010000;
        break;
    case DEVICE_SRV_PWR28V:	      // srv
        CANCmd_power = 0x04000000;
        break;
    default:
        break;
	}
    if(CANCmd_power == 0){
        return -1;  //can't find correct device
    }
    SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&CANCmd_power);

    currentPowerState |= (1u << dev);
    SETDATA(pDataPoolPwr, "PwrCmd", currentPowerState,	OS_U8);
	return 0;
}

OS_S8 PowerOff(POWER_DEVICE dev)
{
	OS_U32 CANCmd_power = 0;
	switch(dev)
	{
    // 014protocol
    case DEVICE_MBAT:	          // MBAT
        CANCmd_power = (0x01000000 << 1);
        break;
    case DEVICE_SCOUT_E28V:      // scout
        CANCmd_power = (0x10000000 << 1);
        break;
    case DEVICE_FUSE28V:	    // fuse deto
        CANCmd_power = (0x40000000 << 1);
        break;
    case DEVICE_FUSE_ISO5V:     // fuse
        CANCmd_power = (0x00010000 << 1);
        break;
    case DEVICE_SRV_PWR28V:	    // srv
        CANCmd_power = (0x04000000 << 1);
        break;
    default:
        break;
	}
    if(CANCmd_power == 0){
        return -1;  //can't find correct device
    }
    SendCanFrame(CAN_RT_POWERSEQ, 0x242, 4, (OS_U8 *)&CANCmd_power);

    currentPowerState |= (0u << dev);
    SETDATA(pDataPoolPwr, "PwrCmd", currentPowerState,	OS_U8);
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
        SETDATA(pDataPoolSelf, "seqDrop", 1,	OS_U8); // cut the umbrella
        break;
    case 1:
        SETDATA(pDataPoolSelf, "seqUmb", 1,	OS_U8);     // open the umbrella
        break;
    case 2:
        SETDATA(pDataPoolSelf, "seqSac1", 1,	OS_U8); // open airbag1
        break;
    case 3:
        SETDATA(pDataPoolSelf, "seqSac2", 1,	OS_U8); // open airbag2
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
