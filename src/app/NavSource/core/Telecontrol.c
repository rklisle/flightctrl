/*
 * Telecontrol.c
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */

#include "Telecontrol.h"
#include "BusInteract.h"
#include "../modules/modFlash.h"
#include "../modules/modGPS.h"
#include "../modules/modFlyCtrl.h"
#include "../controller/controller.h"

OS_U32 CmdHandler(STRU_422_MSG_INFO * frame)
{
	//箭地协议是自己写的，能够保证msgID的有效性及互斥性
	OS_U8 msgId = frame->u8MsgID;
	switch(msgId)
	{
	//FLASH指令
	case CMD_FLASH_CTRL_REQ:		//flahs烧写请求
	case CMD_FLASH_ENCAP_REQ:		//flash烧写内容
	case CMD_FLASH_CHECK_REQ:		//flash烧写校验
	case CMD_FLASH_QUERY_REQ:		//查询文件列表
	case CMD_FLASH_CLEAR_REQ:		//清空发射诸元，不含模飞及程序
	case CMD_FLASH_LOAD_REQ:		//诸元数据加载
		FlashCmdHandler(frame);
		break;
	case CMD_GET_EPH:
	case CMD_SET_EPH:
		//GpsCmdHandler(frame);
        break;
    default:
        FlyctrlRtHandler(frame);
	}
	return 0;
}

OS_U8 CmdResponseHandler(OS_U8 msgID, OS_U16 msgLen, OS_U8* buf)
{
	MsgToDevice(RT_FLYCTRL, msgID, msgLen, buf);
	return 0;
}

