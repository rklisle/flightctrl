/*
 * Telecontrol.c
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */

#include "Telecontrol.h"
#include "BusInteract.h"
#include "../modules/modFlash.h"
#include "../modules/modOnceBattery.h"
#include "../modules/modSrvCtl.h"
#include "../modules/modPwrSeqCtl.h"
#include "../modules/modEngine.h"
#include "../modules/modMEMS.h"
#include "../controller/controller.h"
#include "../modules/modNav.h"
#include "../mission/mission.h"
#include "../payload/scout.h"

OS_U32 CmdHandler(STRU_422_MSG_INFO * frame)//02 //	RT_DATA_LINK
{
	OS_U8 msgId = frame->u8MsgID;
	SETDATA(pDataPoolSelf,	"gmsgId",	msgId,		OS_U8);
	//非交互模式时，不处理地面指令
	// MML 非交互模式时，只能处理“紧急伞降”、“紧急返航”、“开关发动机”...等指令
	if((g_DeviceState.workStage & DOM_INTERACTIVE) !=  DOM_INTERACTIVE)
	{
		if(msgId != CMD_URGENT_LAND && msgId != CMD_URGENT_RETURN && msgId != CMD_START_STOP_ENGINE && msgId != CMD_MSN_NEWPT && msgId != CMD_USER_SETTARGET && msgId != CMD_SET_IMAGEMODE)
		{
			return -1;
		}
	}
	//箭地协议是自己写的，能够保证msgID的有效性及互斥性
	// MML 交互模式时，能处理下列switch中的指令
	switch(msgId)
	{
	case BUS_SLAVER_REPORT:
	case BUS_SLAVER_CMD:
		SlaverHandler(frame);
		break;
    case CMD_MSN_UPDATE://更新任务
    case CMD_MSN_NEWPT://下载任务
        MsnCmdHandler(frame);
        //UpdatePredictMsnByGround(frame);
        break;
	//状态设置指令
	case CMD_MODULE_SET_REQ:		//模式设置
	case CMD_BJTIME_SET:		//设置时间
		//发射指令
    case CMD_ENGINE_START:		//启动发动机
    case CMD_ENGINE_STOP:		//停止
	case CMD_FORE_LAUNCH_REQ:		//发射	//预发射（未使用）
	case CMD_LAUNCH_REQ:		//0xFA 全部解指令
	case CMD_LUANCH_FORCE:
	//case CMD_DATA_REQ:
	//case CMD_DATA_SET:
	case CMD_URGENT_LAND:			//紧急伞降   zhang 20230608
	case CMD_URGENT_RETURN:			//紧急返航   zhang 20230608
	//case CMD_INSTANT_RECOVER:		//紧急回收   zhang 20230608
		ControllerCmdHandler(frame);
		break;
	//时序配电指令
	case CMD_POWER_REQ:				//单机配电
	case CMD_SEQ_POWER_REQ:			//无线配电
		PwrCmdHandler(frame);
		break;
	//惯组指令
	case CMD_HOR_CALC_REQ:			//水平计算请求
	case CMD_TO_NAV_REQ:			//转导航请求
	case CMD_POLAR_TEST_REQ:		//极性测试
		NavCmdHandler(frame);
        ImuCmdHandler(frame);
		break;
    case CMD_TO_AFTER_LUANCH:
        NavCmdHandler(frame);
        break;
	//FLASH指令
	case CMD_FLASH_CTRL_REQ:		//flahs烧写请求
	case CMD_FLASH_ENCAP_REQ:		//flash烧写内容
	case CMD_FLASH_CHECK_REQ:		//flash烧写校验
	case CMD_FLASH_LOAD_REQ:		//诸元数据加载
	case CMD_FLASH_QUERY_REQ:
	case CMD_FLASH_CLEAR_REQ:
		FlashCmdHandler(frame);
		break;
	//伺服指令
	case CMD_SRV_ZERO_ENCAP_REQ:	//零位装订（目前没用）
	case CMD_SRV_CTRL_REQ:			//伺服控制
	case CMD_SRV_MINLOOP_REQ:		//伺服小回路
	case CMD_SRV_BOOKMODE:			//让舵机设置零位
	case CMD_SRV_SET_ID:			//（目前没用）
	case CMD_SRV_SAVE:				//（目前没用）
		ServoCmdHandler(frame);
		break;
	case CMD_START_STOP_ENGINE:
	case CMD_GET_RUNNING_INFO:
	case CMD_GET_RUNNING_PARAM:
	case CMD_GET_START_PARAM:
	case CMD_ECU_RPM_SETTING:
		EngineCmdHandler(frame);
		break;
		//SD卡指令
	case CMD_SD_READ_FILE:	// 280 此处没处理这条case
	case CMD_SD_INIT:		// 280 此处没处理这条case
		ControllerCmdHandler(frame);
		break;
	case CMD_GET_EPH:
	case CMD_SET_EPH:
		//MsgToDevice(RT_NAV, msgId, frame->u16Len, frame->au8Data);
		break;
	case SCOUT_AFRAME:
	case TEMPLATE_TXT:
	case IMAGE_INFO:
	case SOFT_UPDATE:
	case CMD_USER_SETTARGET:
	case CMD_SET_IMAGEMODE:
		ScoutGroundHandler(frame);
	    break;
	}
	
	//发送给导航板的烧写指令，直接透传至导航板
	if(msgId >= 0x70 && msgId <= 0x7F)
	{
		MsgToDevice(RT_NAV, msgId- 0x10, frame->u16Len, frame->au8Data);
	}
	//载荷相关箭地通讯协议
	//{

	//}
	return 0;
}

OS_U8 CmdResponseHandler(OS_U8 msgID, OS_U16 msgLen, OS_U8* buf)
{
	MsgToDevice(RT_DATA_LINK, msgID, msgLen, buf);
    //MsgToDevice(RT_HIL, msgID, msgLen, buf);
    //MsgToDevice(RT_P900, msgID, msgLen, buf);
	return 0;
}

