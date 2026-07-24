/*
 * Telecontrol.c
 *
 *  Created on: 2022��1��21��
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
	g_DeviceStatus.msgFromGCS = 0x01;//SD����ʼ�洢
	
	//�ǽ���ģʽʱ������������ָ��
	if((g_DeviceState.workStage & DOM_INTERACTIVE) !=  DOM_INTERACTIVE)
	{
		// ���е����˵�����������������ɡ�����ʱֻ�ܴ���������ɡ�������������������������ط�������...��ָ��
		if(msgId != CMD_URGENT_LAND && msgId != CMD_URGENT_RETURN && msgId != CMD_START_STOP_ENGINE && msgId != CMD_MSN_NEWPT && msgId != CMD_USER_SETTARGET && msgId != CMD_SET_IMAGEMODE)
		{
			return -1;
		}
	}
	//����Э�����Լ�д�ģ��ܹ���֤msgID����Ч�Լ�������
	// ����ģʽʱ���ܴ�������switch�е�ָ��
	switch(msgId)
	{
	case BUS_SLAVER_REPORT:
	case BUS_SLAVER_CMD:
		SlaverHandler(frame);
		break;
    case CMD_MSN_UPDATE://��������
    case CMD_MSN_NEWPT://��������
        MsnCmdHandler(frame);
        //UpdatePredictMsnByGround(frame);
        break;
	//״̬����ָ��
	case CMD_MODULE_SET_REQ:		//ģʽ����
	case CMD_BJTIME_SET:		//����ʱ��
		//����ָ��
    case CMD_ENGINE_START:		//����������
    case CMD_ENGINE_STOP:		//ֹͣ
	case CMD_FORE_LAUNCH_REQ:		//����	//Ԥ���䣨δʹ�ã�
	case CMD_LAUNCH_REQ:		//0xFA ȫ����ָ��
	case CMD_LUANCH_FORCE:
	//case CMD_DATA_REQ:
	//case CMD_DATA_SET:
	case CMD_URGENT_LAND:			//紧急伞降
	case CMD_URGENT_RETURN:			//紧急返航
	//case CMD_INSTANT_RECOVER:		//即时回收
		ControllerCmdHandler(frame);
		break;
	//ʱ�����ָ��
	case CMD_POWER_REQ:				//�������
	case CMD_SEQ_POWER_REQ:			//�������
		PwrCmdHandler(frame);
		break;
	//����ָ��
	case CMD_HOR_CALC_REQ:			//ˮƽ��������
	case CMD_TO_NAV_REQ:			//ת��������
	case CMD_POLAR_TEST_REQ:		//���Բ���
		NavCmdHandler(frame);
        ImuCmdHandler(frame);
		break;
    case CMD_TO_AFTER_LUANCH:
        NavCmdHandler(frame);
        break;
	//FLASHָ��
	case CMD_FLASH_CTRL_REQ:		//flahs��д����
	case CMD_FLASH_ENCAP_REQ:		//flash��д����
	case CMD_FLASH_CHECK_REQ:		//flash��дУ��
	case CMD_FLASH_LOAD_REQ:		//��Ԫ���ݼ���
	case CMD_FLASH_QUERY_REQ:
	case CMD_FLASH_CLEAR_REQ:
		FlashCmdHandler(frame);
		break;
	//�ŷ�ָ��
	case CMD_SRV_ZERO_ENCAP_REQ:	//��λװ����Ŀǰû�ã�
	case CMD_SRV_CTRL_REQ:			//�ŷ�����
	case CMD_SRV_MINLOOP_REQ:		//�ŷ�С��·
	case CMD_SRV_BOOKMODE:			//�ö��������λ
		ServoCmdHandler(frame);
		break;
	case CMD_START_STOP_ENGINE:
	case CMD_GET_RUNNING_INFO:
	case CMD_GET_RUNNING_PARAM:
	case CMD_GET_START_PARAM:
	case CMD_ECU_RPM_SETTING:
		EngineCmdHandler(frame);
		break;
		//SD��ָ��
	case CMD_SD_READ_FILE:	// 280 �˴�û��������case
	case CMD_SD_INIT:		// 280 �˴�û��������case
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
	
	//���͸����������дָ�ֱ��͸����������
	if(msgId >= 0x70 && msgId <= 0x7F)
	{
		MsgToDevice(RT_NAV, msgId- 0x10, frame->u16Len, frame->au8Data);
	}
	//�غ���ؼ���ͨѶЭ��
	//{

	//}
	return 0;
}

//Ӧ����Ϣ����
OS_U8 CmdResponseHandler(OS_U8 msgID, OS_U16 msgLen, OS_U8* buf)
{
	MsgToDevice(RT_DATA_LINK, msgID, msgLen, buf);
    //MsgToDevice(RT_HIL, msgID, msgLen, buf);
    //MsgToDevice(RT_P900, msgID, msgLen, buf);
	return 0;
}

