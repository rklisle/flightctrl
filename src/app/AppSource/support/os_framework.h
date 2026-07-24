/******************************************************************************

Copyright (C), 2022-2023, SpaceTransportation Co., Ltd.

******************************************************************************/
/******************************************************************************
File Name		: os_framework.h
Version			: 2.0
Author			: �ɺ�Z
Created			: 2022/07/23
******************************************************************************/
#include "os_types.h"
#include <string.h>
#include <stdio.h>
#ifndef _OS_FRAMEWORK_H_
#define _OS_FRAMEWORK_H_

#define WORK			(1)
#define SAFE		    (0)

#define OS_SUCCESS		(0)
#define OS_FAILURE		(-1)

typedef enum
{
	DOM_INTERACTIVE	= 0x1,	// �����(����/����׼��)�������� ��� ��ǰ���̣������˵��Ե���������е�һ�н����߼�;1) ����ɿغ���Ч�����������վ��Ӧ����ɡ����ָ��
	DOM_SIMIMUDAT	= 0x1<<1,	// ģ�����ģʽ��IMU���ݴ�FLASH��ȡ
	DOM_SIMSRVDAT	= 0x1<<2,	// �ŷ�С��·ģʽ���ŷ������ɵ���������Զ����ɲ��滻���
	DOM_TRIGGERON	= 0x1<<3,	// ʱ�򿪹أ��Ƿ����ʱ����
	DOM_AUTOMATIC	= 0x1<<4,	// �Զ����̣�����(�����)���Զ�ִ�еĹ��̣����ڵ���
	DOM_HILSMODE	= 0x1<<5,	// ��ʵ��ģʽ
	DOM_NAVON		= 0x1<<6,	// ��������
}FUNC_DOMAIN;

#pragma pack(1)
typedef struct
{
	OS_U8 u8HeadA;//EB
	OS_U8 u8HeadB;//90
	OS_U16 u16Len;//au8Data����
	OS_U8 u8Seq;//
	OS_U8 u8MsgID;	// ����վ�ͻ���֮��涨�õ�����
	OS_U8 au8Data[2048];
	OS_U8 u8CRCA;
	OS_U8 u8CRCB;
}STRU_422_MSG_INFO;	//422��Ϣ���ݸ�ʽ

typedef struct
{
	OS_U8 u8HeadA;
	OS_U8 u8HeadB;
	OS_U16 datalen;
	OS_U16 sendid;
	OS_U16 receiveid;
	OS_U8 type;
	OS_U8 u8Seq;
	OS_U8 au8Data[2048];
	OS_U8 u8CRC;
}LinkOUT_422_MSG_INFO;	//LinkOUT422��Ϣ���ݸ�ʽ
typedef struct
{
	OS_U8 u8RtIndex;	//����rt������
	STRU_422_MSG_INFO pStand422Data;
}STRU_STANDARD_FRAME;  //�����׼֡��ʽ���ȴ���422��ʽ��һ������422ͨ���ŵı���

typedef struct
{
	OS_U8 CanIndex;
	OS_U8 NodeIndex;	//����ڵ�ţ�Ĭ��0x25	//���
	OS_U16 MsgID;
	OS_U8 MsgLen;
	OS_U8 MsgData[8];
}STRU_CAN_MSG;  //�����׼֡��ʽ���ȴ���422��ʽ��һ������422ͨ���ŵı���

#pragma pack()

#define MathUtils_SignBit(x) (((signed char*)&x)[sizeof(x)-1]>>7|1)
#define LOBYTE(w)       ((BYTE)(w))
#define HIBYTE(w)       ((BYTE)(((UINT)(w) >> 8) & 0xFF))
//**************************************************************************

#define STANDARD_HEADA						(0xEB)
#define STANDARD_HEADB						(0x90)
#define STANDARD_HEADGPSA					(0xEB)
#define STANDARD_HEADGPSB					(0x90)

#define STANDARD_HEADGPSC					(0xFC)	//��δʹ��
#define STANDARD_HEADGPSD					(0x1D)	//��δʹ��

#define IMU_G0								(9.794265)	//�������ٶȳ����������䳡γ������

#define _422_FRAME_SYNCCHAR_LEN				(0x2)
#define _422_FRAME_HEADER_LEN				(0x6)
#define _422_FRAME_TM_HEADER_LEN			(0x7)
#define _422_FRAME_FOOTER_LEN				(0x2)
#define _422_PAYLOAD_MAX_LEN				(0x0FFF)
#define  _422_LinkFRAME_HEADER_LEN			(0xA)
#define _422_LinkFRAME_FOOTER_LEN			(0x1)



//#include "../support/qspi_flash.h"
#include "../support/support.h"
//#include "../support/time.h"
#include "./os_basic.h"
#include "./os_bufferQueue.h"
//#include "./os_bufferLoop.h"
#include "./os_error.h"
//#include "./os_time.h"
#include "../controller/controller.h"

//DEV �豸��
#define CHECK_HEAD					(0x01)	 //Ԥ����ͷ���Ƿ����Ϣͷ���бȶ�У��
#define DEV_CODE_FK					(0x01)  //�豸�ţ�DEV     ID
#define DEV_CODE_POWER				(0x0A)  //�豸�ţ�DEV     ID  ʱ���豸
#define DEV_CODE_SX					(0x0B)  //�豸�ţ�DEV     ID  ����豸

//MSG-ָ��-����
#define CMD_ENGINE_START			(0xF6)	//����������
#define CMD_ENGINE_STOP             (0xF7)	//������ͣ��
#define CMD_FORE_LAUNCH_REQ			(0xF8)	//����Ԥ���ָ��	//Ԥ���䣨δʹ�ã�
#define CMD_FORE_LAUNCH_RSP			(0xF9)	//����Ԥ���ָ����
#define CMD_LAUNCH_REQ				(0xFA)	//ȫ��������
#define CMD_LUANCH_FORCE			(0xFB)	//ǿ�Ʒ���ָ��		// 0xFB ��ҳ - ���

//MSG-ָ��-���ܿ�����״̬
#define CMD_MODULE_SET_REQ			(0x01)	//���ܿ�����ģʽ����
#define CMD_STATUS_REPORT			(0x02)	//���ܿ�����״̬�ϱ�

#define CMD_BJTIME_SET				(0x0A)	//����ʱ����

#define CMD_DATA_REQ				(0x03)	//��Ԫ�б�����
#define CMD_DATA_RSP				(0x04)	//��Ԫ�б��ϴ�
#define CMD_DATA_SET				(0x21)	//��Ԫ�б�����

#define CMD_URGENT_LAND				(0x22)	//紧急伞降
#define CMD_URGENT_RETURN			(0x23)	//紧急返航
#define CMD_INSTANT_RECOVER			(0x24)	//即时回收

//MSG_ָ��_ECU
#define	CMD_START_STOP_ENGINE		(0xC1)	// 0xC1 �ŷ�ʱ�� - ����ֹͣ������ - ���ݴ��Ĳ�����ͬ������ͣ��0x11���� 0x22ֹͣ
#define	CMD_GET_RUNNING_INFO		(0xC2)	// 0xC2 �ŷ�ʱ�� - ���һ���

#define	CMD_GET_RUNNING_PARAM		(0xC3)	// 0xC3 �ŷ�ʱ�� - ��ȡ���в���
#define	CMD_GET_START_PARAM			(0xC4)	// 0xC4 �ŷ�ʱ�� - ��ȡ��������
#define	CMD_ECU_RPM_SETTING			(0xC5)	//ת������	// 0xC5 �ŷ�ʱ�� - �����趨

//MSG-ָ��-���
#define CMD_POWER_REQ				(0x05)	//�����������		  // 0x05 �ŷ�ʱ�� - ��ͨ��ʱ����� - ����/��/��
#define CMD_SEQ_POWER_REQ			(0xEF)	//ʱ���������

//MSG-ָ��-����
#define CMD_NAV_INIT				(0xE4)	//�������Ԫ�͵�����
#define CMD_HOR_CALC_REQ			(0xE0)	//ˮƽ���㣨��׼������	 // 0xE0 �ŷ�ʱ�� - ��׼
#define CMD_TO_NAV_REQ				(0xE2)	//ת��������			// 0xE2 �ŷ�ʱ�� - ת����
#define CMD_TO_AFTER_LUANCH			(0xE3)	//ת��������			// 0xE3 �ŷ�ʱ�� - ת���
#define CMD_POLAR_TEST_REQ			(0x3A)	//���Բ���

//MSG ָ��-����
#define CMD_GET_EPH					(0xE7)	//������ȡ
#define CMD_GET_EPH_RSP				(0xE8)	//������ȡ�ظ�
#define CMD_SET_EPH					(0xE9)	//����װ��
#define CMD_SET_EPH_RSP				(0xEA)	//����װ���ظ�

//MSG-ָ��-SD
#define CMD_SD_READ_FILE			(0x50)	//��ȡSD���ļ�
#define CMD_SD_INIT					(0x51)	//��ȡSD���ļ�

//MSG-ָ��-�ŷ�
#define CMD_SRV_ZERO_ENCAP_REQ		(0x40)	//�ŷ���λװ������
#define CMD_SRV_CTRL_REQ			(0x42)	//�ŷ���������			// 0x42 �ŷ�ʱ�� - �趨
#define CMD_SRV_MINLOOP_REQ			(0x44)	//�ŷ�С��·��������	// 0x44 �ŷ�ʱ�� - С��·���� - ��ʼ
#define CMD_SRV_BOOKMODE			(0x48)	//�ŷ�װ��ģʽʹ��		// 0x48 �ŷ�ʱ�� - ��λװ��

//MSG-ָ��-���
#define CMD_BATT_CMD				(0x46)	//���ָ��

//MSG-ָ��-FLASH
#define CMD_FLASH_CTRL_REQ			(0x60)	//FLASH��������
#define CMD_FLASH_CTRL_RSP			(0x61)	//FLASH���ƽ��
#define CMD_FLASH_ENCAP_REQ			(0x62)	//FLASH������������
#define CMD_FLASH_ENCAP_RSP			(0x63)	//FLASH�������н��

#define CMD_FLASH_QUERY_REQ			(0x64)	//FLASH��ѯ
#define CMD_FLASH_QUERY_RSP			(0x65)	//FLASH��ѯ���
#define CMD_FLASH_CHECK_REQ			(0x66)	//FLASHУ������
#define CMD_FLASH_CHECK_RSP			(0x67)	//FLASHУ����
#define CMD_FLASH_LOAD_REQ			(0x68)	//��Ԫ���ݼ���
#define CMD_FLASH_LOAD_RSP			(0x69)	//��Ԫ���ݼ��ط���
#define CMD_FLASH_CLEAR_REQ			(0x6A)	//��Ԫ�������
#define CMD_FLASH_CLEAR_RSP			(0x6B)	//����ظ�

#define CMD_MSN_UPDATE              (0x13)	// 0x13 ��ҳ - ��������
#define CMD_MSN_NEWPT               (0x21)	// 0x21 ������ ����༭ - ���к���
//---------------------����---------------------------------------------
#define BUS_SLAVER_REPORT			(0x9F)
#define BUS_SLAVER_CMD				(0x9E)

//MSG-����-����
#define BUS_IMU_INFO_REPORT			(0x93)	//���˹��鷢�����ɿؼ�����Ĳ��Խ��
#define BUS_IMU_INFO_SEND			(0x94)	//�ɿؼ���������˹��鷢�͵�����
#define BUS_IMU_INFO_SEND_EPH		(0x3A)	//add by Li@20230514���ɿط��͸���������Э�������
#define BUS_IMU_SIMU_GPSIMU			(0xA0)	//��ʵ����������GPS���ݽṹ��ֱ��

#define BUS_NAV_INIT_DATA			(0xE0)	//��װģʽ����
#define BUS_NAV_FOCUS				(0xE2)	//��׼����
#define BUS_NAV_START_NAV			(0x3A)	//ת����
#define BUS_NAV_IGNATION			(0x3B)	//����
#define BUS_NAV_IMUDATA				(0x3C)	//�߾��������͵�����

//----------------------------ң��--------------------------------------------
//MSG-ң��
#define TM_FLIGHT					(0x9A)
#define TM_OTHER					(0x99)
//-----------------------------�غ�--------------------------------------------
//MSG-�غ�-ָ��

//MSG-�غ�-�ɿ�-����
#define BUS_FLIGHT_REPORT			(0xB0)
#define BUS_FLIGHT_LUNCH			(0xBA)
#define BUS_FLIGHT_INFO				(0xBF)

//MSG-SCOUT
#define SCOUT_AFRAME            	(0XAA)
#define SCOUT_BFRAME            	(0XBB)
#define TEMPLATE_TXT            	(0XB1)
#define IMAGE_INFO              	(0XB2)
#define IMAGE_REBACK            	(0XB3)
#define SOFT_UPDATE             	(0XD1)
#define SOFT_REBACKE            	(0XD2)
#define CMD_USER_SETTARGET    		(0xD3)
#define CMD_SET_IMAGEMODE			(0xD4)
//MSG������-----------------------------------------------------------------------
#pragma pack(1)
typedef struct
{
	/***************
	 * ��ң�ⲿ��
	 * *************/

	 /** workStage	����״̬ */
	// DOM_INTERACTIVE	= 0x1,		// �������̣������˵��Ե���������е�һ�н����߼���Ϊ��ǰ����
	// DOM_SIMIMUDAT	= 0x1<<1,	// ģ�����ģʽ��IMU���ݴ�FLASH��ȡ
	// DOM_SIMSRVDAT	= 0x1<<2,	// �ŷ�С��·ģʽ���ŷ������ɵ���������Զ����ɲ��滻���
	// DOM_TRIGGERON	= 0x1<<3,	// ʱ�򿪹أ��Ƿ����ʱ����
	// DOM_AUTOMATIC	= 0x1<<4,	// �Զ����̣�����Զ�ִ�еĹ��̣����ڵ���
	// DOM_HILSMODE		= 0x1<<5,	// ��ʵ��ģʽ
	// DOM_NAVON		= 0x1<<6,	// ��������
	FUNC_DOMAIN workStage;	//�����׶�
	
	/** ������ͨ��״̬����ʼ����Ϊ200tick��ÿ��tick����-1��ÿ���յ����ݻָ�200.��֤��ͨѶ�ж�1�����ܹ�������ң�� */
	OS_U8 srvCountDown;	// �豸ͨ��״̬
	OS_U8 battCountDown;	// �豸ͨ��״̬
	OS_U8 navCountDown;	// �豸ͨ��״̬
    OS_U8 powerCountDown;	// �豸ͨ��״̬
	OS_U8 ecuCountDown;	// �豸ͨ��״̬
	OS_U8 pwrStatePos;
	OS_U8 hilCountDown;	// �豸ͨ��״̬
    OS_U8 imuCountDown;	// �豸ͨ��״̬
    OS_U8 scoutCountDown;	// �豸ͨ��״̬
    OS_U8 fuseCountDown;	// �豸ͨ��״̬���ǲ���û���ϣ���
    
	//ʱ������������״̬
	OS_U8 pwrStateB1:1;
	OS_U8 pwrStateB2:1;
	OS_U8 pwrStateB3:1;
	OS_U8 pwrStateB4:1;
	OS_U8 pwrStateB6:1;
	OS_U8 pwrStateP15:1;
	OS_U8 pwrStateN15:1;
	OS_U8 pwrState5V:1;

	OS_U8 luanchState;//δ�õ�	
	OS_U8 detachState;//δ�õ�
	OS_FLOAT temperature;	//�ɿ��¶�
	OS_U8 luanchStart;		//����׼����ɱ�ʶ�����ڽ���
	
	OS_DOUBLE flightStartTime;	// ���ʱ�䣬���ɿس�ʼ��ʱ��ȥ�����ʱ�̵õ��ɿ�ʱ�䣬��λ s
	OS_U64 CurrTick;	// �����ʼ��ʱ��0���ӣ�ÿ5ms+1�����ʱ��DoIgnition�����У���ֵ0����״̬���У�ÿ5ms+1
	OS_DOUBLE currTime;// �����ʼ����0����+0.005s�����ʱ��0�����´�0����+0.005s	// ��λ s
	OS_U64 BJTimeSecond;//��ϵ����������ʱ�䣬�����գ�ʱ����
	OS_U16 BJTimeMS;	//��ʱ����
}DeviceState;

typedef struct
{
	OS_DOUBLE FzOnStamp_s;	// fuse power-on time	// ��λ s
	// OS_DOUBLE sysTime_s;	// ϵͳʱ�䣬��������ʱ0	// ��λ s
	OS_U8 msgFromGCS;	// message from Ground Control Station // ��������SD���ļ�д�롣��ʼ��ʱ����Ϊ0x00���յ���������������ʱ����Ϊ0x01
}DeviceStatus;
#pragma pack()
extern DeviceState g_DeviceState;
extern DeviceStatus g_DeviceStatus;

#endif
