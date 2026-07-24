/*
 * modIMU.c
 *
 *  Created on: 2021��10��16��
 *      Author: QL
 */
#include "../StateMachine.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../core/Telecontrol.h"
#include "../interface/interface_uart.h"
#include "modPwrSeqCtl.h"
#include "modSD.h"
#include "../FlightSupport.h"
#include <math.h>
#include "modNav.h"
#include "../support/common.h"
#include "modHil.h"
#include "../interface/interface_power.h"
#include "../payload/fuse.h"

STRU_NAV_INPUT navInput = {0, 0, 0, 0, 1, 210, 0};
#define PI (3.1415926)
static double northDir = 0;//��λ��
static double Global = 0;  //���ؼ��ٶ�
static double Sigma0 = 0;  //������ת���ٶ�
double dpitch;
double dyaw;
double dpitchUnHor;
double dyawUnHor;

//�򵼺�����ָ����Ϣ
OS_U8 MsgToNAV(OS_U8 msgID, OS_U8 *data, OS_U8 len)
{
	MsgToDevice(RT_NAV, msgID, len, data);

	return 0;
}

/***********************************************************
 * ��������: DoHorizonCalc()
 * ��������: ˮƽ�����㷨������
 * �����Ϣ�������ǡ�����ǡ���ת�ǡ���γ�ߡ��ͼ��ٶȡ��ͽ��ٶ�
 * ����:	���Ʋ���
 ***********************************************************/
static OS_S32 HorizontalCalc(OS_DOUBLE ax,
					   		 OS_DOUBLE ay,
					   		 OS_DOUBLE az,
					   		 OS_DOUBLE wx,
					   		 OS_DOUBLE wy,
					   		 OS_DOUBLE wz,
					   		 OS_DOUBLE lon,
					   		 OS_DOUBLE lat,
					   		 OS_DOUBLE height,
					   		 OS_DOUBLE* pPItch,
					   		 OS_DOUBLE* pYaw,
					   		 OS_DOUBLE* pNorth,
					   		 OS_DOUBLE* pLon,
					   		 OS_DOUBLE* pLat,
					   		 OS_DOUBLE* pHeight,
					   		 OS_DOUBLE* pGlocal,
					   		 OS_DOUBLE* pSigma0,
					   		 OS_U32 u8CalcCnt)
{
	static OS_DOUBLE s_AxsumRocket = 0;
	static OS_DOUBLE s_AysumRocket = 0;
	static OS_DOUBLE s_AzsumRocket = 0;
	static OS_DOUBLE s_WxsumRocket = 0;
	static OS_DOUBLE s_WysumRocket = 0;
	static OS_DOUBLE s_WzsumRocket = 0;

	static OS_DOUBLE s_AxsumIMU = 0;
	static OS_DOUBLE s_AysumIMU = 0;
	static OS_DOUBLE s_AzsumIMU = 0;
	static OS_DOUBLE s_WxsumIMU = 0;
	static OS_DOUBLE s_WysumIMU = 0;
	static OS_DOUBLE s_WzsumIMU = 0;

	static OS_DOUBLE s_Lonsum = 0;
	static OS_DOUBLE s_Latsum = 0;
	static OS_DOUBLE s_Heightsum = 0;
	OS_DOUBLE axAverage,ayAverage,azAverage,wxAverage,wyAverage,wzAverage,d_pitch,pitch,yaw,globalIn;

	switch(u8CalcCnt)
	{
	case 0:	//��ʼ����
		s_AxsumIMU= s_AysumIMU= s_AzsumIMU=
		s_WxsumIMU= s_WysumIMU= s_WzsumIMU=
		s_Lonsum= s_Latsum= s_Heightsum=0;
		break;
	default: 
		//������ٶȡ����ٶȡ���γ�߾�ֵ
		s_AxsumIMU += ax;
		s_AysumIMU += ay;
		s_AzsumIMU += az;
		s_WxsumIMU += wx;
		s_WysumIMU += wy;
		s_WzsumIMU += wz;
		s_Lonsum += lon;
		s_Latsum += lat;
		s_Heightsum += height;

		s_AxsumRocket = s_AxsumIMU; //��������ϵ����������ϵ��ת���������ͺ��Լ���װ�����Ӧ
		s_AysumRocket = s_AysumIMU;
		s_AzsumRocket = s_AzsumIMU;
		s_WxsumRocket = s_WxsumIMU;
		s_WysumRocket = s_WysumIMU;
		s_WzsumRocket = s_WzsumIMU;

		axAverage = s_AxsumRocket / u8CalcCnt;   //ȡˮƽ����180s�ڵ�ƽ��ֵ
		ayAverage = s_AysumRocket / u8CalcCnt;
		azAverage = s_AzsumRocket / u8CalcCnt;
		wxAverage = s_WxsumRocket / u8CalcCnt;
		wyAverage = s_WysumRocket / u8CalcCnt;
		wzAverage = s_WzsumRocket / u8CalcCnt;

		if(pLon) *pLon = s_Lonsum / u8CalcCnt;
		if(pLat) *pLat = s_Latsum / u8CalcCnt;
		if(pHeight) *pHeight = s_Heightsum / u8CalcCnt;

		//�ͼ��ٶȡ����ٶ�
		globalIn = sqrt(axAverage * axAverage + ayAverage * ayAverage + azAverage * azAverage);
		if(pGlocal) *pGlocal = globalIn;
		//*pGlocal = axAverage;
		if(pSigma0) *pSigma0 = sqrt(wxAverage * wxAverage + wyAverage * wyAverage + wzAverage * wzAverage)*3600;

		///1���޳�����
		d_pitch = - asin(ayAverage / globalIn); //������ = ������ٶ�/�ϼ��ٶ�
		pitch = d_pitch + PI / 2;
		yaw = asin(azAverage / globalIn / sin(pitch)); //�����

		///3���޳�����
		//d_pitch = asin(ayAverage / globalIn); //���ݷ�������ϵ���������ϵ��ϵȷ������
		//pitch = d_pitch + PI / 2;
		//yaw = - asin(azAverage / globalIn / sin(pitch)); //ͬ��

		//��������ǡ�����ǡ���ת��
		if(pPItch) *pPItch = toDeg(d_pitch);
		if(pYaw) *pYaw = toDeg(yaw);
		if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage));//Y1 3���޳����򣬲���Ҫ�ټ�pi
		if(*pNorth < 0)
			*pNorth += 360;
		//Y6 if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage)+ PI); //g_HorizontalCalc_north �����ΧΪ-pi/2~pi/2,��Է�λ�Ƿ�ĸ�������0 2021521 ��
		break;
	}
	return 1;
}

/***********************************************************
 * ��������: DoHorizonCalc()
 * ��������: ˮƽ���������������������Ϊ:
 * 			(1)hCalcCnt	ˮƽ���㵱ǰ�ļ���
 * 			(2)hCalcTotalCnt ˮƽ����Ľ��������
 * 			����������Ϊ0ʱ������Ҫ����ˮƽ���㡣
 * 			���յ�����ˮƽ��������󣬻ὫhCalcTotalCnt��Ϊ180��hCalcCnt��Ϊ0������ʼ�����ۼӼ���������ˮƽ���㡣
 * ����:	�ɺ�Z
 ***********************************************************/
static OS_U32 hCalcCnt = 0;//ˮƽ��̬�������
static OS_U32 hCalcTotalCnt = 0;//ת������46s = 46 * 200 * 5ms
static OS_U8 DoHorizonCalc()
{
	//�ϼ��ٶȡ����ٶȸ���
	if(hCalcTotalCnt)
	{
		//�������ʱ������㣬���ٸ�������
		if(hCalcCnt == hCalcTotalCnt)
		{
			hCalcCnt = 0;
			hCalcTotalCnt = 0;
			return 0;
		}
		//ÿ1s = 5ms*200������һ������
		else if(hCalcCnt % 200 == 0)
		{//�ڳ�������Ĺ����У�Ҳÿ��1���´�һ�μ�������
			SETDATA(pDataPoolNav, "Global", Global, OS_DOUBLE);//���ؼ��ٶ�
			SETDATA(pDataPoolNav, "Sigma0", Sigma0, OS_DOUBLE);//�ϳɽ��ٶ�
		}
	}

	//ˮƽ����㣺�����ǡ�����ǡ���ת�ǡ���γ�ߡ��ͼ��ٶȡ��ͽ��ٶȼ���
	float fax,fay,faz,fwx,fwy,fwz;
	GetDataFast(pDataPoolNav, "navWx", &fwx);//ԭʼ���ٶ�X
	GetDataFast(pDataPoolNav, "navWy", &fwy);//ԭʼ���ٶ�Y
	GetDataFast(pDataPoolNav, "navWz", &fwz);//ԭʼ���ٶ�Z
	GetDataFast(pDataPoolNav, "navAx", &fax);//ԭʼ���ٶ�X
	GetDataFast(pDataPoolNav, "navAy", &fay);//ԭʼ���ٶ�Y
	GetDataFast(pDataPoolNav, "navAz", &faz);//ԭʼ���ٶ�Z
	//���hCalcTotalCntΪ0,˵��ˮƽ����δ����
	//���hCalcCnt�Ѿ�����180*200ʱ��˵��ˮƽ�����Ѿ�����
	//ֻ�е����淢��ˮƽ����ָ���hCalcTotalCnt��ֵ����hCalcCnt��0,��ʼÿ5ms��һ��ˮƽ����
	double ax = fax,ay=fay,az=faz,wx=fwx,wy=fwy,wz=fwz;
	if(hCalcTotalCnt && hCalcCnt<hCalcTotalCnt)
	{
		HorizontalCalc(ax,
						ay,
						az,
						wx,
						wy,
						wz,
						0,
						0,
						0,
						&dpitchUnHor,//������
						&dyawUnHor,
						&northDir,//�淽λ��
						NULL,
						NULL,
						NULL,
						&Global,
						&Sigma0,
						hCalcCnt++);
	}

	return 1;
}

//�򵼺�����ָ��
OS_U32 NavCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case CMD_NAV_INIT:
		{
			//�ڶ�׼��ʱ����ע��ʼ����
			OS_S8 InitData[100];
			//ˮƽ��װ
			//InitData[0] = 1;
			//InitData[1] = -3;
			//InitData[2] = 2;
			//��ֱ��װ
			InitData[0] = 1;
			InitData[1] = 3;
			InitData[2] = -2;
    
			memcpy(InitData + 3, &navInput.InitLon, 8);
			memcpy(InitData + 11, &navInput.InitLat, 8);
			memcpy(InitData + 19, &navInput.InitHigh, 8);
			memcpy(InitData + 27, &navInput.InitYaw, 8);
			// InitData[35] = (OS_S8)navInput.navAlignMode;
    		// InitData[36] = (OS_U8)(navInput.navAlignTime & 0xFF);
    		// InitData[37] = (OS_U8)(navInput.navAlignTime >> 8);
			MsgToDevice(RT_NAV , BUS_NAV_INIT_DATA, 35, (OS_U8*)InitData);
			//MsgToDevice(RT_NAV , BUS_NAV_INIT_DATA, 38, (OS_U8*)InitData);
		}
		break;
		case CMD_HOR_CALC_REQ://��׼���� ��ˮƽ��׼�ʹ�ֱ��׼����ģʽ
		{

			if(frame->au8Data[0] == 0)//ˮƽ��׼
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 2,		OS_U8);
			}
			else if(frame->au8Data[0] == 1)//��ֱ��׼
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 1,		OS_U8);
			}
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_FOCUS, 0, (OS_U8*)&toNav);
		}
		break;
		case CMD_TO_NAV_REQ: //������ϵ�������
		{
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_START_NAV, 0, (OS_U8*)&toNav);
			g_DeviceState.workStage |= DOM_NAVON;//ת����ģʽ

			//����������ˮƽ����
			hCalcCnt = 0;
			OS_U8 calcSecond = 46;
			hCalcTotalCnt = calcSecond * 200;
		}
			break;
        case CMD_TO_AFTER_LUANCH:
        {
			OS_U8 toNav[1] = {0};
            MsgToNAV(BUS_NAV_IGNATION, toNav, 0);
        }
            break;
		default:
			break;
	}
	return 0;
}

//���յ��������ݺ󣬻������ݵ����ݳأ����ڷɿ�ʹ�ú�ң�⣬�Լ�����վ��ϢӦ��
OS_U32 NavRtHandler(STRU_422_MSG_INFO * frame)	// RT_NAV
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	//��ʱ������ϵ�������֡ (����5ms)
	case BUS_IMU_INFO_REPORT:
		{
			//�������ݵ����ݳأ����ڷɿ�ʹ�ú�ң��
			STRU_NAV_INFO navInfo;
			memcpy(&navInfo, frame->au8Data, sizeof(navInfo));
			SaveNavInDataPool(&navInfo);	//�洢��������
			g_DeviceState.navCountDown = 200;

			//������׼
			if(navInfo.navStatus == 0x44)
			{
				STRU_422_MSG_INFO frame;
				frame.u8MsgID = CMD_NAV_INIT;
				NavCmdHandler(&frame);
			}

			//ˮƽ���㿴ָ���Ƿ�Ѽ���ʱ�������ˣ���������˾����������û���þ�����
			DoHorizonCalc();
		}
		break;
	//����Ӧ������֡
	case CMD_GET_EPH_RSP:
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
		break;
	}
	
	//����վ��ϢӦ��
	if(msgID >= 0x60 && msgID <= 0x6F)
	{
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
	}
	return 0;
}

// NAV�����������ݣ�����������Ҫ����Щ���ݸ�ֵ��IMU�����ݳ�
OS_U8 SaveNavInDataPool(STRU_NAV_INFO *navInfo)
{
	SETDATA(pDataPoolNav, "gpsMod", navInfo->GPSstate,	OS_U8);//GPS״̬
	SETDATA(pDataPoolNav, "gpsLoCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS��λ����
	SETDATA(pDataPoolNav, "gpsLoMas", navInfo->gpsDirEnable[0] == 'V'?1:0, OS_U8);
	SETDATA(pDataPoolNav, "gpsLoSla", navInfo->gpsDirEnable[1] == 'V'?1:0, OS_U8);
	SETDATA(pDataPoolNav, "gpsLon", navInfo->GPSlon,	OS_S32);//GPS����
	SETDATA(pDataPoolNav, "gpsLat", navInfo->GPSlat,	OS_S32);//GPSγ��
	SETDATA(pDataPoolNav, "gpsHigh", navInfo->GPShigh * 1e-3,	OS_S16);//GPS�߶�
	SETDATA(pDataPoolNav, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS����
	SETDATA(pDataPoolNav, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS����
	SETDATA(pDataPoolNav, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS����
	SETDATA(pDataPoolNav, "gpsPdop", navInfo->PDOP,		OS_U16);//PDOP
	SETDATA(pDataPoolNav, "gpsGdop", navInfo->GDOP,		OS_U16);//GDOP
	SETDATA(pDataPoolNav, "gpsDelay", navInfo->Deltime,	OS_U8);//PPS
	SETDATA(pDataPoolNav, "gpsUload", navInfo->uploadEphStatus,	OS_U8);//����װ�����

	SETDATA(pDataPoolNav, "gpsYear", navInfo->year,	OS_U8);//GPS��
	SETDATA(pDataPoolNav, "gpsMonth", navInfo->month,	OS_U8);//GPS��
	SETDATA(pDataPoolNav, "gpsDay", navInfo->day,	OS_U8);//GPS��
	SETDATA(pDataPoolNav, "gpsHour", navInfo->hour,	OS_U8);//GPSʱ
	SETDATA(pDataPoolNav, "gpsMinit", navInfo->minite,	OS_U8);//GPS��
	SETDATA(pDataPoolNav, "gpsSec", navInfo->second,	OS_U8);//GPS��
	SETDATA(pDataPoolNav, "gpsMs", navInfo->ms,	OS_U16);//GPS����
	SETDATA(pDataPoolNav, "gpsTrack", navInfo->gpsTrack,	OS_U16);//GPS������
	SETDATA(pDataPoolNav, "gpsDir", navInfo->gpsDir,	OS_U16);//GPS�����
	SETDATA(pDataPoolNav, "gpsDirOK", navInfo->gpsDirEffect,	OS_U8);//GPS������Ч��־

	if((g_DeviceState.workStage & DOM_HILSMODE) && hilInput.useNav == 0)
	{
	}
	else
	{
		SETDATA(pDataPoolImu, "gpsLon", navInfo->GPSlon,			OS_S32);//GPS����
		SETDATA(pDataPoolImu, "gpsLat", navInfo->GPSlat,			OS_S32);//GPSγ��
		SETDATA(pDataPoolImu, "gpsAlt", navInfo->GPShigh * 1e-3,	OS_S16);//GPS�߶�
		SETDATA(pDataPoolImu, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS����
		SETDATA(pDataPoolImu, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS����
		SETDATA(pDataPoolImu, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS����
		SETDATA(pDataPoolImu, "dirEffec",	navInfo->gpsDirEffect,	OS_U8);//GPS������Ч��־

		SETDATA(pDataPoolImu, "gpsYear",  navInfo->year,   OS_U8);	//GPS��
		SETDATA(pDataPoolImu, "gpsMonth", navInfo->month,  OS_U8);	//GPS��
		SETDATA(pDataPoolImu, "gpsDay",   navInfo->day,    OS_U8);	//GPS��
		SETDATA(pDataPoolImu, "gpsHour",  navInfo->hour,   OS_U8);	//GPSʱ
		SETDATA(pDataPoolImu, "gpsMinit", navInfo->minite, OS_U8);	//GPS��
		SETDATA(pDataPoolImu, "gpsSec",   navInfo->second, OS_U8);	//GPS��
		SETDATA(pDataPoolImu, "gpsMSec",  navInfo->ms,     OS_U8);	//GPS����
		SETDATA(pDataPoolImu, "gpsDir", navInfo->gpsDir,  OS_U16);
		SETDATA(pDataPoolImu, "gpsScCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS��λ����

		SETDATA(pDataPoolImu, "imuWx", navInfo->imuWx16507,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuWy", navInfo->imuWy16507,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuWz", navInfo->imuWz16507,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAx", navInfo->imuAx16507,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAy", navInfo->imuAy16507,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAz", navInfo->imuAz16507,	OS_FLOAT);

		SETDATA(pDataPoolImu, "navLon",  navInfo->s32navLon,			OS_S32	);
		SETDATA(pDataPoolImu, "navLat",  navInfo->s32navLat,			OS_S32	);
		SETDATA(pDataPoolImu, "navHigh", navInfo->s32navHigh * 1e-3,	OS_FLOAT);

		SETDATA(pDataPoolImu, "navVn", navInfo->s32navVn * 1e-1,	OS_S16);
		SETDATA(pDataPoolImu, "navVs", navInfo->s32navVs * 1e-1,	OS_S16);
		SETDATA(pDataPoolImu, "navVe", navInfo->s32navVe * 1e-1,	OS_S16);

		SETDATA(pDataPoolImu, "navPitch", navInfo->s16pitch,	OS_S16);	
		SETDATA(pDataPoolImu, "navRoll", navInfo->s16roll,	OS_S16);  
		SETDATA(pDataPoolImu, "navState", navInfo->navStatus,	OS_U8);
		SETDATA(pDataPoolImu, "navDir", navInfo->s16dir,	OS_U16);

		//�����жϵ���״̬׼�����Һ�����Ч��־��Ч
		if((navInfo->navStatus == 0) && (navInfo->gpsDirEffect == 1))
		{
			float navdirmid;
			navdirmid = navInfo->gpsDir / 100 + 180;
			if(navdirmid > 360)
				navdirmid = navdirmid - 360;
			SETDATA(pDataPoolImu, "navDir", navdirmid * 1e2,	OS_U16);
			SETDATA(pDataPoolImu, "navLon", navInfo->GPSlon,	OS_S32);
			SETDATA(pDataPoolImu, "navLat", navInfo->GPSlat,	OS_S32);
			SETDATA(pDataPoolImu, "navHigh", navInfo->GPShigh * 1e-3,	OS_FLOAT);
		}
	}

	SETDATA(pDataPoolNav, "navLon", navInfo->s32navLon,	OS_S32);//��������
	SETDATA(pDataPoolNav, "navLat", navInfo->s32navLat,	OS_S32);//����γ��
	SETDATA(pDataPoolNav, "navHigh", navInfo->s32navHigh * 1e-3,OS_FLOAT);//�����߶�
	SETDATA(pDataPoolNav, "navVn", navInfo->s32navVn * 1e-1,	OS_S16);//��������
	SETDATA(pDataPoolNav, "navVs", navInfo->s32navVs * 1e-1,	OS_S16);//��������
	SETDATA(pDataPoolNav, "navVe", navInfo->s32navVe * 1e-1,	OS_S16);//��������

	SETDATA(pDataPoolNav, "navPitch", navInfo->s16pitch,	OS_S16);//��������
	SETDATA(pDataPoolNav, "navRoll", navInfo->s16roll,	OS_S16);//������ת
	SETDATA(pDataPoolNav, "navDir", navInfo->s16dir,	OS_U16);//������λ��

	SETDATA(pDataPoolNav, "navWx", navInfo->imuWx16507,	OS_FLOAT);//ԭʼ���ٶ�X
	SETDATA(pDataPoolNav, "navWy", navInfo->imuWy16507,	OS_FLOAT);//ԭʼ���ٶ�Y
	SETDATA(pDataPoolNav, "navWz", navInfo->imuWz16507,	OS_FLOAT);//ԭʼ���ٶ�Z
	SETDATA(pDataPoolNav, "navAx", navInfo->imuAx16507,	OS_FLOAT);//ԭʼ���ٶ�X
	SETDATA(pDataPoolNav, "navAy", navInfo->imuAy16507,	OS_FLOAT);//ԭʼ���ٶ�Y
	SETDATA(pDataPoolNav, "navAz", navInfo->imuAz16507,	OS_FLOAT);//ԭʼ���ٶ�Z


	// SETDATA(pDataPoolNav, "navWx2", navInfo->imuWx20689,	OS_FLOAT);//ԭʼ���ٶ�X
	// SETDATA(pDataPoolNav, "navWy2", navInfo->imuWy20689,	OS_FLOAT);//ԭʼ���ٶ�Y
	// SETDATA(pDataPoolNav, "navWz2", navInfo->imuWz20689,	OS_FLOAT);//ԭʼ���ٶ�Z
	// SETDATA(pDataPoolNav, "navAx2", navInfo->imuAx20689,	OS_FLOAT);//ԭʼ���ٶ�X
	// SETDATA(pDataPoolNav, "navAy2", navInfo->imuAy20689,	OS_FLOAT);//ԭʼ���ٶ�Y
	// SETDATA(pDataPoolNav, "navAz2", navInfo->imuAz20689,	OS_FLOAT);//ԭʼ���ٶ�Z
    
	// SETDATA(pDataPoolNav, "navWx3", navInfo->imuWx42688,	OS_FLOAT);//ԭʼ���ٶ�X
	// SETDATA(pDataPoolNav, "navWy3", navInfo->imuWy42688,	OS_FLOAT);//ԭʼ���ٶ�Y
	// SETDATA(pDataPoolNav, "navWz3", navInfo->imuWz42688,	OS_FLOAT);//ԭʼ���ٶ�Z
	// SETDATA(pDataPoolNav, "navAx3", navInfo->imuAx42688,	OS_FLOAT);//ԭʼ���ٶ�X
	SETDATA(pDataPoolNav, "navAy3", navInfo->imuAy42688,	OS_FLOAT);//ԭʼ���ٶ�Y
	SETDATA(pDataPoolNav, "navAz3", navInfo->imuAz42688,	OS_FLOAT);//ԭʼ���ٶ�Z

	SETDATA(pDataPoolNav, "navState", navInfo->navStatus ,	OS_U8);
	//cpu0�����ʱ�������壩
	SETDATA(pDataPoolNav, "navUs", navInfo->navUs,	OS_U16);
	SETDATA(pDataPoolNav, "navUsKa", navInfo->navUsKa,	OS_U16);
    SETDATA(pDataPoolSelf, "cpuTemp2", navInfo->cpuTemp,	OS_S16);
    

//	CalcXYZ();
/*
	char sdRow[2000] = {0};
	sprintf(sdRow, "%.3f,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%c%c,%d,%d,%.2f\n", g_DeviceState.currTime,
			navInfo->s32GPSlon*1e-7, navInfo->s32GPSlat*1e-7, navInfo->s32GPShigh*1e-3,
			navInfo->s32GPSVn*1e-2, navInfo->s32GPSVs*1e-2,navInfo->s32GPSVe*1e-2,
			navInfo->StanumberGPS + navInfo->StanumberBD,navInfo->GPSstate,
			navInfo->PDOP * 1e-2, navInfo->gpsDir* 1e-2, navInfo->gpsDirEnable[0],
			navInfo->gpsDirEnable[1], navInfo->gpsupdate, navInfo->gpsDirEffect,
			navInfo->gpsTrack * 1e-2);
	WriteToSD(2, (OS_U8 *)sdRow, strlen(sdRow));

	char sdRow1[2000] = {0};
	sprintf(sdRow1, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", g_DeviceState.currTime,
				navInfo->f32imuAx, navInfo->f32imuAy, navInfo->f32imuAz,
				navInfo->f32imuWx, navInfo->f32imuWy, navInfo->f32imuWz);
	WriteToSD(3, (OS_U8 *)sdRow1, strlen(sdRow1));

	char sdRow2[2000] = {0};
	sprintf(sdRow2, "%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.7f,%.7f,%.3f,%d,%.2f,%.2f,%.2f\n", g_DeviceState.currTime,
			navInfo->x * 0.1, navInfo->y * 0.1, navInfo->z * 0.1,
			navInfo->Vx * 0.1, navInfo->Vy * 0.1, navInfo->Vz * 0.1,
			navInfo->s16pitch * 0.01,navInfo->s16yaw * 0.01,navInfo->s16gama * 0.01,
			navInfo->s32navLon * 1e-7, navInfo->s32navLat * 1e-7, navInfo->s32navHigh * 1e-3,
			navInfo->navStatus, navInfo->s32navVn* 0.001, navInfo->s32navVs* 0.001, navInfo->s32navVe* 0.001);
	WriteToSD(4, (OS_U8 *)sdRow2, strlen(sdRow2));
    */
	return 0;
}

//���ݵ�����Ϣ������ֱ������ϵ����
OS_U8 CalcXYZ()
{
	double x = 0, y = 0, z = 0;
	double vx = 0, vy = 0, vz = 0;
	double navlon,navlat,navhigh;
	double navVn,navVs, navVe;

	OS_S32 inavlon,inavlat;
	OS_S16 inavVn, inavVs, inavVe;
    OS_FLOAT fnavhigh;
	OS_U8 navState = 0;
    if(g_DeviceState.imuCountDown == 0 && ((g_DeviceState.workStage & DOM_HILSMODE) == 0))
    {
		// ��IMU��������nav���ݳ���ȡ����
        GetDataFast(pDataPoolNav, "navLon",  &inavlon);
        GetDataFast(pDataPoolNav, "navLat",  &inavlat);
        GetDataFast(pDataPoolNav, "navHigh", &fnavhigh);

        GetDataFast(pDataPoolNav, "navVn", &inavVn);
        GetDataFast(pDataPoolNav, "navVs", &inavVs);
        GetDataFast(pDataPoolNav, "navVe", &inavVe);
        
        GetDataFast(pDataPoolNav, "navState", &navState);//
    }
    else
    {
		// ��IMU������������IMU���ݳ���ȡ����
        GetDataFast(pDataPoolImu, "navLon",  &inavlon);
        GetDataFast(pDataPoolImu, "navLat",  &inavlat);
        GetDataFast(pDataPoolImu, "navHigh", &fnavhigh);

        GetDataFast(pDataPoolImu, "navVn", &inavVn);
        GetDataFast(pDataPoolImu, "navVs", &inavVs);
        GetDataFast(pDataPoolImu, "navVe", &inavVe);
        
        GetDataFast(pDataPoolImu, "navState", &navState);//
    }
	navlon = inavlon * 1e-7;
	navlat = inavlat * 1e-7;
	navhigh = fnavhigh;
	navVn = inavVn * 1e-2;
	navVs = inavVs * 1e-2;
	navVe = inavVe * 1e-2;

	OS_S32 luanchLon,luanchLat;
	OS_S16 luanchHigh;
	OS_U16 luanchDir;


	GetDataFast(pDataPoolFly, "DataLon", &luanchLon);//
	GetDataFast(pDataPoolFly, "DataLat", &luanchLat);//
	GetDataFast(pDataPoolFly, "DataHigh", &luanchHigh);//
	GetDataFast(pDataPoolFly, "DataDir", &luanchDir);//

	if(navState == 0x60 || navState == 0x64)//60��ϵ�����64���Ե���
	{
		DoCalcXYZ(	luanchLon * 1e-7, 
					luanchLat*1e-7, 
					luanchHigh, 
					luanchDir*1e-2, 
					navlon, navlat, navhigh, navVn, navVs, navVe, 
					&x, &y, &z, &vx, &vy, &vz);
	}

	SETDATA(pDataPoolFly, "navX", x, OS_FLOAT);
	SETDATA(pDataPoolFly, "navY", y, OS_FLOAT);
	SETDATA(pDataPoolFly, "navZ", z, OS_FLOAT);
	SETDATA(pDataPoolFly, "navVx", vx * 10, OS_S16);
	SETDATA(pDataPoolFly, "navVy", vy * 10, OS_S16);
	SETDATA(pDataPoolFly, "navVz", vz * 10, OS_S16);

	return 0;
}
