/*
 * modIMU.c
 *
 *  Created on: 2021锟斤拷10锟斤拷16锟斤拷
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

#define PI (3.1415926)

STRU_NAV_INPUT navInput = {0, 0, 0, 0, 1, 210, 0};//鐩�鍓嶄笉浣跨敤
static double northDir = 0;//鏂逛綅瑙�
static double Global = 0;  //褰撳湴鍔犻€熷害
static double Sigma0 = 0;  //褰撳湴鑷�杞�瑙掗€熷害
double dpitch;
double dyaw;
double dpitchUnHor;
double dyawUnHor;

//锟津导猴拷锟斤拷锟斤拷指锟斤拷锟斤拷息
OS_U8 MsgToNAV(OS_U8 msgID, OS_U8 *data, OS_U8 len)
{
	MsgToDevice(RT_NAV, msgID, len, data);

	return 0;
}

/***********************************************************
 * 鍑芥暟鍚嶇О: DoHorizonCalc()
 * 鍑芥暟鍔熻兘: 姘村钩璁＄畻绠楁硶鍑芥暟銆�
 * 浣滆€�:	鎺у埗閮ㄩ棬
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
	case 0:	//寮€濮嬭�＄畻
		s_AxsumIMU= s_AysumIMU= s_AzsumIMU=
		s_WxsumIMU= s_WysumIMU= s_WzsumIMU=
		s_Lonsum= s_Latsum= s_Heightsum=0;
		break;
	default: //绉�鍒�//璁＄畻
		s_AxsumIMU += ax;
		s_AysumIMU += ay;
		s_AzsumIMU += az;
		s_WxsumIMU += wx;
		s_WysumIMU += wy;
		s_WzsumIMU += wz;
		s_Lonsum += lon;
		s_Latsum += lat;
		s_Heightsum += height;

		s_AxsumRocket = s_AxsumIMU; //鎯�缁勫潗鏍囩郴鍒扮��浣撳潗鏍囩郴鐨勮浆鎹�锛屼緷鎹�鍨嬪彿浠ュ強瀹夎�呮儏鍐靛�瑰簲
		s_AysumRocket = s_AysumIMU;
		s_AzsumRocket = s_AzsumIMU;
		s_WxsumRocket = s_WxsumIMU;
		s_WysumRocket = s_WysumIMU;
		s_WzsumRocket = s_WzsumIMU;

		axAverage = s_AxsumRocket / u8CalcCnt;   //鍙栨按骞宠�＄畻180s鍐呯殑骞冲潎鍊�
		ayAverage = s_AysumRocket / u8CalcCnt;
		azAverage = s_AzsumRocket / u8CalcCnt;
		wxAverage = s_WxsumRocket / u8CalcCnt;
		wyAverage = s_WysumRocket / u8CalcCnt;
		wzAverage = s_WzsumRocket / u8CalcCnt;

		if(pLon) *pLon = s_Lonsum / u8CalcCnt;
		if(pLat) *pLat = s_Latsum / u8CalcCnt;
		if(pHeight) *pHeight = s_Heightsum / u8CalcCnt;

		//锟酵硷拷锟劫度★拷锟斤拷锟劫讹拷
		globalIn = sqrt(axAverage * axAverage + ayAverage * ayAverage + azAverage * azAverage);
		if(pGlocal) *pGlocal = globalIn;
		//*pGlocal = axAverage;
		if(pSigma0) *pSigma0 = sqrt(wxAverage * wxAverage + wyAverage * wyAverage + wzAverage * wzAverage)*3600;

		//1璞￠檺鏈濆皠鍚�
		d_pitch = - asin(ayAverage / globalIn); //渚濇嵁鍙戝皠鍧愭爣绯讳笌绠�浣撳潗鏍囩郴鍏崇郴纭�瀹氱�﹀彿
		pitch = d_pitch + PI / 2;
		yaw = asin(azAverage / globalIn / sin(pitch)); //鍚屼笂

		//3璞￠檺鏈濆皠鍚�
		//d_pitch = asin(ayAverage / globalIn); //渚濇嵁鍙戝皠鍧愭爣绯讳笌绠�浣撳潗鏍囩郴鍏崇郴纭�瀹氱�﹀彿
		//pitch = d_pitch + PI / 2;
		//yaw = - asin(azAverage / globalIn / sin(pitch)); //鍚屼笂

		//锟斤拷锟斤拷锟斤拷锟斤拷恰锟斤拷锟斤拷锟角★拷锟斤拷转锟斤拷
		if(pPItch) *pPItch = toDeg(d_pitch);
		if(pYaw) *pYaw = toDeg(yaw);
		if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage));//Y1 3璞￠檺鏈濆皠鍚戯紝涓嶉渶瑕佸啀鍔爌i
		if(*pNorth < 0)
			*pNorth += 360;
		//Y6 if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage)+ PI); //g_HorizontalCalc_north 缁撴灉鑼冨洿涓�-pi/2~pi/2,閽堝�规柟浣嶈�掑垎姣嶅繀椤诲ぇ浜�0 2021521 浣�
		break;
	}
	return 1;
}

/***********************************************************
 * 鍑芥暟鍚嶇О: DoHorizonCalc()
 * 鍑芥暟鍔熻兘: 姘村钩璁＄畻鍚�鍔ㄥ嚱鏁帮紝杈撳叆鍙傛暟涓�:
 * 			(1)hCalcCnt	姘村钩璁＄畻褰撳墠鐨勮�℃暟
 * 			(2)hCalcTotalCnt 姘村钩璁＄畻鐨勬埅鑷崇�掕�℃暟
 * 			褰撴埅鑷宠�℃暟涓�0鏃讹紝涓嶉渶瑕佽繘琛屾按骞宠�＄畻銆�
 * 			褰撴敹鍒板湴闈㈡按骞宠�＄畻璇锋眰鍚庯紝浼氬皢hCalcTotalCnt缃�涓�180锛宧CalcCnt缃�涓�0锛屽苟寮€濮嬭繘琛岀疮鍔犺�℃暟骞惰繘琛屾按骞宠�＄畻銆�
 * 浣滆€�:	鎴愬畯鐠�
 ***********************************************************/
static OS_U32 hCalcCnt = 0;//水平锟斤拷态锟斤拷锟斤拷锟斤拷锟�
static OS_U32 hCalcTotalCnt = 0;//转锟斤拷锟斤拷锟斤拷46s = 46 * 200 * 5ms
static OS_U8 DoHorizonCalc()
{
	//锟较硷拷锟劫度★拷锟斤拷锟劫度革拷锟斤拷
	if(hCalcTotalCnt)
	{
		//锟斤拷锟斤拷锟斤拷锟绞憋拷锟斤拷锟斤拷锟姐，锟斤拷锟劫革拷锟斤拷锟斤拷锟斤拷
		if(hCalcCnt == hCalcTotalCnt)
		{
			hCalcCnt = 0;
			hCalcTotalCnt = 0;
			return 0;
		}
		//每1s = 5ms*200锟斤拷锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷
		else if(hCalcCnt % 200 == 0)
		{//鍦ㄦ寔缁�璁＄畻鐨勮繃绋嬩腑锛屼篃姣忛殧1绉掍笅浼犱竴娆¤�＄畻缁撴灉銆�
			SETDATA(pDataPoolNav, "Global", Global, OS_DOUBLE);//褰撳湴鍔犻€熷害
			SETDATA(pDataPoolNav, "Sigma0", Sigma0, OS_DOUBLE);//鍚堟垚瑙掗€熷害
		}
	}

	//水平锟斤拷锟斤拷悖猴拷锟斤拷锟斤拷恰锟斤拷锟斤拷锟角★拷锟斤拷转锟角★拷锟斤拷纬锟竭★拷锟酵硷拷锟劫度★拷锟酵斤拷锟劫度硷拷锟斤拷
	float fax,fay,faz,fwx,fwy,fwz;
	GetDataFast(pDataPoolNav, "navWx", &fwx);//鍘熷�嬭�掗€熷害X
	GetDataFast(pDataPoolNav, "navWy", &fwy);//鍘熷�嬭�掗€熷害Y
	GetDataFast(pDataPoolNav, "navWz", &fwz);//鍘熷�嬭�掗€熷害Z
	GetDataFast(pDataPoolNav, "navAx", &fax);//鍘熷�嬪姞閫熷害X
	GetDataFast(pDataPoolNav, "navAy", &fay);//鍘熷�嬪姞閫熷害Y
	GetDataFast(pDataPoolNav, "navAz", &faz);//鍘熷�嬪姞閫熷害Z
	//锟斤拷锟絟CalcTotalCnt为0,说锟斤拷水平锟斤拷锟斤拷未锟斤拷锟斤拷
	//锟斤拷锟絟CalcCnt锟窖撅拷锟斤拷锟斤拷180*200时锟斤拷说锟斤拷水平锟斤拷锟斤拷锟窖撅拷锟斤拷锟斤拷
	//只锟叫碉拷锟斤拷锟芥发锟斤拷水平锟斤拷锟斤拷指锟斤拷锟絟CalcTotalCnt锟斤拷值锟斤拷锟斤拷hCalcCnt锟斤拷0,锟斤拷始每5ms锟斤拷一锟斤拷水平锟斤拷锟斤拷
	double ax = fax,ay=fay,az=faz,wx=fwx,wy=fwy,wz=fwz;

	//濡傛灉hCalcTotalCnt涓�0,璇存槑姘村钩璁＄畻鏈�鍚�鍔�
	//濡傛灉hCalcCnt宸茬粡澶т簬180*200鏃讹紝璇存槑姘村钩璁＄畻宸茬粡缁撴潫
	//鍙�鏈夊綋鍦伴潰鍙戦€佹按骞宠�＄畻鎸囦护鍚庯紝hCalcTotalCnt璧嬪€硷紝涓攈CalcCnt缃�0,寮€濮嬫瘡5ms鍋氫竴娆℃按骞宠�＄畻
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
						&dpitchUnHor,//锟斤拷锟斤拷锟斤拷
						&dyawUnHor,
						&northDir,//锟芥方位锟斤拷
						NULL,
						NULL,
						NULL,
						&Global,
						&Sigma0,
						hCalcCnt++);
	}

	return 1;
}

//锟津导猴拷锟斤拷锟斤拷指锟斤拷
OS_U32 NavCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case CMD_NAV_INIT:
		{
			//鍦ㄥ�瑰噯鐨勬椂鍊欙紝涓婃敞鍒濆�嬫暟鎹�
			OS_S8 InitData[100];
			//姘村钩瀹夎��
			//InitData[0] = 1;
			//InitData[1] = -3;
			//InitData[2] = 2;
			//鍨傜洿瀹夎��
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
		case CMD_HOR_CALC_REQ://瀵瑰噯璇锋眰 鍒嗘按骞冲�瑰噯鍜屽瀭鐩村�瑰噯涓ょ�嶆ā寮�
		{

			if(frame->au8Data[0] == 0)//姘村钩瀵瑰噯
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 2,		OS_U8);
			}
			else if(frame->au8Data[0] == 1)//鍨傜洿瀵瑰噯
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 1,		OS_U8);
			}
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_FOCUS, 0, (OS_U8*)&toNav);
		}
		break;
		case CMD_TO_NAV_REQ: //鍚�鍔ㄧ粍鍚堝�艰埅璇锋眰
		{
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_START_NAV, 0, (OS_U8*)&toNav);
			g_DeviceState.workStage |= DOM_NAVON;//杞�瀵艰埅妯″紡

			//鍚�鍔ㄦ湰鏈虹殑姘村钩璁＄畻
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

//锟斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷锟捷后，伙拷锟斤拷锟斤拷锟捷碉拷锟斤拷锟捷池ｏ拷锟斤拷锟节飞匡拷使锟矫猴拷遥锟解，锟皆硷拷锟斤拷锟斤拷站锟斤拷息应锟斤拷
OS_U32 NavRtHandler(STRU_422_MSG_INFO * frame)	// RT_NAV
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case BUS_IMU_INFO_REPORT://缁勫悎瀵艰埅鐗堝畾鏃跺彂閫佸抚(5ms)
		{
			//锟斤拷锟斤拷锟斤拷锟捷碉拷锟斤拷锟捷池ｏ拷锟斤拷锟节飞匡拷使锟矫猴拷遥锟斤拷
			STRU_NAV_INFO navInfo;
			memcpy(&navInfo, frame->au8Data, sizeof(navInfo));
			SaveNavInDataPool(&navInfo);	//锟芥储锟斤拷锟斤拷锟斤拷锟斤拷
			g_DeviceState.navCountDown = 200;

			//锟斤拷锟斤拷锟斤拷准
			if(navInfo.navStatus == 0x44)
			{
				STRU_422_MSG_INFO frame;
				frame.u8MsgID = CMD_NAV_INIT;
				NavCmdHandler(&frame);
			}

			//姘村钩璁＄畻鐪嬫寚浠ゆ槸鍚︽妸璁＄畻鏃堕棿閲嶇疆浜嗭紝濡傛灉閲嶇疆浜嗗氨閲嶅仛锛屽�傛灉娌￠噸缃�灏辫烦杩�
			DoHorizonCalc();
		}
		break;
	//锟斤拷锟斤拷应锟斤拷锟斤拷锟斤拷帧
	case CMD_GET_EPH_RSP:
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
		break;
	}
	
	//锟斤拷锟斤拷站锟斤拷息应锟斤拷
	if(msgID >= 0x60 && msgID <= 0x6F)
	{
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
	}
	return 0;
}

// NAV浼犺繃鏉ョ殑鏁版嵁锛岀幇鍦ㄦ垜浠�闇€瑕佹妸杩欎簺鏁版嵁璧嬪€肩粰IMU鐨勬暟鎹�姹�
OS_U8 SaveNavInDataPool(STRU_NAV_INFO *navInfo)
{
	SETDATA(pDataPoolNav, "gpsMod", navInfo->GPSstate,	OS_U8);//GPS鐘舵€�
	SETDATA(pDataPoolNav, "gpsLoCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS锟斤拷位锟斤拷锟斤拷
	SETDATA(pDataPoolNav, "gpsLoMas", navInfo->gpsDirEnable[0] == 'V'?1:0, OS_U8);
	SETDATA(pDataPoolNav, "gpsLoSla", navInfo->gpsDirEnable[1], OS_U8);
	SETDATA(pDataPoolNav, "gpsLon", navInfo->GPSlon,	OS_S32);//GPS缁忓害
	SETDATA(pDataPoolNav, "gpsLat", navInfo->GPSlat,	OS_S32);//GPS绾�搴�
	SETDATA(pDataPoolNav, "gpsHigh", navInfo->GPShigh * 1e-3,	OS_S16);//GPS楂樺害
	SETDATA(pDataPoolNav, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS鍖楅€�
	SETDATA(pDataPoolNav, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS澶╅€�
	SETDATA(pDataPoolNav, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS涓滈€�
	SETDATA(pDataPoolNav, "gpsPdop", navInfo->PDOP,		OS_U16);//PDOP
	SETDATA(pDataPoolNav, "gpsGdop", navInfo->GDOP,		OS_U16);//GDOP
	SETDATA(pDataPoolNav, "gpsDelay", navInfo->Deltime,	OS_U8);//PPS
	SETDATA(pDataPoolNav, "gpsUload", navInfo->uploadEphStatus,	OS_U8);//鏄熷巻瑁呰�㈢粨鏋�

	SETDATA(pDataPoolNav, "gpsYear", navInfo->year,	OS_U8);//GPS骞�
	SETDATA(pDataPoolNav, "gpsMonth", navInfo->month,	OS_U8);//GPS鏈�
	SETDATA(pDataPoolNav, "gpsDay", navInfo->day,	OS_U8);//GPS鏃�
	SETDATA(pDataPoolNav, "gpsHour", navInfo->hour,	OS_U8);//GPS鏃�
	SETDATA(pDataPoolNav, "gpsMinit", navInfo->minite,	OS_U8);//GPS鍒�
	SETDATA(pDataPoolNav, "gpsSec", navInfo->second,	OS_U8);//GPS绉�
	SETDATA(pDataPoolNav, "gpsMs", navInfo->ms,	OS_U16);//GPS姣�绉�
	SETDATA(pDataPoolNav, "gpsTrack", navInfo->gpsTrack,	OS_U16);//GPS鑸�杩硅��
	SETDATA(pDataPoolNav, "gpsDir", navInfo->gpsDir,	OS_U16);//GPS鑸�鍚戣��
	SETDATA(pDataPoolNav, "gpsDirOK", navInfo->gpsDirEffect,	OS_U8);//GPS鑸�鍚戞湁鏁堟爣蹇�

	if((g_DeviceState.workStage & DOM_HILSMODE) && hilInput.useNav == 0)
	{
	}
	else
	{
SETDATA(pDataPoolImu, "gpsLon", navInfo->GPSlon,			OS_S32);//GPS缁忓害
SETDATA(pDataPoolImu, "gpsLat", navInfo->GPSlat,			OS_S32);//GPS绾�搴�
SETDATA(pDataPoolImu, "gpsAlt", navInfo->GPShigh * 1e-3,	OS_S16);//GPS楂樺害
SETDATA(pDataPoolImu, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS鍖楅€�
SETDATA(pDataPoolImu, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS澶╅€�
SETDATA(pDataPoolImu, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS涓滈€�
SETDATA(pDataPoolImu, "dirEffec",	navInfo->gpsDirEffect,	OS_U8);//GPS鑸�鍚戞湁鏁堟爣蹇�

SETDATA(pDataPoolImu, "gpsYear",  navInfo->year,   OS_U8);	//GPS骞�
SETDATA(pDataPoolImu, "gpsMonth", navInfo->month,  OS_U8);	//GPS鏈�
SETDATA(pDataPoolImu, "gpsDay",   navInfo->day,    OS_U8);	//GPS鏃�
SETDATA(pDataPoolImu, "gpsHour",  navInfo->hour,   OS_U8);	//GPS鏃�
SETDATA(pDataPoolImu, "gpsMinit", navInfo->minite, OS_U8);	//GPS鍒�
SETDATA(pDataPoolImu, "gpsSec",   navInfo->second, OS_U8);	//GPS绉�
SETDATA(pDataPoolImu, "gpsMSec",  navInfo->ms,     OS_U8);	//GPS姣�绉�
SETDATA(pDataPoolImu, "gpsDir", navInfo->gpsDir,  OS_U16);
SETDATA(pDataPoolImu, "gpsScCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS瀹氫綅鏄熸暟

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

//鏂板�炲垽鏂�瀵艰埅鐘舵€佸噯澶囦腑涓旇埅鍚戞湁鏁堟爣蹇楁湁鏁�
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

	SETDATA(pDataPoolNav, "navLon", navInfo->s32navLon,	OS_S32);//瀵艰埅缁忓害
	SETDATA(pDataPoolNav, "navLat", navInfo->s32navLat,	OS_S32);//瀵艰埅绾�搴�
	SETDATA(pDataPoolNav, "navHigh", navInfo->s32navHigh * 1e-3,OS_FLOAT);//瀵艰埅楂樺害
	SETDATA(pDataPoolNav, "navVn", navInfo->s32navVn * 1e-1,	OS_S16);//瀵艰埅鍖楅€�
	SETDATA(pDataPoolNav, "navVs", navInfo->s32navVs * 1e-1,	OS_S16);//瀵艰埅澶╅€�
	SETDATA(pDataPoolNav, "navVe", navInfo->s32navVe * 1e-1,	OS_S16);//瀵艰埅涓滈€�

	SETDATA(pDataPoolNav, "navPitch", navInfo->s16pitch,	OS_S16);//瀵艰埅淇�浠�
	SETDATA(pDataPoolNav, "navRoll", navInfo->s16roll,	OS_S16);//瀵艰埅婊氳浆
	SETDATA(pDataPoolNav, "navDir", navInfo->s16dir,	OS_U16);//瀵艰埅鏂逛綅瑙�

	SETDATA(pDataPoolNav, "navWx", navInfo->imuWx16507,	OS_FLOAT);//鍘熷�嬭�掗€熷害X
	SETDATA(pDataPoolNav, "navWy", navInfo->imuWy16507,	OS_FLOAT);//鍘熷�嬭�掗€熷害Y
	SETDATA(pDataPoolNav, "navWz", navInfo->imuWz16507,	OS_FLOAT);//鍘熷�嬭�掗€熷害Z
	SETDATA(pDataPoolNav, "navAx", navInfo->imuAx16507,	OS_FLOAT);//鍘熷�嬪姞閫熷害X
	SETDATA(pDataPoolNav, "navAy", navInfo->imuAy16507,	OS_FLOAT);//鍘熷�嬪姞閫熷害Y
	SETDATA(pDataPoolNav, "navAz", navInfo->imuAz16507,	OS_FLOAT);//鍘熷�嬪姞閫熷害Z


	// SETDATA(pDataPoolNav, "navWx2", navInfo->imuWx20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷X
	// SETDATA(pDataPoolNav, "navWy2", navInfo->imuWy20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷Y
	// SETDATA(pDataPoolNav, "navWz2", navInfo->imuWz20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷Z
	// SETDATA(pDataPoolNav, "navAx2", navInfo->imuAx20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷X
	// SETDATA(pDataPoolNav, "navAy2", navInfo->imuAy20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷Y
	// SETDATA(pDataPoolNav, "navAz2", navInfo->imuAz20689,	OS_FLOAT);//原始锟斤拷锟劫讹拷Z
    
	// SETDATA(pDataPoolNav, "navWx3", navInfo->imuWx42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷X
	// SETDATA(pDataPoolNav, "navWy3", navInfo->imuWy42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷Y
	// SETDATA(pDataPoolNav, "navWz3", navInfo->imuWz42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷Z
	// SETDATA(pDataPoolNav, "navAx3", navInfo->imuAx42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷X
	SETDATA(pDataPoolNav, "navAy3", navInfo->imuAy42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷Y
	SETDATA(pDataPoolNav, "navAz3", navInfo->imuAz42688,	OS_FLOAT);//原始锟斤拷锟劫讹拷Z

	SETDATA(pDataPoolNav, "navState", navInfo->navStatus ,	OS_U8);
	//cpu0杩愮畻鑰楁椂锛堝�艰埅鏉匡級
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

//锟斤拷锟捷碉拷锟斤拷锟斤拷息锟斤拷锟斤拷锟斤拷直锟斤拷锟斤拷锟斤拷系锟斤拷锟斤拷
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
    if(g_DeviceState.imuCountDown == 0 && ((g_DeviceState.workStage & DOM_HILSMODE) != DOM_HILSMODE))
    {
		// 涓嶪MU鏂�杩烇紝浠巒av鏁版嵁姹犱腑鍙栨暟鎹�
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
		// 涓嶪MU杩炴帴姝ｅ父锛屼粠IMU鏁版嵁姹犱腑鍙栨暟鎹�
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

	if(navState == 0x60 || navState == 0x64)//60缁勫悎瀵艰埅锛�64鎯�鎬у�艰埅
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
