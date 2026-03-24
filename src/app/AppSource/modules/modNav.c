/*
 * modIMU.c
 *
 *  Created on: 2021年10月16日
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

STRU_NAV_INPUT navInput;
#define PI (3.1415926)
static double northDir = 0;//方位角
static double Global = 0;  //当地加速度
static double Sigma0 = 0;  //当地自转角速度
double dpitch;
double dyaw;
double dpitchUnHor;
double dyawUnHor;
OS_U8 CalcXYZ();

OS_U8 SaveNavInDataPool(STRU_NAV_INFO *navInfo);

OS_U8 MsgToNAV(OS_U8 msgID, OS_U8 *data, OS_U8 len)
{
	MsgToDevice(RT_NAV, msgID, len, data);

	return 0;
}

/***********************************************************
 * 函数名称: DoHorizonCalc()
 * 函数功能: 水平计算算法函数。
 * 作者:	控制部门
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
	case 0:	//开始计算
		s_AxsumIMU= s_AysumIMU= s_AzsumIMU=
		s_WxsumIMU= s_WysumIMU= s_WzsumIMU=
		s_Lonsum= s_Latsum= s_Heightsum=0;
		break;
	default: //积分//计算
		s_AxsumIMU += ax;
		s_AysumIMU += ay;
		s_AzsumIMU += az;
		s_WxsumIMU += wx;
		s_WysumIMU += wy;
		s_WzsumIMU += wz;
		s_Lonsum += lon;
		s_Latsum += lat;
		s_Heightsum += height;

		s_AxsumRocket = s_AxsumIMU; //惯组坐标系到箭体坐标系的转换，依据型号以及安装情况对应
		s_AysumRocket = s_AysumIMU;
		s_AzsumRocket = s_AzsumIMU;
		s_WxsumRocket = s_WxsumIMU;
		s_WysumRocket = s_WysumIMU;
		s_WzsumRocket = s_WzsumIMU;

		axAverage = s_AxsumRocket / u8CalcCnt;   //取水平计算180s内的平均值
		ayAverage = s_AysumRocket / u8CalcCnt;
		azAverage = s_AzsumRocket / u8CalcCnt;
		wxAverage = s_WxsumRocket / u8CalcCnt;
		wyAverage = s_WysumRocket / u8CalcCnt;
		wzAverage = s_WzsumRocket / u8CalcCnt;

		if(pLon) *pLon = s_Lonsum / u8CalcCnt;
		if(pLat) *pLat = s_Latsum / u8CalcCnt;
		if(pHeight) *pHeight = s_Heightsum / u8CalcCnt;

		globalIn = sqrt(axAverage * axAverage + ayAverage * ayAverage + azAverage * azAverage);
		if(pGlocal) *pGlocal = globalIn;
		//*pGlocal = axAverage;
		if(pSigma0) *pSigma0 = sqrt(wxAverage * wxAverage + wyAverage * wyAverage + wzAverage * wzAverage)*3600;

		//1象限朝射向
		d_pitch = - asin(ayAverage / globalIn); //依据发射坐标系与箭体坐标系关系确定符号
		pitch = d_pitch + PI / 2;
		yaw = asin(azAverage / globalIn / sin(pitch)); //同上

		//3象限朝射向
		//d_pitch = asin(ayAverage / globalIn); //依据发射坐标系与箭体坐标系关系确定符号
		//pitch = d_pitch + PI / 2;
		//yaw = - asin(azAverage / globalIn / sin(pitch)); //同上

		if(pPItch) *pPItch = toDeg(d_pitch);
		if(pYaw) *pYaw = toDeg(yaw);
		if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage));//Y1 3象限朝射向，不需要再加pi
		if(*pNorth < 0)
			*pNorth += 360;
		//Y6 if(pNorth) *pNorth = toDeg(atan2(wzAverage, wyAverage)+ PI); //g_HorizontalCalc_north 结果范围为-pi/2~pi/2,针对方位角分母必须大于0 2021521 余
		break;
	}
	return 1;
}

/***********************************************************
 * 函数名称: DoHorizonCalc()
 * 函数功能: 水平计算启动函数，输入参数为:
 * 			(1)hCalcCnt	水平计算当前的计数
 * 			(2)hCalcTotalCnt 水平计算的截至秒计数
 * 			当截至计数为0时，不需要进行水平计算。
 * 			当收到地面水平计算请求后，会将hCalcTotalCnt置为180，hCalcCnt置为0，并开始进行累加计数并进行水平计算。
 * 作者:	成宏璟
 ***********************************************************/
static OS_U32 hCalcCnt = 0;
static OS_U32 hCalcTotalCnt = 0;
static OS_U8 DoHorizonCalc()
{
	if(hCalcTotalCnt)
	{
		if(hCalcCnt == hCalcTotalCnt)
		{
			hCalcCnt = 0;
			hCalcTotalCnt = 0;
			return 0;
		}
		else if(hCalcCnt % 200 == 0)
		{//在持续计算的过程中，也每隔1秒下传一次计算结果。
			SETDATA(pDataPoolNav, "Global", Global, OS_DOUBLE);//当地加速度
			SETDATA(pDataPoolNav, "Sigma0", Sigma0, OS_DOUBLE);//合成角速度
		}
	}
	float fax,fay,faz,fwx,fwy,fwz;
	GetDataFast(pDataPoolNav, "navWx", &fwx);//原始角速度X
	GetDataFast(pDataPoolNav, "navWy", &fwy);//原始角速度Y
	GetDataFast(pDataPoolNav, "navWz", &fwz);//原始角速度Z
	GetDataFast(pDataPoolNav, "navAx", &fax);//原始加速度X
	GetDataFast(pDataPoolNav, "navAy", &fay);//原始加速度Y
	GetDataFast(pDataPoolNav, "navAz", &faz);//原始加速度Z


	double ax = fax,ay=fay,az=faz,wx=fwx,wy=fwy,wz=fwz;

	//如果hCalcTotalCnt为0,说明水平计算未启动
	//如果hCalcCnt已经大于180*200时，说明水平计算已经结束
	//只有当地面发送水平计算指令后，hCalcTotalCnt赋值，且hCalcCnt置0,开始每5ms做一次水平计算
	if(hCalcTotalCnt && hCalcCnt<hCalcTotalCnt)
	{
		HorizontalCalc(	ax,
						ay,
						az,
						wx,
						wy,
						wz,
						0,
						0,
						0,
						&dpitchUnHor,
						&dyawUnHor,
						&northDir,
						NULL,
						NULL,
						NULL,
						&Global,
						&Sigma0,
						hCalcCnt++);
	}

	return 1;
}

OS_U32 NavCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case CMD_NAV_INIT:
		{
			//在对准的时候，上注初始数据
			OS_S8 InitData[100];
			//水平安装
			//InitData[0] = 1;
			//InitData[1] = -3;
			//InitData[2] = 2;
			//垂直安装
			InitData[0] = 1;
			InitData[1] = 3;
			InitData[2] = -2;
            
            //MINI
            //InitData[0] = 2;
			//InitData[1] = -3;
		//	InitData[2] = -1;
			memcpy(InitData + 3, &navInput.InitLon, 8);
			memcpy(InitData + 11, &navInput.InitLat, 8);
			memcpy(InitData + 19, &navInput.InitHigh, 8);
			memcpy(InitData + 27, &navInput.InitYaw, 8);
			MsgToDevice(RT_NAV , BUS_NAV_INIT_DATA, 35, (OS_U8*)InitData);
		}
		break;
		case CMD_HOR_CALC_REQ://对准请求 分水平对准和垂直对准两种模式
		{

			if(frame->au8Data[0] == 0)//水平对准
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 2,		OS_U8);
			}
			else if(frame->au8Data[0] == 1)//垂直对准
			{
				SETDATA(pDataPoolNav,	"imuFocus",	 1,		OS_U8);
			}
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_FOCUS, 0, (OS_U8*)&toNav);
		}
		break;
		case CMD_TO_NAV_REQ: //启动组合导航请求
		{
			OS_U8 toNav[1];
			MsgToDevice(RT_NAV, BUS_NAV_START_NAV, 0, (OS_U8*)&toNav);
			g_DeviceState.workStage |= DOM_NAVON;//转导航模式

			//启动本机的水平计算
			hCalcCnt = 0;
			OS_U8 calcSecond = 46;
			hCalcTotalCnt = calcSecond * 200;
		}
			break;
        case CMD_TO_AFTER_LUANCH:
        {
            MsgToNAV(BUS_NAV_IGNATION, PTR_NULL, 0);
        }
            break;
		default:
			break;
	}
	return 0;
}

OS_U32 NavRtHandler(STRU_422_MSG_INFO * frame)	// RT_NAV
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case BUS_IMU_INFO_REPORT://组合导航版定时发送帧(5ms)
		{
			STRU_NAV_INFO navInfo;
			memcpy(&navInfo, frame->au8Data, sizeof(navInfo));
			SaveNavInDataPool(&navInfo);
			g_DeviceState.navCountDown = 200;

			if(navInfo.navStatus == 0x44)
			{
				STRU_422_MSG_INFO frame;
				frame.u8MsgID = CMD_NAV_INIT;
				NavCmdHandler(&frame);
			}

			//水平计算看指令是否把计算时间重置了，如果重置了就重做，如果没重置就跳过
			DoHorizonCalc();
		}
		break;
	case CMD_GET_EPH_RSP:
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
		break;
	}
	if(msgID >= 0x60 && msgID <= 0x6F)
	{
		CmdResponseHandler(msgID, frame->u16Len, frame->au8Data);
	}
	return 0;
}

// BUG改这里 NAV传过来的数据，现在我们需要把这些数据赋值给IMU的数据池
OS_U8 SaveNavInDataPool(STRU_NAV_INFO *navInfo)
{
	SETDATA(pDataPoolNav, "gpsMod", navInfo->GPSstate,	OS_U8);//GPS状态
	SETDATA(pDataPoolNav, "gpsLoCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS定位星数
	SETDATA(pDataPoolNav, "gpsLoMas", navInfo->gpsDirEnable[0] == 'V'?1:0, OS_U8);
	SETDATA(pDataPoolNav, "gpsLoSla", navInfo->gpsDirEnable[1] == 'V'?1:0, OS_U8);
	SETDATA(pDataPoolNav, "gpsLon", navInfo->GPSlon,	OS_S32);//GPS经度
	SETDATA(pDataPoolNav, "gpsLat", navInfo->GPSlat,	OS_S32);//GPS纬度
	SETDATA(pDataPoolNav, "gpsHigh", navInfo->GPShigh * 1e-3,	OS_S16);//GPS高度
	SETDATA(pDataPoolNav, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS北速
	SETDATA(pDataPoolNav, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS天速
	SETDATA(pDataPoolNav, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS东速
	SETDATA(pDataPoolNav, "gpsPdop", navInfo->PDOP,		OS_U16);//PDOP
	SETDATA(pDataPoolNav, "gpsGdop", navInfo->GDOP,		OS_U16);//GDOP
	SETDATA(pDataPoolNav, "gpsDelay", navInfo->Deltime,	OS_U8);//PPS
	SETDATA(pDataPoolNav, "gpsUload", navInfo->uploadEphStatus,	OS_U8);//星历装订结果

	SETDATA(pDataPoolNav, "gpsYear", navInfo->year,	OS_U8);//GPS年
	SETDATA(pDataPoolNav, "gpsMonth", navInfo->month,	OS_U8);//GPS月
	SETDATA(pDataPoolNav, "gpsDay", navInfo->day,	OS_U8);//GPS日
	SETDATA(pDataPoolNav, "gpsHour", navInfo->hour,	OS_U8);//GPS时
	SETDATA(pDataPoolNav, "gpsMinit", navInfo->minite,	OS_U8);//GPS分
	SETDATA(pDataPoolNav, "gpsSec", navInfo->second,	OS_U8);//GPS秒
	SETDATA(pDataPoolNav, "gpsMs", navInfo->ms,	OS_U16);//GPS毫秒
	SETDATA(pDataPoolNav, "gpsTrack", navInfo->gpsTrack,	OS_U16);//GPS航迹角
	SETDATA(pDataPoolNav, "gpsDir", navInfo->gpsDir,	OS_U16);//GPS航向角
	SETDATA(pDataPoolNav, "gpsDirOK", navInfo->gpsDirEffect,	OS_U8);//GPS航向有效标志


SETDATA(pDataPoolImu, "gpsLon", navInfo->GPSlon,			OS_S32);//GPS经度
SETDATA(pDataPoolImu, "gpsLat", navInfo->GPSlat,			OS_S32);//GPS纬度
SETDATA(pDataPoolImu, "gpsAlt", navInfo->GPShigh * 1e-3,	OS_S16);//GPS高度
SETDATA(pDataPoolImu, "gpsVn", navInfo->GPSVn,	OS_S16);//GPS北速
SETDATA(pDataPoolImu, "gpsVs", navInfo->GPSVs,	OS_S16);//GPS天速
SETDATA(pDataPoolImu, "gpsVe", navInfo->GPSVe,	OS_S16);//GPS东速
SETDATA(pDataPoolImu, "dirEffec",	navInfo->gpsDirEffect,	OS_U8);//GPS航向有效标志

SETDATA(pDataPoolImu, "gpsYear",  navInfo->year,   OS_U8);	//GPS年
SETDATA(pDataPoolImu, "gpsMonth", navInfo->month,  OS_U8);	//GPS月
SETDATA(pDataPoolImu, "gpsDay",   navInfo->day,    OS_U8);	//GPS日
SETDATA(pDataPoolImu, "gpsHour",  navInfo->hour,   OS_U8);	//GPS时
SETDATA(pDataPoolImu, "gpsMinit", navInfo->minite, OS_U8);	//GPS分
SETDATA(pDataPoolImu, "gpsSec",   navInfo->second, OS_U8);	//GPS秒
SETDATA(pDataPoolImu, "gpsMSec",  navInfo->ms,     OS_U8);	//GPS毫秒
SETDATA(pDataPoolImu, "gpsDir", navInfo->gpsDir,  OS_U16);
SETDATA(pDataPoolImu, "gpsScCnt", navInfo->StanumberMaster>navInfo->StanumberSlave?navInfo->StanumberMaster:navInfo->StanumberSlave,	OS_U8);//GPS定位星数

if(g_DeviceState.hilCountDown > 0 && hilInput.useNav == 0)
{
}
else
{
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

//新增判断导航状态准备中且航向有效标志有效
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

SETDATA(pDataPoolNav, "navLon", navInfo->s32navLon,	OS_S32);//导航经度
	SETDATA(pDataPoolNav, "navLat", navInfo->s32navLat,	OS_S32);//导航纬度
	SETDATA(pDataPoolNav, "navHigh", navInfo->s32navHigh * 1e-3,OS_FLOAT);//导航高度
	SETDATA(pDataPoolNav, "navVn", navInfo->s32navVn * 1e-1,	OS_S16);//导航北速
	SETDATA(pDataPoolNav, "navVs", navInfo->s32navVs * 1e-1,	OS_S16);//导航天速
	SETDATA(pDataPoolNav, "navVe", navInfo->s32navVe * 1e-1,	OS_S16);//导航东速

	SETDATA(pDataPoolNav, "navPitch", navInfo->s16pitch,	OS_S16);//导航俯仰
	SETDATA(pDataPoolNav, "navRoll", navInfo->s16roll,	OS_S16);//导航滚转
	SETDATA(pDataPoolNav, "navDir", navInfo->s16dir,	OS_U16);//导航方位角

	SETDATA(pDataPoolNav, "navWx", navInfo->imuWx16507,	OS_FLOAT);//原始角速度X
	SETDATA(pDataPoolNav, "navWy", navInfo->imuWy16507,	OS_FLOAT);//原始角速度Y
	SETDATA(pDataPoolNav, "navWz", navInfo->imuWz16507,	OS_FLOAT);//原始角速度Z
	SETDATA(pDataPoolNav, "navAx", navInfo->imuAx16507,	OS_FLOAT);//原始加速度X
	SETDATA(pDataPoolNav, "navAy", navInfo->imuAy16507,	OS_FLOAT);//原始加速度Y
	SETDATA(pDataPoolNav, "navAz", navInfo->imuAz16507,	OS_FLOAT);//原始加速度Z


//  SETDATA(pDataPoolNav, "navWx2", navInfo->imuWx20689,	OS_FLOAT);//原始角速度X
//	SETDATA(pDataPoolNav, "navWy2", navInfo->imuWy20689,	OS_FLOAT);//原始角速度Y
//	SETDATA(pDataPoolNav, "navWz2", navInfo->imuWz20689,	OS_FLOAT);//原始角速度Z
//	SETDATA(pDataPoolNav, "navAx2", navInfo->imuAx20689,	OS_FLOAT);//原始加速度X
	SETDATA(pDataPoolNav, "navAy2", navInfo->imuAy20689,	OS_FLOAT);//原始加速度Y
	SETDATA(pDataPoolNav, "navAz2", navInfo->imuAz20689,	OS_FLOAT);//原始加速度Z
    
	SETDATA(pDataPoolNav, "navWx3", navInfo->imuWx42688,	OS_FLOAT);//原始角速度X
	SETDATA(pDataPoolNav, "navWy3", navInfo->imuWy42688,	OS_FLOAT);//原始角速度Y
	SETDATA(pDataPoolNav, "navWz3", navInfo->imuWz42688,	OS_FLOAT);//原始角速度Z
	SETDATA(pDataPoolNav, "navAx3", navInfo->imuAx42688,	OS_FLOAT);//原始加速度X
	SETDATA(pDataPoolNav, "navAy3", navInfo->imuAy42688,	OS_FLOAT);//原始加速度Y
	SETDATA(pDataPoolNav, "navAz3", navInfo->imuAz42688,	OS_FLOAT);//原始加速度Z

	SETDATA(pDataPoolNav, "navState", navInfo->navStatus ,	OS_U8);
	//cpu0运算耗时（导航板）
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
    if(g_DeviceState.imuCountDown == 0)
    {
		// 与IMU断连，从nav数据池中取数据
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
		// 与IMU连接正常，从IMU数据池中取数据
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

	if(navState == 0x60 || navState == 0x64)//60组合导航，64惯性导航
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
