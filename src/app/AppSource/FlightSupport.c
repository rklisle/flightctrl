/*
 * FlightSupport.c
 *
 *  Created on: 2022年3月23日
 *      Author: Lenovo
 */
#include "FlightSupport.h"
#include "stmToZynq.h"
#include "./modules/modPwrSeqCtl.h"
#include "./modules/modSrvCtl.h"
#include "./modules/modHil.h"
#include "./modules/modEngine.h"
#include "./modules/modFlash.h"
#include "./modules/modNav.h"
#include "./controller/controller.h"
//#include "./comm/CommHandler.h"
#include "./support/common.h"
#include "./core/DataPool.h"
#include "./flight/os_flight_io.h"
#include "./flight/RoutePoint.h"
#include "./interface/interface_power.h"
#include "./mission/mission.h"
#include "math.h"
#include <stdlib.h>
#include "interface_timer.h"

//#include "flight_data.h"
Point safePoints[20];
int airHighArray[1000] = {0};
int safePointCount;

MISSION homeMsn = {0};

#define d2r		(57.29577951308402)
/***********************************************************
 * 函数名称:FlightSeqOutputHandle()
 * 函数功能:根据飞控运算的时序结果，触发时序开关
  ***********************************************************/
static void FlightSeqOutputHandle()	//02 如果收到伞降命令，关发动机、开伞、控舵机角度、气囊、舵机断电、割伞、发动机断电
{
	if(flightSeq.umOpen == 1)
	{
		// 加一个对引信的控制
		FuseSend(0x7A);	// 取消电激活
		//收到伞降命令
		struct EngineStatus engineStatus = {0};
		static int waitOneSec = 0;
		//立刻关闭发动机，持续发送发动机转速到怠速指令，及发动机停机指令
		if(waitOneSec >= 0 && waitOneSec <10000)
		{
			//20ms后关闭发动机
            if(waitOneSec % 4 == 0)
            {
				modECU_GetEngineStatus(&engineStatus);
				if(engineStatus.CntState != ENGINE_STOPED)
				{
					StopEngine();
				}
            }
		}
		//200ms秒后开伞
		if(waitOneSec == 40)
		{
			// TrigerSeqWithWidth(TEST_1 + 1, 100);//开伞操作
			AngleServo_SetAngle(SERVO_PWM7, 50.0f);//MML开伞舵机7 控制7号舵机至50°（此处角度根据需要修改）
		}
		
		if(waitOneSec == 200)//1秒时保持舵机回0
		{			
			ServoCtlOnce_6Rudder(0, 0, 0, 0, 0, 0);	//MML舵机
		}
		if(waitOneSec == 3000)//15秒时开气囊前
		{
			// TrigerSeqWithWidth(TEST_1 + 2, 100);//前气囊操作
		}
		if(waitOneSec == 3080)//15.4秒时开气囊后
		{
			// TrigerSeqWithWidth(TEST_1 + 3, 100);//后气囊操作
		}
		if(waitOneSec == 4000)//20秒舵机下电
		{
			//断伺服电
			PowerOff(DEVICE_SRV_PWR28V);
		}
		static OS_U8 yijinggesan = 0;
		if(waitOneSec > 4000 && yijinggesan == 0)
		{
			//开始判断割伞
			OS_S16 navVs;
			GetDataFast(pDataPoolImu, "navVs", 	&navVs);
			float fvs = navVs * 0.01;	// 得到实际导航天速
            if(g_DeviceState.imuCountDown != 0)
            {
				// 表示 与IMU通信正常
                if(fabs(fvs) < 1.0)
                {
					// 速度很慢了，说明降落伞起作用了
                    TrigerSeqWithWidth(TEST_1 + 0, 100);//割伞操作
                    yijinggesan = 1;
                }
            }
            else
            {
				// 表示 与IMU通信不上
                OS_S16 airHigh;
                GetDataFast(pDataPoolSelf, "AirHigh", &airHigh);//气压高度
                memmove(&airHighArray[1], &airHighArray[0], sizeof(int) * 999);
                airHighArray[0] = airHigh;
                OS_U8 airStable = 1;
                for(int i=0; i<1000; i++)
                {
                    if(fabs(airHigh - airHighArray[i]) > 5)
                    {
						// 表示 高度下降或上升很快，说明降落伞还没将飞机速度拉下来
                        airStable = 0;
                        break;
                    }                        
                }
                if(airStable == 1)
                {
                    TrigerSeqWithWidth(TEST_1 + 0, 100);//割伞操作
                    yijinggesan = 1;
                }
            }
		}
		else
		{
			waitOneSec++;
		}
		OS_U16 ecuTemp;
		GetDataFast(pDataPoolSelf,  "ecuTemp", 	&ecuTemp);
		if(ecuTemp * 0.1 < 65)
		{
				//断发动机24v供电
				// PowerOff(DEVICE_BATT_ENGINE);	//MML 发动机
		}
	}
}

/***********************************************************
 * 函数名称:FlightTMOutputHandle()
 * 函数功能:分析飞控运算后的输出数据，存数据池
  ***********************************************************/
void FlightTMOutputHandle()	//02 控制输出——>存入数据池
{
	SETDATA(pDataPoolFly,	"pitchCmd",	pOutput->rudderPitchCmd * 100,	OS_S16);	//起控标志
	SETDATA(pDataPoolFly,	"rollCmd",	pOutput->rudderRollCmd* 100,	OS_S16);	//制导级数
	SETDATA(pDataPoolFly,	"yawCmd",	pOutput->rudderYawCmd* 100,		OS_S16);	//控制级数
    
	SETDATA(pDataPoolFly,	"dRn",	pOutput->dRn,		OS_S16);	//控制级数
	SETDATA(pDataPoolFly,	"dRu",	pOutput->dRu,		OS_S16);	//控制级数
	SETDATA(pDataPoolFly,	"dRe",	pOutput->dRe,		OS_S16);	//控制级数
	
	SETDATA(pDataPoolFly,	"PitchPre",	pOutput->Pitch_Preset_Angle * 100,		OS_S16);	//控制级数
	SETDATA(pDataPoolFly,	"YawPre",	pOutput->Yaw_Preset_Angle * 100,		OS_S16);	//控制级数
    
	SETDATA(pDataPoolFly,	"fPitchSp",	pOutput->pitch_rate_nT_filterOut * 500,		OS_S16);	//控制级数
	SETDATA(pDataPoolFly,	"fYawSp",	pOutput->yaw_rate_nT_filterOut * 500,		OS_S16);	//控制级数
 
	SETDATA(pDataPoolMsn,	"WP_cur",	pOutput->curPtNo,	OS_U8);//航点号

	SETDATA(pDataPoolMsn,	"tarLon",	pOutput->curTargetLon * 1e7, OS_S32 );//目标航点经度
	SETDATA(pDataPoolMsn,	"tarLat",	pOutput->curTargetLat * 1e7, OS_S32 );//目标航点纬度
	SETDATA(pDataPoolMsn,	"tarAlt",	pOutput->curTargetAlt * 1, OS_S16 );//目标航点高度

  SETDATA(pDataPoolFly,	"adrc_Mx",	pOutput->mx_ESO * 10, OS_S16 );//adrc
	SETDATA(pDataPoolFly,	"gamaCmd",	pOutput->gamaCmd * 1e2, OS_S16 );//滚转角指令
	SETDATA(pDataPoolFly,	"thetaCmd",	pOutput->varthetaCmd * 1e2, OS_S16 );//俯仰角指令
	SETDATA(pDataPoolFly,	"nycCmd",	pOutput->nycCmd * 1e3, OS_S16 );//过载指令
	SETDATA(pDataPoolFly,	"highCmd",	pOutput->heightCmd , OS_FLOAT );///高度指令
	SETDATA(pDataPoolFly,	"ac_dL",	pOutput->ac_dL * 10, OS_S32 );//待飞距
	SETDATA(pDataPoolFly,	"ac_dZ",	pOutput->ac_dZ * 10, OS_S16 );//侧边距
	SETDATA(pDataPoolFly,	"tokenlon",	pOutput->token_long * 1, OS_U8 );//纵向令牌
	SETDATA(pDataPoolFly,	"tokenlat",	pOutput->token_late * 1, OS_U8 );//侧向令牌
	SETDATA(pDataPoolFly,	"ac_dPsi",	pOutput->ac_dPsi * 100, OS_S16 );//航向角偏差
	
	SETDATA(pDataPoolFly,	"ac_dR",	pOutput->ac_dR * 10, OS_S16 );//圆轨迹侧边距
	SETDATA(pDataPoolFly,	"thetav",	pOutput->cur_thetav * 10, OS_S16 );//轨迹倾角
	SETDATA(pDataPoolFly,	"Vcmd",	pOutput->Vcmd * 10, OS_S16 );//速度指令
    
	SETDATA(pDataPoolFly,	"nyCmd",	pOutput->nyCmd_Guidance * 100 , OS_S16 );//速度指令
	SETDATA(pDataPoolFly,	"nzCmd",	pOutput->nzCmd_Guidance * 100, OS_S16 );//速度指令

	SETDATA(pDataPoolFly,	"DbsLen",	pOutput->Dubins_length , OS_FLOAT );//速度指令
	SETDATA(pDataPoolFly,	"ADRC",	    pOutput->fduox_ADRC * 100, OS_S16 );//速度指令
	//暂时借用导航版数据		
  SETDATA(pDataPoolNav, "navWx2", pOutput->arp_ins,	OS_FLOAT);//惯测攻角
	SETDATA(pDataPoolNav, "navWy2", pOutput->beta_ins,	OS_FLOAT);//惯测侧滑角
	
	SETDATA(pDataPoolNav, "navWz2", pOutput->MaxRpm,	OS_FLOAT);//最大转速
	SETDATA(pDataPoolNav, "navAx2", pOutput->DFT_freq_max,	OS_FLOAT);//辨识运动频率
    
	if(pOutput->on_takeoff == 1)
	{
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xEE , OS_U8 );//速度指令
	}
}

void FlightSrvOutputHandle()	//02 控制输出角度——>操作舵机
{
	if((DOM_AUTOMATIC & g_DeviceState.workStage) && flightSeq.luanched == 1)
	{
		ServoCtlOnce_6Rudder(	pOutput->rudder1Cmd,
								pOutput->rudder2Cmd,
								pOutput->rudder3Cmd,
								pOutput->rudder4Cmd,
								pOutput->rudder5Cmd,
								pOutput->rudder6Cmd
								);
	}
}
OS_U8 OutSafeCount = 0;
OS_U8 OutSafeArea = 0;	// 0安全区里	1出安全区，且等了一会儿确定出了安全区
void FlightEngineOutputHandle()//02 在各种情况下（是否起飞？是否出安全区？）控制输出油门开度——>操作油门开度全局变量
{
	if(flightSeq.luanched == 1 && ((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC))
	{
		if(OutSafeArea)
		{
			//离开安全区，降低发动机转速
			// CurEngineRpm = 18000;
		}
		else
		{
			//安全区内，正常飞行，听控制的
			CurEngineRpm = (OS_U32)(pOutput->engineSet * 10.0f);
			// if(CurEngineRpm == 0)
			// {
			// 	int a = 0;
			// }
			
		}
	}
    else if(flightSeq.luanched == 0 && ((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC))
    {
		//此时，马上就要飞了，油门应该很大才对
        CurEngineRpm = 800;	// TODO: ECU 油门设置需要再调整，目前是写软件时的临时设置。 油门暂且设置80.0%	//50500;
    }

	SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
}

/***********************************************************
 * 函数名称:FlightOutputHandle()
 * 函数功能:调用飞控主运算函数后，处理飞控主函数输出，这些输出包括:
 * 1.时序信息，用以触发时序，包括分离、火箭点火、开伞等需求
 * 2.遥测信息，用以下传地面
  ***********************************************************/
void FlightOutputHandle()  //01 5ms运行一次
{
	if(DOM_AUTOMATIC & g_DeviceState.workStage)
	{		
		FlightSrvOutputHandle();//飞控输出——>控舵机
		FlightEngineOutputHandle();//飞控输出——>控发动机
		FlightTMOutputHandle();//存储飞控输出
		JudgeHomeward();//判断安全区，判断是否需要伞降
	}
	FlightSeqOutputHandle();//02 如果收到伞降命令，关发动机、开伞、控舵机角度、气囊、舵机断电、割伞、发动机断电
	CalcXYZ();// 坐标系转换
}

/***********************************************************
 * 函数名称:FlightInputGenerate()
 * 函数功能:调用飞控主运算函数前，准备给飞控的输入数据，主要包括：
 * 1.惯组数据
 * 2.GPS数据
 * 3.时间(double,单位秒,精确到0.001)
 * 当当前为模飞模式时，惯组、GPS数据均来自于flash
  ***********************************************************/
double curAirSpd;
void FlightInputGenerate()
{
		if((g_DeviceState.workStage & DOM_HILSMODE) == DOM_HILSMODE)
		{
			//半实物仿真模式
			g_DeviceState.currTime = GetCurTime()-g_DeviceState.flightStartTime;
			HilFlightStage();
		}
		else//正式飞行模式
		{
			g_DeviceState.currTime = GetCurTime()-g_DeviceState.flightStartTime;        
			float tempf;
			GetDataFast(pDataPoolImu, "imuWx",	&tempf); pInput->wx = tempf;
			GetDataFast(pDataPoolImu, "imuWy",	&tempf); pInput->wy = tempf;
			GetDataFast(pDataPoolImu, "imuWz", 	&tempf); pInput->wz = tempf;
			GetDataFast(pDataPoolImu, "imuAx", 	&tempf); pInput->ax = tempf;
			GetDataFast(pDataPoolImu, "imuAy", 	&tempf); pInput->ay = tempf;
			GetDataFast(pDataPoolImu, "imuAz", 	&tempf); pInput->az = tempf;

			OS_S32 temps32;
			OS_S16 temps16;
			GetDataFast(pDataPoolImu, "navLon", 	&temps32);pInput->navLon = temps32 * 1e-7;
			GetDataFast(pDataPoolImu, "navLat", 	&temps32);pInput->navLat = temps32 * 1e-7;
			GetDataFast(pDataPoolImu, "navHigh", 	&tempf);pInput->navHigh = tempf;
					
					//test
					//SETDATA(pDataPoolImu, "abc", tempf * 10,	OS_U16);
					//OS_S16 temp16Alt;
					//GetDataFast(pDataPoolImu, "abc", 	&temp16Alt);
					//pInput->navHigh = temp16Alt * 0.1;
					//test end
					
			GetDataFast(pDataPoolImu, "navVn", &temps16);pInput->navVn = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navVs", &temps16);pInput->navVs = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navVe", &temps16);pInput->navVe = temps16 * 0.01;
			
			OS_U16 tempu16; //10-19修改OS_S16为OS_U16
			GetDataFast(pDataPoolImu, "navPitch", 	&temps16);pInput->pitch = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navDir", 	&tempu16);pInput->yaw = tempu16 * 0.01;
			GetDataFast(pDataPoolImu, "navRoll", 	&temps16);pInput->roll = temps16 * 0.01;
			pInput->yaw = -pInput->yaw; //北偏东转北偏西
			if(pInput->yaw > 180)
				pInput->yaw -= 360;
			if(pInput->yaw < -180)
				pInput->yaw += 360;
			
			pInput->Luanched = flightSeq.luanched;
			
			double V   = sqrt(pow(pInput->navVn,2) + pow(pInput->navVs,2) + pow(pInput->navVe,2));   // 地速计算
			//空速就是地速，高速飞机的空速没用
			pInput->airSpd = V;
					
			SETDATA(pDataPoolSelf, "GrdSpd", V * 10, OS_S16);//
			
			OS_S16 srv1,srv2;
			GetDataFast(pDataPoolSrv, "Sr1Read", &srv1);
			GetDataFast(pDataPoolSrv, "Sr2Read", &srv2);
			pInput->DD1 = srv1 * 0.01;
			pInput->DD2 = srv2 * 0.01;
			if(g_DeviceState.srvCountDown == 0)
			{
					pInput->DD1 = pOutput->rudder1Cmd;	//MML舵机
					pInput->DD2 = pOutput->rudder2Cmd;
			}				
			pInput->DD5 = 0;
			pInput->DD6 = 0;
		}
		//设置视线角和锁定状态
		GetDataFast(pDataPoolFly,	"sctLock",	 &(pInput->scoutLocked));
		OS_S16 tempss16;
		GetDataFast(pDataPoolFly,	"vPitchSp",	 &tempss16);
		pInput->scoutPitchSpd = (double)tempss16 * 0.002 / 57.3;
		GetDataFast(pDataPoolFly,	"vYawSp",	 &tempss16);
		pInput->scoutYawSpd = (double)tempss16 * 0.002 / 57.3;	
		OS_U16 fuel_n;
		GetDataFast(pDataPoolSelf,	"fuelRate",	 &fuel_n);
		pInput->mass_fuel = (double)fuel_n;
}
#define d2r		(57.29577951308402)

/***********************************************************
 * 函数名称:DoFlightRun()
 * 函数功能:调用飞控主运算函数，仅当发射后进入调用
  ***********************************************************/
void DoFlightRun()
{
	if((DOM_AUTOMATIC ) & g_DeviceState.workStage)
	{
		FlightRun(pInput, pOutput);
		pInput->Msn_updatesig = 0;
		
	}
}

OS_U8 InitSafeArea(OS_U8* buf)
{
	OS_S32 lon,lat;
	safePointCount = buf[0];// 围栏点数
	for(int i=0;i<safePointCount;i++)
	{
		lon = *((OS_S32*)(buf+ 2 + i*8));
		lat = *((OS_S32*)(buf+ 6 + i*8));
        safePoints[i].x = lon * 1e-7;   // 经度 
        safePoints[i].y = lat * 1e-7; 	// 纬度 
	}
    
	return 0;
    
}

// 计算两个向量的点积
double dotProduct(Point a, Point b) {
    return a.x * b.x + a.y * b.y;
}

// 计算向量的模
double vectorMagnitude(Point a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

// 计算三个点之间的夹角
double calculateAngle(Point p1, Point p2, Point p3)
{
    // 计算向量P1P3和P2P3
    Point vectorP1P3 = {p3.x - p1.x, p3.y - p1.y};
    Point vectorP2P3 = {p3.x - p2.x, p3.y - p2.y};

    // 计算向量的点积
    double dot = dotProduct(vectorP1P3, vectorP2P3);

    // 计算向量的模
    double magnitudeP1P3 = vectorMagnitude(vectorP1P3);
    double magnitudeP2P3 = vectorMagnitude(vectorP2P3);

    // 计算夹角的余弦值
    double cosTheta = dot / (magnitudeP1P3 * magnitudeP2P3);

    // 计算并返回夹角（以弧度为单位）
    return acos(cosTheta);
}

OS_BOOL JudgeInSafe2(double lon, double lat)
{
	Point curPt;
	curPt.x = lon;
	curPt.y = lat;
	double angle[100];
	int i;
	for(i=0; i<safePointCount-1; i++)
	{
		angle[i] = calculateAngle(safePoints[i], safePoints[i+1], curPt);
	}
	angle[i] = calculateAngle(safePoints[i], safePoints[0], curPt);

	double allAngle = 0;
	for(i=0;i<safePointCount;i++)
	{
		allAngle += angle[i];
	}
	OS_BOOL is_inside;
	if(allAngle> 2*3.1 && allAngle < 2*3.2)//2pai
	{
		is_inside = TRUE;
	}
	else
	{
		is_inside = FALSE;
	}
	return is_inside;
}

extern OS_U8 RecoverMark;
OS_U8 JudgeHomeward()	//02 起飞2s后，每1s判断一次，看是否出了安全区？是否需要伞降？是否地面发出紧急返航？
{
	//每秒判断一次是否出安全区返航和任务机失联
	if(g_DeviceState.CurrTick % 200 != 0)
		return 1;

	//判断安全区驶出
	if(g_DeviceState.CurrTick < 200 * 2)//2s后起判安全区
		return 1;

	int ilon,ilat;
	double lon,lat;
	GetDataFast(pDataPoolImu, "navLon", &ilon);
	GetDataFast(pDataPoolImu, "navLat", &ilat);
	lon = ilon * 1e-7;
	lat = ilat * 1e-7;

	if(OutSafeArea)
	{
		// 出安全区了
		if(OutSafeCount == 30)
		{
			StopEngine();
			CurEngineRpm = 0;
		}
		if(OutSafeCount == 0)
		{
			DoOpenUm();//安全区开伞
		}
		else
		{
			OutSafeCount--;
		}
	}
	//出安全区开?
  static int judgeError = 0;
	if(JudgeInSafe2(lon,lat) == FALSE)
	{
		// 出安全区了
		if(RecoverMark == 1)	// 地面发出紧急返航，该值置为1
		{
				;
		}
		else
		{
			if(judgeError >= 3)
			{
				SETDATA(pDataPoolSelf,  "flyError", 0xCC,	OS_U8);
				if(OutSafeArea == false)
					OutSafeCount = 35;
				OutSafeArea = true;
				CurEngineRpm = 200;	// TODO: ECU 油门设置需要再调整，目前是写软件时的临时设置。 油门暂且设置20.0%	//18000;
			}
			else
			{
					judgeError++;
			}
		}
	}
	else
	{
		// 在安全区内
		judgeError = 0;
		OutSafeArea = false;
	}

	//地面指令紧急返航，到返航点后开伞
	if(homeMsn.MsnCmdType != 0)
	{
		double dist = haversine_distance(lat, lon, homeMsn.targetLat, homeMsn.targetLon);
		if(dist < 150)
		{
			SETDATA(pDataPoolSelf,  "flyError", 0x3,	OS_U8);
			DoOpenUm();//紧急返航开伞
		}
	}
	return 0;
}

//extern RoutePoint rp[RP_MAX_NUMBER];
extern int RP_NUMBER;



OS_U8 DoOpenUm()
{
	//解除飞控算法对控制系统的管理
	g_DeviceState.workStage &= ~DOM_AUTOMATIC;

	//开伞
	flightSeq.umOpen = 1;
	//CurEngineRpm = 18000;
	//SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
	//SETDATA(pDataPoolMsn, "OpenUm", 1,	OS_U8);
	return 0;
}


