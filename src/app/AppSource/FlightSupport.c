/*
 * FlightSupport.c
 *
 *  Created on: 2022??3??23??
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
// #include "./flight/os_flight_io.h"
// #include "./flight/RoutePoint.h"
#include "./interface/interface_power.h"
#include "./mission/mission.h"
#include "math.h"
#include <stdlib.h>
#include "interface_timer.h"
#include "fuse.h"
#include "StateMachine.h"
#include "./core/BusInteract.h"
#include <stdio.h>
#include <string.h>


//#include "flight_data.h"
Point safePoints[20];	// 加载任务后，得到的安全区域围栏
int airHighArray[1000] = {0};
int safePointCount;

// MISSION homeMsn = {0};
extern Stru_Data_Controller_To_DatalinkTel  g_CtrltoDL_tel;
extern double	g_flight_time;
extern int		g_time_tick;

#define d2r		(57.29577951308402)

/* 弹目距离等写入 int16 遥测：超出范围饱和到 ±32767 */
static OS_S16 TmClampS16(double v)
{
	if(v > 32767.0)
		return (OS_S16)32767;
	if(v < -32767.0)
		return (OS_S16)(-32767);
	return (OS_S16)v;
}
/***********************************************************
 * FlightSeqOutputHandle()
 * Parachute / recovery sequence after DoOpenUm() sets umOpen=1.
 * Called from FlightOutputHandle() every ~5 ms (waitOneSec +1 per call).
 ***********************************************************/
static void FlightSeqOutputHandle()
{
	if(flightSeq.umOpen == 1)
	{
		OS_U8 CurState;
		static int waitOneSec = 0;

		/* Keep commanding engine stop until ECU reports stopped (~every 20 ms) */
		if(waitOneSec >= 0 && waitOneSec < 10000)
		{
			if(waitOneSec % 4 == 0)
			{
				GetDataFast(pDataPoolSelf, "ecuState", &CurState);
				if(CurState != ENGINE_STOPED)
				{
					StopEngine();
				}
			}
		}
		/* T+200 ms: drive parachute servo (PWM7) to 50 deg */
		if(waitOneSec == 40)
		{
			AngleServo_SetAngle(SERVO_PWM7, 50.0f);
		}
		/* T+1 s: center all 6 flight control rudders */
		if(waitOneSec == 200)
		{
			ServoCtlOnce_6Rudder(0, 0, 0, 0, 0, 0);
		}
		/* T+15 s / T+15.4 s: reserved pyrotechnic channels (disabled) */
		if(waitOneSec == 3000)
		{
			/* TrigerSeqWithWidth(SEQ_1 + 2, 100); */
		}
		if(waitOneSec == 3080)
		{
			/* TrigerSeqWithWidth(SEQ_1 + 3, 100); */
		}
		/* T+20 s: cut servo bus 28 V */
		if(waitOneSec == 4000)
		{
			PowerOff(DEVICE_SRV_PWR28V);
		}
		/* After T+20 s: fire landing pyrotechnic once when touchdown is detected */
		static OS_U8 yijinggesan = 0;
		if(waitOneSec > 4000 && yijinggesan == 0)//ZXT-014不需要割伞
		{
			// OS_S16 navVs;
			// GetDataFast(pDataPoolImu, "navVs", &navVs);
			// float fvs = navVs * 0.01f;	/* vertical speed m/s */
			// if(g_DeviceState.imuCountDown != 0)
			// {
			// 	/* IMU link alive: touchdown if |Vz| < 1 m/s */
			// 	if(fabs(fvs) < 1.0)
			// 	{
			// 		TrigerSeqWithWidth(TEST_1 + 0, 100);
			// 		yijinggesan = 1;
			// 	}
			// }
			// else
			// {
			// 	/* IMU down: touchdown if baro height stable within 5 m for ~5 s */
			// 	OS_S16 airHigh;
			// 	GetDataFast(pDataPoolSelf, "AirHigh", &airHigh);
			// 	memmove(&airHighArray[1], &airHighArray[0], sizeof(int) * 999);
			// 	airHighArray[0] = airHigh;
			// 	OS_U8 airStable = 1;
			// 	for(int i = 0; i < 1000; i++)
			// 	{
			// 		if(fabs(airHigh - airHighArray[i]) > 5)
			// 		{
			// 			airStable = 0;
			// 			break;
			// 		}
			// 	}
			// 	if(airStable == 1)
			// 	{
			// 		TrigerSeqWithWidth(TEST_1 + 0, 100);
			// 		yijinggesan = 1;
			// 	}
			// }
		}
		else
		{
			waitOneSec++;
		}
		OS_U16 ecuTemp;
		GetDataFast(pDataPoolSelf, "ecuTemp", &ecuTemp);
		if(ecuTemp * 0.1 < 65)
		{
			/* Optional: PowerOff(DEVICE_BATT_ENGINE) when ECU cools below 65 C */
		}
	}
}

/***********************************************************
 * 
 * 函数名称:FlightTMOutputHandle()
 * 
***********************************************************/
void FlightTMOutputHandle()	
{
	SETDATA(pDataPoolMsn,	"curPtNo",	g_CtrltoDL_tel.curPtNo,	OS_U8);         		//当前航点号
	SETDATA(pDataPoolMsn,	"tarLon",	g_CtrltoDL_tel.curTargetLon * 1e7, OS_S32 );	//当前目标航点经度
	SETDATA(pDataPoolMsn,	"tarLat",	g_CtrltoDL_tel.curTargetLat * 1e7, OS_S32 );	//当前目标航点纬度
	SETDATA(pDataPoolMsn,	"tarAlt",	g_CtrltoDL_tel.curTargetAlt, OS_S16 );			//当前目标航点高度
	SETDATA(pDataPoolFly,	"rollCmd",	g_CtrltoDL_tel.rudderRollCmd* 100,	OS_S16);	//通道舵副翼
	SETDATA(pDataPoolFly,	"pitchCmd",	g_CtrltoDL_tel.rudderPitchCmd * 100,OS_S16);	//通道舵升降	
	SETDATA(pDataPoolFly,	"yawCmd",	g_CtrltoDL_tel.rudderYawCmd* 100,	OS_S16);	//通道舵航向

	SETDATA(pDataPoolFly,	"gamaCmd",	g_CtrltoDL_tel.gamaCmd * 1e2, OS_S16 );			//滚转角指令
	SETDATA(pDataPoolFly,	"nycCmd",	g_CtrltoDL_tel.nycCmd * 1e2, OS_S16 ); 			//升力面过载指令
	SETDATA(pDataPoolFly,	"varthCmd",	g_CtrltoDL_tel.varthetaCmd * 1e2, OS_S16 );     //俯仰角指令
	SETDATA(pDataPoolFly,	"highCmd",	g_CtrltoDL_tel.heightCmd/0.1,OS_S16 );			//高度指令
	SETDATA(pDataPoolFly,	"vyCmd",	g_CtrltoDL_tel.vyCmd/0.003 , OS_S16 );			//垂速指令
	SETDATA(pDataPoolFly,	"ac_dL",	g_CtrltoDL_tel.ac_dL * 100, OS_U32);			//待飞距
	SETDATA(pDataPoolFly,	"ac_dZ",	g_CtrltoDL_tel.ac_dZ * 100, OS_U16);			//侧边距
	SETDATA(pDataPoolFly,   "fcstate",  g_CtrltoDL_tel.flight_control_state,OS_U32);    //飞行状态
	
	SETDATA(pDataPoolFly,   "thrusCmd", g_CtrltoDL_tel.thrustCmd*10,OS_U16);            //推力指令，例如油门开度Kc 无符号
	SETDATA(pDataPoolFly,   "rpmState", g_CtrltoDL_tel.rpmState/2,OS_U16);             	//发动机状态转速
	SETDATA(pDataPoolFly,	"ac_Vz",	g_CtrltoDL_tel.ac_Vz/0.003,	OS_S16);			//侧向速度
	SETDATA(pDataPoolFly,	"ac_Vy",	g_CtrltoDL_tel.ac_Vy/0.003,OS_S16);				//天向速度	
	SETDATA(pDataPoolFly,	"ac_Vx",	g_CtrltoDL_tel.ac_Vx/0.003,	OS_S16);			//射向速度
	SETDATA(pDataPoolFly,	"ac_dR",	g_CtrltoDL_tel.ac_dR * 10, OS_S16 );			//圆轨迹侧边距，未去掉转弯半径
	SETDATA(pDataPoolFly,	"azimuth",	g_CtrltoDL_tel.cur_azimuth/0.006, OS_S16 );		//航段方位角
	SETDATA(pDataPoolFly,	"thetav",	g_CtrltoDL_tel.cur_thetav/0.003, OS_S16 );		//轨迹倾角
	SETDATA(pDataPoolFly,	"psicv",	g_CtrltoDL_tel.cur_psicv/0.006, OS_S16 );		//轨迹偏角
	SETDATA(pDataPoolFly,	"Vcmd",	    g_CtrltoDL_tel.Vcmd * 10, OS_S16 );				//速度指令 
	SETDATA(pDataPoolFly,	"nyCmd",	g_CtrltoDL_tel.nyCmd_Guidance * 100 , OS_S16 );	//末制导纵向过载指令
	SETDATA(pDataPoolFly,	"nzCmd",	g_CtrltoDL_tel.nzCmd_Guidance * 100, OS_S16 );	//末制导侧向过载指令
	SETDATA(pDataPoolFly,	"wyCmd",	g_CtrltoDL_tel.wyCmd * 100, OS_S16 );			//航向角速度指令

	SETDATA(pDataPoolFly,	"pitch_nT",	g_CtrltoDL_tel.pitch_rate_nT_filterOut * 1000, OS_S16 );//俯仰视线角速度滤波
	SETDATA(pDataPoolFly,	"yaw_nT",	g_CtrltoDL_tel.yaw_rate_nT_filterOut * 1000, OS_S16 );  //偏航视线角速度滤波
	SETDATA(pDataPoolFly,	"deltaR",	g_CtrltoDL_tel.deltaR*100,	OS_U32);			//弹目距离，打击点
	SETDATA(pDataPoolFly,	"dRn",		TmClampS16(g_CtrltoDL_tel.dRn),	OS_S16);
	SETDATA(pDataPoolFly,	"dRu",		TmClampS16(g_CtrltoDL_tel.dRu),	OS_S16);
	SETDATA(pDataPoolFly,	"dRe",		TmClampS16(g_CtrltoDL_tel.dRe),	OS_S16);
	SETDATA(pDataPoolFly,	"PitchPre",	g_CtrltoDL_tel.Pitch_Preset_Angle * 100,OS_S16);//理论俯仰框架角
	SETDATA(pDataPoolFly,	"YawPre",	g_CtrltoDL_tel.Yaw_Preset_Angle * 100,OS_S16);	//理论偏航框架角
	SETDATA(pDataPoolFly,	"DusState",	g_CtrltoDL_tel.Dubins_stage, OS_S32);			//杜宾斯段
	SETDATA(pDataPoolFly,	"DusType1",	g_CtrltoDL_tel.dubins_type1, OS_S32);			//杜宾斯类型
	SETDATA(pDataPoolFly,	"DusType2",	g_CtrltoDL_tel.dubins_type2, OS_S32);			//杜宾斯类型
	SETDATA(pDataPoolFly,	"DusType3",	g_CtrltoDL_tel.dubins_type3, OS_S32);			//杜宾斯类型
	SETDATA(pDataPoolFly,	"DbsLen",	g_CtrltoDL_tel.Dubins_length , OS_FLOAT);		//杜宾斯段航程
	SETDATA(pDataPoolFly,	"gamacCom",	g_CtrltoDL_tel.gamac_compensate/0.003,OS_S16);	//滚转角指令补偿量
	SETDATA(pDataPoolFly,	"uz_gamac",	g_CtrltoDL_tel.uz_gamac/0.003,OS_S16);			//侧偏控制量
	SETDATA(pDataPoolFly,	"mx_ESO",	g_CtrltoDL_tel.mx_ESO*100,OS_S16);				//干扰估计状态量z2
	SETDATA(pDataPoolFly,	"ADRC",	    g_CtrltoDL_tel.fduox_ADRC * 100, OS_S16);		//ADRC舵偏
	SETDATA(pDataPoolFly,	"Qv",	    g_CtrltoDL_tel.Qv * 100, OS_U32);				//动压
	SETDATA(pDataPoolFly,   "alphaIns", g_CtrltoDL_tel.alpha_ins*100,	OS_S16);		//地速攻角
	SETDATA(pDataPoolFly,   "betaIns",  g_CtrltoDL_tel.beta_ins*100,	OS_S16);        //地速侧滑角	
	SETDATA(pDataPoolFly,	"nyflt",	g_CtrltoDL_tel.nyflt/0.001, OS_S16 );			//体轴法向过载
	SETDATA(pDataPoolFly,	"nzflt",	g_CtrltoDL_tel.nzflt/0.001, OS_S16 );			//体轴侧向过载
	SETDATA(pDataPoolFly,   "cnt_alti", g_CtrltoDL_tel.count_altitude_change,OS_U32);   //高度机动次数
	SETDATA(pDataPoolFly,	"mass_cal",	g_CtrltoDL_tel.mass_calc/0.01, OS_S16 );		//质量估计
	SETDATA(pDataPoolFly,	"ugfZetac", g_CtrltoDL_tel.ugf_zetac/0.001, OS_S16 );		//高度控制量
	SETDATA(pDataPoolFly,	"uqkf",		g_CtrltoDL_tel.uqkf/0.001, OS_S16 );			//前馈控制量

	SETDATA(pDataPoolFly,	"curLon",	g_CtrltoDL_tel.curLon * 1e7, OS_S32 );    //当前经度
	SETDATA(pDataPoolFly,	"curLat",	g_CtrltoDL_tel.curLat * 1e7, OS_S32 );	  //当前纬度
	SETDATA(pDataPoolFly,	"curAlt",	g_CtrltoDL_tel.curAlt/0.1,OS_S16 );		  //当前高度

	// SETDATA(pDataPoolFly,	"tokenlon",	g_CtrltoDL_tel.token_long, OS_U8 );			//纵向令牌
	// SETDATA(pDataPoolFly,	"tokenlat",	g_CtrltoDL_tel.token_late, OS_U8 );			//侧向令牌
    // SETDATA(pDataPoolNav, "MaxRpm", g_CtrltoDL_tel.MaxRpm*100,	OS_S16);			//最大转速
	// SETDATA(pDataPoolNav, "DFT_freq", g_CtrltoDL_tel.DFT_freq_max*100,	OS_S16);	//辨识运动频率
	if(g_controller_to_switch.flag_missle_takeoff == 1)
	{
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xEE , OS_U8 );//
	}

}

void FlightSrvOutputHandle()	//02 控制输出角度—�?>操作舵机
{
	/* currTime is seconds since DoIgnition (HIL sim start also calls it) */
	// if(g_DeviceState.currTime > 200.0 && fabs(g_controller_to_actuator.control_voltage_I) > 10.0)
	// {
	// 	int a=0;
	// }
	ServoCtlOnce_6Rudder(	g_controller_to_actuator.control_voltage_I	,
							g_controller_to_actuator.control_voltage_II	,
							g_controller_to_actuator.control_voltage_III,
							g_controller_to_actuator.control_voltage_IV	,
							g_controller_to_actuator.control_voltage_V	,
							g_controller_to_actuator.control_voltage_VI
							);
	//ServoCtlOnce_6Rudder(g_CtrltoDL_tel.rudder_I_cmd,g_CtrltoDL_tel.rudder_II_cmd,g_CtrltoDL_tel.rudder_III_cmd,g_CtrltoDL_tel.rudder_IV_cmd,g_CtrltoDL_tel.rudder_V_cmd,g_CtrltoDL_tel.rudder_VI_cmd);
}
OS_U8 OutSafeArea = 0;	// 0安全区里	1出安全区，且等了一会儿�?定出了安全区
void FlightEngineOutputHandle()//02 在各种情况下（是否起飞？�?否出安全区？）控制输出油门开度—�?>操作油门开度全局变量
{
	OS_U32 curEngineRpm;

	if ((g_DeviceState.workStage & DOM_HILSMODE) && g_DeviceState.hilCountDown > 0) {
		curEngineRpm = (OS_U32)(g_controller_to_engine.control_Kc * 10.0);
		SETDATA(pDataPoolFly, "EngineRp", curEngineRpm, OS_U16);
		return;
	}

	//if(flightSeq.luanched == 1 && ((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC))
	if(flightSeq.luanched == 1)
	{
		if(OutSafeArea)
		{
		}
		else
		{
			//安全区内，�?�常飞�?�，�?控制�?
			CurEngineRpm = (OS_U32)(g_controller_to_engine.control_Kc * 10.0f);
		}
	}
    else
    {
        CurEngineRpm = 300;// about to take off
    }
	SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
}

/***********************************************************
 * FlightOutputHandle()
 * 5 ms flight output dispatcher when AUTOMATIC or HIL:
 * 1) Servo / engine / TM publish + homeward check
 * 2) Parachute sequence + ECEF position update
 ***********************************************************/
void FlightOutputHandle()  /* 5 ms period */
{
	if((DOM_AUTOMATIC & g_DeviceState.workStage) || (DOM_HILSMODE & g_DeviceState.workStage))
	{
		FlightSrvOutputHandle();       /* control voltage -> servos */
		FlightEngineOutputHandle();    /* throttle / rpm command */
		FlightTMOutputHandle();        /* pack control TM to data pool */
		JudgeHomeward();               /* geofence / homeward logic */
	}
	FlightSeqOutputHandle();           /* parachute / recovery timing */
	CalcXYZ();                         /* update ECEF XYZ */
}

/***********************************************************
 * FlightInputGenerate()
 * Build controller inputs each 5 ms cycle:
 * 1) Flight / control time (currTime, g_flight_time, g_time_tick)
 * 2) HIL: fill g_ins_data / engine / baro from simulator
 * 3) Real flight: map IMU/NAV pool to control-frame INS + engine + baro
 *    Pool scales: angles/speeds *0.01; lon/lat *1e-7
 ***********************************************************/
extern float static_pressure,total_pressure;
void FlightInputGenerate()
{
	/* Update flight clock first so ControlRun sees the new currTime same cycle */
	g_DeviceState.currTime = GetCurTime() - g_DeviceState.flightStartTime;
	g_flight_time = g_DeviceState.currTime;
	g_time_tick   = g_DeviceState.CurrTick;

	/* HIL / simulation path */
	if((g_DeviceState.workStage & DOM_HILSMODE) == DOM_HILSMODE)
	{
		HilFlightStage();
	}
	/* Real flight: onboard IMU/NAV -> control body frame */
	else
	{
		float tempf;
		float imu1,imu2,imu3,imu4,imu5,imu6;
		OS_S32 temps32;
		OS_S16 temps16;
		OS_U16 tempu16;

		GetDataFast(pDataPoolImu, "imuWx",	&tempf); imu1 = tempf;
		GetDataFast(pDataPoolImu, "imuWy",	&tempf); imu2 = tempf;
		GetDataFast(pDataPoolImu, "imuWz", 	&tempf); imu3 = tempf;
		GetDataFast(pDataPoolImu, "imuAx", 	&tempf); imu4 = tempf;
		GetDataFast(pDataPoolImu, "imuAy", 	&tempf); imu5 = tempf;
		GetDataFast(pDataPoolImu, "imuAz", 	&tempf); imu6 = tempf;
		/* Axis remap to control body frame */
		g_ins_data.wx = imu2; /* imuWy */
		g_ins_data.wy = imu3; /* imuWz */
		g_ins_data.wz = imu1; /* imuWx */
		g_ins_data.ax = imu5; /* imuAy */
		g_ins_data.ay = imu6; /* imuAz */
		g_ins_data.az = imu4; /* imuAx */
		GetDataFast(pDataPoolImu, "navPitch", 	&temps16);g_ins_data.zeta = temps16 * 0.01;
		GetDataFast(pDataPoolImu, "navDir", 	&tempu16);g_ins_data.psi = tempu16 * 0.01;
		GetDataFast(pDataPoolImu, "navRoll", 	&temps16);g_ins_data.gama = temps16 * 0.01;
		/* Heading: negate and wrap 0~360 deg into -180~180 deg */
		g_ins_data.psi = -g_ins_data.psi;
		if(g_ins_data.psi > 180)
			g_ins_data.psi -= 360;
		if(g_ins_data.psi < -180)
			g_ins_data.psi += 360;
		/* NED velocities and geodetic position */
		GetDataFast(pDataPoolImu, "navVn", &temps16);g_ins_data.vtx = temps16 * 0.01;
		GetDataFast(pDataPoolImu, "navVs", &temps16);g_ins_data.vty = temps16 * 0.01;
		GetDataFast(pDataPoolImu, "navVe", &temps16);g_ins_data.vtz = temps16 * 0.01;
		GetDataFast(pDataPoolImu, "navLon", 	&temps32);	g_ins_data.longitude = temps32 * 1e-7;
		GetDataFast(pDataPoolImu, "navLat", 	&temps32);	g_ins_data.latitude = temps32 * 1e-7;
		GetDataFast(pDataPoolImu, "navHigh", 	&tempf);	g_ins_data.height = tempf;
		GetDataFast(pDataPoolImu, "navState", &g_ins_data.GPS_status);

		GetDataFast(pDataPoolSelf, "ecuGetRp", &g_engine_data.rpm_engine);
		GetDataFast(pDataPoolSelf, "ecuState", &g_engine_data.ECU_work_status);

		g_baro_data.static_pressure = static_pressure;
		g_baro_data.total_pressure = total_pressure;
		//导引头后续赋值
		g_seeker_data.flag_combat_status=0;
		g_seeker_data.flag_combat_status=0;

		double V   = sqrt(pow(g_ins_data.vtx,2) + pow(g_ins_data.vty,2) + pow(g_ins_data.vtz,2)); /* ground speed m/s */
		SETDATA(pDataPoolSelf, "GrdSpd", V * 10, OS_S16);
	}
}
#define d2r		(57.29577951308402)

/***********************************************************
 * DoFlightRun()
 * Run flight-control math once when AUTOMATIC or HIL is active.
 ***********************************************************/
extern void *g_pControl;
void DoFlightRun()
{
	if((DOM_AUTOMATIC & g_DeviceState.workStage) || (DOM_HILSMODE & g_DeviceState.workStage))
	{
		ControlRun(g_pControl);
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

/**函数功能：判断某个经纬度点是否在安全区内
 * 返回值：	1	安全区内
 * 			0	安全区外*/
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
OS_U8 JudgeHomeward()	//02 起飞2s后，每1s判断一次，是否出了安全区？是否需要伞降？是否地面发出紧急返航？
{
	static OS_U8 OutSafeCount = 0;

	//每秒判断一次
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
		if(OutSafeCount == 5)
		{
			StopEngine();
			CurEngineRpm = 0;
		}

		if(OutSafeCount == 0)
		{
			DoOpenUm();//安全区外开伞
		}
		else
		{
			OutSafeCount--;
		}
	}

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
				// 出安全区，且时间超过3s
				SETDATA(pDataPoolSelf,  "flyError", 0xCC,	OS_U8);
				
				if(OutSafeArea == false)
					OutSafeCount = 35;
				OutSafeArea = true;
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

	// //???????????????????????????
	//if(homeMsn.MsnCmdType != 0)	// TODO:????????????????????????????????????????????????????
	//{
	// 	double dist = haversine_distance(lat, lon, homeMsn.targetLat, homeMsn.targetLon);
	// 	if(dist < 150)
	// 	{
	// 		SETDATA(pDataPoolSelf,  "flyError", 0x3,	OS_U8);
	// 		DoOpenUm();//????????????
	// 	}
	//}
	return 0;
}

//extern RoutePoint rp[RP_MAX_NUMBER];
// extern int RP_NUMBER;


/** Start parachute sequence (urgent land / recovery).
 * Clears DOM_AUTOMATIC so autopilot stops; FlightSeqOutputHandle()
 * then runs stop-engine / open-chute / power-down timing via umOpen.
 */
OS_U8 DoOpenUm()
{
	g_DeviceState.workStage &= ~DOM_AUTOMATIC;
	flightSeq.umOpen = 1;
	/* 与控制律开伞同步：遥测时序开伞标志*/
	SETDATA(pDataPoolSelf, "openUmb", 1, OS_U8);
	return 0;
}


