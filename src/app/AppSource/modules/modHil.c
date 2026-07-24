/*
 * modHil.c
 *
 *  Created on: 2024??4??6??
 *      Author: lenovo
 */
#include "modHil.h"
#include "../FlightSupport.h"
#include "./modPwrSeqCtl.h"
#include "../core/DataPool.h"
#include "../core/BusInteract.h"
#include "../interface/interface_power.h"
// #include "../flight/os_flight_io.h"
#include "./modNav.h"
#include "../StateMachine.h"
#include "flightPort.h"
#include "../controller/controller.h"
#include <math.h>
#define d2r		(57.29577951308402)
STRU_HIL_INPUT hilInput;
OS_U16 g_hilEchoSequence = 0;
static OS_U16 g_hilPendingSequence = 0;
float fwxhil,fwyhil,fwzhil;
OS_DOUBLE pitchspd, yawspd;
extern void *g_pControl;
//OS_U16 enableDelayTick = 240;
void StopHilTest();

static void HilReinitControl(void)
{
	if (g_pControl != NULL)
	{
		deleteCMathControlMain(g_pControl);
		g_pControl = NULL;
	}
	g_pControl = ControlInitial();
}

static void HilSimOutputToInput(const Stru_Sim_Data_OUTPUT *src, STRU_HIL_INPUT *dst)
{
	memset(dst, 0, sizeof(STRU_HIL_INPUT));
	dst->runStop = src->runStop;
	dst->useNav = src->useNav;
	dst->wx = src->wx;
	dst->wy = src->wy;
	dst->wz = src->wz;
	dst->ax = src->ax;
	dst->ay = src->ay;
	dst->az = src->az;
	dst->pitch = src->pitch;
	dst->yaw = src->yaw;
	dst->roll = src->roll;
	dst->airSpd = src->airSpd;
	dst->lon = src->lon;
	dst->lat = src->lat;
	dst->alt = src->alt;
	dst->vn = src->vn;
	dst->vs = src->vs;
	dst->ve = src->ve;
	dst->DD1 = src->DD1;
	dst->DD2 = src->DD2;
	dst->rpm_engine_state = src->rpm_state;
	dst->qf = src->qf;
	dst->qh = src->qh;
	dst->dqf = src->dqf;
	dst->dqh = src->dqh;
	dst->seeker_state = (OS_U16)src->TargetLocked;
}
OS_U8 SaveHilInDataPool(STRU_HIL_INPUT *hilInfo)
{
	memcpy(&hilInput, hilInfo, sizeof(STRU_HIL_INPUT));
	return 0;
		// SETDATA(pDataPoolImu, "imuWx", hilInput.wx,	OS_FLOAT);
		// SETDATA(pDataPoolImu, "imuWy", hilInput.wy,	OS_FLOAT);
		// SETDATA(pDataPoolImu, "imuWz", hilInput.wz,	OS_FLOAT);
		// SETDATA(pDataPoolImu, "imuAx", hilInput.ax,	OS_FLOAT);
		// SETDATA(pDataPoolImu, "imuAy", hilInput.ay,	OS_FLOAT);
		// SETDATA(pDataPoolImu, "imuAz", hilInput.az,	OS_FLOAT);

		// SETDATA(pDataPoolImu, "navLon", hilInput.lon * 1e7,	OS_S32);
		// SETDATA(pDataPoolImu, "navLat", hilInput.lat * 1e7,	OS_S32);
		// SETDATA(pDataPoolImu, "navHigh", hilInput.alt,	    OS_FLOAT);
		// SETDATA(pDataPoolImu, "navVn", hilInput.vn * 1e2,	OS_S16);
		// SETDATA(pDataPoolImu, "navVs", hilInput.vs * 1e2,	OS_S16);
		// SETDATA(pDataPoolImu, "navVe", hilInput.ve * 1e2,	OS_S16);

		// double dir = hilInput.yaw;
		// dir = -dir;
		// //if(dir < 0)
		// //	dir += 360;
		// SETDATA(pDataPoolImu, "navPitch", hilInput.pitch * 1e2,	OS_S16);
		// SETDATA(pDataPoolImu, "navDir", dir * 1e2,	OS_U16);//
		// SETDATA(pDataPoolImu, "navRoll", hilInput.roll * 1e2,	OS_S16);
		
		// if(g_DeviceState.srvCountDown == 0)
		// {
		// 	// 280 ?????????????????100?????hilInfo->DD1?????????100???????
		// 	SETDATA(pDataPoolSrv, "Sr1Read", hilInput.DD1, OS_S16);	
		// 	SETDATA(pDataPoolSrv, "Sr2Read", hilInput.DD2, OS_S16);	
		// }
}

OS_U32 HilRtHandler(STRU_422_MSG_INFO * frame)// RT_HIL
{
	g_DeviceState.hilCountDown = 200;
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case 0xAA://???????
	{	
		OS_U8 cmd = frame->au8Data[0];
		if(cmd == CMD_HOR_CALC_REQ || cmd == CMD_TO_NAV_REQ)
		{
			STRU_422_MSG_INFO frame1;
			frame1.u8MsgID = frame->au8Data[0];
			NavCmdHandler(&frame1);
		}
	}
		break;
	case 0xAB://???????
	{
		OS_DOUBLE lon,lat,high;
		OS_DOUBLE pitch,yaw;
		memcpy(&lon, frame->au8Data, 8);
		memcpy(&lat, frame->au8Data + 8, 8);
		memcpy(&high, frame->au8Data + 16, 8);
		memcpy(&pitchspd, frame->au8Data + 24, 8);
		memcpy(&yawspd, frame->au8Data + 32, 8);
		memcpy(&pitch, frame->au8Data + 40, 8);
		memcpy(&yaw, frame->au8Data + 48, 8);
    	if(g_DeviceState.scoutCountDown == 0)
		{
			SETDATA(pDataPoolFly,	"sctLock",	   1,		OS_U8);
			SETDATA(pDataPoolFly,	"sctPitch",	   0,		OS_S16);
			SETDATA(pDataPoolFly,	"sctYaw",	   0,		OS_S16);

			SETDATA(pDataPoolFly, 	"viewPitc",    pitch/0.01, 	OS_S16);//s16 90/32767
			SETDATA(pDataPoolFly, 	"viewYaw",     yaw/0.01, 	OS_S16);//u16 	360/65535
        
			SETDATA(pDataPoolFly,	"vPitchSp",	   pitchspd/0.002,		OS_S16);
			SETDATA(pDataPoolFly,	"vYawSp",	   yawspd/0.002,		OS_S16);           
		}
		else
		{
           
		}
        
	}
		break;
	case 0x22://????????
	case 0x23://????????
		ControllerCmdHandler(frame);
		break;
	case CMD_MSN_NEWPT://??????????????	// 280 ???????? ????
		{
			// double lon,lat,high,track,speed,arriveTime;
			// memcpy(&lon, frame->au8Data + 1, 8);
			// memcpy(&lat, frame->au8Data + 9, 8);
			// memcpy(&high, frame->au8Data + 17, 8);
			// memcpy(&track, frame->au8Data + 25, 8);
			// memcpy(&speed, frame->au8Data + 33, 8);
			// memcpy(&arriveTime, frame->au8Data + 41, 8);
			// MISSION msn;
			// msn.MsnCmdType = frame->au8Data[0];
			// msn.targetLon = lon;
			// msn.targetLat = lat;
			// msn.targetHigh = high;
			// msn.speed = speed;
			// msn.outTrack = track;
			// UpdateMission(msn);
		}
		break;
	case 0x20:	// : ????????????
		{
			Stru_Sim_Data_OUTPUT hilInfoSrc = {0};
			memcpy(&hilInfoSrc, frame->au8Data, sizeof(hilInfoSrc));
			/* DD4 在模型协议中未使用，作为 16 位 HIL 环回序号。 */
			if(hilInfoSrc.DD4 >= 0.0 && hilInfoSrc.DD4 <= 65535.0)
				g_hilPendingSequence = (OS_U16)hilInfoSrc.DD4;
			STRU_HIL_INPUT hilInfo = {0};
			HilSimOutputToInput(&hilInfoSrc, &hilInfo);
			SaveHilInDataPool(&hilInfo);
			
			//???????????J
			if(hilInfo.runStop == 0)
			{
				StopHilTest();
				return 0;
			}
			
			//???????????????????????
			if((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC)
			{
				g_DeviceState.workStage |= DOM_HILSMODE;//???????
				if(flightSeq.luanched != 1)
				{
					DoIgnition();
					HilReinitControl();//???????
				}
			}
			SETDATA(pDataPoolMsn,	"autoStep",	15,	OS_U8);
		}
		break;
	case CMD_LAUNCH_REQ:	//0xFA ???????
		{

		}
			break;
	}
	return 0;
}

static double HilYawToNavDirTelemetry(double yaw_deg)
{
	double dir = -yaw_deg;

	while (dir < 0.0) {
		dir += 360.0;
	}
	while (dir >= 360.0) {
		dir -= 360.0;
	}
	return dir;
}

static void HilWriteSimAirTelemetry(void)
{
	SETDATA(pDataPoolSelf, "AirSpd", (OS_S16)(hilInput.airSpd * 10.0), OS_S16);
	SETDATA(pDataPoolSelf, "AirPress", (OS_S16)(hilInput.DD1 * 0.1), OS_S16);
	//SETDATA(pDataPoolSelf, "AirHigh", (OS_S16)g_ins_data.height, OS_S16);
}

static void HilWriteSimSeekerTelemetry(void)
{
	SETDATA(pDataPoolFly, "sctLock", (OS_U8)(hilInput.seeker_state != 0U), OS_U8);
	SETDATA(pDataPoolFly, "sctPitch", (OS_S16)(hilInput.qf / 0.01), OS_S16);
	SETDATA(pDataPoolFly, "sctYaw", (OS_S16)(hilInput.qh / 0.01), OS_S16);
	SETDATA(pDataPoolFly, "viewPitc", (OS_S16)(hilInput.qf / 0.01), OS_S16);
	SETDATA(pDataPoolFly, "viewYaw", (OS_S16)(hilInput.qh / 0.01), OS_S16);
	SETDATA(pDataPoolFly, "vPitchSp", (OS_S16)(hilInput.dqf / 0.002), OS_S16);
	SETDATA(pDataPoolFly, "vYawSp", (OS_S16)(hilInput.dqh / 0.002), OS_S16);
}

static void HilWriteSimNavTelemetry(void)
{
	double dir = HilYawToNavDirTelemetry(g_ins_data.psi);
	double groundSpd = sqrt(g_ins_data.vtx * g_ins_data.vtx
		+ g_ins_data.vty * g_ins_data.vty
		+ g_ins_data.vtz * g_ins_data.vtz);

	SETDATA(pDataPoolImu, "imuAx", g_ins_data.ax, OS_FLOAT);
	SETDATA(pDataPoolImu, "imuAy", g_ins_data.ay, OS_FLOAT);
	SETDATA(pDataPoolImu, "imuAz", g_ins_data.az, OS_FLOAT);
	SETDATA(pDataPoolImu, "imuWx", g_ins_data.wx, OS_FLOAT);
	SETDATA(pDataPoolImu, "imuWy", g_ins_data.wy, OS_FLOAT);
	SETDATA(pDataPoolImu, "imuWz", g_ins_data.wz, OS_FLOAT);

	SETDATA(pDataPoolImu, "navPitch", (OS_S16)(g_ins_data.zeta * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "navRoll", (OS_S16)(g_ins_data.gama * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "navDir", (OS_U16)(dir * 100.0), OS_U16);

	SETDATA(pDataPoolImu, "navLon", (OS_S32)(g_ins_data.longitude * 1e7), OS_S32);
	SETDATA(pDataPoolImu, "navLat", (OS_S32)(g_ins_data.latitude * 1e7), OS_S32);
	SETDATA(pDataPoolImu, "navHigh", g_ins_data.height, OS_FLOAT);
	SETDATA(pDataPoolImu, "navVn", (OS_S16)(g_ins_data.vtx * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "navVs", (OS_S16)(g_ins_data.vty * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "navVe", (OS_S16)(g_ins_data.vtz * 100.0), OS_S16);

	SETDATA(pDataPoolImu, "gpsLon", (OS_S32)(g_ins_data.longitude * 1e7), OS_S32);
	SETDATA(pDataPoolImu, "gpsLat", (OS_S32)(g_ins_data.latitude * 1e7), OS_S32);
	SETDATA(pDataPoolImu, "gpsAlt", (OS_S16)g_ins_data.height, OS_S16);
	SETDATA(pDataPoolImu, "gpsVn", (OS_S16)(g_ins_data.vtx * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "gpsVs", (OS_S16)(g_ins_data.vty * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "gpsVe", (OS_S16)(g_ins_data.vtz * 100.0), OS_S16);
	SETDATA(pDataPoolImu, "gpsDir", (OS_U16)(dir * 100.0), OS_U16);

	if (flightSeq.luanched) {
		SETDATA(pDataPoolImu, "navState", (OS_U8)0x64, OS_U8);
	} else if (hilInput.nav_state != 0U) {
		SETDATA(pDataPoolImu, "navState", hilInput.nav_state, OS_U8);
	} else {
		SETDATA(pDataPoolImu, "navState", (OS_U8)0, OS_U8);
	}

	SETDATA(pDataPoolSelf, "GrdSpd", (OS_S16)(groundSpd * 10.0), OS_S16);
	HilWriteSimAirTelemetry();
	SETDATA(pDataPoolSelf, "ecuGetRp", (OS_U16)(hilInput.rpm_engine_state * 10.0), OS_U16);
	SETDATA(pDataPoolSelf, "ecuState", hilInput.engine_state, OS_U16);
	SETDATA(pDataPoolFly, "EngineRp", (OS_U16)(hilInput.rpm_engine_state * 10.0), OS_U16);

	//HilWriteSimSeekerTelemetry();
}

static void HilLoadNavFromDataPool(void)
{
	OS_FLOAT tempf;
	OS_S16 temps16;
	OS_U16 tempu16;

	GetDataFast(pDataPoolImu, "imuWx", &tempf);
	g_ins_data.wx = tempf;
	GetDataFast(pDataPoolImu, "imuWy", &tempf);
	g_ins_data.wy = tempf;
	GetDataFast(pDataPoolImu, "imuWz", &tempf);
	g_ins_data.wz = tempf;
	GetDataFast(pDataPoolImu, "imuAx", &tempf);
	g_ins_data.ax = tempf;
	GetDataFast(pDataPoolImu, "imuAy", &tempf);
	g_ins_data.ay = tempf;
	GetDataFast(pDataPoolImu, "imuAz", &tempf);
	g_ins_data.az = tempf;

	GetDataFast(pDataPoolImu, "navPitch", &temps16);
	g_ins_data.zeta = temps16 * 0.01;
	GetDataFast(pDataPoolImu, "navDir", &tempu16);
	g_ins_data.psi = -(tempu16 * 0.01);
	if (g_ins_data.psi > 180.0) {
		g_ins_data.psi -= 360.0;
	}
	if (g_ins_data.psi < -180.0) {
		g_ins_data.psi += 360.0;
	}
	GetDataFast(pDataPoolImu, "navRoll", &temps16);
	g_ins_data.gama = temps16 * 0.01;

	GetDataFast(pDataPoolImu, "navVn", &temps16);
	g_ins_data.vtx = temps16 * 0.01;
	GetDataFast(pDataPoolImu, "navVs", &temps16);
	g_ins_data.vty = temps16 * 0.01;
	GetDataFast(pDataPoolImu, "navVe", &temps16);
	g_ins_data.vtz = temps16 * 0.01;
}

OS_U8 HilFlightStage()
{
	/* 此序号对应本周期实际送入 ControlRun 的 HIL 样本。 */
	g_hilEchoSequence = g_hilPendingSequence;
	g_ins_data.wx = hilInput.wx;
	g_ins_data.wy = hilInput.wy;
	g_ins_data.wz = hilInput.wz;
	g_ins_data.ax = hilInput.ax;
	g_ins_data.ay = hilInput.ay;
	g_ins_data.az = hilInput.az;

	g_ins_data.zeta = hilInput.pitch;
	g_ins_data.psi  = hilInput.yaw;
	g_ins_data.gama = hilInput.roll;

	g_ins_data.vtx = hilInput.vn;
	g_ins_data.vty = hilInput.vs;
	g_ins_data.vtz = hilInput.ve;

	g_ins_data.longitude = hilInput.lon;
	g_ins_data.latitude = hilInput.lat;
	g_ins_data.height = hilInput.alt;
	g_ins_data.GPS_status = hilInput.nav_state;

	g_engine_data.rpm_engine = hilInput.rpm_engine_state;
	g_engine_data.ECU_work_status = hilInput.engine_state;

	g_baro_data.static_pressure = hilInput.DD1;
	g_baro_data.total_pressure = hilInput.DD2;

	g_seeker_data.pitch_LOS_rate = hilInput.dqf;
	g_seeker_data.yaw_LOS_rate = hilInput.dqh;
	g_seeker_data.pitch_LOS_angle = hilInput.qf;
	g_seeker_data.yaw_LOS_angle = hilInput.qh;
	g_seeker_data.flag_seize_stable = hilInput.seeker_state;
	g_seeker_data.flag_seize_stable = 0;

	if (hilInput.useNav) {
		HilLoadNavFromDataPool();
		HilWriteSimSeekerTelemetry();
		SETDATA(pDataPoolFly, "EngineRp", (OS_U16)(hilInput.rpm_engine_state * 10.0), OS_U16);
	} else {
		HilWriteSimNavTelemetry();
	}

	return 0;
}


void StopHilTest()
{
	g_DeviceState.workStage &= ~((unsigned int)DOM_AUTOMATIC);
	g_DeviceState.workStage &= ~((unsigned int)DOM_NAVON);
	g_DeviceState.workStage &= ~((unsigned int)DOM_HILSMODE);
	g_DeviceState.workStage |= DOM_INTERACTIVE;

	HilReinitControl();
}
