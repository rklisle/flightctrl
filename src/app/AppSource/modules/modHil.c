/*
 * modHil.c
 *
 *  Created on: 2024年4月6日
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
#include <math.h>
#define d2r		(57.29577951308402)
STRU_HIL_INPUT hilInput;
float fwxhil,fwyhil,fwzhil;
OS_DOUBLE pitchspd, yawspd;
//OS_U16 enableDelayTick = 240;
void StopHilTest();
OS_U8 SaveHilInDataPool(STRU_HIL_INPUT *hilInfo)
{
		memcpy(&hilInput, hilInfo, sizeof(STRU_HIL_INPUT));
		SETDATA(pDataPoolImu, "imuWx", hilInfo->wx,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuWy", hilInfo->wy,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuWz", hilInfo->wz,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAx", hilInfo->ax,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAy", hilInfo->ay,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAz", hilInfo->az,	OS_FLOAT);

		SETDATA(pDataPoolImu, "navLon", hilInfo->lon * 1e7,	OS_S32);
		SETDATA(pDataPoolImu, "navLat", hilInfo->lat * 1e7,	OS_S32);
		SETDATA(pDataPoolImu, "navHigh", hilInfo->alt,	    OS_FLOAT);
		SETDATA(pDataPoolImu, "navVn", hilInfo->vn * 1e2,	OS_S16);
		SETDATA(pDataPoolImu, "navVs", hilInfo->vs * 1e2,	OS_S16);
		SETDATA(pDataPoolImu, "navVe", hilInfo->ve * 1e2,	OS_S16);

		double dir = hilInfo->yaw;
		dir = -dir;
		if(dir < 0)
				dir += 360;
		SETDATA(pDataPoolImu, "navPitch", hilInfo->pitch * 1e2,	OS_S16);
		SETDATA(pDataPoolImu, "navDir", dir * 1e2,	OS_U16);//
		SETDATA(pDataPoolImu, "navRoll", hilInfo->roll * 1e2,	OS_S16);
		if(g_DeviceState.srvCountDown == 0)
		{
			// 280 半实物过来的数据就是100倍的，hilInfo->DD1这个应该就是100倍的角度值
			SETDATA(pDataPoolSrv, "Sr1Read", hilInfo->DD1, OS_S16);	
			SETDATA(pDataPoolSrv, "Sr2Read", hilInfo->DD2, OS_S16);	
		}
    
	return 0;
}

OS_U32 HilRtHandler(STRU_422_MSG_INFO * frame)// RT_HIL
{
	g_DeviceState.hilCountDown = 200;
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case 0xAA:
	{//主控指令
		OS_U8 cmd = frame->au8Data[0];
		if(cmd == CMD_HOR_CALC_REQ || cmd == CMD_TO_NAV_REQ)
		{
			STRU_422_MSG_INFO frame1;
			frame1.u8MsgID = frame->au8Data[0];
			NavCmdHandler(&frame1);
		}
	}
		break;
	case 0xAB:
		//攻击目标
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
	case 0x22://紧急伞降
	case 0x23://紧急返航
		ControllerCmdHandler(frame);
		break;
	case CMD_MSN_NEWPT://仿真上注任务指令	// 280 该命令不用 注释掉
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
	case 0x20:	// : 仿真已经起飞了
		{
			STRU_HIL_INPUT hilInfo = {0};
			memcpy(&hilInfo, frame->au8Data, sizeof(hilInfo));
			SaveHilInDataPool(&hilInfo);
			if(hilInfo.runStop == 0)
			{
				StopHilTest();
				return 0;
			}
			if((g_DeviceState.workStage & DOM_AUTOMATIC) != DOM_AUTOMATIC)
			{
				g_DeviceState.workStage |= DOM_HILSMODE;
				if(flightSeq.luanched != 1)
				{
					Ignition();
					DoIgnition();                    
					MsgToNAV(BUS_NAV_IGNATION,PTR_NULL,0);
				}
			}
			SETDATA(pDataPoolMsn,	"autoStep",	13,	OS_U8);
		}
		break;
	case CMD_LAUNCH_REQ:	//0xFA 发射指令
		{

		}
			break;
	}
	return 0;
}

OS_U8 HilFlightStage()
{
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

		GetDataFast(pDataPoolImu, "navState", &g_ins_data.GPS_status);// pDataPoolNav


		GetDataFast(pDataPoolSelf, "ecuGetRp", &g_engine_data.rpm_engine);
		GetDataFast(pDataPoolSelf, "ecuState", &g_engine_data.ECU_work_status);

	// pInput->Luanched = flightSeq.luanched;	// 014不使用

	// double V   = sqrt(pow(g_ins_data.vtx,2) + pow(g_ins_data.vty,2) + pow(g_ins_data.vtz,2));   // 地速计算
    // //空速就是地速，高速飞机的空速没用
    // pInput->airSpd = V;

	// pInput->DD1 = hilInput.DD1; // 014不使用
    // pInput->DD2 = hilInput.DD2;
    
    // if(g_DeviceState.srvCountDown > 0)
    // {
    //     OS_S16 srv1,srv2;
    //     GetDataFast(pDataPoolSrv, "Sr1Read", &srv1);
    //     GetDataFast(pDataPoolSrv, "Sr2Read", &srv2);
    //     pInput->DD1 = srv1 * 0.01; // 014不使用
    //     pInput->DD2 = srv2 * 0.01;
    // }

		SETDATA(pDataPoolImu, "imuAx", g_ins_data.ax,	OS_FLOAT);//加速度
		SETDATA(pDataPoolImu, "imuAy", g_ins_data.ay,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAz", g_ins_data.az,	OS_FLOAT);

		SETDATA(pDataPoolSelf, "AirPress", hilInput.DD1*0.1, OS_S16);	// MML新增
	g_baro_data.static_pressure = hilInput.DD1;
	g_baro_data.total_pressure  = hilInput.DD2;

   	if(hilInput.useNav)
	{
        OS_FLOAT tempf;
				GetDataFast(pDataPoolImu, "imuWx",	&tempf);g_ins_data.wx = tempf;
				GetDataFast(pDataPoolImu, "imuWy",	&tempf);g_ins_data.wy = tempf;
				GetDataFast(pDataPoolImu, "imuWz", 	&tempf);g_ins_data.wz = tempf;
				OS_S16 temps16;
				OS_U16 tempu16;
				GetDataFast(pDataPoolImu, "navPitch", 	&temps16);//
				g_ins_data.zeta = temps16 * 0.01;
				GetDataFast(pDataPoolImu, "navDir", 	&tempu16);
				g_ins_data.psi = -tempu16 * 0.01;
				GetDataFast(pDataPoolImu, "navRoll", 	&temps16);
				g_ins_data.gama = temps16 * 0.01;
        if(g_ins_data.psi < -180)
            g_ins_data.psi += 360;

    }
    double groundSpd = sqrt(pow(g_ins_data.vtx,2) + pow(g_ins_data.vty,2) + pow(g_ins_data.vtz,2));
    SETDATA(pDataPoolSelf, "GrdSpd", groundSpd * 10, OS_S16);//
   
//			SETDATA(pDataPoolImu, "imuWx", g_ins_data.wx,	OS_FLOAT);//角速度
//			SETDATA(pDataPoolImu, "imuWY", g_ins_data.wy ,	OS_FLOAT);
//			SETDATA(pDataPoolImu, "imuWZ", g_ins_data.wz,	OS_FLOAT);


		SETDATA(pDataPoolImu, "navLon", g_ins_data.longitude * 1e7,		OS_S32);//惯组经度
		SETDATA(pDataPoolImu, "navLat", g_ins_data.latitude * 1e7,		OS_S32);//惯组纬度
		SETDATA(pDataPoolImu, "navHigh", g_ins_data.height , OS_FLOAT);//惯组高

		SETDATA(pDataPoolImu, "navVn", g_ins_data.vtx* 100,	OS_S16);//
		SETDATA(pDataPoolImu, "navVs", g_ins_data.vty* 100,	OS_S16);//
		SETDATA(pDataPoolImu, "navVe", g_ins_data.vtz* 100,	OS_S16);//

//			SETDATA(pDataPoolImu, "navRoll", g_ins_data.gama * 100,	OS_S16);//
//			SETDATA(pDataPoolImu, "navPitch", g_ins_data.zeta * 100,	OS_S16);//
//**************************  MML 20260417*****************************
		if(flightSeq.luanched)	
		{
			SETDATA(pDataPoolImu, "navState", 0x64, OS_U8);//导航状态
		}
		else
		{
			SETDATA(pDataPoolImu, "navState", 0, OS_U8);//导航状态
		}

		double dir = -g_ins_data.psi;
		if(dir < 0)
		{
			dir += 360;
		}
//		SETDATA(pDataPoolImu, "navDir", dir * 100,		OS_U16);

		// SETDATA(pDataPoolSelf, "AirSpd", pInput->airSpd * 10, OS_S16);
		
		// OS_S16 temps16;
		// OS_U8 locked;
		// GetDataFast(pDataPoolFly,	"sctLock",	 &(locked));
		// GetDataFast(pDataPoolFly,	"vPitchSp",	 &temps16);
		// pInput->scoutPitchSpd = (double)temps16 * 0.002 / 57.3;	// 014不使用
		// GetDataFast(pDataPoolFly,	"vYawSp",	 &temps16);
		// pInput->scoutYawSpd = (double)temps16 * 0.002 / 57.3;	// 014不使用
		// pInput->scoutLocked = locked;	// 014不使用
//		if(pInput->scoutLocked == 0 && locked == 1)	// 014不使用
//		{
//			enableDelayTick--;
//			if(enableDelayTick == 0)
//			{
//				pInput->scoutLocked = 1;	// 014不使用
//			}
//			else if(pInput->scoutLocked == 0)	// 014不使用
//			{
//				enableDelayTick = 2;
//			}
//		}
//		pInput->scoutLocked = 1;	// 014不使用
//		pInput->scoutPitchSpd = pitchspd / 57.3;	// 014不使用
//		pInput->scoutYawSpd = yawspd /57.3;	// 014不使用
		return 0;
}

extern void *g_pControl;

void StopHilTest()
{
	g_DeviceState.workStage &= ~((unsigned int)DOM_AUTOMATIC);
	g_DeviceState.workStage &= ~((unsigned int)DOM_NAVON);
	g_DeviceState.workStage |= DOM_INTERACTIVE;
	// FlightInit();
  g_pControl = ControlInitial();

	return;
}
