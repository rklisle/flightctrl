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
#include "../flight/os_flight_io.h"
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
//		SETDATA(pDataPoolImu, "imuWx", hilInfo->wx,	OS_FLOAT);
//		SETDATA(pDataPoolImu, "imuWy", hilInfo->wy,	OS_FLOAT);
//		SETDATA(pDataPoolImu, "imuWz", hilInfo->wz,	OS_FLOAT);
//		SETDATA(pDataPoolImu, "imuAx", hilInfo->ax,	OS_FLOAT);
//		SETDATA(pDataPoolImu, "imuAy", hilInfo->ay,	OS_FLOAT);
//		SETDATA(pDataPoolImu, "imuAz", hilInfo->az,	OS_FLOAT);

//		SETDATA(pDataPoolImu, "navLon", hilInfo->lon * 1e7,	OS_S32);
//		SETDATA(pDataPoolImu, "navLat", hilInfo->lat * 1e7,	OS_S32);
//		SETDATA(pDataPoolImu, "navHigh", hilInfo->alt,	    OS_FLOAT);
//		SETDATA(pDataPoolImu, "navVn", hilInfo->vn * 1e2,	OS_S16);
//		SETDATA(pDataPoolImu, "navVs", hilInfo->vs * 1e2,	OS_S16);
//		SETDATA(pDataPoolImu, "navVe", hilInfo->ve * 1e2,	OS_S16);

//		double dir = hilInfo->yaw;
//		dir = -dir;
//		if(dir < 0)
//				dir += 360;
//		SETDATA(pDataPoolImu, "navPitch", hilInfo->pitch * 1e2,	OS_S16);
//		SETDATA(pDataPoolImu, "navDir", dir * 1e2,	OS_U16);//
//		SETDATA(pDataPoolImu, "navRoll", hilInfo->roll * 1e2,	OS_S16);
		if(g_DeviceState.srvCountDown == 0)
		{
			// MML 半实物过来的数据就是100倍的，hilInfo->DD1这个应该就是100倍的角度值
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
	case CMD_MSN_NEWPT://仿真上注任务指令
		{
			double lon,lat,high,track,speed,arriveTime;
			memcpy(&lon, frame->au8Data + 1, 8);
			memcpy(&lat, frame->au8Data + 9, 8);
			memcpy(&high, frame->au8Data + 17, 8);
			memcpy(&track, frame->au8Data + 25, 8);
			memcpy(&speed, frame->au8Data + 33, 8);
			memcpy(&arriveTime, frame->au8Data + 41, 8);
			MISSION msn;
			msn.MsnCmdType = frame->au8Data[0];
			msn.targetLon = lon;
			msn.targetLat = lat;
			msn.targetHigh = high;
			msn.speed = speed;
			msn.outTrack = track;
			// UpdateMission(msn);	// MML 20260413 临时注销
		}
		break;
	case 0x20:	// MML: 仿真已经起飞了
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
/* *********************************************** MML 20260413*****************************************************

		pInput->Luanched = flightSeq.luanched;
		pInput->ax = hilInput.ax;
		pInput->ay = hilInput.ay;
		pInput->az = hilInput.az;
    
		pInput->wx = hilInput.wx ;
		pInput->wy = hilInput.wy;
		pInput->wz = hilInput.wz;

		pInput->navLon = hilInput.lon;
		pInput->navLat = hilInput.lat;
		pInput->navHigh = hilInput.alt;
		pInput->navVn = hilInput.vn;
		pInput->navVs = hilInput.vs;
		pInput->navVe = hilInput.ve;
    
    double V   = sqrt(pow(pInput->navVn,2) + pow(pInput->navVs,2) + pow(pInput->navVe,2));   // 地速计算
    //空速就是地速，高速飞机的空速没用
    pInput->airSpd = V;

		pInput->pitch = hilInput.pitch;
		pInput->yaw = hilInput.yaw;
		pInput->roll = hilInput.roll;
    
    
    
    pInput->DD1 = hilInput.DD1;
    pInput->DD2 = hilInput.DD2;
    
    if(g_DeviceState.srvCountDown > 0)
    {
        OS_S16 srv1,srv2;
        GetDataFast(pDataPoolSrv, "Sr1Read", &srv1);
        GetDataFast(pDataPoolSrv, "Sr2Read", &srv2);
        pInput->DD1 = srv1 * 0.01;
        pInput->DD2 = srv2 * 0.01;
    }
    

		SETDATA(pDataPoolImu, "imuAx", pInput->ax,	OS_FLOAT);//加速度
		SETDATA(pDataPoolImu, "imuAy", pInput->ay,	OS_FLOAT);
		SETDATA(pDataPoolImu, "imuAz", pInput->az,	OS_FLOAT);

   	if(hilInput.useNav)
		{
        OS_FLOAT tempf;
				GetDataFast(pDataPoolImu, "imuWx",	&tempf);pInput->wx = tempf;
				GetDataFast(pDataPoolImu, "imuWy",	&tempf);pInput->wy = tempf;
				GetDataFast(pDataPoolImu, "imuWz", 	&tempf);pInput->wz = tempf;
				OS_S16 temps16;
				OS_U16 tempu16;
				GetDataFast(pDataPoolImu, "navPitch", 	&temps16);//
				pInput->pitch = temps16 * 0.01;
				GetDataFast(pDataPoolImu, "navDir", 	&tempu16);
				pInput->yaw = -tempu16 * 0.01;
				GetDataFast(pDataPoolImu, "navRoll", 	&temps16);
				pInput->roll = temps16 * 0.01;
        if(pInput->yaw < -180)
            pInput->yaw += 360;

    }
    double groundSpd = sqrt(pow(pInput->navVn,2) + pow(pInput->navVs,2) + pow(pInput->navVe,2));
    SETDATA(pDataPoolSelf, "GrdSpd", groundSpd * 10, OS_S16);//
   
//			SETDATA(pDataPoolImu, "imuWx", pInput->wx,	OS_FLOAT);//角速度
//			SETDATA(pDataPoolImu, "imuWY", pInput->wy ,	OS_FLOAT);
//			SETDATA(pDataPoolImu, "imuWZ", pInput->wz,	OS_FLOAT);


		SETDATA(pDataPoolImu, "navLon", pInput->navLon * 1e7,		OS_S32);//惯组经度
		SETDATA(pDataPoolImu, "navLat", pInput->navLat * 1e7,		OS_S32);//惯组纬度
		SETDATA(pDataPoolImu, "navHigh", pInput->navHigh , OS_FLOAT);//惯组高

		SETDATA(pDataPoolImu, "navVn", pInput->navVn* 100,	OS_S16);//
		SETDATA(pDataPoolImu, "navVs", pInput->navVs* 100,	OS_S16);//
		SETDATA(pDataPoolImu, "navVe", pInput->navVe* 100,	OS_S16);//

//			SETDATA(pDataPoolImu, "navRoll", pInput->roll * 100,	OS_S16);//
//			SETDATA(pDataPoolImu, "navPitch", pInput->pitch * 100,	OS_S16);//
		if(pInput->Luanched)	
		{
			SETDATA(pDataPoolImu, "navState", 0x64, OS_U8);//导航状态
		}
		else
		{
			SETDATA(pDataPoolImu, "navState", 0, OS_U8);//导航状态
		}
		double dir = -pInput->yaw;
		if(dir < 0)
		{
			dir += 360;
		}
//		SETDATA(pDataPoolImu, "navDir", dir * 100,		OS_U16);

		SETDATA(pDataPoolSelf, "AirSpd", pInput->airSpd * 10, OS_S16);
		
		OS_S16 temps16;
		OS_U8 locked;
		GetDataFast(pDataPoolFly,	"sctLock",	 &(locked));
		GetDataFast(pDataPoolFly,	"vPitchSp",	 &temps16);
		pInput->scoutPitchSpd = (double)temps16 * 0.002 / 57.3;
		GetDataFast(pDataPoolFly,	"vYawSp",	 &temps16);
		pInput->scoutYawSpd = (double)temps16 * 0.002 / 57.3;
		pInput->scoutLocked = locked;
//		if(pInput->scoutLocked == 0 && locked == 1)
//		{
//			enableDelayTick--;
//			if(enableDelayTick == 0)
//			{
//				pInput->scoutLocked = 1;
//			}
//			else if(pInput->scoutLocked == 0)
//			{
//				enableDelayTick = 2;
//			}
//		}
//		pInput->scoutLocked = 1;
//		pInput->scoutPitchSpd = pitchspd / 57.3;
//		pInput->scoutYawSpd = yawspd /57.3;
*/
		return 0;
}

extern CMathControlMain *g_pControl;

void StopHilTest()
{
	g_DeviceState.workStage &= ~((unsigned int)DOM_AUTOMATIC);
	g_DeviceState.workStage &= ~((unsigned int)DOM_NAVON);
	g_DeviceState.workStage |= DOM_INTERACTIVE;
	// FlightInit();
ControlInitial(g_pControl);

	return;
}
