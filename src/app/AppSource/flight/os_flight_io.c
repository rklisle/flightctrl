#include "os_flight_io.h"
#include <math.h>
#include "Control.h"
#include "WP.h"
#include "PGuide.h"

FLIGHT_INPUT input = {0};
FLIGHT_OUTPUT output = {0};


FLIGHT_INPUT *pInput = &input;
FLIGHT_OUTPUT *pOutput = &output;

#define d2r		(57.29577951308402)
extern MISSION Flight_mission;
extern DubinsStruc Dubins_para; //Dubins结构体
void FlightInit()
{
	WP_InitRoutePoint();
	
}

void FlightRun()
{
	static float flight_time = 0;
	static int luanched = 0;
	if (luanched == 0)
	{
		PGuide_init(pInput);
		Control_init(pInput);
		luanched = 1;
	}
	SetPGuide(pInput);
	SetControlPara(pInput); //制导控制周期赋值
	FlyEngineControl(); //发动机先启动
	ESO_RollAutopilot(7 * 6.28, control_para.RollCompensate_ESO); //ESO观测器
	if (pInput->Luanched == 1)//启控
	{
		control_para.driv_time += control_para.con_fTimerStep; // 需处理
		PGuide();
		FlyStabilityControl();
	}
	pOutput->rudderPitchCmd = control_para.fduoz;//degree
	pOutput->rudderYawCmd = control_para.fduoy;//degree
	pOutput->rudderRollCmd = control_para.fduox;//degree

	pOutput->rudder1Cmd = control_para.g_fUdelta01;
	pOutput->rudder2Cmd = control_para.g_fUdelta02;
	pOutput->rudder3Cmd = control_para.g_fUdelta03;
	pOutput->rudder4Cmd = control_para.g_fUdelta04;
	pOutput->rudder5Cmd = control_para.g_fUdelta05;
	pOutput->rudder6Cmd = control_para.g_fUdelta06;

	pOutput->engineSet = control_para.EngineSet;
	pOutput->curPtNo = pGuide_para.ac_dot + 1; //从0开始
	pOutput->curTargetLon = pGuide_para.AB.lon;
	pOutput->curTargetLat = pGuide_para.AB.lat;
	pOutput->curTargetAlt = pGuide_para.AB.alt;

	pOutput->gamaCmd = control_para.gama_cmd_tran;//滚转角指令
	pOutput->nycCmd = control_para.ny_cmd*9.8;//过载指令
	pOutput->varthetaCmd = control_para.thetac;//俯仰角指令
	pOutput->heightCmd = control_para.height_cmd_tran;//高度指令
//	pOutput->Qv = control_para.qv;//动压计算
	pOutput->ac_dL = pGuide_para.ac_dL;//待飞距
	pOutput->ac_dZ = pGuide_para.ac_dZ;// 侧边距
	pOutput->token_long = pGuide_para.token_long;// 纵向令牌
	pOutput->token_late = pGuide_para.token_late;// 侧向令牌
	pOutput->nyCmd_Guidance = pGuide_para.ny_cmd * 9.8;//末制导纵向过载指令
	pOutput->nzCmd_Guidance = pGuide_para.nz_cmd * 9.8;//末制导侧向过载指令
	pOutput->pitch_rate_nT_filterOut = pGuide_para.pitch_rate_nT_filterOut[0] * 57.3;//俯仰角速度滤波
	pOutput->yaw_rate_nT_filterOut = pGuide_para.yaw_rate_nT_filterOut[0] * 57.3;//偏航角速度滤波
	pOutput->deltaR = pGuide_para.deltaR;//剩余弹目距离
	pOutput->dRn  = pGuide_para.Rnue[0];
	pOutput->dRu  = pGuide_para.Rnue[1];
	pOutput->dRe  = pGuide_para.Rnue[2];//剩余弹目北天东距离
	pOutput->Pitch_Preset_Angle = pGuide_para.Pitch_Preset_Angle;
	pOutput->Yaw_Preset_Angle = pGuide_para.Yaw_Preset_Angle; //理论框架角

	pOutput->ac_dPsi = pGuide_para.ac_dPsi;
	pOutput->ac_dR = pGuide_para.ac_dR;// 圆轨迹侧边距
	pOutput->cur_thetav = pGuide_para.cur_thetav;
	pOutput->Vcmd = control_para.Vcmd;
	pOutput->Dubins_stage = pGuide_para.Dubins_stage;
	pOutput->dubins_type1 = Dubins_para.dubins_type[0];
	pOutput->dubins_type2 = Dubins_para.dubins_type[1];
	pOutput->dubins_type3 = Dubins_para.dubins_type[2];
	pOutput->Dubins_length = Dubins_para.dubins_len;
	pOutput->fduox_ADRC = control_para.fduox_ADRC;//fduox_ADRC;
	pOutput->mx_ESO = control_para.mx_ESO;
	pOutput->on_takeoff = pGuide_para.on_takeoff; //起飞标志
	pOutput->open_umbrella = pGuide_para.tag_Open_Umbrella; //开伞标志
	pOutput->enginge_off = pGuide_para.tag_Eng_off; //动力停车标志
	
	pOutput->Min_IAS2Vel = pGuide_para.Min_IAS2Vel; //最低速度
	pOutput->arp_ins = pGuide_para.arf_ins;
	pOutput->beta_ins = pGuide_para.beta_ins;
	pOutput->MaxRpm = control_para.MaxRpmValue;//最大转速
	pOutput->DFT_freq_max = control_para.state_freq_max;//
	flight_time = flight_time+0.005;
	/*
	if ((flight_time > 20) && (flight_time < 20.01))
	{
		MISSION new_mission;
		new_mission.MsnCmdType = 1;
		new_mission.targetLon = 113.621106;
		new_mission.targetLat = 35.129102;
		new_mission.outTrack = 90;
		new_mission.speed = 190;
		new_mission.targetHigh = 1600;
		new_mission.radis = 0; //转弯半径
		Update_Mission(new_mission);
	}
	if ((flight_time > 118.48) && (flight_time < 118.49))
	{
		MISSION new_mission;
		new_mission.MsnCmdType = 1;
		new_mission.targetLon = 113.766867;
		new_mission.targetLat = 35.301569;
		new_mission.outTrack = 270;
		new_mission.speed = 190;
		new_mission.targetHigh = 1600;
		new_mission.radis = 3000; //转弯半径
		Update_Mission(new_mission);
	}
	if ((flight_time > 293.71) && (flight_time < 293.72))
	{
		MISSION new_mission;
		new_mission.MsnCmdType = 1;
		new_mission.targetLon = 114.011746;
		new_mission.targetLat = 35.008171;
		new_mission.outTrack = 270;
		new_mission.speed = 190;
		new_mission.targetHigh = 1600;
		new_mission.radis = 0; //转弯半径
		Update_Mission(new_mission);
	}*/
}

void UpdateMission(MISSION msn)
{
	//Update_Mission(msn);
}

void setFlightInput(FLIGHT_INPUT input1)
{
	memcpy(pInput, &input1, sizeof(FLIGHT_INPUT));
}

void getFlightOutput(FLIGHT_OUTPUT* output1) 
{
	memcpy(output1, pOutput, sizeof(FLIGHT_OUTPUT));
}

void updateRP(RoutePointIn* inrp, int num)
{
	RP_NUMBER = num;
	memset(rp, 0, sizeof(RoutePointIn) * RP_MAX_NUMBER);
	//memcpy(rp, inrp, sizeof(RoutePointIn) * num);
	int i;
	for(i = 0;i < num;i++)
	{
		rp[i].sn = inrp[i].sn;
		rp[i].lon = inrp[i].lon;
		rp[i].lat = inrp[i].lat;
		rp[i].h = inrp[i].h;
		rp[i].w = inrp[i].w;
		rp[i].t = inrp[i].t;
		rp[i].V_cmd = inrp[i].V_cmd;
		rp[i].outTrack = inrp[i].outTrack;
		rp[i].if_airspeed_used = inrp[i].if_airspeed_used;
		//rp[i].if_GuideFlight = inrp[i].if_GuideFlight;
		rp[i].hover_radis = 0.0;
		rp[i].AttackAngle = 0.0;
		if(rp[i].w == 3 ||rp[i].w == 4)
		{
			rp[i].AttackAngle = -50.0;
		}
		if(rp[i].w == 2)
		{
			rp[i].hover_radis = 0.0;
		}
	}
}

void updateNewRP(RoutePointIn* newinrp, int num, OS_U16 curPtNo, int pt)
{
	int i;
	if(pt > (num + curPtNo))
	{
		memset(rp + curPtNo + num, 0, sizeof(RoutePointIn) * (pt - num - curPtNo));
		//memcpy(rp + curPtNo, newinrp, sizeof(RoutePointIn) * num);
		for(i = curPtNo;i < (num + curPtNo) ;i++)
		{
			rp[i].sn = newinrp[i].sn;
			rp[i].lon = newinrp[i].lon;
			rp[i].lat = newinrp[i].lat;
			rp[i].h = newinrp[i].h;
			rp[i].w = newinrp[i].w;
			rp[i].t = newinrp[i].t;
			rp[i].V_cmd = newinrp[i].V_cmd;
			rp[i].outTrack = newinrp[i].outTrack;
			rp[i].if_airspeed_used = newinrp[i].if_airspeed_used;
			//rp[i].if_GuideFlight = newinrp[i].if_GuideFlight;
			rp[i].hover_radis = 0.0;
			rp[i].AttackAngle = 0.0;
			if(rp[i].w == 3 ||rp[i].w == 4)
			{
				rp[i].AttackAngle = 0.0;
			}
			if(rp[i].w == 2)
			{
				rp[i].hover_radis = 0.0;
			}
		}
	}
	else
	{		
		//memcpy(rp + curPtNo, newinrp, sizeof(RoutePointIn) * num);
		for(i = curPtNo;i < (num + curPtNo) ;i++)
		{
			rp[i].sn = newinrp[i].sn;
			rp[i].lon = newinrp[i].lon;
			rp[i].lat = newinrp[i].lat;
			rp[i].h = newinrp[i].h;
			rp[i].w = newinrp[i].w;
			rp[i].t = newinrp[i].t;
			rp[i].V_cmd = newinrp[i].V_cmd;
			rp[i].outTrack = newinrp[i].outTrack;
			rp[i].if_airspeed_used = newinrp[i].if_airspeed_used;
			//rp[i].if_GuideFlight = newinrp[i].if_GuideFlight;
			rp[i].hover_radis = 0.0;
			rp[i].AttackAngle = 0.0;
			if(rp[i].w == 3 ||rp[i].w == 4)
			{
				rp[i].AttackAngle = 0.0;
			}
			if(rp[i].w == 2)
			{
				rp[i].hover_radis = 0.0;
			}
		}
	}
  RP_NUMBER = curPtNo + 1;
	pInput->Msn_updatesig = 1;
}

void getDubinsParam(DubinsStrucIn* paramss)
{
	memcpy(paramss , &Dubins_para,  sizeof(DubinsStrucIn));
}
