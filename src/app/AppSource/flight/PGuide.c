//#include <math.h>
#include "PGuide.h"
#include "Control.h"
#include "Ctrl_Law_Typedef.h"

#include "FCC_Lib.h"

#define BTT90
//#define BTT180

PGuidPara pGuide_para;
MISSION Flight_mission;
DubinsStruc Dubins_para; //Dubins结构体
RoutePoint rp[RP_MAX_NUMBER];
unsigned char Mission_Updated_sign;
extern float flight_len;
extern double Hover_lon;
extern double Hover_lat; //盘旋点坐标
//离轨时刻初始化
void PGuide_init(FLIGHT_INPUT *Flight_Input)
{
	memset(&pGuide_para, 0, sizeof(pGuide_para));
	memset(&Flight_mission, 0, sizeof(Flight_mission));
	memset(&Dubins_para, 0, sizeof(DubinsStruc));
	pGuide_para.con_fTimerStep = 0.005f;
	pGuide_para.tan_psi = 1.0f;
	pGuide_para.tag_Eng_off=0;
	pGuide_para.cmd_guid=PW_GuideNone;
	//	pGuide_para.tag_land=0;
	pGuide_para.tag_NextLineEnd=0;
	pGuide_para.gama_cmd=0;
	pGuide_para.Vm_cmd=180;
	pGuide_para.tag_RealVmCal = 0;
	pGuide_para.Circ_make=0;
	pGuide_para.nav_guid=PW_NavNone;
	pGuide_para.mode_long=PW_LongNone;
	pGuide_para.step_enter=0;
	pGuide_para.ac_dot=0;
	pGuide_para.token_late=TOKEN_LateNone;
	pGuide_para.step_WayLate=0;
	pGuide_para.token_long=TOKEN_LongNone;
	pGuide_para.Climbing_failed=0;
	pGuide_para.tag_Vm_control=0;
	pGuide_para.ax_cmd_pre=0.0f;
	pGuide_para.ax_cmd_cur=0.0f;
	pGuide_para.Vm_cmd_pre=0.0f;
	pGuide_para.Vm_cmd_cur=0.0f;
	pGuide_para.cur_horizVm=0;
	pGuide_para.tag_endwp=0;
	pGuide_para.cur_Vm=0;
	pGuide_para.height_cmd_max = 12000;
	/**************航迹zd*****************/
	pGuide_para.fzd=0;   //侧边距误差
	pGuide_para.fzdi=0;  //侧边距误差积分
	pGuide_para.fzd_last=0;
	pGuide_para.fzdi_last=0;
	pGuide_para.fdzd=0;
	pGuide_para.tag_ac_dL = 0;		
	
	rp[0].sn=0;
	rp[0].lon = Flight_Input->initLon;
	rp[0].lat = Flight_Input->initLat;
	rp[0].h = Flight_Input->initHigh;
	rp[0].w=0;
}

void SetPGuide(FLIGHT_INPUT *Flight_Input)
{
	//判断是否进入PW_NavWayEnter状态
	if (Flight_Input->Luanched == 1 && (control_para.driv_time>Takeoff_Time) && (fabs(Flight_Input->pitch - control_para.thetac) < 0.5) && pGuide_para.nav_guid == PW_NavNone)
	{
		if((fabs(Flight_Input->wx) < Omega_Limit) && (fabs(Flight_Input->wy) < Omega_Limit) && (fabs(Flight_Input->wz) < Omega_Limit))
		{
			pGuide_para.nav_guid = PW_NavWayEnter;
			pGuide_para.on_takeoff = 1;//起飞完成
		}
	}
	//PW_NavWayEnter备保时间
	if (Flight_Input->Luanched == 1 && (control_para.driv_time > Takeoff_Protect_Time) && pGuide_para.nav_guid == PW_NavNone)
	{
		pGuide_para.nav_guid = PW_NavWayEnter;
		pGuide_para.on_takeoff = 1;//起飞完成
	}
       // 外回路取数
    // 输入
	pGuide_para.cur_lon = Flight_Input->navLon;
	pGuide_para.cur_lat = Flight_Input->navLat;

	pGuide_para.cur_high = Flight_Input->navHigh;

	pGuide_para.cur_psi = WP_Psi2Psi360(atan2(Flight_Input->navVe,
                                              Flight_Input->navVn)*RADIAN_2_DEGREE); //计算实时航迹角

	pGuide_para.cur_Vm = sqrt(pow(Flight_Input->navVe, 2) 
                          + pow(Flight_Input->navVn, 2)
                          + pow(Flight_Input->navVs, 2) );	// 合速度
	// 20231107 补加水平和速度
	pGuide_para.cur_Vh = sqrt( pow(Flight_Input->navVe, 2)
							 + pow(Flight_Input->navVn, 2) );
	
	pGuide_para.cur_horizon_distance += (pGuide_para.cur_Vh*SCAN_FLY_PROCESS_PERIOD/1000.0f);

	// 20231121 补加总航程计算
	pGuide_para.cur_total_distance += (pGuide_para.cur_Vm*SCAN_FLY_PROCESS_PERIOD/1000.0f);
	
	pGuide_para.cur_VN = Flight_Input->navVn;	
	pGuide_para.cur_VU = Flight_Input->navVs;
	pGuide_para.cur_VE = Flight_Input->navVe;

	pGuide_para.cur_thetav = atan(pGuide_para.cur_VU/pGuide_para.cur_Vh)*RADIAN_2_DEGREE; //计算实时弹道倾角

	pGuide_para.QB_ENU = Flight_Input->pitch;
	pGuide_para.QH_ENU = Flight_Input->yaw; //输入为北偏西-180~180 PB20251018
	pGuide_para.QK_ENU = Flight_Input->roll;

	control_para.acc_xin[1] = control_para.acc_xin[0];
	control_para.acc_xin[0] = Flight_Input->ax;
	Tustin_FirstIO(0,1,0.02,1,control_para.acc_xin,control_para.acc_xout, pGuide_para.con_fTimerStep);
	control_para.acc_xout[1] = control_para.acc_xout[0];
	pGuide_para.nx_T = control_para.acc_xout[0]/9.8;

	control_para.acc_yin[1] = control_para.acc_yin[0];
	control_para.acc_yin[0] = Flight_Input->ay;
	Tustin_FirstIO(0,1,0.02,1,control_para.acc_yin,control_para.acc_yout, pGuide_para.con_fTimerStep);
	control_para.acc_yout[1] = control_para.acc_yout[0];
	pGuide_para.ny_T = control_para.acc_yout[0]/9.8;

	control_para.acc_zin[1] = control_para.acc_zin[0];
	control_para.acc_zin[0] = Flight_Input->az;
	Tustin_FirstIO(0,1,0.02,1,control_para.acc_zin,control_para.acc_zout, pGuide_para.con_fTimerStep);
	control_para.acc_zout[1] = control_para.acc_zout[0];
	pGuide_para.nz_T = control_para.acc_zout[0] / 9.8;

	pGuide_para.Terminal_Guidance = Flight_Input->scoutLocked; //20250617 末制导导引头锁定标志位输入
	pGuide_para.DYT_Wxyz[2] = Flight_Input->scoutPitchSpd;//20250617 导引头视线角速度输入
	pGuide_para.DYT_Wxyz[1] = Flight_Input->scoutYawSpd;//
	pGuide_para.DYT_Frame[2] = Flight_Input->scoutPitch;//20250617 导引头框架角输入
	pGuide_para.DYT_Frame[1] = Flight_Input->scoutYaw;//

	//----航点更新标志输入----//
	pGuide_para.tag_RpUpdata = Flight_Input->Msn_updatesig;
} 

void PGuide()
{
	GUID_Axis_Change();
	GUID_Enter();
	GUID_VcmdUpdate();      //  速度更新管理状态机
//	GUID_MISSION();//实时接收任务规划
	GUID_RouteUpdate();     //航路点更新管理状态机
	GUID_Way();             //航路跟踪状态机
	GUID_GlideAngleUpdate();//下滑角更新管理状态机
	GUID_LongSafe();        //  爬升安全管理状态机
}

void  GUID_LongMan (void)
{
    if (pGuide_para.cur_high < (pGuide_para.height_cmd - Height_Climb_Threshold))
    { /*[H-Height_Climb_Threshold]*/
        pGuide_para.step_WayLong = 0;
        pGuide_para.token_long=TOKEN_WayClimb;
    }
    else if (pGuide_para.cur_high>(pGuide_para.height_cmd + Height_Glide_Threshold))
    { /*[H+Height_Glide_Threshold]*/
        pGuide_para.step_WayLong = 0;
        pGuide_para.token_long=TOKEN_WayDive;
    }
    else
    {
        // if(pGuide_para.mode_long!=PW_LongLevel || fabs(pGuide_para.height_var-pGuide_para.height_cmd)>1){
		// 20231101 在-100 到 100 继续平飞，配合着GUID_VcmdUpdate继续切令牌，切速度指令
        pGuide_para.token_long=TOKEN_WayLevel;
		// }
    }
}
//=====速度指令更新管理状态机=====//
void GUID_VcmdUpdate(void)
{
	//------最低飞行表速折算速度计算----//
	pGuide_para.Min_IAS2Vel = sqrt(1.225f * PW_Min_IAS * PW_Min_IAS / control_para.ru);

	if((pGuide_para.token_longpre==TOKEN_LongNone)&& (pGuide_para.token_long==TOKEN_WayClimb)) //稳态切换爬升时刻
	{
		pGuide_para.Vm_cmd =pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	if ((pGuide_para.token_longpre == TOKEN_LongNone) && (pGuide_para.token_long == TOKEN_WayDive)) //稳态切换下滑时刻
	{
		if (pGuide_para.AB.Velocity_cmd < pGuide_para.Vm_cmd) {  //当预定速度指令<当前速度指令
			pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
		}
	}
	if ((pGuide_para.token_longpre == TOKEN_WayClimb) && (pGuide_para.token_long == TOKEN_WayDive)) //爬升切换下滑时刻
	{
		if (pGuide_para.AB.Velocity_cmd < pGuide_para.Vm_cmd) {  //当预定速度指令<当前速度指令
			pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
		}
	}
	if((pGuide_para.token_longpre==TOKEN_LongNone)&& (pGuide_para.token_long==TOKEN_WayLevel)) //稳态切换平飞时刻
	{
		pGuide_para.Vm_cmd =pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	if((pGuide_para.token_longpre==TOKEN_LongNone)&& (pGuide_para.token_long==TOKEN_LongNone)) //稳态切换稳态时刻
	{
		pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	if((pGuide_para.token_longpre== TOKEN_WayDive)&& (pGuide_para.token_long==TOKEN_LongNone)) //下滑切换稳态时刻
	{
		pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	if ((pGuide_para.token_longpre == TOKEN_WayDive) && (pGuide_para.token_long == TOKEN_WayClimb)) //下滑切换爬升时刻
	{
		pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	if((pGuide_para.token_longpre == TOKEN_WayClimb)&& (pGuide_para.token_long == TOKEN_LongNone)) //爬升切换稳态时刻
	{
		pGuide_para.Vm_cmd =pGuide_para.AB.Velocity_cmd;  //速度指令更新
	}
	pGuide_para.token_longpre = pGuide_para.token_long; //飞行模态记录

	//----返航开伞段终端速度指令----//
	if (pGuide_para.AB.vxd2 == RP_TW_RETURN && pGuide_para.tag_Vm_cmd_Return != 1)
	{
		pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd;  //速度指令更新
		//---回收开伞航段终端速度指令:取装订返航速度与最低开伞表速的最大值---//
		pGuide_para.Vm_cmd_ReturnEnd = ((pGuide_para.Vm_cmd > pGuide_para.Min_OpenUmIAS2Vel) ? pGuide_para.Vm_cmd : pGuide_para.Min_OpenUmIAS2Vel);
		//---回收开伞航段速度指令计算完成----//
		pGuide_para.tag_Vm_cmd_Return = 1;
	}
}

//=====下滑角指令更新管理状态机=====//
void GUID_GlideAngleUpdate(void)
{
	//if ((pGuide_para.token_longpre == TOKEN_LongNone) && (pGuide_para.token_long == TOKEN_WayDive)) //稳态切换下滑时刻
	if (pGuide_para.token_long == TOKEN_WayDive) //切换下滑时刻
	{
	//	pGuide_para.Route_Glide_Angle = pGuide_para.Route_Ele_Angle; //航段下滑角指令更新;  
		pGuide_para.Route_Glide_Angle = pGuide_para.Route_Ele_Angle_GNC; //航段制导下滑角指令更新;  
	}
}

//=====航路点更新管理状态机=====//
void GUID_RouteUpdate(void)
{
	if (pGuide_para.tag_RpUpdata == 1) //接收到航路点更新标志
	{
		pGuide_para.tag_RouteSwitch = 1;

		pGuide_para.tag_RpUpdata = 0; //更新标志清0
		//-----航点即时更新时相应标志位处理---PB20251120----//
		pGuide_para.tag_endwp = 0; //航点更新后将：最后航点标志位清0
		pGuide_para.tag_NextLineEnd = 0; //航点更新后将：最后航段标志位清0
	}
}
//=====空速控制管理====//
void GUID_Airspeed_Ctrl(unsigned char tag)
{
	pGuide_para.tag_Airspeed_ctrl = tag;  //空速控制指令传递
}

void  GUID_Spec (int vxd, unsigned char vxd_guide)
{
    int   SP1;	
    SP1 = vxd;

	unsigned char SP_guide;
	SP_guide = vxd_guide;
    
    if (SP1== RP_TW_HOVER)
    { /*[盘旋飞行]*/
		pGuide_para.hover_radis = pGuide_para.AB.hover_radis;
		pGuide_para.token_late = TOKEN_TrackHover;
		pGuide_para.step_WayLate = 0;
		//----从航段信息中取出盘旋圈数----//
		pGuide_para.hover_round = pGuide_para.AB.Hover_Round;

		pGuide_para.hover_flydis = 0; //盘旋距离置0计算
    }
	else if(SP1 == RP_TW_GUIDANCE || SP1 == RP_TW_FEINT) { /*[末端打击]*/
		pGuide_para.mode_guid = PW_GuideDim4;

		pGuide_para.tar_lon = pGuide_para.AB.lon;
		pGuide_para.tar_lat = pGuide_para.AB.lat;
		pGuide_para.tar_alt = pGuide_para.AB.alt;

		pGuide_para.token_late = TOKEN_LateGuidance;
		pGuide_para.step_WayLate = 0;
		pGuide_para.token_long = TOKEN_LongGuidance; //切换末制导令牌
		pGuide_para.step_WayLong = 0;

		//---攻击角度约束---//
		pGuide_para.AttackAngle = -pGuide_para.AB.AttackAngle;//取反
		//---BTT-STT切换标志---//
		pGuide_para.BTT_STT_Switch = 0;
	}
	else{
		if(SP_guide == RP_TW_DUBINS)
		{
			pGuide_para.mode_guid = PW_GuideDim5; //mode切换

			memset(&flight_len, 0, sizeof(flight_len)); //每一次开始dubins航段长度清零
			Flight_mission.radis = pGuide_para.cur_Vm * pGuide_para.cur_Vm / 9.8f / 0.7f; //35度滚转角计算转弯半径
			Flight_mission.targetLon = pGuide_para.AB.lon;
			Flight_mission.targetLat = pGuide_para.AB.lat;
			Flight_mission.targetHigh = pGuide_para.AB.alt;
			Flight_mission.outTrack = pGuide_para.AB.outTrack;

			/*----杜宾斯圆计算----*/
			float cur_psi1_NtoW = 360 - pGuide_para.cur_psi; //北偏西角度
			float cur_psi2_NtoW = 360 - Flight_mission.outTrack; //北偏西角度
			Dubins_para = WP_DubinsCal(pGuide_para.cur_lon, pGuide_para.cur_lat, cur_psi1_NtoW, Flight_mission.targetLon, Flight_mission.targetLat, cur_psi2_NtoW, Flight_mission.radis);
			/*----根据杜宾斯两个切点计算D1D2航段---*/
			RoutePoint pointD1, pointD2;
			memset(&pointD1, 0, sizeof(pointD1));
			memset(&pointD2, 0, sizeof(pointD2));
			pointD1.lon = Dubins_para.lon_D1;
			pointD1.lat = Dubins_para.lat_D1;  //切点1赋值给A
			pointD2.lon = Dubins_para.lon_D2;
			pointD2.lat = Dubins_para.lat_D2;  //切点2赋值给B
			WP_GetLine2Point(pointD1, pointD2, &pGuide_para.D1D2);

			/*----定义预设航点----*/
			RoutePoint pointA, pointB;
			memcpy(&pointA, &rp[pGuide_para.ac_dot + 1], sizeof(RoutePoint));
			memcpy(&pointB, &rp[pGuide_para.ac_dot + 1], sizeof(RoutePoint));//需保证飞完DUBINS后的预设点特征和DUBINS指点一致		
			/*----根据杜宾斯切出点计算切出后航段---*/
			float MakeCD_len = 1000;
			WP_DubinsMakeCD(Flight_mission.targetLon, Flight_mission.targetLat, Dubins_para.outTrack, MakeCD_len, &Dubins_para.lon_preset, &Dubins_para.lat_preset); //航段长度可调
			pointA.lon = Flight_mission.targetLon;
			pointA.lat = Flight_mission.targetLat;
			pointB.lon = Dubins_para.lon_preset;
			pointB.lat = Dubins_para.lat_preset;
			WP_GetLine2Point(pointA, pointB, &pGuide_para.AB_preset);
			
			/*----航段高低角计算----*/
			pGuide_para.Route_Ele_Angle = atand((pGuide_para.AB.alt- pGuide_para.cur_high) / Dubins_para.dubins_len); //根据无人机当前点与B点高度差和航段总长度计算
			pGuide_para.ac_Dis2Fly = Dubins_para.dubins_len;  //切换时刻点的待飞距

			pGuide_para.token_late = TOKEN_TrackDubins;
			pGuide_para.step_WayLate = 0;		
		}
		else
		{	/*[预装航迹飞行]*/
			pGuide_para.mode_guid = PW_GuideDim2; //L1        
			if (pGuide_para.Circ_make)
			{
				pGuide_para.token_late = TOKEN_TrackCirc;
				pGuide_para.step_WayLate = 0;
			}
			WP_LateWay(&pGuide_para.AB, pGuide_para.cur_lon, pGuide_para.cur_lat, pGuide_para.cur_psi, &pGuide_para.ac_dZ, &pGuide_para.ac_dL, &pGuide_para.ac_dPsi);
			/*----航段高低角计算----*/
			pGuide_para.Route_Ele_Angle = atand((pGuide_para.AB.alt - pGuide_para.cur_high) / pGuide_para.AB.len); //根据无人机当前点与B点高度差和航段总长度计算
			pGuide_para.ac_Dis2Fly = pGuide_para.ac_dL;    //切换时刻点的待飞距
		}

		pGuide_para.flightTgo_ini=(int)(pGuide_para.AB.FlightTime-pGuide_para.AB.ac_FlightTime);//航段切换时计算一次

		/*若下一高度点低于此刻高度,边转弯边下滑*/
		pGuide_para.height_var=pGuide_para.height_cmd; //上一阶段高度指令
		pGuide_para.Height_cmd_comp=(pGuide_para.AB.alt>pGuide_para.height_var)?0:1;
		
		if (pGuide_para.Fient_sign == 0)  //非佯攻点，正常赋值高度指令
		{
			if (pGuide_para.Height_cmd_comp)
			{
				pGuide_para.height_cmd = pGuide_para.AB.alt;
				GUID_LongMan();
			}
			else //下一高度点高于此刻高度
			{
				if (pGuide_para.Climbing_failed == 1)        //上一航段爬升失败，判断下一航段高度指令，若新指令<限制指令，赋值新指令，否则保持指令不变
					pGuide_para.height_cmd = (pGuide_para.AB.alt < pGuide_para.height_cmd_max) ? pGuide_para.AB.alt : pGuide_para.height_cmd_max;
				else
					pGuide_para.height_cmd = pGuide_para.AB.alt;

				GUID_LongMan(); //高度指令更新	
			}
		}
		else if (pGuide_para.Fient_sign == 1)//佯攻点状态1，高度指令增加PW_dHeightFeint
		{
			//----高度指令更新为B点+dh高度-----//
			pGuide_para.height_cmd = pGuide_para.AB.alt + PW_dHeightFeint;
			GUID_LongMan(); //调用纵向状态机
			pGuide_para.Fient_sign = 2;  //状态机标志位更新2
		}
		else
		{
		};

		switch (SP1) {
		case RP_TW_LAND:/**----判断是否为着陆航段---**/
			pGuide_para.tag_ac_dL = 1;
			pGuide_para.token_long = TOKEN_WayLevel;// 更新定高令牌
			pGuide_para.height_cmd = pGuide_para.height_var; //最后保持第一拍高度指令
			pGuide_para.ac_dL_LandStart = pGuide_para.ac_dL;//记录着陆段起始待飞距
			break;
		case RP_TW_RETURN:/**----判断是否为返航回收开伞航段---**/
			pGuide_para.ru_Return = 1.225f * pow((1 - pGuide_para.height_cmd / 44332.3), 4.2559); //回收点密度计算
			//------最低开伞表速折算速度计算----//
			pGuide_para.Min_OpenUmIAS2Vel = sqrt(1.225f * PW_OpenUm_IAS * PW_OpenUm_IAS / pGuide_para.ru_Return);
			//---回收开伞航段初始速度指令---//
			pGuide_para.Vm_cmd_ReturnStart = pGuide_para.Vm_cmd;
			//记录返航段起始待飞距
			pGuide_para.ac_dL_ReturnStart = pGuide_para.ac_Dis2Fly;
			break;
		default:
			break;
		}
    }
}

void  GUID_Way (void)
{
    double  psi_tmp, radius, len2turn; 
    LineStruc  next;
	
	if (pGuide_para.nav_guid==PW_NavWay) {
		pGuide_para.AB.ac_FlightTime+= pGuide_para.con_fTimerStep; //当前航段实际飞行时间累加
		pGuide_para.cur_horizVm=sqrt(pGuide_para.cur_Vm*pGuide_para.cur_Vm-pGuide_para.cur_VU*pGuide_para.cur_VU);//计算当前水平速度

		WP_LateWay(&pGuide_para.AB, pGuide_para.cur_lon,pGuide_para.cur_lat,pGuide_para.cur_psi, &pGuide_para.ac_dZ,&pGuide_para.ac_dL,&pGuide_para.ac_dPsi);
		
		if (pGuide_para.cur_high > Low_Attitude_Threshold)
			radius = pGuide_para.cur_Vm*pGuide_para.cur_Vm/9.8f/0.7f;         /*[0.577=tan(30)]/[0.7=tan(35)]/[0.466=tan(25)]/[0.346=tan(20)]/[1.2=tan(50)]/[0.84=tan(40)]*///转弯半径
		else //低空
			radius = pGuide_para.cur_Vm * pGuide_para.cur_Vm / 9.8f / 0.466f;

		len2turn = radius*pGuide_para.tan_psi;   //提前转弯量
		
		pGuide_para.ac_flightTgo = WP_LateTgoCal(pGuide_para.ac_dL,pGuide_para.cur_horizVm,pGuide_para.ac_dPsi); //实际待飞时间计算
		pGuide_para.flightTgo = (int)(pGuide_para.AB.FlightTime-pGuide_para.AB.ac_FlightTime); //50;//指令待飞时间计算

		switch (pGuide_para.token_late) {
		case TOKEN_TrackWay:  //直线轨迹跟踪滚转角指令生成
			Guide_Line(pGuide_para.cur_Vm,pGuide_para.ac_dZ,pGuide_para.ac_dPsi);
			// 20231118 实时记录转弯半径
			pGuide_para.turn_radius = radius;
			//----航线导引航路切换时机判断----//
			if (pGuide_para.ac_dL < (len2turn + 100.0f)) {  //提前距离量
				pGuide_para.tag_RouteSwitch = 1;
			}
			break;
		case TOKEN_TrackCirc:  //圆形轨迹跟踪滚转角指令生成
			// 20231118 进入圆轨迹后保持转弯半径不变
			// WP_LateCirc(pGuide_para.Circ_lon,pGuide_para.Circ_lat,radius,pGuide_para.Circ_psi,pGuide_para.Route_psi,pGuide_para.cur_lon,pGuide_para.cur_lat,pGuide_para.cur_psi,&pGuide_para.ac_dR,&pGuide_para.ac_L1,&pGuide_para.ac_Circ_dPsi);
			// Guide_L1(pGuide_para.cur_Vm,pGuide_para.ac_L1,pGuide_para.ac_dR,pGuide_para.ac_Circ_dPsi,radius);
			WP_LateCirc(pGuide_para.Circ_lon,pGuide_para.Circ_lat,pGuide_para.turn_radius,pGuide_para.Circ_psi,pGuide_para.Route_psi,pGuide_para.cur_lon,pGuide_para.cur_lat,pGuide_para.cur_psi,&pGuide_para.ac_dR,&pGuide_para.ac_L1,&pGuide_para.ac_Circ_dPsi);
			Guide_L1(pGuide_para.cur_Vm,pGuide_para.ac_L1,pGuide_para.ac_dR,pGuide_para.ac_Circ_dPsi,pGuide_para.turn_radius);
			break;
		case TOKEN_TrackHover: //盘旋跟踪滚转角指令生成
			WP_LateCirc(Hover_lon, Hover_lat, pGuide_para.hover_radis, 0.0, 0.0, pGuide_para.cur_lon, pGuide_para.cur_lat, pGuide_para.cur_psi, &pGuide_para.ac_dR, &pGuide_para.ac_L1, &pGuide_para.ac_Circ_dPsi);
			Guide_Circ(pGuide_para.cur_Vm, pGuide_para.ac_dR, pGuide_para.ac_Circ_dPsi, pGuide_para.hover_radis);
			//---判断盘旋距离是否满足既定圈数---//
			pGuide_para.hover_flydis += pGuide_para.cur_horizVm * pGuide_para.con_fTimerStep;//已盘旋距离
			//----盘旋导引航路切换时机判断----//
			if (pGuide_para.hover_flydis > (2 * 3.1415926 * pGuide_para.hover_radis * pGuide_para.hover_round)) { //盘旋距离超过既定圈数
				pGuide_para.tag_RouteSwitch = 1;
			}
			break;
		case TOKEN_TrackDubins: //Dubins跟踪
			WP_LateDubins(&pGuide_para.D1D2, &pGuide_para.AB_preset, pGuide_para.cur_lon, pGuide_para.cur_lat, pGuide_para.cur_psi, Flight_mission.radis, Dubins_para, pGuide_para.cur_horizVm, &pGuide_para.ac_dZ, &pGuide_para.ac_dL, &pGuide_para.ac_dPsi, &pGuide_para.Dubins_stage);
			switch (pGuide_para.Dubins_stage) {
			case 1:
				Guide_Circ(pGuide_para.cur_Vm, pGuide_para.ac_dZ, pGuide_para.ac_dPsi, Flight_mission.radis);
				break;
			case 2:
				if (strcmp(Dubins_para.dubins_type, "RLR") == 0 || strcmp(Dubins_para.dubins_type, "LRL") == 0) //CCC
				{
					Guide_Circ(pGuide_para.cur_Vm, pGuide_para.ac_dZ, pGuide_para.ac_dPsi, Flight_mission.radis);
				}
				else //CSC
				{
					Guide_Line(pGuide_para.cur_Vm, pGuide_para.ac_dZ, pGuide_para.ac_dPsi);
				}
				break;
			case 3:
				Guide_Circ(pGuide_para.cur_Vm, pGuide_para.ac_dZ, pGuide_para.ac_dPsi, Flight_mission.radis);
				break;
			case 4: //未收到新点，沿预设点直线飞
				Guide_Line(pGuide_para.cur_Vm, pGuide_para.ac_dZ, pGuide_para.ac_dPsi);
				//----指点导引航路切换时机判断----//
				//----如果当前为预盘旋状态，不进行航路切换而是进入正式盘旋----//
				if (pGuide_para.AB.vxd_PreHover == 1) //如果为预盘旋状态
				{
					pGuide_para.AB.vxd2 = RP_TW_HOVER;//进入正式盘旋
					GUID_Spec(pGuide_para.AB.vxd2, pGuide_para.AB.vxd_guide);  /*[处理任务特征]*/
					pGuide_para.AB.vxd_PreHover = 0; //预盘旋结束
				}
				else
				{
					pGuide_para.tag_RouteSwitch = 1;
				}
				break;
			default:
				break;
			}
			break;
		case TOKEN_LateGuidance: //末端打击
			GUID_WxyzCal(); //视线角速度计算

			GUID_GuidanceLaw(pGuide_para.Law_Cmd);
			//BTT制导指令
			pGuide_para.gama_cmd = pGuide_para.Law_Cmd[0];
			pGuide_para.ny_cmd = pGuide_para.Law_Cmd[1];
			pGuide_para.nz_cmd = pGuide_para.Law_Cmd[2];

			/**----佯攻触发判断---**/
			if (pGuide_para.AB.vxd2 == RP_TW_FEINT)
			{
				if ((pGuide_para.cur_high - pGuide_para.AB.alt) < PW_HeightFeint)//相对高度差低于PW_HeightFeint时触发佯攻
				{
					pGuide_para.token_late = TOKEN_TrackWay; //赋值横向令牌
					//----佯攻标志设置----//
					pGuide_para.Fient_sign = 1;
					//----侧向过载指令置0----//
					pGuide_para.nz_cmd = 0;  //控0侧滑
				}
			}
			break;
		default:
			break;
		}
		/*----飞行制导高低角在线计算----*/
		pGuide_para.Route_Ele_Angle_GNC = atand((pGuide_para.AB.alt - pGuide_para.cur_high) / pGuide_para.ac_dL); //根据目标高度与D点高度差和航段长度计算
	    
		/**----着陆航段状态机---**/
		if(((pGuide_para.AB.vxd2==RP_TW_LAND) )&&(pGuide_para.tag_ac_dL == 1) ) 
		{
			pGuide_para.height_cmd = pGuide_para.height_var - (pGuide_para.ac_dL_LandStart - pGuide_para.ac_dL)/PW_Gilde_ratio; //高度指令按照预定滑翔比更新 

			if(((pGuide_para.cur_high-pGuide_para.AB.alt)<PW_HeightLand)&&(!pGuide_para.tag_land)) //着陆高度达到判断高度时时切换为着陆令牌
			{
				pGuide_para.token_long=TOKEN_WayLand;
				pGuide_para.tag_Eng_off = 1;
				pGuide_para.tag_land  = 1;
			}
		}
		/**----返场回收航段状态机---**/
		if (pGuide_para.AB.vxd2 == RP_TW_RETURN)
		{
			//---速度指令过渡计算----// 
			pGuide_para.Vm_cmd = pGuide_para.Vm_cmd_ReturnStart + (pGuide_para.ac_dL_ReturnStart - pGuide_para.ac_dL) * (pGuide_para.Vm_cmd_ReturnEnd - pGuide_para.Vm_cmd_ReturnStart) / pGuide_para.ac_dL_ReturnStart; //高度指令按照预定滑翔比更新 
			//---速度在初始速度与终端速度限幅----//
			if (pGuide_para.Vm_cmd_ReturnStart > pGuide_para.Vm_cmd_ReturnEnd)
				pGuide_para.Vm_cmd = Limit_In(pGuide_para.Vm_cmd_ReturnEnd, pGuide_para.Vm_cmd_ReturnStart, pGuide_para.Vm_cmd);
			else
				pGuide_para.Vm_cmd = Limit_In(pGuide_para.Vm_cmd_ReturnStart, pGuide_para.Vm_cmd_ReturnEnd, pGuide_para.Vm_cmd);

			//-----动力停车+开伞条件-----//
			if (pGuide_para.ac_dL <= PW_EngOffLen) //动力停车条件：第一步满足待飞距小于动力停车回收圈
			{
				pGuide_para.tag_Eng_off = 1; //发出停车指令
				//开伞判断条件1:在动力停车回收圈内
				if (pGuide_para.ac_dL >= -PW_EngOffLen)
				{
					if (pGuide_para.cur_Vm <= pGuide_para.Min_OpenUmIAS2Vel) {  //分支1：速度<表速90
						pGuide_para.tag_Open_Umbrella = 1;
					}
					else {
						if (pGuide_para.cur_Vm <= pGuide_para.Min_OpenUmIAS2Vel + 3) {//分支2：速度<表速93且高度差±30m
							if (fabs(pGuide_para.cur_high - pGuide_para.height_cmd) < 30)
								pGuide_para.tag_Open_Umbrella = 1;
						}
					}
				}
				else //开伞判断条件2(备保):在动力停车回收圈外
				{
					if (pGuide_para.cur_Vm <= pGuide_para.Min_OpenUmIAS2Vel + 3) //分支1：速度<表速93
						pGuide_para.tag_Open_Umbrella = 1;
					if ((pGuide_para.cur_high - pGuide_para.height_cmd) < -50)  //分支2：高度差-50m
						pGuide_para.tag_Open_Umbrella = 1;
					if (pGuide_para.ac_dL < -(PW_EngOffLen + 1000))  //分支3：出动力停车回收圈1000m
						pGuide_para.tag_Open_Umbrella = 1;
				}
			}
		}
		/**----佯攻点状态机----**/
		if (pGuide_para.Fient_sign == 2) //满足佯攻条件后，且高度指令已完成赋值，开始判断
		{
			//满足条件1:到达稳定高度;或条件2:姿态(爬升率>10m/s)达到稳定条件
			if (pGuide_para.token_long == TOKEN_LongNone || pGuide_para.cur_VU > PW_Dive_Pull_VU)  
			{					
				//----高度指令更新为B点高度-----//
				pGuide_para.height_cmd = pGuide_para.AB.alt;
				GUID_LongMan(); //调用纵向状态机

				//----佯攻标志设置----//
				pGuide_para.Fient_sign = 0;
			}
		}
		/**----  航路点更新状态机模块----**/
		if ((!pGuide_para.tag_endwp)&&(!pGuide_para.tag_NextLineEnd)) { //考虑航点即时更新，屏蔽该判断条件 
			if (pGuide_para.tag_RouteSwitch == 1) //满足航路切换条件
			{
				pGuide_para.tag_RouteSwitch = 0; //切换标志置0,重新判断
				if(fabs(pGuide_para.Route_psi)>=2.5f){  
					pGuide_para.Circ_make=WP_MakeCirc(pGuide_para.AB.lon,pGuide_para.AB.lat,pGuide_para.Route_psi,pGuide_para.AB.psi,len2turn,&pGuide_para.Circ_lon,&pGuide_para.Circ_lat,&pGuide_para.Circ_psi);
				}         
					pGuide_para.ac_dot+=1; 
					pGuide_para.tag_RealVmCal = 0; //航段切换，实时速度解算关闭

					if (WP_GetLine(pGuide_para.ac_dot, &pGuide_para.AB)) {

						GUID_Spec(pGuide_para.AB.vxd2, pGuide_para.AB.vxd_guide);  /*[处理任务特征]*/
						GUID_Airspeed_Ctrl(pGuide_para.AB.if_airspeed_used); /*[空速控制管理]*/

						if (WP_GetLine(pGuide_para.ac_dot+1, &next)) {//取完下第一个点立刻取下第二个点,若第一个为指点，继续取点无影响
							psi_tmp = WP_Psi2Psi(next.psi-pGuide_para.AB.psi);
							pGuide_para.Route_doublePsi=psi_tmp;
							if (psi_tmp<0.0f) psi_tmp=-psi_tmp;
							if (psi_tmp<150.0f) { 
								pGuide_para.tan_psi=sin(psi_tmp/RADIAN_2_DEGREE*0.5f)/cos(psi_tmp/RADIAN_2_DEGREE*0.5f);
							}
							else pGuide_para.tan_psi=3.73f;
						}
						else //未取到下第二个点
						{
							pGuide_para.tan_psi=0.0f;    	
							pGuide_para.tag_NextLineEnd = 1; //下一航段为最终航段
						}
					}
					else pGuide_para.tag_endwp=1; //未取到下第一个点
			}
		}

		if (pGuide_para.mode_guid==PW_GuideDim2) { /*[航段转换时监控航向角]*/
			psi_tmp = WP_Psi2Psi(pGuide_para.AB.psi-pGuide_para.cur_psi);
			if ((fabs(psi_tmp)<PW_dPsiThreshold) || (fabs(pGuide_para.ac_dZ)<PW_dZThreshold)) {
				pGuide_para.Circ_make = 0; 
				
				//轉彎完成后更新1/2航段夾角
				pGuide_para.Route_psi=0.5f*pGuide_para.Route_doublePsi;

				/***初始速度指令计算***/
//				pGuide_para.Vm_cmd = Guid_VcmdInit(pGuide_para.AB.len,pGuide_para.flightTgo_ini,pGuide_para.ac_dPsi); //初始速度指令计算	

				pGuide_para.tag_RealVmCal = 1; //实时速度解算打开
				
				pGuide_para.mode_guid=PW_GuideDim3;
				if (pGuide_para.cmd_guid==PW_GuideDim3) pGuide_para.mode_guid=PW_GuideDim3;
				if (pGuide_para.cmd_guid==PW_GuideDim2) pGuide_para.mode_guid=PW_GuideDim2;
				if (pGuide_para.mode_guid==PW_GuideDim3) { 
				}			
				pGuide_para.token_late=TOKEN_TrackWay;
				GUID_Inte_Zero(&pGuide_para.fzd,&pGuide_para.fzd_last,&pGuide_para.fzdi,&pGuide_para.fzdi_last); //侧边距控制项清零
				pGuide_para.step_WayLate=0;
			}
		} 
	}
}


/*----纵向安全模块----*/
void  GUID_LongSafe(void)
{
//	if(pGuide_para.token_long==TOKEN_WayClimb)  //当前状态为爬升
	if(pGuide_para.mode_long==PW_LongClimb)  //当前状态为爬升
	{
		if((pGuide_para.cur_high>PW_HeightThreshold)&&(pGuide_para.cur_VU<PW_ViClimb)) //当前高度大于门限，当前速度低于最低爬升速度
		{
			pGuide_para.step_Climbing_failed++;   //爬升失败次数累计
		}
		
		if(pGuide_para.step_Climbing_failed>=PW_StepThreshold)  //爬升失败累计时间达到3s
		{
			pGuide_para.Climbing_failed = 1;                   //判断爬升失败
            pGuide_para.height_cmd = pGuide_para.cur_high - PW_HeightDecline;
			pGuide_para.height_cmd_max = pGuide_para.height_cmd;
			GUID_LongMan(); //高度指令更新
		}			
	}
	else
	{
		pGuide_para.step_Climbing_failed = 0;
	}
}

void  GUID_Enter(void)
{
//    double  radius;
	LineStruc next;
	double  psi_tmp;



	if(pGuide_para.nav_guid==PW_NavWayEnter){
		switch (pGuide_para.step_enter) {
		case 0:
			if (WP_GetLine(pGuide_para.ac_dot, &pGuide_para.AB)) 
			{
				pGuide_para.AB.ac_FlightTime=control_para.driv_time;
				if (WP_GetLine(pGuide_para.ac_dot+1, &next)) 
				{
					psi_tmp = WP_Psi2Psi(next.psi-pGuide_para.AB.psi);
					pGuide_para.Route_psi=0.5*psi_tmp;
					if (psi_tmp<0.0f) psi_tmp=-psi_tmp;
					if (psi_tmp<150.0f) 
					{ 
						pGuide_para.tan_psi=sin(psi_tmp/RADIAN_2_DEGREE*0.5f)/cos(psi_tmp/RADIAN_2_DEGREE*0.5f);
					}
					else pGuide_para.tan_psi=3.73f;
				}
				else pGuide_para.tan_psi=0.0f;
			}
			pGuide_para.step_enter++;
			break;
		case 1:
			pGuide_para.mode_guid=PW_GuideDim3;
			WP_LateWay(&pGuide_para.AB, pGuide_para.cur_lon,pGuide_para.cur_lat,pGuide_para.cur_psi, &pGuide_para.ac_dZ,&pGuide_para.ac_dL,&pGuide_para.ac_dPsi);
			if (pGuide_para.cmd_guid==PW_GuideDim3) pGuide_para.mode_guid=PW_GuideDim3;
			if (pGuide_para.cmd_guid==PW_GuideDim2) pGuide_para.mode_guid=PW_GuideDim2;
			pGuide_para.token_late=TOKEN_TrackWay;   
			pGuide_para.step_WayLate=0;
			pGuide_para.nav_guid=PW_NavWay;
			if (pGuide_para.mode_guid==PW_GuideDim3) { 
				pGuide_para.height_cmd=pGuide_para.AB.alt; 
				pGuide_para.flightTgo = (int)(pGuide_para.AB.FlightTime-pGuide_para.AB.ac_FlightTime); //指令待飞时间计算
//				pGuide_para.Vm_cmd = Guid_VcmdInit(pGuide_para.ac_dL,pGuide_para.flightTgo,pGuide_para.ac_dPsi); //初始速度指令计算	
				pGuide_para.Vm_cmd = pGuide_para.AB.Velocity_cmd; // 210
				GUID_LongMan(); 
			}
			break;
		}
	}
	// 20231107补加应急策略
	else if(pGuide_para.nav_guid == PW_NavNone)
	{
		switch(pGuide_para.step_glide)
		{
		case 0:
			if(WP_GetLine(pGuide_para.ac_dot, &pGuide_para.AB))
			{
				pGuide_para.step_glide ++;
			}
			break;
		case 1:
			WP_LateWay(&pGuide_para.AB, pGuide_para.cur_lon,pGuide_para.cur_lat,pGuide_para.cur_psi, &pGuide_para.ac_dZ,&pGuide_para.ac_dL,&pGuide_para.ac_dPsi);
			// 航向提前修正
		//	if( (control_para.driv_time > PW_GLIDE_TIME) || ( pGuide_para.engine_startup_failed_flag == 1) )
			if (control_para.driv_time > PW_GLIDE_TIME)
			{
				Guide_Line(pGuide_para.cur_Vm,pGuide_para.ac_dZ,pGuide_para.ac_dPsi);
			}
			break;
		}
	}

	else{
		pGuide_para.step_enter=0;
	}
}
/**理论视线角速度解算**/
void GUID_WxyzCal(void)
{
	double	fR_Los[3] = { 0 };//导航系下单位视线矢量
	double	Rxyz1[3] = { 0 };//弹体系下视线角单位向量
	float	sinR = 0.0, cosR = 0.0, sinY = 0.0, cosY = 0.0, sinP = 0.0, cosP = 0.0;
	float	Cbn[9] = { 0.0 };//导航系到弹体系坐标转换矩阵

	static unsigned char Flag_Inti = 0;// 初始赋值标志位

	//计算弹目距离(计算动力学得到理想的)
	pGuide_para.Rnue[0] = 6371000 * (pGuide_para.tar_lat / RADIAN_2_DEGREE - pGuide_para.cur_lat / RADIAN_2_DEGREE);							//北向距离
	pGuide_para.Rnue[1] = pGuide_para.tar_alt - pGuide_para.cur_high;										//天向距离
	pGuide_para.Rnue[2] = 6371000 * cos(pGuide_para.cur_lat / RADIAN_2_DEGREE) * (pGuide_para.tar_lon / RADIAN_2_DEGREE - pGuide_para.cur_lon / RADIAN_2_DEGREE);	//东向距离

	pGuide_para.deltaR = sqrt(pGuide_para.Rnue[0] * pGuide_para.Rnue[0] + pGuide_para.Rnue[1] * pGuide_para.Rnue[1] + pGuide_para.Rnue[2] * pGuide_para.Rnue[2]);
	if (pGuide_para.deltaR < PW_delatRlimit)
		pGuide_para.deltaR = PW_delatRlimit; //弹目距离限制

	pGuide_para.lambdaD = atan2(pGuide_para.Rnue[1], sqrt(pGuide_para.Rnue[0] * pGuide_para.Rnue[0] + pGuide_para.Rnue[2] * pGuide_para.Rnue[2])) * RADIAN_2_DEGREE;
	pGuide_para.lambdaT = -atan2(pGuide_para.Rnue[2], pGuide_para.Rnue[0]) * RADIAN_2_DEGREE;//视线高低角

	if (Flag_Inti == 0)
	{
		pGuide_para.Bomb_R0 = sqrt(pGuide_para.Rnue[0] * pGuide_para.Rnue[0] + pGuide_para.Rnue[1] * pGuide_para.Rnue[1] + pGuide_para.Rnue[2] * pGuide_para.Rnue[2]);   //弹目初始水平距离,即射程
		Flag_Inti = 1;
	}

	//导航系下单位视线矢量
	fR_Los[0] = pGuide_para.Rnue[0] / pGuide_para.deltaR;
	fR_Los[1] = pGuide_para.Rnue[1] / pGuide_para.deltaR;
	fR_Los[2] = pGuide_para.Rnue[2] / pGuide_para.deltaR;

	//Cbn是导航系到弹体系的转换矩阵，Cnb是弹体系到导航系的转换矩阵
	sinR = sin(pGuide_para.QK_ENU / RADIAN_2_DEGREE);
	cosR = cos(pGuide_para.QK_ENU / RADIAN_2_DEGREE);
	sinY = sin((pGuide_para.QH_ENU) / RADIAN_2_DEGREE);
	cosY = cos((pGuide_para.QH_ENU) / RADIAN_2_DEGREE);
	sinP = sin(pGuide_para.QB_ENU / RADIAN_2_DEGREE);
	cosP = cos(pGuide_para.QB_ENU / RADIAN_2_DEGREE);
	//Cbn = 2-3-1转换矩阵×"东-北-天"-->"前上右"转换矩阵

	Cbn[0] = cosY * cosP;
	Cbn[1] = sinP;
	Cbn[2] = -cosP * sinY;
	Cbn[3] = -cosY * sinP * cosR + sinY * sinR;
	Cbn[4] = cosP * cosR;
	Cbn[5] = sinY * sinP * cosR + cosY * sinR;
	Cbn[6] = cosY * sinP * sinR + sinY * cosR;
	Cbn[7] = -cosP * sinR;
	Cbn[8] = -sinY * sinP * sinR + cosY * cosR;
	//视线角单位向量从导航系转换到弹体系
	Rxyz1[0] = Cbn[0] * fR_Los[0] + Cbn[1] * fR_Los[1] + Cbn[2] * fR_Los[2];
	Rxyz1[1] = Cbn[3] * fR_Los[0] + Cbn[4] * fR_Los[1] + Cbn[5] * fR_Los[2];
	Rxyz1[2] = Cbn[6] * fR_Los[0] + Cbn[7] * fR_Los[1] + Cbn[8] * fR_Los[2];
	//外框俯仰，内框偏航
	pGuide_para.Pitch_Preset_Angle = atan2(Rxyz1[1], Rxyz1[0]) * RADIAN_2_DEGREE;				//上正
	pGuide_para.Yaw_Preset_Angle = -atan2(Rxyz1[2], sqrt(Rxyz1[0] * Rxyz1[0] + Rxyz1[1] * Rxyz1[1])) * RADIAN_2_DEGREE;	//左正
	//---框架角限幅----//
	pGuide_para.Pitch_Preset_Angle = Limit_In(-40, 10, pGuide_para.Pitch_Preset_Angle);//俯仰-40°-10°
	pGuide_para.Yaw_Preset_Angle   = Limit_In(-30, 30, pGuide_para.Yaw_Preset_Angle);//偏航-30°-30°
	//导航系下视线角速度(未修正目标运动产生的视线角速度)
	pGuide_para.Wnue[0] = (-fR_Los[1] * (pGuide_para.cur_VE - 0.0) + fR_Los[2] * (pGuide_para.cur_VU - 0.0)) / pGuide_para.deltaR;
	pGuide_para.Wnue[1] = (-fR_Los[2] * (pGuide_para.cur_VN - 0.0) + fR_Los[0] * (pGuide_para.cur_VE - 0.0)) / pGuide_para.deltaR;
	pGuide_para.Wnue[2] = (-fR_Los[0] * (pGuide_para.cur_VU - 0.0) + fR_Los[1] * (pGuide_para.cur_VN - 0.0)) / pGuide_para.deltaR;

	//视线角速度从导航系转到弹体系
	pGuide_para.Wxyz[0] = Cbn[0] * pGuide_para.Wnue[0] + Cbn[1] * pGuide_para.Wnue[1] + Cbn[2] * pGuide_para.Wnue[2];
	pGuide_para.Wxyz[1] = Cbn[3] * pGuide_para.Wnue[0] + Cbn[4] * pGuide_para.Wnue[1] + Cbn[5] * pGuide_para.Wnue[2];
	pGuide_para.Wxyz[2] = Cbn[6] * pGuide_para.Wnue[0] + Cbn[7] * pGuide_para.Wnue[1] + Cbn[8] * pGuide_para.Wnue[2];

}
/**L1制导律**/
void Guide_L1(double Vm,double L1,double dZ,double dPsi,double radius)
{
	double c        = 0.0f;
	double diff_dZ  = 0.0f;
	const double Kgain    = 1.5f;
	double tmp1,tmp2,tmp3;

	if(L1<=100){
		L1=100;
	}
	
	diff_dZ = Vm*sin(dPsi/RADIAN_2_DEGREE);   /*[侧偏距的变化率]*/
	c       = sqrt(1-L1*L1/(4*radius*radius));
	tmp1    = 2*Kgain*Vm*Vm*c*c/L1/L1;
	tmp2    = 2*Kgain*Vm*c/L1;
	if (pGuide_para.mode_line == PW_LineLeft)
		tmp3 = -Vm * Vm / radius;
	else if (pGuide_para.mode_line == PW_LineRight)
		tmp3 = Vm * Vm / radius;
	else
		tmp3 = 0;
	pGuide_para.az_cmd  = -tmp1*dZ-tmp2*diff_dZ+tmp3;
	
	pGuide_para.gama_cmd= atan(pGuide_para.az_cmd/9.8f)*RADIAN_2_DEGREE;
	pGuide_para.gama_cmd=Limit_In(-PW_gama_cmd_limit,PW_gama_cmd_limit,pGuide_para.gama_cmd);	
	
}
/**直线侧边距控制**/
void Guide_Line(double Vm,double dZ,double dPsi)
{
	//====航迹控制增益===//
	const double fkzd=0.1;
	const double fkzdi=0.001;
	const double fkpsi=3.5;
	
	double fpsi,fdpsi;

	/**************航迹zd*****************/	
	fpsi=-dPsi;	
	fpsi=Limit_In(-90.,90.,fpsi); //限幅90度
	
	fdpsi=fkpsi*fpsi;  //航向——》滚转角指令
	
	
	pGuide_para.fzd=-dZ;   //侧边距误差积分项限幅+死区	

	pGuide_para.fzd=Limit_In(-20.,20.,pGuide_para.fzd);	
	pGuide_para.fzd=deadzone(-1,1,pGuide_para.fzd);
	
	pGuide_para.fzdi=0.5* pGuide_para.con_fTimerStep *(pGuide_para.fzd+pGuide_para.fzd_last)+pGuide_para.fzdi_last;  //侧边距误差积分	
	pGuide_para.fzd_last=pGuide_para.fzd;	
	pGuide_para.fzdi_last=pGuide_para.fzdi; 
		
	
	pGuide_para.fzd=-dZ; //侧边距误差比例项
	
	pGuide_para.fzd=Limit_In(-200.,200.,pGuide_para.fzd);//侧边距误差,只限幅
	
	pGuide_para.fdzd=fkzd*pGuide_para.fzd+fkzdi*pGuide_para.fzdi;
	
	pGuide_para.gama_cmd=fdpsi+pGuide_para.fdzd;	
	pGuide_para.gama_cmd=Limit_In(-PW_gama_cmd_limit,PW_gama_cmd_limit,pGuide_para.gama_cmd);	

	// 20231026 只遥测
	pGuide_para.fdpsi = fdpsi;
}
/**圆轨迹侧边距控制**/
void Guide_Circ(double Vm, double dZ, double dPsi, double radius)
{
	double diff_dZ = 0.0f;
	const double Kgain = 1.0;
	double tmp1, tmp2, tmp3, tmp4;

	pGuide_para.dR = dZ; //
	pGuide_para.dR_I = 0.5f * pGuide_para.con_fTimerStep * (pGuide_para.dR + pGuide_para.dR_last) + pGuide_para.dR_I_last;
	pGuide_para.dR_last = pGuide_para.dR;
	pGuide_para.dR_I_last = pGuide_para.dR_I;  //侧边距积分值

	diff_dZ = Vm * sin(dPsi / RADIAN_2_DEGREE);   /*[侧偏距的变化率]*/
	tmp1 = 0.03;
	tmp2 = 0.15;  //202550918pb
	tmp3 = tmp1*0.05;
	tmp4 = 0;
	if (pGuide_para.mode_line == PW_LineLeft)
		tmp4 = -Vm * Vm / radius;
	if (pGuide_para.mode_line == PW_LineRight)
		tmp4 = Vm * Vm / radius;
	pGuide_para.az_cmd = -tmp1 * pGuide_para.dR - tmp2 * diff_dZ - tmp3 * pGuide_para.dR_I + tmp4;

	pGuide_para.gama_cmd = atan(pGuide_para.az_cmd / 9.8f) * RADIAN_2_DEGREE;
	pGuide_para.gama_cmd = Limit_In(-PW_gama_cmd_limit, PW_gama_cmd_limit, pGuide_para.gama_cmd);

}
/*---末制导律----*/

void GUID_GuidanceLaw(float* value)
{
	float GuidanceLaw_Cmd[3]; //制导指令
	float Ky, Kz, Kg;

	pGuide_para.GNC_time += pGuide_para.con_fTimerStep;

	if (pGuide_para.Terminal_Guidance == 0) //坐标打击
	{
		pGuide_para.pitch_rate_nT = pGuide_para.Wxyz[2] * cosd(pGuide_para.QK_ENU) + pGuide_para.Wxyz[1] * sind(pGuide_para.QK_ENU);
		pGuide_para.yaw_rate_nT = -pGuide_para.Wxyz[2] * sind(pGuide_para.QK_ENU) + pGuide_para.Wxyz[1] * cosd(pGuide_para.QK_ENU);
	}
	else //DYT攻击
	{
		pGuide_para.pitch_rate_nT = pGuide_para.DYT_Wxyz[2] * cosd(pGuide_para.QK_ENU) + pGuide_para.DYT_Wxyz[1] * sind(pGuide_para.QK_ENU);
		pGuide_para.yaw_rate_nT  = -pGuide_para.DYT_Wxyz[2] * sind(pGuide_para.QK_ENU) + pGuide_para.DYT_Wxyz[1] * cosd(pGuide_para.QK_ENU);
	}

	Los_Filter(); //视线角速度滤波

	if (pGuide_para.deltaR > (pGuide_para.Bomb_R0 / 2))
	{
		Ky = 4.5;  //过载驾驶仪开环时，需增大导航比补偿增益 5.5-6.0
		Kg = 1.0;
	}
	else if (pGuide_para.deltaR >= (pGuide_para.Bomb_R0 / 2 - 500))
	{
		Ky = 4.5 + 0. * (pGuide_para.Bomb_R0 / 2 - pGuide_para.deltaR) / 500;
		Kg = 1.0 - 0.0 * (pGuide_para.Bomb_R0 / 2 - pGuide_para.deltaR) / 500;
	}
	else
	{
		Ky = 4.5;
		Kg = 1.0;
	}
	Kz = 4.5;

	//---------BTT/STT切换逻辑-------//
	pGuide_para.nz_zl_judge = -Kz * pGuide_para.yaw_rate_nT * fabs(pGuide_para.cur_Vm) / 9.8;

	switch ((fabs(pGuide_para.nz_zl_judge) < nz_Constraint) || (pGuide_para.BTT_STT_Switch)) { //转入STT后不可退出，退出需重新规划目标点
	case 0:     //BTT制导
		pGuide_para.BTT_STT_Switch = 0; //切换为STT
		pGuide_para.ny_zl = Ky * pGuide_para.pitch_rate_nT_filterOut[0] * fabs(pGuide_para.cur_Vm) / 9.8 + Kg * cosd(pGuide_para.QB_ENU);
		pGuide_para.nz_zl = -Kz * pGuide_para.yaw_rate_nT_filterOut[0] * fabs(pGuide_para.cur_Vm) / 9.8;
		//====主升力面过载指令计算===//
#ifdef BTT90
		pGuide_para.nc_zl = sqrt(pGuide_para.ny_zl * pGuide_para.ny_zl + pGuide_para.nz_zl * pGuide_para.nz_zl) * sign(pGuide_para.ny_zl); //BTT-90
#endif
#ifdef BTT180
		pGuide_para.nc_zl = sqrt(pGuide_para.ny_zl * pGuide_para.ny_zl + pGuide_para.nz_zl * pGuide_para.nz_zl) * (-sign(pGuide_para.ny_zl)); //BTT-180
#endif

		GuidanceLaw_Cmd[1] = pGuide_para.nc_zl;

		if (fabs(pGuide_para.nc_zl) < nc_zl_min)
		{
			GuidanceLaw_Cmd[0] = pGuide_para.gama_cmd_last;//滚转角指令保持
		}
		else
		{
#ifdef BTT90 //BTT-90指令分配
			GuidanceLaw_Cmd[0] = atand(pGuide_para.nz_zl / pGuide_para.ny_zl); //BTT-90
			if (fabs(GuidanceLaw_Cmd[0]) >= PW_Guidance_gama_cmd_limit)
			{
				GuidanceLaw_Cmd[0] = sign(GuidanceLaw_Cmd[0]) * PW_Guidance_gama_cmd_limit;
				GuidanceLaw_Cmd[1] = pGuide_para.ny_zl / cosd(PW_Guidance_gama_cmd_limit); //优先保证纵向
			}
			//------纵向过载指令穿0时限制滚转角指令----//
			if (fabs(pGuide_para.ny_zl) < 0.5)
				GuidanceLaw_Cmd[0] = Limit_In(-PW_Guidance_gama_cmd_min_limit, PW_Guidance_gama_cmd_min_limit, GuidanceLaw_Cmd[0]);
#endif

#ifdef BTT180 //BTT-180指令分配
			GuidanceLaw_Cmd[0] = atand(pGuide_para.nz_zl / pGuide_para.ny_zl);
			GuidanceLaw_Cmd[0] = 180 + GuidanceLaw_Cmd[0]; //BTT-180 滚转角指令反向
			if (GuidanceLaw_Cmd[0] >= (180 + PW_Guidance_gama_cmd_limit))
			{
				GuidanceLaw_Cmd[0] = 180 + PW_Guidance_gama_cmd_limit;
				GuidanceLaw_Cmd[1] = -ny_zl / cosd(PW_Guidance_gama_cmd_limit); //优先保证纵向
			}
			if (GuidanceLaw_Cmd[0] <= (180 - PW_Guidance_gama_cmd_limit))
			{
				GuidanceLaw_Cmd[0] = 180 - PW_Guidance_gama_cmd_limit;
				GuidanceLaw_Cmd[1] = -ny_zl / cosd(PW_Guidance_gama_cmd_limit); //优先保证纵向
			}
			if (fabs(pGuide_para.ny_zl) < 0.5)
				GuidanceLaw_Cmd[0] = Limit_In(180 - PW_Guidance_gama_cmd_min_limit, 180 + PW_Guidance_gama_cmd_min_limit, GuidanceLaw_Cmd[0]);
#endif
		}
		pGuide_para.gama_cmd_last = GuidanceLaw_Cmd[0];//保持指令

		GuidanceLaw_Cmd[2] = 0;
		break;

	case 1:     //STT制导

		pGuide_para.BTT_STT_Switch = 1; //切换为STT

		pGuide_para.ny_zl = Ky * pGuide_para.pitch_rate_nT_filterOut[0] * fabs(pGuide_para.cur_Vm) / 9.8 + Kg * cosd(pGuide_para.QB_ENU);
		pGuide_para.nz_zl = -Kz * pGuide_para.yaw_rate_nT_filterOut[0] * fabs(pGuide_para.cur_Vm) / 9.8;
		
		pGuide_para.ny_PPN_zl = Ky * pGuide_para.pitch_rate_nT_filterOut[0] * fabs(pGuide_para.cur_Vm) / 9.8; //比例导引项
		//--------攻击角约束项-----//
		if (pGuide_para.AttackAngle + pGuide_para.lambdaD > 0) {  //未达到期望攻击角 增加攻击角约束
			pGuide_para.FBEC = 0.2 * pGuide_para.cur_Vm * (pGuide_para.AttackAngle + pGuide_para.lambdaD) / 9.8 / 57.3;  //攻击角约束补偿项
		}
		else {
			pGuide_para.FBEC = 0;
		}
		pGuide_para.FBEC = Limit_In(-fabs(pGuide_para.ny_PPN_zl) * 1.0, fabs(pGuide_para.ny_PPN_zl) * 1.0, pGuide_para.FBEC); //限制在1.0倍比例导引项内
		pGuide_para.ny_zl = pGuide_para.ny_zl + pGuide_para.FBEC;

#ifdef BTT90 
		GuidanceLaw_Cmd[0] = 0;//滚转角保持
		GuidanceLaw_Cmd[1] = pGuide_para.ny_zl;//俯仰过载指令
		GuidanceLaw_Cmd[2] = pGuide_para.nz_zl;//偏航过载指令
#endif
#ifdef BTT180 //STT翻身
		GuidanceLaw_Cmd[0] = 180;
		GuidanceLaw_Cmd[1] = -pGuide_para.ny_zl;
		GuidanceLaw_Cmd[2] = -pGuide_para.nz_zl; //翻身制导
#endif
		break;

	default:
		break;
	}
	memcpy(value, GuidanceLaw_Cmd, sizeof(float) * 3);
	return ;
}
/**制导信息滤波模型**/
void Los_Filter(void)
{
	const float filter_coff = 0.25;
	pGuide_para.pitch_rate_nT_filterIn[1] = pGuide_para.pitch_rate_nT_filterIn[0];
	pGuide_para.pitch_rate_nT_filterIn[0] = pGuide_para.pitch_rate_nT;
	Tustin_FirstIO(0, 1, filter_coff, 1, pGuide_para.pitch_rate_nT_filterIn, pGuide_para.pitch_rate_nT_filterOut, pGuide_para.con_fTimerStep); //con_fTimerStep
	pGuide_para.pitch_rate_nT_filterOut[1] = pGuide_para.pitch_rate_nT_filterOut[0];

	pGuide_para.yaw_rate_nT_filterIn[1] = pGuide_para.yaw_rate_nT_filterIn[0];
	pGuide_para.yaw_rate_nT_filterIn[0] = pGuide_para.yaw_rate_nT;
	Tustin_FirstIO(0, 1, filter_coff, 1, pGuide_para.yaw_rate_nT_filterIn, pGuide_para.yaw_rate_nT_filterOut, pGuide_para.con_fTimerStep); //con_fTimerStep
	pGuide_para.yaw_rate_nT_filterOut[1] = pGuide_para.yaw_rate_nT_filterOut[0];
}
/**初始速度指令模型**/
double Guid_VcmdInit(double len,int flightTimeToGo,double dpsi)
{
	double V_len = 0;
	double V_cmd  = 0;

	V_len = len/flightTimeToGo;
	V_cmd = V_len;
//	V_cmd = V_len/cos(dpsi/RADIAN_2_DEGREE);
	
	pGuide_para.Vm_cmd_pre = V_cmd;
	
	return V_cmd;

}
/**实时速度指令模型**/
double Guid_VcmdReal(int flightTimeToGo,int ac_flightTimeToGo)
{
	double Kt = 0.5;   //调节增益
	double V_cmd  = 0;

	pGuide_para.ax_cmd_cur = Kt*(ac_flightTimeToGo - flightTimeToGo);
	pGuide_para.Vm_cmd_cur = 0.5f * pGuide_para.con_fTimerStep * (pGuide_para.ax_cmd_cur + pGuide_para.ax_cmd_pre) + pGuide_para.Vm_cmd_pre;

	pGuide_para.ax_cmd_pre = pGuide_para.ax_cmd_cur;
	pGuide_para.Vm_cmd_pre = pGuide_para.Vm_cmd_cur;

	V_cmd = pGuide_para.Vm_cmd_cur;

	return V_cmd;
	
}

void GUID_Inte_Zero(double *dError,double *dError_last,double *dError_I,double *dError_I_last)
{
	*dError        =  0.0;
	*dError_last   =  0.0;
	*dError_I      =  0.0;
	*dError_I_last =  0.0;
}

void GUID_Axis_Change (void)
{
	double nxyz_T[3],nxyz_D[3],nxyz_V[3];

	nxyz_T[0]=pGuide_para.nx_T;
	nxyz_T[1]=pGuide_para.ny_T;
	nxyz_T[2]=pGuide_para.nz_T;
		
	pGuide_para.matr_NUE2bQH[0][0] = cosd(pGuide_para.QH_ENU);
	pGuide_para.matr_NUE2bQH[0][1] = 0.;
	pGuide_para.matr_NUE2bQH[0][2] = -sind(pGuide_para.QH_ENU);
	pGuide_para.matr_NUE2bQH[1][0] = 0.;
	pGuide_para.matr_NUE2bQH[1][1] = 1.;
	pGuide_para.matr_NUE2bQH[1][2] = 0.;
	pGuide_para.matr_NUE2bQH[2][0] = sind(pGuide_para.QH_ENU);
	pGuide_para.matr_NUE2bQH[2][1] = 0.;
	pGuide_para.matr_NUE2bQH[2][2] = cosd(pGuide_para.QH_ENU);
	pGuide_para.matr_NUE2bQB[0][0] = cosd(pGuide_para.QB_ENU);
	pGuide_para.matr_NUE2bQB[0][1] = sind(pGuide_para.QB_ENU);
	pGuide_para.matr_NUE2bQB[0][2] = 0.;
	pGuide_para.matr_NUE2bQB[1][0] = -sind(pGuide_para.QB_ENU);
	pGuide_para.matr_NUE2bQB[1][1] = cosd(pGuide_para.QB_ENU);
	pGuide_para.matr_NUE2bQB[1][2] = 0.;
	pGuide_para.matr_NUE2bQB[2][0] = 0.;
	pGuide_para.matr_NUE2bQB[2][1] = 0.;
	pGuide_para.matr_NUE2bQB[2][2] = 1.;
	pGuide_para.matr_NUE2bQK[0][0] = 1.;
	pGuide_para.matr_NUE2bQK[0][1] = 0.;
	pGuide_para.matr_NUE2bQK[0][2] = 0.;
	pGuide_para.matr_NUE2bQK[1][0] = 0.;
	pGuide_para.matr_NUE2bQK[1][1] = cosd(pGuide_para.QK_ENU);
	pGuide_para.matr_NUE2bQK[1][2] = sind(pGuide_para.QK_ENU);
	pGuide_para.matr_NUE2bQK[2][0] = 0.;
	pGuide_para.matr_NUE2bQK[2][1] = -sind(pGuide_para.QK_ENU);
	pGuide_para.matr_NUE2bQK[2][2] = cosd(pGuide_para.QK_ENU); 	//地面系—》弹体系
	pGuide_para.cur_VNUEvector[0] = pGuide_para.cur_VN;
	pGuide_para.cur_VNUEvector[1] = pGuide_para.cur_VU;
	pGuide_para.cur_VNUEvector[2] = pGuide_para.cur_VE;
	//惯测攻角侧滑角
	matmat(pGuide_para.matr_NUE2bQK, pGuide_para.matr_NUE2bQB, pGuide_para.matr_NUE2bQH, pGuide_para.cur_VNUEvector, pGuide_para.cur_VXYZvector);
	pGuide_para.arf_ins = -atan2(pGuide_para.cur_VXYZvector[1], pGuide_para.cur_VXYZvector[0]) * 57.3;
	if (pGuide_para.cur_Vm == 0)
		pGuide_para.beta_ins = 0;
	else
		pGuide_para.beta_ins = asin(pGuide_para.cur_VXYZvector[2] / pGuide_para.cur_Vm) * 57.3;
	pGuide_para.matr_b2NUEQH[0][0]=cosd(-pGuide_para.QH_ENU);
	pGuide_para.matr_b2NUEQH[0][1]=0.;
	pGuide_para.matr_b2NUEQH[0][2]=-sind(-pGuide_para.QH_ENU);
	pGuide_para.matr_b2NUEQH[1][0]=0.;
	pGuide_para.matr_b2NUEQH[1][1]=1.;
	pGuide_para.matr_b2NUEQH[1][2]=0.;
	pGuide_para.matr_b2NUEQH[2][0]=sind(-pGuide_para.QH_ENU);
	pGuide_para.matr_b2NUEQH[2][1]=0.;
	pGuide_para.matr_b2NUEQH[2][2]=cosd(-pGuide_para.QH_ENU);
	
	pGuide_para.matr_b2NUEQB[0][0]=cosd(-pGuide_para.QB_ENU);
	pGuide_para.matr_b2NUEQB[0][1]=sind(-pGuide_para.QB_ENU);
	pGuide_para.matr_b2NUEQB[0][2]=0.;
	pGuide_para.matr_b2NUEQB[1][0]=-sind(-pGuide_para.QB_ENU);
	pGuide_para.matr_b2NUEQB[1][1]=cosd(-pGuide_para.QB_ENU);
	pGuide_para.matr_b2NUEQB[1][2]=0.;
	pGuide_para.matr_b2NUEQB[2][0]=0.;
	pGuide_para.matr_b2NUEQB[2][1]=0.;
	pGuide_para.matr_b2NUEQB[2][2]=1.;
	
	pGuide_para.matr_b2NUEQK[0][0]=1.;
	pGuide_para.matr_b2NUEQK[0][1]=0.;
	pGuide_para.matr_b2NUEQK[0][2]=0.;
	pGuide_para.matr_b2NUEQK[1][0]=0.;
	pGuide_para.matr_b2NUEQK[1][1]=cosd(-pGuide_para.QK_ENU);
	pGuide_para.matr_b2NUEQK[1][2]=sind(-pGuide_para.QK_ENU);
	pGuide_para.matr_b2NUEQK[2][0]=0.;
	pGuide_para.matr_b2NUEQK[2][1]=-sind(-pGuide_para.QK_ENU);
	pGuide_para.matr_b2NUEQK[2][2]=cosd(-pGuide_para.QK_ENU);  //体轴系—》地面系  导航坐标系

	matmat(pGuide_para.matr_b2NUEQH,pGuide_para.matr_b2NUEQB,pGuide_para.matr_b2NUEQK,nxyz_T,nxyz_D); 
	
	nxyz_D[1] = nxyz_D[1] - 1;//考虑重力的合过载

	pGuide_para.matr_NUE2TRAQH[0][0]=cosd(-pGuide_para.cur_psi); //北偏东为正，需＋负号转为北偏西
	pGuide_para.matr_NUE2TRAQH[0][1]=0.;
	pGuide_para.matr_NUE2TRAQH[0][2]=-sind(-pGuide_para.cur_psi);
	pGuide_para.matr_NUE2TRAQH[1][0]=0.;
	pGuide_para.matr_NUE2TRAQH[1][1]=1.;
	pGuide_para.matr_NUE2TRAQH[1][2]=0.;
	pGuide_para.matr_NUE2TRAQH[2][0]=sind(-pGuide_para.cur_psi);
	pGuide_para.matr_NUE2TRAQH[2][1]=0.;
	pGuide_para.matr_NUE2TRAQH[2][2]=cosd(-pGuide_para.cur_psi);
	pGuide_para.matr_NUE2TRAQB[0][0]=cosd(pGuide_para.cur_thetav);
	pGuide_para.matr_NUE2TRAQB[0][1]=sind(pGuide_para.cur_thetav);
	pGuide_para.matr_NUE2TRAQB[0][2]=0.;
	pGuide_para.matr_NUE2TRAQB[1][0]=-sind(pGuide_para.cur_thetav);
	pGuide_para.matr_NUE2TRAQB[1][1]=cosd(pGuide_para.cur_thetav);
	pGuide_para.matr_NUE2TRAQB[1][2]=0.;
	pGuide_para.matr_NUE2TRAQB[2][0]=0.;
	pGuide_para.matr_NUE2TRAQB[2][1]=0.;
	pGuide_para.matr_NUE2TRAQB[2][2]=1.;
	pGuide_para.matr_I[0][0]=1;
	pGuide_para.matr_I[0][1]=0;
	pGuide_para.matr_I[0][2]=0.;
	pGuide_para.matr_I[1][0]=0;
	pGuide_para.matr_I[1][1]=1;
	pGuide_para.matr_I[1][2]=0.;
	pGuide_para.matr_I[2][0]=0.;
	pGuide_para.matr_I[2][1]=0.;
	pGuide_para.matr_I[2][2]=1.;//地面系—》弹道系 
	
	matmat(pGuide_para.matr_I,pGuide_para.matr_NUE2TRAQB,pGuide_para.matr_NUE2TRAQH,nxyz_D,nxyz_V);  
	pGuide_para.aVtotal = nxyz_V[0]*9.8; //合加速度
}
//----任务规划更新----//
void Update_Mission(MISSION new_Mission)
{
	memcpy(&Flight_mission, &new_Mission, sizeof(MISSION));

	Mission_Updated_sign = 1; //任务更新状态机更新
}
//----任务规划处理----//
void GUID_MISSION(void)
{
	if (Mission_Updated_sign == 1) //已经更新
	{
		GUID_Spec(Flight_mission.MsnCmdType, 1);  /*[处理任务特征]*/

		Mission_Updated_sign = 0; //恢复状态机
	}
	else
	{

	}
}
//
