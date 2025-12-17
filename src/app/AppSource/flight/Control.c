/********************************************************************************/
/*Write By: Fig                                                                 */
/********************************************************************************/

#include "PGuide.h"
#include "Control.h"
#include "math.h"
#include "FCC_Lib.h"
#include <stdlib.h>
ControlPara control_para = { 0 };


//离轨时刻初始化

void Control_init(FLIGHT_INPUT *Flight_Input) 
{
     memset(&control_para, 0, sizeof(ControlPara));
	 control_para.con_fTimerStep = 0.005f;
	 control_para.SREF = 0.5;
	 control_para.LREF = 1.73;
	 control_para.mass = 210.5;
	 control_para.ESO_step = 0.005;
	 control_para.Vcmd = 0;
	// //====初始发射速度====//
	 control_para.g_fVm_init=sqrt(pow(Flight_Input->navVe, 2) 
                           + pow(Flight_Input->navVn, 2)
                           + pow(Flight_Input->navVs, 2) );
	 	// 将初始发射速度给	g_fVm				  
	control_para.g_fVm = control_para.g_fVm_init;

	control_para.Height_Rate = 20.0; //爬升/下滑率

	control_para.gama_ESO = Flight_Input->roll / RADIAN_2_DEGREE;//
	control_para.wx_ESO = Flight_Input->wx / RADIAN_2_DEGREE;

	control_para.mass_fuel = Flight_Input->mass_fuel;//
}

void SetControlPara(FLIGHT_INPUT *Flight_Input) 
{
    //系统时间

	control_para.launched_sign = Flight_Input->Luanched;

	control_para.fwxin[1] = control_para.fwxin[0];
	control_para.fwxin[0] = Flight_Input->wx;
	Tustin_FirstIO(0,1,0.005,1,control_para.fwxin,control_para.fwxout, control_para.con_fTimerStep);
	control_para.fwxout[1] = control_para.fwxout[0];

	control_para.fwzin[1] = control_para.fwzin[0];
	control_para.fwzin[0] = Flight_Input->wz;
	Tustin_FirstIO(0,1,0.005,1,control_para.fwzin,control_para.fwzout, control_para.con_fTimerStep);
	control_para.fwzout[1] = control_para.fwzout[0];

	control_para.fwyin[1] = control_para.fwyin[0];
	control_para.fwyin[0] = Flight_Input->wy;
	Tustin_FirstIO(0,1,0.005,1,control_para.fwyin,control_para.fwyout, control_para.con_fTimerStep);
	control_para.fwyout[1] = control_para.fwyout[0];

	control_para.theta = Flight_Input->pitch;
	control_para.gama  = Flight_Input->roll;

	if(control_para.driv_time<=0.01f) //初始俯仰角确定
	{
		control_para.thetac0 = control_para.theta;
	}

	control_para.driv_YDin[1] = control_para.driv_YDin[0];
	control_para.driv_YDin[0] = Flight_Input->navHigh;
	Tustin_FirstIO(0,1,0.1,1,control_para.driv_YDin,control_para.driv_YDout, control_para.con_fTimerStep);
	control_para.driv_YDout[1] = control_para.driv_YDout[0];

	control_para.vu = Flight_Input->navVs;
	control_para.g_fVm = sqrt(pow(Flight_Input->navVe, 2) 
                           + pow(Flight_Input->navVn, 2)
                           + pow(Flight_Input->navVs, 2) );
	control_para.cur_alt = Flight_Input->navHigh;

	control_para.airSpd = Flight_Input->airSpd;
	control_para.Vcmd = pGuide_para.Vm_cmd;

	control_para.feedback_duo_x = 0.5 * (Flight_Input->DD1 + Flight_Input->DD2);
	
	//control_para.feedback_duo_y = 0.5 * (Flight_Input->DD5 + Flight_Input->DD6);
	control_para.feedback_duo_y = 0;
}

/*
Function: Control Guidance Law
*/

void FlyStabilityControl()
{
	/*====控制回路积分限幅====*/
	const float RollLoop_I_Limit = 10; //滚转回路积分舵面限幅
	const float PitchLoop_I_Limit = 15; //俯仰角回路积分舵面限幅
	const float HeightLoop_I_Limit = 1.5; //高度回路积分过载限幅
	const float NycLoop_I_Limit = 10; //过载回路积分舵面限幅
	/*====控制回路通道舵限幅====*/
	const float RudderX_Limit = 15;
	const float RudderY_Limit = 15;
	const float RudderZ_Limit = 15;
	/*====控制回路物理舵限幅====*/
	const float Rudder_Limit = 15;

	// control_para.ru        = 1.225f*pow((1-pGuide_para.cur_high/44332.3), 4.2559); //密度计算
	control_para.ru        = 1.225f*pow((1-control_para.driv_YDout[0]/44332.3), 4.2559); //密度计算
//	control_para.qv        = 0.5*control_para.ru*control_para.g_fVm*control_para.g_fVm; //动压计算
	control_para.qv        = 0.5 * control_para.ru * control_para.airSpd * control_para.airSpd; //动压计算
	control_para.qvs       = control_para.qv*control_para.SREF;
	control_para.qvl       = control_para.qvs*control_para.LREF;
	control_para.qv        = Limit_In(4000, 40000, control_para.qv);  //7000->4000 202050719

	/****************************滚转方向*****************/	

	/**************滚转ADRC控制*****************/
	control_para.fkwx=0.1*10000/control_para.qv;
	
	control_para.fkgama=0.4*10000/control_para.qv;
	
	control_para.fkgamai=0.2*control_para.fkgama;//滚转角姿态PID控制系数
	if (pGuide_para.token_late == TOKEN_LateGuidance) {
		control_para.fkgamai = 1.0 * control_para.fkgama;//滚转角姿态PID控制系数
	}
	/**************ADRC估计补偿所需滚转舵*****************/
	control_para.mxdx   = -0.001; //滚转舵效
	control_para.mxdy   = -0.0004; //偏航舵滚转耦合项
	control_para.jcx    =  4.0;   //极转动惯量

	control_para.mx_ESO     =  -control_para.RollDisturbance_ESO*control_para.jcx;  //ESO估计干扰力矩
	control_para.fduox_ADRC = control_para.mx_ESO/(control_para.mxdx*control_para.qvl); //补偿干扰所需滚转舵面
	control_para.fduox_ADRC = Limit_In(-5,5,control_para.fduox_ADRC);//ADRC舵面限幅
	
	/**************ADRC end*****************/

	/**************滚转角指令误差*****************/
	
	if(pGuide_para.nav_guid!=PW_NavWay){ //滑翔模式
		if (pGuide_para.step_WayLate == 0) {
			control_para.gama0 = control_para.gama;//记录当前时刻实际滚转角
			control_para.gama_cmd_tran_In[0] = control_para.gama0; //指令滤波器初值
			control_para.gama_cmd_tran_In[1] = control_para.gama0; //指令滤波器初值
			control_para.gama_cmd_tran_Out[0] = control_para.gama0; //指令滤波器初值
			control_para.gama_cmd_tran_Out[1] = control_para.gama0; //指令滤波器初值
			control_para.gamma0_time = control_para.driv_time;
			pGuide_para.step_WayLate = 1;
		}
		control_para.gama_cmd_tran_In[1] = control_para.gama_cmd_tran_In[0];
		control_para.gama_cmd_tran_In[0] = pGuide_para.gama_cmd;
		Tustin_FirstIO(0, 1, 1.0, 1, control_para.gama_cmd_tran_In, control_para.gama_cmd_tran_Out, control_para.con_fTimerStep); //con_fTimerStep
		control_para.gama_cmd_tran_Out[1] = control_para.gama_cmd_tran_Out[0];
		control_para.gama_cmd_tran = control_para.gama_cmd_tran_Out[0];

		control_para.fgama= control_para.gama - control_para.gama_cmd_tran;   //滚转角指令误差			
	}
	else{
		switch(pGuide_para.token_late){
		case TOKEN_LateNone:
			control_para.gama_cmd_tran = 0.;
			control_para.fgama=control_para.gama - control_para.gama_cmd_tran;   //滚转角指令误差
			break;
		case TOKEN_TrackCirc:
			if(pGuide_para.step_WayLate==0){
				control_para.gama0=control_para.gama_cmd_tran; //记录当前时刻滚转角过渡指令
				control_para.gamma0_time=control_para.driv_time;
				pGuide_para.step_WayLate=1;
			}
			control_para.gama_cmd_tran = pGuide_para.gama_cmd+(control_para.gama0-pGuide_para.gama_cmd)*exp(-1.0f*(control_para.driv_time-control_para.gamma0_time)); //滚转角指数过渡指令			
			control_para.fgama=control_para.gama-control_para.gama_cmd_tran;
			break;
		case TOKEN_TrackHover:
			if (pGuide_para.step_WayLate == 0) {
				control_para.gama0 = control_para.gama_cmd_tran; //记录当前时刻滚转角过渡指令
				control_para.gamma0_time = control_para.driv_time;
				pGuide_para.step_WayLate = 1;
			}
			control_para.gama_cmd_tran = pGuide_para.gama_cmd + (control_para.gama0 - pGuide_para.gama_cmd) * exp(-1.0f * (control_para.driv_time - control_para.gamma0_time)); //滚转角指数过渡指令			
			control_para.fgama = control_para.gama - control_para.gama_cmd_tran;
			break;
		case TOKEN_TrackWay:
			if(pGuide_para.step_WayLate==0){
				control_para.gama0=control_para.gama_cmd_tran; //记录当前时刻滚转角过渡指令
				control_para.gamma0_time=control_para.driv_time;
				pGuide_para.step_WayLate=1;
			}
			control_para.gama_cmd_tran = pGuide_para.gama_cmd+(control_para.gama0-pGuide_para.gama_cmd)*exp(-1.0f*(control_para.driv_time-control_para.gamma0_time)); //滚转角指数过渡指令			
			control_para.fgama=control_para.gama-control_para.gama_cmd_tran;
			break;
		case TOKEN_LateGuidance:
			if (pGuide_para.step_WayLate == 0) {
				control_para.gama0 = control_para.gama_cmd_tran; //记录当前时刻滚转角过渡指令
				control_para.gamma0_time = control_para.driv_time;
				pGuide_para.step_WayLate = 1;
			}
			control_para.gama_cmd_tran = pGuide_para.gama_cmd + (control_para.gama0 - pGuide_para.gama_cmd) * exp(-1.0f * (control_para.driv_time - control_para.gamma0_time)); //滚转角指数过渡指令			
			control_para.fgama = control_para.gama - control_para.gama_cmd_tran;
			//----限制误差不超过180deg----//
			if (control_para.fgama > 180)
				control_para.fgama = control_para.fgama - 360;
			if (control_para.fgama < -180)
				control_para.fgama = control_para.fgama + 360;

			break;
		case TOKEN_TrackDubins:
			if (pGuide_para.step_WayLate == 0) {
				control_para.gama0 = control_para.gama_cmd_tran; //记录当前时刻滚转角过渡指令
				control_para.gama_cmd_tran_In[0] = control_para.gama0; //指令滤波器初值
				control_para.gama_cmd_tran_In[1] = control_para.gama0; //指令滤波器初值
				control_para.gama_cmd_tran_Out[0] = control_para.gama0; //指令滤波器初值
				control_para.gama_cmd_tran_Out[1] = control_para.gama0; //指令滤波器初值
				control_para.gamma0_time = control_para.driv_time;
				pGuide_para.step_WayLate = 1;
			}
		//	control_para.gama_cmd_tran = pGuide_para.gama_cmd + (control_para.gama0 - pGuide_para.gama_cmd) * exp(-1.0f * (control_para.driv_time - control_para.gamma0_time)); //滚转角指数过渡指令			
			control_para.gama_cmd_tran_In[1] = control_para.gama_cmd_tran_In[0];
			control_para.gama_cmd_tran_In[0] = pGuide_para.gama_cmd;
			Tustin_FirstIO(0,1,1.0,1,control_para.gama_cmd_tran_In, control_para.gama_cmd_tran_Out, control_para.con_fTimerStep); //con_fTimerStep
			control_para.gama_cmd_tran_Out[1] = control_para.gama_cmd_tran_Out[0];
			control_para.gama_cmd_tran = control_para.gama_cmd_tran_Out[0];
			control_para.fgama = control_para.gama - control_para.gama_cmd_tran;
			break;
		default:						
			control_para.gama_cmd_tran = 0; 			
			control_para.fgama=control_para.gama-control_para.gama_cmd_tran;//滚转角指令误差
			break;
		}
	}
	
	control_para.fgamai = 0.5f* control_para.con_fTimerStep*(control_para.fgama+control_para.fgama_last)+control_para.fgamai_last;
	control_para.fgamai = Limit_In(-RollLoop_I_Limit / control_para.fkgamai, RollLoop_I_Limit / control_para.fkgamai, control_para.fgamai);//积分限幅
	control_para.fduox = control_para.fkgama*control_para.fgama+control_para.fkgamai*control_para.fgamai+control_para.fkwx*control_para.fwxout[0];
	control_para.fduox = control_para.fduox +control_para.fduox_ADRC;
	control_para.fduox = Limit_In(-RudderX_Limit, RudderX_Limit, control_para.fduox);

	// control_para.RollCompensate_ESO = control_para.fduox*control_para.mxdx*control_para.qvl/control_para.jcx;  //滚转舵产生补偿力矩,需要引入ESO观测器
	// 20231029 此处使用实际舵角
	control_para.RollCompensate_ESO = (control_para.feedback_duo_x*control_para.mxdx + control_para.feedback_duo_y * control_para.mxdy)*control_para.qvl/control_para.jcx;  //滚转舵产生补偿力矩,需要引入ESO观测器
	
	/**************偏航方向*****************/
	if (control_para.qv < 18000)																						//偏航通道参数(mywy = -1)----【mazg】------20250408
	{
		control_para.fkwy = -0.0273 * (control_para.qv - 7000) / 1000 + 0.7;//
		control_para.fknz = -1.4545 * (control_para.qv - 7000) / 1000 + 25.0;//
		control_para.fknzi = -2.0909 * (control_para.qv - 7000) / 1000 + 50.0;
	}
	else if (control_para.qv < 29000)
	{
		control_para.fkwy = -0.0091 * (control_para.qv - 18000) / 1000 + 0.4;//
		control_para.fknz = -0.4091 * (control_para.qv - 18000) / 1000 + 9.0;//
		control_para.fknzi = -0.8182 * (control_para.qv - 18000) / 1000 + 27.0;
	}
	else
	{
		control_para.fkwy = -0.0091 * (control_para.qv - 29000) / 1000 + 0.3;//
		control_para.fknz = -0.1636 * (control_para.qv - 29000) / 1000 + 4.5;//
		control_para.fknzi = -0.4091 * (control_para.qv - 29000) / 1000 + 18.0;
	}
	//---频率稳定边界补偿---//
	//if (control_para.DFT_Stable_sign == 1) {  //触发频率稳定边界时增益降低
	//	control_para.fknz = control_para.fknz * 0.7;
	//	control_para.fknzi = control_para.fknzi * 0.7;
	//}
	control_para.fknz = control_para.fknz * 0.3;
	control_para.fknzi = control_para.fknzi * 0.3; //增益调整 
	control_para.fkwy = control_para.fkwy * 0.8;	

	control_para.fnz = pGuide_para.nz_cmd - pGuide_para.nz_T;//过载误差
	control_para.fnz = Limit_In(-2.0, 2.0, control_para.fnz);

	control_para.fnzi = 0.5 * control_para.con_fTimerStep * (control_para.fnz + control_para.fnz_last) + control_para.fnzi_last;
	control_para.fnzi = Limit_In(-NycLoop_I_Limit / control_para.fknzi, NycLoop_I_Limit / control_para.fknzi, control_para.fnzi); //过载误差积分限幅

	control_para.fduoy = control_para.fknz * control_para.fnz + control_para.fknzi * control_para.fnzi + control_para.fkwy * control_para.fwyout[0];

    /**************俯仰方向*****************/
	if (control_para.qv < 18000)					//俯仰角参数without(mzwz)----【mazg】------20250328
	{
		control_para.fkwz = -0.03454 * (control_para.qv - 7000) / 1000 + 0.6;//
		control_para.fktheta = -0.09091 * (control_para.qv - 7000) / 1000 + 1.8;
	}
	else if (control_para.qv < 29000)
	{
		control_para.fkwz = -0.00727 * (control_para.qv - 18000) / 1000 + 0.22;//
		control_para.fktheta = -0.02727 * (control_para.qv - 18000) / 1000 + 0.8;
	}
	else
	{
		control_para.fkwz = -0.00236 * (control_para.qv - 29000) / 1000 + 0.14;//
		control_para.fktheta = -0.01091 * (control_para.qv - 29000) / 1000 + 0.5;
	}
	control_para.fkthetai=control_para.fktheta*0.15f;    //俯仰姿态角控制系数PID设置
	
	/******过载控制增益*****/
	if (control_para.qv < 18000)					//过载控制参数(mzwz = -1)----【mazg】------20250418
	{
	//	control_para.fkny = -0.57273 * (control_para.qv - 7000) / 1000 + 8.0;//
	//	control_para.fknyi = -1.06818 * (control_para.qv - 7000) / 1000 + 16.0;
		control_para.fkny = -0.4909 * (control_para.qv - 7000) / 1000 + 8.0;//
		control_para.fknyi = -0.8636 * (control_para.qv - 7000) / 1000 + 16.0;	//pb改	
	}
	else if (control_para.qv < 29000)
	{
	//	control_para.fkny = -0.08182 * (control_para.qv - 18000) / 1000 + 1.7;//
	//	control_para.fknyi = -0.13182 * (control_para.qv - 18000) / 1000 + 4.25;
		control_para.fkny = -0.1455 * (control_para.qv - 18000) / 1000 + 2.6;//
		control_para.fknyi = -0.3182 * (control_para.qv - 18000) / 1000 + 6.5;
	}
	else
	{
	//	control_para.fkny = -0.02727 * (control_para.qv - 29000) / 1000 + 0.8;//
	//	control_para.fknyi = -0.07273 * (control_para.qv - 29000) / 1000 + 2.8;
		control_para.fkny = -0.0227 * (control_para.qv - 29000) / 1000 + 1.0;//
		control_para.fknyi = -0.0341 * (control_para.qv - 29000) / 1000 + 3.0;
	}   //过载控制系数PI设置

	/******高度回路控制增益*****/
	control_para.fkvyd = -0.05 * 1.0;//-0.06;//
	control_para.fkyd = -0.01 * 1.0;//-0.01;//
	control_para.fkydi = control_para.fkyd * 0.05;//0.05;//高度控制外回路高度速度参数设置

	control_para.fkyd2theta = 0.2;
	control_para.fkyd2thetai = control_para.fkyd2theta * 0.05; //高度-俯仰角控制系数
	/**************俯仰方向舵指令模型*****************/

	if(pGuide_para.nav_guid!=PW_NavWay){ //滑翔模式
		
		control_para.thetac= Climb_Angle_Launch +(control_para.thetac0 - Climb_Angle_Launch)*exp(-0.2f*(control_para.driv_time-0.0f));
				
		control_para.ftheta= control_para.theta-control_para.thetac;
		control_para.fthetai = 0.5f* control_para.con_fTimerStep*(control_para.ftheta+control_para.ftheta_last)+control_para.fthetai_last;
		control_para.fthetai = Limit_In(-PitchLoop_I_Limit / control_para.fkthetai, PitchLoop_I_Limit / control_para.fkthetai, control_para.fthetai);//积分限幅	
		
		control_para.fduoz = control_para.fktheta* control_para.ftheta + control_para.fkthetai * control_para.fthetai +control_para.fkwz * control_para.fwzout[0];// 

		control_para.fduoz = Limit_In(-RudderZ_Limit + 5, RudderZ_Limit - 5, control_para.fduoz);
	}
	else{ //航迹跟踪模式
		switch(pGuide_para.token_long){
		case TOKEN_LongNone:
			break;
		case TOKEN_WayClimb:
			if((control_para.driv_YDout[0]<(pGuide_para.height_cmd- Height_Climb_Threshold)) && (pGuide_para.step_WayLong==0)){
				control_para.climb_thita_cmd=Climb_Angle;
				control_para.thetac0=control_para.theta;
				control_para.thetac0_time=control_para.driv_time;
				
				control_para.fduoz0 =control_para.fduoz;
				control_para.fthetai=control_para.fduoz0/control_para.fkthetai;

				control_para.ftheta_last =0;
				control_para.fthetai_last=control_para.fthetai;
				
				control_para.mode_flag=Climb_Ctrl;


				pGuide_para.step_WayLong=1;
				pGuide_para.mode_long=PW_LongClimb; //PGuide
			}
			if(control_para.driv_YDout[0]>=(pGuide_para.height_cmd- Height_Climb_Threshold)){
				control_para.thetac0_time=control_para.driv_time;

				control_para.fduoz0=control_para.fduoz;
				control_para.fy0=control_para.driv_YDout[0];

				control_para.Height_Rate = 20.0; //爬升速度赋值

				control_para.Height_cmd_tran_ke = -control_para.Height_Rate / Height_Climb_Threshold;//-0.2;//过渡系数计算  //6.0：爬升/下滑率
				
				control_para.fydi=0.0;				
				control_para.fyd_last=0;
				control_para.fydi_last=control_para.fydi;

				control_para.fny_last = 0.;
				control_para.fnyi = 0;
				control_para.fnyi_last = control_para.fnyi;

				control_para.mode_flag=Level_Ctrl;

				pGuide_para.token_long=TOKEN_LongNone;
				pGuide_para.mode_long=PW_LongLevel; //PGuide

				control_para.vy_limit = 30; // 20231129 将速度限幅改为25 20; //速度限幅
			}
			break;
		case TOKEN_WayLevel:
			control_para.thetac0_time=control_para.driv_time;

			control_para.fduoz0= control_para.fduoz;	
			control_para.fy0=control_para.driv_YDout[0];

			control_para.Height_cmd_tran_ke = -control_para.Height_Rate / Height_Climb_Threshold;//-0.2;//过渡系数计算  //6.0：爬升/下滑率
			
			control_para.fydi=0.0;
			control_para.fyd_last=0;
			control_para.fydi_last=control_para.fydi;

			control_para.fny_last = 0.;
			control_para.fnyi = 0;
			control_para.fnyi_last = control_para.fnyi;
			
			control_para.mode_flag=Level_Ctrl;
			
			pGuide_para.token_long=TOKEN_LongNone;
			pGuide_para.mode_long=PW_LongLevel; //PGuide
			
			control_para.vy_limit = 30; //速度限幅
			break;
		case TOKEN_WayDive:
			if((control_para.driv_YDout[0]>(pGuide_para.height_cmd+ Height_Glide_Threshold)) && (pGuide_para.step_WayLong==0)){
				pGuide_para.Route_Glide_Angle = Limit_In(Glide_Angle, Climb_Angle, pGuide_para.Route_Glide_Angle); //下滑角限幅
				control_para.climb_thita_cmd = pGuide_para.Route_Glide_Angle;//Glide_Angle;
				control_para.thetac0=control_para.theta;
		//		control_para.thetac0 = pGuide_para.cur_thetav; //控制轨迹倾角

				control_para.thetac0_time=control_para.driv_time;

				control_para.fduoz0 = control_para.fduoz;
				control_para.fthetai=control_para.fduoz0/control_para.fkthetai;
//				control_para.fthetai=Limit_In(-80,-20,control_para.fthetai);

				control_para.ftheta_last =0;
				control_para.fthetai_last=control_para.fthetai;
				
				control_para.mode_flag=Glide_Ctrl;

				pGuide_para.step_WayLong=1;
				pGuide_para.mode_long=PW_LongDive; //PGuide
			}
			//----实时制导下滑角更新----//
			pGuide_para.Route_Glide_Angle = Limit_In(Glide_Angle, Climb_Angle, pGuide_para.Route_Glide_Angle); //下滑角限幅
			control_para.climb_thita_cmd = pGuide_para.Route_Glide_Angle;
			
			if(control_para.driv_YDout[0]<=(pGuide_para.height_cmd+ Height_Glide_Threshold)){
				control_para.thetac0_time=control_para.driv_time;
				
				control_para.fduoz0=control_para.fduoz;
				control_para.fy0=control_para.driv_YDout[0];

				control_para.Height_Rate = -control_para.vu; //实时下滑速度赋值

				control_para.Height_cmd_tran_ke = -control_para.Height_Rate / Height_Glide_Threshold;//-0.2;//过渡系数计算  //6.0：爬升/下滑率
				
				control_para.fydi=0.0;				
				control_para.fyd_last=0;
				control_para.fydi_last=control_para.fydi;

				control_para.fny_last = 0.;
				control_para.fnyi = 0;
				control_para.fnyi_last = control_para.fnyi;
				
				control_para.mode_flag=Level_Ctrl;
				
				pGuide_para.token_long=TOKEN_LongNone;
				pGuide_para.mode_long=PW_LongLevel; //PGuide
				
				// control_para.vy_limit = 20; 	//速度限幅
				control_para.vy_limit = 25; 	// 20231118速度限幅调整
			}
			break;
		case TOKEN_LongGuidance: //末端打击
			control_para.thetac0_time = control_para.driv_time;
			control_para.fduoz0 = control_para.fduoz;

			control_para.fny_last = 0.;
			control_para.fnyi = 0;
			control_para.fnyi_last = control_para.fnyi;

			control_para.mode_flag = Guidance_Ctrl;

			pGuide_para.step_WayLong = 1;
			pGuide_para.mode_long = PW_Guidance;
			pGuide_para.token_long = TOKEN_LongNone;
			break;
		default:
			break;
		}
		switch(control_para.mode_flag){
		case Climb_Ctrl: //爬升
			control_para.thetac =control_para.climb_thita_cmd+(control_para.thetac0-control_para.climb_thita_cmd)*exp(-0.2f*(control_para.driv_time-control_para.thetac0_time));
			if (pGuide_para.Fient_sign == 2)
				control_para.thetac = control_para.climb_thita_cmd + (control_para.thetac0 - control_para.climb_thita_cmd) * exp(-0.1f * (control_para.driv_time - control_para.thetac0_time));

			control_para.ftheta = control_para.theta-control_para.thetac;
			control_para.fthetai=0.5f* control_para.con_fTimerStep*(control_para.ftheta+control_para.ftheta_last)+control_para.fthetai_last;
			control_para.fthetai = Limit_In(-PitchLoop_I_Limit/control_para.fkthetai, PitchLoop_I_Limit/control_para.fkthetai, control_para.fthetai);

			control_para.fduoz= control_para.fktheta* control_para.ftheta + control_para.fkthetai * control_para.fthetai + control_para.fkwz* control_para.fwzout[0];//
			RudderTrans();
			break;
		case Level_Ctrl: //平飞
			//-----TECS总能量控制器俯仰角指令---//
//			control_para.thetac = (pGuide_para.height_cmd - control_para.driv_YDout[0]) * control_para.fkyd2theta;
//			control_para.thetac = Limit_In(Glide_Angle, Climb_Angle, control_para.thetac);//俯仰角指令限幅

			control_para.height_cmd_tran=pGuide_para.height_cmd+(control_para.fy0-pGuide_para.height_cmd)*exp(control_para.Height_cmd_tran_ke*(control_para.driv_time-control_para.thetac0_time));								
			control_para.height_cmd_tran_rate = control_para.Height_cmd_tran_ke*(control_para.fy0-pGuide_para.height_cmd)*exp(control_para.Height_cmd_tran_ke*(control_para.driv_time-control_para.thetac0_time));
			
			control_para.fyd=control_para.driv_YDout[0] - control_para.height_cmd_tran;			
		//	control_para.fyd=control_para.driv_YDout[0] - pGuide_para.height_cmd;			
			control_para.fyd=Limit_In(-20,20,control_para.fyd);
			
			control_para.fydi=0.5f*control_para.con_fTimerStep*(control_para.fyd+control_para.fyd_last)+control_para.fydi_last;
			control_para.fydi = Limit_In(HeightLoop_I_Limit / control_para.fkydi, -HeightLoop_I_Limit / control_para.fkydi,control_para.fydi);

			control_para.vu=Limit_In(-control_para.vy_limit,control_para.vy_limit,control_para.vu);
			control_para.driv_vy_Vertical2cmd = control_para.vu - control_para.height_cmd_tran_rate;

			control_para.ny_cmd_I=control_para.fkydi*control_para.fydi;

			control_para.gama_HeightLoop = Limit_In(-PW_gama_cmd_limit, PW_gama_cmd_limit, control_para.gama); //滚转角限制
			control_para.ny_cmd = control_para.fkvyd* control_para.driv_vy_Vertical2cmd + control_para.ny_cmd_I + control_para.fkyd * control_para.fyd + cosd(pGuide_para.cur_thetav) / cosd(control_para.gama_HeightLoop);;//指数曲线跟踪
			
			control_para.fny = pGuide_para.ny_T - control_para.ny_cmd;//过载误差
			control_para.fny = Limit_In(-1.0, 1.0, control_para.fny);

			control_para.fnyi = 0.5 * control_para.con_fTimerStep * (control_para.fny + control_para.fny_last) + control_para.fnyi_last;
			control_para.fnyi = Limit_In(-NycLoop_I_Limit / control_para.fknyi, NycLoop_I_Limit / control_para.fknyi, control_para.fnyi); //过载误差积分限幅

			control_para.fduoz = control_para.fkny * control_para.fny + control_para.fknyi * control_para.fnyi + control_para.fkwz * control_para.fwzout[0];
			
			RudderTrans();
			break;
		case Glide_Ctrl: //下滑
			control_para.arf_ins = Limit_In(0.0, 3.0, pGuide_para.arf_ins); //攻角限幅
			control_para.thetac=(control_para.climb_thita_cmd+ control_para.arf_ins )+(control_para.thetac0-(control_para.climb_thita_cmd+ control_para.arf_ins))*exp(-0.2f*(control_para.driv_time-control_para.thetac0_time));//考虑惯测攻角，间接控制轨迹倾角
			control_para.ftheta= control_para.theta-control_para.thetac;
		//	control_para.ftheta = pGuide_para.cur_thetav - control_para.thetac; //控制轨迹倾角
			control_para.fthetai=0.5f* control_para.con_fTimerStep*(control_para.ftheta+control_para.ftheta_last)+control_para.fthetai_last;
			
			control_para.fthetai = Limit_In(-PitchLoop_I_Limit / control_para.fkthetai, PitchLoop_I_Limit / control_para.fkthetai, control_para.fthetai);

			control_para.fduoz= control_para.fktheta* control_para.ftheta+ control_para.fkthetai* control_para.fthetai+ control_para.fkwz* control_para.fwzout[0];
			RudderTrans();
			break;
		case Guidance_Ctrl: //末端打击
			control_para.fny = pGuide_para.ny_T - pGuide_para.ny_cmd;//过载误差
			control_para.fny = Limit_In(-2.0, 2.0, control_para.fny);

			control_para.fnyi = 0.5 * control_para.con_fTimerStep * (control_para.fny + control_para.fny_last) + control_para.fnyi_last;
			control_para.fnyi = Limit_In(-NycLoop_I_Limit / control_para.fknyi, NycLoop_I_Limit / control_para.fknyi, control_para.fnyi); //过载误差积分限幅

			control_para.fduoz = control_para.fkny * control_para.fny + control_para.fknyi * control_para.fnyi + control_para.fkwz * control_para.fwzout[0];
			RudderTrans();
			break;
		default:
			break;
		}
		control_para.fduoz = Limit_In(-RudderZ_Limit, RudderZ_Limit, control_para.fduoz);
	}
	//===12舵:1左2右--->滚转舵===//
	control_para.g_fUdelta01 = control_para.fduox;
	control_para.g_fUdelta02 = control_para.fduox;
	//===34舵:3左4右--->俯仰舵===//
	control_para.g_fUdelta03 = -control_para.fduoz;
	control_para.g_fUdelta04 =  control_para.fduoz;
	//===56舵:5左6右--->方向舵===//
	control_para.g_fUdelta05 = control_para.fduoy;
	control_para.g_fUdelta06 = control_para.fduoy;


	control_para.g_fUdelta01 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta01);
	control_para.g_fUdelta02 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta02);
	control_para.g_fUdelta03 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta03);
	control_para.g_fUdelta04 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta04);
	control_para.g_fUdelta05 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta05);
	control_para.g_fUdelta06 = Limit_In(-Rudder_Limit, Rudder_Limit, control_para.g_fUdelta06);

	//前后拍迭代
	control_para.ftheta_last= control_para.ftheta;
	control_para.fthetai_last= control_para.fthetai;
	control_para.fgama_last= control_para.fgama;
	control_para.fgamai_last= control_para.fgamai;
	control_para.fbeta_last=control_para.fbeta;
	control_para.fbetai_last=control_para.fbetai;
	control_para.fyd_last=control_para.fyd;
	control_para.fydi_last=control_para.fydi;
	control_para.fny_last = control_para.fny;
	control_para.fnyi_last = control_para.fnyi;
	control_para.fnz_last = control_para.fnz;
	control_para.fnzi_last = control_para.fnzi;
}

//--------滚转回路扩张状态观测器----//

void ESO_RollAutopilot(double omega_ESO,double u0_ESO)
{
	float beta1,beta2,beta3;

	beta1 = 3*omega_ESO;
	beta2 = 3*omega_ESO*omega_ESO;
	beta3 = omega_ESO*omega_ESO*omega_ESO;
	
	control_para.gama_error = control_para.gama_ESO - control_para.gama/RADIAN_2_DEGREE;//滚转角误差
	
	//----限制误差不超过180deg----//
	if (control_para.gama_error > (180 / RADIAN_2_DEGREE))
		control_para.gama_error = control_para.gama_error - 360 / RADIAN_2_DEGREE;
	if (control_para.gama_error < (-180 / RADIAN_2_DEGREE))
		control_para.gama_error = control_para.gama_error + 360 / RADIAN_2_DEGREE;
	
	control_para.RollDisturbance_ESO += -beta3*control_para.gama_error*control_para.ESO_step;
	control_para. wx_ESO             +=  (control_para.RollDisturbance_ESO + u0_ESO - beta2*control_para.gama_error)*control_para.ESO_step; 
	//		   control_para.gama_ESO +=  (control_para.wx_ESO - beta1*control_para.gama_error)*control_para.ESO_step;
			   //考虑横侧向运动对滚转角度耦合的影响,对角速度的估计精度更高
	control_para.gama_ESO            +=  (control_para.wx_ESO - tand(control_para.theta)*(control_para.fwy/RADIAN_2_DEGREE*cosd(control_para.gama)-control_para.fwz/RADIAN_2_DEGREE*sind(control_para.gama)) -beta1*control_para.gama_error)*control_para.ESO_step;
	
}

//
void FlyEngineControl()
{
	const float Kvp = 500,Kvi = Kvp*0.02; //速度控制回路参数 PB20250818 0.03->0.02 PB20251117
	const float cx_coff = 0.035; //阻力系数
	const float VelLoop_I_Limit = 37500;//速度积分限幅
	float Cx_Force,Gravity_Force;
	


	//if(control_para.launched_sign == 1) //启动
	if (1) //启动
	{
		Cx_Force      = cx_coff*control_para.qvs;
		//--TECS俯仰角指令--//
		//Gravity_Force = control_para.mass*9.8*sind(control_para.thetac);
		//control_para.ThrustF_init  = Cx_Force + Gravity_Force;  //推力平衡值

		control_para.fdv = control_para.Vcmd - control_para.airSpd;
		if (control_para.mode_flag == Level_Ctrl) //平飞时，积分阈值15m/s PB20250719
		{
			control_para.fdv = Limit_In(-30.0, 30.0, control_para.fdv); //误差限幅，保证大误差时输出小推力指令
			if (control_para.Vcmd < PW_VcmdMax) {//如果最大速度指令小于PW_VcmdMAX，积分阈值增大
				if (fabs(control_para.fdv) > 0.2 && control_para.fdv < 30.0) //积分分离法,误差>0.2，<30时采用积分,减速时积分持续作用
				{
					control_para.fdvi = 0.5 * control_para.con_fTimerStep * (control_para.fdv + control_para.fdv_last) + control_para.fdvi_last;
					control_para.fdvi = Limit_In(-VelLoop_I_Limit / Kvi, VelLoop_I_Limit / Kvi, control_para.fdvi);//积分限幅，当响应达不到指令时积分持续增大，后续控制很难退出			
					control_para.fdvi_last = control_para.fdvi;
				}
			}
			else {//最大速度指令超过PW_VcmdMAX
				if (fabs(control_para.fdv) > 0.2 && control_para.fdv < 15.0) //积分分离法,误差>0.2,<15,时采用积分,减速时积分持续作用
				{
					control_para.fdvi = 0.5 * control_para.con_fTimerStep * (control_para.fdv + control_para.fdv_last) + control_para.fdvi_last;
					control_para.fdvi = Limit_In(-VelLoop_I_Limit / Kvi, VelLoop_I_Limit / Kvi, control_para.fdvi);//积分限幅，当响应达不到指令时积分持续增大，后续控制很难退出			
					control_para.fdvi_last = control_para.fdvi;
				}
			}
		}
		else if (control_para.mode_flag == Glide_Ctrl)//下滑时，积分阈值10m/s PB20251017
		{
			control_para.fdv = Limit_In(-10.0, 30.0, control_para.fdv); //误差限幅，保证大误差时输出小推力指令
			if (fabs(control_para.fdv) > 0.2 && control_para.fdv < 10.0) //积分分离法,误差>0.2,<10,时采用积分,减速时积分持续作用
			{
				control_para.fdvi = 0.5 * control_para.con_fTimerStep * (control_para.fdv + control_para.fdv_last) + control_para.fdvi_last;
				control_para.fdvi = Limit_In(-VelLoop_I_Limit / Kvi, VelLoop_I_Limit / Kvi, control_para.fdvi);//积分限幅，当响应达不到指令时积分持续增大，后续控制很难退出			
				control_para.fdvi_last = control_para.fdvi;
			}
		}
		else  //爬升时，积分阈值5m/s PB20251017
		{ 
			control_para.fdv = Limit_In(-30.0, 30.0, control_para.fdv); //误差限幅，保证大误差时输出小推力指令
			if (fabs(control_para.fdv) > 0.2 && fabs(control_para.fdv) < 5.0) //积分分离法,误差>0.2,<5,时采用积分
			{
				control_para.fdvi = 0.5 * control_para.con_fTimerStep * (control_para.fdv + control_para.fdv_last) + control_para.fdvi_last;
				control_para.fdvi = Limit_In(-VelLoop_I_Limit / Kvi, VelLoop_I_Limit / Kvi, control_para.fdvi);//积分限幅，当响应达不到指令时积分持续增大，后续控制很难退出			
				control_para.fdvi_last = control_para.fdvi;
			}
		}
		control_para.fdv_last = control_para.fdv;
		control_para.rpm_delt = (control_para.fdv * Kvp + control_para.fdvi * Kvi); //转速控制增量

		control_para.rpm_delt_P = control_para.fdv * Kvp;  //用于遥测
		control_para.rpm_delt_I = control_para.fdvi * Kvi; //用于遥测
		
		//========最小物理转速限制=======//
		//-----Idle随高度变化关系----Idle=H*2.25+17500---//
		control_para.Rpm_Idle = control_para.driv_YDout[0] * 2.25 + 17500; //怠速转速拟合计算

		if (control_para.mode_flag == Glide_Ctrl)//下滑状态时:基础转速=70%  //油门最小怠速转速*1.2  
		{
			control_para.EngineSet = 0.7 * 50500 + control_para.rpm_delt;//control_para.Rpm_Idle * 1.2 + control_para.rpm_delt;//PB20251017
		}
		else if (control_para.mode_flag == Level_Ctrl) //平飞状态时:基础转速=80% //最小怠速转速*1.5  //
		{
			control_para.EngineSet = 0.8 * 50500 + control_para.rpm_delt;
		}
		else if (control_para.mode_flag == Guidance_Ctrl) //打击状态时:基础转速=最小怠速转速+500  //
		{
			if(control_para.fdv>=0) //需要加速时用80%油门+1.2倍PID
				control_para.EngineSet = 0.8 * 50500 + 1.2*control_para.rpm_delt;
			else //需要减速时进行基础转速+比例控制
				control_para.EngineSet = control_para.Rpm_Idle + 500 + control_para.rpm_delt_P;
		}
		else  //爬升状态时:基础转速=85%油门
		{
			control_para.EngineSet = 0.85 * 50500 + control_para.rpm_delt;
		}
	}
	else
		control_para.EngineSet = 0;
	EngineMaxRpm();
	//control_para.EngineSet = Limit_In(control_para.Rpm_Idle + 500, 48875, control_para.EngineSet);  //根据飞行高度限制转速,最大95%
	control_para.EngineSet = Limit_In(control_para.Rpm_Idle + 500, control_para.MaxRpmValue, control_para.EngineSet);  //根据飞行高度限制转速,最大95%
	//----起飞10s满油门---//
	if (control_para.driv_time < (Takeoff_Time-2))
	{
		control_para.EngineSet = 50500;
	}
	//----怠速指令处理----//
	if (pGuide_para.tag_Eng_idle == 1)
		control_para.EngineSet = 18000;
	//----动力停车指令处理----//
	if (pGuide_para.tag_Eng_off == 1)
		control_para.EngineSet = 0;
}
//--------发动机最大转速调度计算--------//
void EngineMaxRpm()
{
	//---根据设定大车工作最大时间进行最大转速调度---//
	//---调度机制:100%工作1分钟,95%10分钟,循环---//
	switch (control_para.Eng_Rpm_mode) {
	case 0: //100%大车工作1分钟
		control_para.MaxRpmValue = 50500;//最大值100%
		if (control_para.EngineSet >= (control_para.MaxRpmValue - 100)) {
			control_para.step_MaxRpm0++;   //100%大车累计次数
		}
		else {  //转速指令低于大车,则认为大车使用完成一次,切换为95%
			control_para.Eng_Rpm_mode = 1;//切换为95%大车状态
			control_para.step_MaxRpm0 = 0;//计数清零
		}
		if ((control_para.step_MaxRpm0 * control_para.con_fTimerStep) >= PW_MaxRpm0Time)  //100%大车累计时间达到60s
		{
			control_para.Eng_Rpm_mode = 1;//切换为95%大车状态
			control_para.step_MaxRpm0 = 0;//计数清零
		}
		break;
	case 1://95%大车工作10分钟后可循环
		control_para.MaxRpmValue = 48875;//最大值100%
		control_para.step_MaxRpm1++;   //95%大车累计次数		
		if ((control_para.step_MaxRpm1 * control_para.con_fTimerStep) >= PW_MaxRpm1Time)
		{
			control_para.Eng_Rpm_mode = 0;//切换为100%大车状态
			control_para.step_MaxRpm1 = 0;//计数清零
		}
		break;
	default:
		break;
	}
}
//------舵指令过渡模型----//
void RudderTrans(void)
{
	const float dtime = 2.0;
	const float Trans_time = 1.0;

	if (control_para.driv_time - control_para.thetac0_time < dtime)
	{
		control_para.fduoz = control_para.fduoz + (control_para.fduoz0 - control_para.fduoz) * exp(-Trans_time * (control_para.driv_time - control_para.thetac0_time));
	}
	else
	{
		control_para.fduoz = control_para.fduoz;
	}
}
//--------DFT辨识飞行频率--------//
void DftFreqIdentify(float flight_state)  //20251029PB
{
	//---50ms周期存储数据---离散DFT采样点100个---//
	//---频率最小分辨率=1/(0.05*100)=0.2Hz---//
	int num_loop = 0;
	memset(&control_para.state_real, 0, sizeof(control_para.state_real));
	memset(&control_para.state_imag, 0, sizeof(control_para.state_imag));

	//---离散傅里叶变换正余弦形式---//
	for (num_loop = PW_DFTNUM - 1; num_loop > 0; num_loop--)  //100个点滑动窗口，每次更新一个数据，始终保持窗口内100个点
	{
		control_para.flight_state_DFT[num_loop] = control_para.flight_state_DFT[num_loop - 1];
	}
	control_para.flight_state_DFT[0] = flight_state;

	//-----N*N循环计算---//
	for (int k = 0;k < PW_DFTNUM;k++)
	{
		for (int n = 0;n < PW_DFTNUM;n++)
		{
			control_para.angle_val = 2 * 3.1415926 * k * n / PW_DFTNUM;
			control_para.state_real[k] = control_para.state_real[k] + control_para.flight_state_DFT[n] * cos(control_para.angle_val); //余弦分量
			control_para.state_imag[k] = control_para.state_imag[k] - control_para.flight_state_DFT[n] * sin(control_para.angle_val); //正弦分量		
		}
		control_para.state_mag[k] = sqrt(control_para.state_real[k] * control_para.state_real[k] + control_para.state_imag[k] * control_para.state_imag[k]); //幅值

		if (abs(control_para.state_real[k]) > 1e-10)  //求取相位
			control_para.state_phase[k] = atan2(control_para.state_imag[k], control_para.state_real[k]);
		else
			control_para.state_phase[k] = sign(control_para.state_imag[k]) * 3.1415926 / 2;

		//---幅值归一化---//
		if (k == 0 || k == PW_DFTNUM / 2)   //直流分量和奈奎斯特频率
			control_para.state_amp[k] = control_para.state_mag[k] / PW_DFTNUM;
		else
			control_para.state_amp[k] = 2 * control_para.state_mag[k] / PW_DFTNUM;  //-----不同频率对应幅值----//
		//---找出幅值最大频率点(Hz)---//
		if (k >= 1 && k <= floor(PW_DFTNUM / 2) + 1) //跳过直流分量
		{
			if (k == 1) {
				control_para.state_amp_max = control_para.state_amp[k]; //第一次最大值赋值
				control_para.state_freq_max = 1 / (0.05 * PW_DFTNUM) * k;
			}
			if (control_para.state_amp_max > control_para.state_amp[k + 1]) {
				control_para.state_amp_max = control_para.state_amp_max;
				control_para.state_freq_max = control_para.state_freq_max;
			}
			else {
				control_para.state_amp_max = control_para.state_amp[k + 1];
				control_para.state_freq_max = 1 / (0.05 * PW_DFTNUM) * (k + 1);
			}
		}
	}
	//-----频率稳定边界状态标志----//
	if (control_para.state_freq_max >= 2.0 && control_para.state_amp_max >= 0.3) //连续满足频率>2Hz，幅值>0.3开始触发边界判定
		control_para.DFT_Stable_Num++;
	else
		control_para.DFT_Stable_Num = 0; //不连续清0
	if (control_para.DFT_Stable_Num > PW_DFTStableTime) //连续2s满足稳定边界条件
		control_para.DFT_Stable_sign = 1;//稳定边界状态置1
	//	else
	//		DFT_Stable_sign = 0;//稳定边界状态清0
}