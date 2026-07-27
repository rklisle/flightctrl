#ifndef _CONTROL_OUT_H_
#define _CONTROL_OUT_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

#define RUD_G_ANGLE_MAX	20.0//deg  当俯仰与滚转共用物理舵时，滚转舵最大角度
#define RUD_H_ANGLE_MAX	20.0//deg  水平舵最大角度，滚转舵和俯仰舵不共用，后续优化为发射初段
#define RUD_V_ANGLE_MAX	20.0//deg  航向舵最大角度

typedef struct  _Stru_Control_Out_Input
{
	double u25g;//PD
	double u4g;	//I
	double ug_adrc;//内回路ADRC
	double uqkf;//前馈
	double u2f;//P
	double u4f;//I
	double u5f;//D
	double ugf;//高度控制PID
	double u5h;//D
	double urf_zd;//纵向制导
	double urh_zd;//侧向制导
	double gama;//滚转角解耦
	
	double time_control_on;//开始控制时间
	double time_separate_booster;//助推器分离时间
	double time_combat_status;//战斗或末制导时间
	double time_combat_delay;
}Stru_Control_Out_Input;

typedef struct  _Stru_Control_Out_Output
{
	double u1;//物理舵对应的电机角度
	double u2;
	double u3;
	double u4;
	double u5;
	double u6;

	double uf;//逻辑舵
	double uh;
	double ug;
}Stru_Control_Out_Output;

class CMathControlOut
{
public:
	CMathControlOut();
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor		* p_st_debug_monitor;
	Stru_Control_Out_Input		* p_st_control_out_input;
	Stru_Control_Out_Output	* p_st_control_out_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	//void Monitor_Data();
	
	double RudAgl2MotorAglH1(double rud_angle);//-51~＋30
	double RudAgl2MotorAglH2(double rud_angle);//-30~＋51
	double RudAgl2MotorAglH3(double rud_angle);//-51~＋30
	double RudAgl2MotorAglH4(double rud_angle);//-30~＋51
	double RudAgl2MotorAglV1(double rud_angle);//-36~26
	double RudAgl2MotorAglV2(double rud_angle);//-26~+36
	
	double m_gama;//舵控输出，滚转解耦
	
	double m_u_control[6];//物理舵角
	
	double m_urg;	//逻辑舵
	double m_urg_record;
	double m_urh;
	double m_urf;
	double m_urf_record;
	
	double m_u25g;
	double m_u4g;
	double m_ug_adrc;
	double m_uqkf;
	double m_u2f;
	double m_u4f;
	double m_u5f;
	double m_u5h;
	double m_ugf;
	double m_urf_zd;
	double m_urh_zd;

	double m_time_control_on;
	double m_time_separate_booster;
	double m_time_combat_status;
	double m_time_combat_delay;
};

#endif
