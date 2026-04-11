#ifndef _CONTROL_OUT_H_
#define _CONTROL_OUT_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 飞控输出模块，包括舵解耦，舵限幅，舵输出等
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

typedef struct  _Stru_Control_Out_Input
{
	double u25g;
	double u4g;
	double ug_adrc;
	double u2f;
	double u5f;
	double ugf;
	double u5h;
	double urf_zd;
	double urh_zd;
	double gama;
	double time_control_on;
	double time_separate_booster;
	double time_combat_status;
	double time_combat_delay;
}Stru_Control_Out_Input;

typedef struct  _Stru_Control_Out_Output
{
	double u1;
	double u2;
	double u3;
	double u4;
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
	void Monitor_Data();
	double m_gama;
	double m_u_control[6];
	double m_urg;
	double m_urg_record;
	double m_urh;
	double m_urf;
	double m_u25g;
	double m_u4g;
	double m_ug_adrc;
	double m_u2f;
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