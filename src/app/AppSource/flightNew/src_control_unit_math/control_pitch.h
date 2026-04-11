#ifndef _CONTROL_PITCH_H_
#define _CONTROL_PITCH_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 飞控俯仰通道控制模块
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

typedef struct  _Stru_Pitch_Control_Input
{
	double gama;
	double zeta;
	double wz;
	double hz;
	double mass;
	double v;
	double vs;
	double g;
	double dqf;
	double nby;
	double distance_target;
	double distance_target_t_combat;
	double target_height;
	double gama_command_compensate;
	double time_control;
	double time_separate_booster;
	double time_altitude_control;
	double time_altitude_change_start;
	double time_combat_status;
	double time_combat_delay;
	int	   count_altitude_change;
}Stru_Pitch_Control_Input;

typedef struct  _Stru_Pitch_Control_Output
{
	double u2f;
	double u5f;
	double ugf;
	double urf_zd;
}Stru_Pitch_Control_Output;

class CMathControlPitch
{
public:
	CMathControlPitch();
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor		    * p_st_debug_monitor;
	Stru_Pitch_Control_Input	* p_st_pitch_control_input;
	Stru_Pitch_Control_Output	* p_st_pitch_control_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	void Monitor_Data();
	void Calc_Control_Gain();
	void Calc_Control_Commond();
	int	m_count_altitude_change;
	int	m_count_altitude_change_record;
	double m_k2f;//内环比例
	double m_k5f;//内环微分
	double m_k3f;//外环比例
	double m_k7f;//外环微分
	double m_k6f;//外环积分
	double m_knif;//制导段，伪攻角三回路过载控制
	double m_kwif;//制导段，伪攻角三回路过载控制
	
	double m_u2f;	//内回路比例舵控
	double m_u5f;	//内回路微分舵控
	double m_u3f;
	double m_u7f;
	double m_u6f;
	double m_ugf;	//外回路舵控
	double m_unwif;	//进入导引前和舵控= 2+5+g
	double m_urf_zd;	//导引段和舵控

	double m_mass;	//估计质量
	double m_v;
	double m_vs;
	double m_vs_t_altitude_control;
	double m_g;
	double m_zeta;
	double m_zeta_command;
	double m_zeta_command_record1;//开始控制时刻指令俯仰角
	double m_zeta_command_record2;//助推器分离时刻指令俯仰角
	double m_wz;
	double m_hz;
	double m_hz_t_altitude_control;
	double m_h_command;
	double m_h_command_t_change;
	double m_h_command_t_change_record;
	double m_h_rate_command;
	double m_h_command1;
	double m_h_rate_command1;
	double m_h_command2;
	double m_h_rate_command2;
	double m_nby;
	double m_ny_command;
	double m_dqf;
	double m_time_separate_booster;
	double m_time_combat_status;
	double m_time_combat_delay;
	double m_time_altitude_control;
	double m_time_altitude_change_start;
	double m_time_altitude_change;
	double m_time_altitude_change_record;
	double m_time_control;
	double m_time_v70;
	double m_time_v120;
	double m_time_v60;
	double m_time_v100;
	double m_time_v140;
	double m_altitude_change_gain;
	double m_altitude_change_gain_seg1;
	double m_altitude_change_gain_record;
	double m_h_target_in;
	double m_h_target;
	double m_h_target_record;
	double m_distance_target;
	double m_distance_target_t_combat;
	double m_gama_command_compensate;
	double m_unif;
	double m_uaf;
	double m_uaf_record;
	double m_wz_record;
	bool   m_flag_altitude_integral_set;
};

#endif