#ifndef _CONTROL_ROLL_H_
#define _CONTROL_ROLL_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 飞控滚动通道控制模块
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"
const double ROLL_RATE_COMMAND	= 25.0; //deg per second
const double ROLL_COMMAND_STATIC_LIMIT = 30.0;//deg 
const double ROLL_COMMAND_DYNMIC_LIMIT = 60.0;//deg 

typedef struct  _Stru_Roll_Control_Input
{
	double gama;
	double wx;
	double sz;
	double mass;//质量
	double v;	//空速
	double vnz;
	double g;
	double dqh;
	double angle_zw;//转弯角度
	double radius_zw;//转弯半径
	double target_velocity;
	double gama_turn_nominal;
	double time_control;
	double time_separate_booster;
	double time_engine_start;
	double time_combat_status;
	double time_turn_in_start;
	double time_turn_in_end;
	double time_turn_out_start;
	double time_turn_out_end;
	bool flag_launch_turn;
}Stru_Roll_Control_Input;

typedef struct  _Stru_Roll_Control_Output
{
	double u25g;
	double u4g;
	double ug_adrc;
	double time_combat_delay;
	double gama_command_compensate;
}Stru_Roll_Control_Output;

class CMathControlRoll
{
public:
	CMathControlRoll();
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor		    * p_st_debug_monitor;
	Stru_Roll_Control_Input		* p_st_roll_control_input;
	Stru_Roll_Control_Output	* p_st_roll_control_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	// void Monitor_Data();
	void Calc_Control_Gain();
	void Calc_Control_Commond();
	double m_k2g;//内环比例
	double m_k5g;//内环微分
	double m_k4g;//内环积分
	double m_k3g;//外环侧偏比例
	double m_k7g;//外环侧偏微分
	double m_k6g;//外环侧偏积分
	double m_w0;
	double m_k;
	double m_z1;
	double m_z2;
	double m_last_k2g;
	double m_last_k5g;
	double m_last_k4g;
	double m_tar_k2g;
	double m_tar_k5g;
	double m_tar_k4g; 
	double m_u2g;
	double m_u5g;
	double m_u4g;
	double m_ug_adrc;
	double m_u3g;
	double m_u7g;
	double m_u6g;
	double m_u25g;
	double m_urg_record;
	double m_uz;	//侧向干扰
	double m_uz1_record;
	double m_sz;

	double m_mass;	//估计质量
	double m_v;		//空速
	double m_vnz;
	double m_g;
	double m_gama;
	double m_gama_record1;//助推器分离时刻滚转角
	double m_gama_record2;//入转弯准备后，滚转角
	double m_gama_command;
	double m_gama_command_compensate;
	double m_gama_command_compensate_out;
	double m_wx;
	double m_wx_command;
	double m_gama_turn_nominal;
	double m_turn_angle;
	double m_turn_radius;
	double m_target_velocity;
	double m_time_control;
	double m_time_combat_delay;
	double m_time_separate_booster;
	double m_time_engine_start;
	double m_time_combat_status;
	double m_time_turn_in_start;
	double m_time_turn_in_end;
	double m_time_turn_out_start;
	double m_time_turn_out_end;
	double m_time_v70;
	double m_time_v120;
	double m_time_v60;
	double m_time_v100;
	double m_time_v140;
	double m_time_start;
	double m_time_BTT_guidance_in;
	double m_dqh;
	bool m_flag_record;
	bool m_flag_changing;
	bool m_flag_turning;
	bool m_flag_distance_overlimit;
	bool m_flag_launch_turn;
};

#endif