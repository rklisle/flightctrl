#ifndef _CONTROL_YAW_H_
#define _CONTROL_YAW_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 飞控航向通道控制模块
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

typedef struct  _Stru_Yaw_Control_Input
{
	double gama;
	double wy;
	double mass;
	double hz;//组合高度
	double v; //空速
	double g;
	double qh;
	double dqh;
	double nby;
	double nbz;
	double tgo;
	double qh_leader;
	double det_qh;
	double distance_target;
	double gama_command_compensate;
	double time_separate_booster;
	double time_combat_status;
	double time_combat_delay;
}Stru_Yaw_Control_Input;

typedef struct  _Stru_Yaw_Control_Output
{
	double u5h;
	double urh_zd;
}Stru_Yaw_Control_Output;

class CMathControlYaw
{
public:
	CMathControlYaw();
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor			* p_st_debug_monitor;
	Stru_Yaw_Control_Input		* p_st_yaw_control_input;
	Stru_Yaw_Control_Output		* p_st_yaw_control_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	//void Monitor_Data();
	void Calc_Control_Gain();
	void Calc_Control_Commond();
	
	double m_k5h;
	double m_knih;
	double m_kwih;
	
	double m_u5h;	//阻尼
	double m_unwih;//未用到
	double m_unih;	//过载积分
	double m_ubh;	//伪攻角项
	double m_ubh_record;
	double m_urh_zd;//制导指令
	double m_urh_record_separate_booster;//助推到巡航
	double m_urh_record_begin_combat;//巡航到制导

	//double m_time_record_separate_booster;//时刻
	//double m_time_record_begin_combat;
	
	double m_time_separate_booster;
	double m_time_combat_status;
	double m_time_combat_delay;
	double m_wy;
	double m_wy_record;
	double m_wy_command;
	double m_distance_target;
	double m_nz_command;
	double m_nvz;//速度系侧向过载，用于侧滑角为零控制
	double m_nby;
	double m_nbz;
	double m_qh;
	double m_dqh;
	double m_tgo;
	double m_qh_leader;
	double m_det_qh;

	double m_mass;
	double m_v;
	double m_hz;
	double m_g;
	double m_gama;
	double m_time_v70;
	double m_time_v60;
	double m_time_v100;
	double m_time_v140;
	
	double m_gama_command_compensate;
};

#endif