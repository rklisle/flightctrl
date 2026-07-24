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
	//初始装订数据
	double ktheta_lauch_enc;//发射俯仰角
	double ktheta_climb_enc;//助推器分离后，爬升俯仰角，典型值8deg
	double ktheta_hight_enc;//定高巡航，俯仰角近似等于平衡攻角
	double ktheta_decline_enc;//下滑俯仰角，典型值-6deg
	
	//实时更新数据
	double gama;
	double zeta;
	double wz;
	double hz;	//组合高度
	double v;	//空速
	double mass;//质量估计
	double q;//动压
	double vs;	//垂速
	double g;	//重力加速度
	double dqf;	//俯仰视线角速度
	double nby;	//法向过载
	double ny_command_guidance;//升力面过载指令 或 纵向过载指令
	double distance_target;//(实时)弹目距离
	double distance_target_t_combat;//进入末制导时刻，弹目距离
	double target_height;	//目标高度
	double gama_command_compensate;//滚转角补偿
	
	double time_control;//开始控制时刻
	double time_separate_booster; //助推器分离时刻
	double time_missile_takeoff;	//起飞完成时刻
	double time_altitude_control;//开始高度控制时刻
	
	double time_altitude_change_start;//高度机动开始
	double time_altitude_change_end;//高度机动开始
	double time_combat_status;//战斗指令时刻
	double time_combat_delay;	//战斗指令延迟时间
	double time_combat_dive_pullup;//虚拟打击后，俯冲拉起
	
	int	   count_altitude_change;//高度机动次数
}Stru_Pitch_Control_Input;

typedef struct  _Stru_Pitch_Control_Output
{
	double uqkf;//俯仰前馈量
	double u2f;
	double u5f;
	double ugf;
	double urf_zd;

	//过载指令
	double h_command;
	double h_rate_command;
	double zeta_command;
	double ny_command;
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
	//void Monitor_Data();
	void Calc_Control_Gain();
	void Calc_Control_Commond();
	int	m_count_altitude_change;
	int	m_count_altitude_change_record;
	double m_k0f;//内环前馈
	double m_k2f;//内环比例
	double m_k5f;//内环微分
	double m_k3f;//外环比例
	double m_k7f;//外环微分
	double m_k6f;//外环积分
	double m_knif;//制导段，伪攻角三回路过载控制，过载积分控制
	double m_kwif;//制导段，伪攻角三回路过载控制，角速度积分攻角控制

	double m_ktheta_lauch_enc;//发射俯仰角
	double m_ktheta_climb_enc;//助推器分离后，爬升俯仰角
	double m_ktheta_hight_enc;//巡航段平飞攻角，巡航段，可根据重量、速度插值获得
	double m_ktheta_decline_enc;//下滑角
	double m_alpha_b;

	double m_uqkf;	//前馈量
	double m_u2f;	//内回路比例舵控，输出给舵控分配1
	double m_u5f;	//内回路微分舵控，积分形式伪攻角三回路过载控制复用为“角速度控制系数”，输出给舵控分配2
	double m_u3f;	//外回路比例，高度偏差
	double m_u7f;	//外回路微分，速度(与指令速度偏差）
	double m_u6f;	//外回路积分，高度偏差积分
	double m_ugf;	//外回路舵控，输出给舵控分配3
	double m_ugf_record;

	double m_unif; //过载积分控制，积分形式伪攻角三回路过载控制用
	double m_uaf;	//攻角控制系数(角速度小时间常数积分），积分形式伪攻角三回路过载控制用
	double m_uaf_record;
	double m_wz_record;
	
	double m_unwif;	//进入导引前和舵控 = 2+5+g高度，用于分析
	double m_urf_zd;//导引段和舵控，输出给舵控分配4
	
	double m_mass;	//估计质量
	double m_q;		//动压
	double m_v;		//空速
	double m_vs;	//垂速
	double m_vs_t_altitude_control;//进入首次高度控制时，组合垂速
	double m_g;
	double m_zeta;
	double m_zeta_command;
	double m_zeta_command_record1;//开始控制时刻指令俯仰角
	double m_zeta_command_record2;//助推器分离时刻指令俯仰角
	double m_zeta_t_change;//进入高度机动时刻，俯仰角
	double m_wz;
	double m_hz;
	double m_hz_t_altitude_control;//进入首次高度控制时，组合高度
	double m_h_command;
	double m_h_command_t_change;	//进入高度机动时，组合高度
	double m_h_command_t_change_record;
	double m_h_rate_command;
	double m_h_command1;
	double m_h_rate_command1;
	double m_h_command2;
	double m_h_rate_command2;
	double m_nby;
	double m_ny_command;
	double m_ny_command_guidance;
	double m_dqf;
	//外部输入时刻
	double m_time_separate_booster;
	double m_time_missile_takeoff;
	double m_time_combat_status;
	double m_time_combat_delay;
	double m_time_combat_dive_pullup;
	double m_time_altitude_control;//首次高度控制时刻
	double m_time_altitude_change_start;//输入
	double m_time_altitude_change_end;//输入
	//梳理输出时刻
	double m_time_altitude_change;//高度机动时刻(开始）
	double m_time_altitude_change_record;//前一次
	double m_time_altitude_change_tocruise;//高度机动 转 巡航控制时刻
	double m_time_zeta_change;//指令俯仰角切换时刻
	double m_time_zeta_change_record;//前一次
	
	//double m_time_zeta_change;
	double m_time_control;
	double m_time_v70;
	double m_time_v120;
	double m_time_v60;
	double m_time_v100;
	double m_time_v140;
	double m_altitude_change_gain;	//过渡时间
	double m_altitude_change_gain_seg1;//起飞过渡时间
	double m_altitude_change_gain_record;//前一次过渡时间
	double m_h_target_in;	//目标高度
	double m_h_target;		//进入高度控制时刻，目标高度
	double m_h_target_record;
	double m_distance_target;
	double m_distance_target_t_combat;
	double m_gama_command_compensate;

	bool   m_flag_altitude_integral_set;//高度积分
	int m_high_maneuver_state_record;
	int m_high_maneuver_state;//高度机动状态，0初始，1起飞过程，2定高控制，3爬升，4下滑，5末制导，6末制导(测试，需要俯冲拉起)，7俯冲拉起(测试末制导结束后)，8伞降回收状态
};

#endif