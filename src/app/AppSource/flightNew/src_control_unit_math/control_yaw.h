#ifndef _CONTROL_YAW_H_
#define _CONTROL_YAW_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

typedef struct  _Stru_Yaw_Control_Input
{
	double gama;
	double wx;	//用于BTT机动过程，运动学、动力学解耦
	double wy;
	double mass;
	double hz;//组合高度
	double v; //空速
	double g;
	double qh;
	double dqh;
	double nby;
	double nbz;
	double nz_command_guidance;//速度系过载指令（BTT为零） 或 体轴系侧向过载指令（STT制导，滚转角近似于零解耦，体轴系即制导系）
	double tgo;
	double qh_leader;
	double det_qh;
	double distance_target;
	double gama_command_compensate;
	
	double time_separate_booster;
	double time_missile_takeoff;
	double time_combat_status;
	double time_combat_delay;
}Stru_Yaw_Control_Input;

typedef struct  _Stru_Yaw_Control_Output
{
	double u5h;
	double urh_zd;

	//用于遥测及分析
	double wy_command;
	double nz_command;
}Stru_Yaw_Control_Output;

// 状态观测器结构体
typedef struct  _Stru_Para_y_eso
{
    double z1;       // 当前观测状态1：角速度
    double z2;       // 当前观测状态2：总扰动
    double z1_pre;   // 上一时刻 z1
    double z2_pre;   // 上一时刻 z2
    double uy;  	// 【输入】（上一时刻）总舵控量（控制器计算值）
    double y;       // 【输入】当前实测滚转角速度（传感器值）
    double y_pre;   // 【输入】上一时刻实测滚转角速度
    double k;
    double Ts;       // 采样时间
}Stru_Para_y_eso;


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
	void Calc_Control_Gain_ADRC();

	void y_eso_init_takeoff();
	void y_eso_calc_takeoff(double b, double d, double Keso);
	void y_eso_init_cruise();
	void y_eso_calc_cruise(double b, double d, double Keso);
	void y_eso_init_attack();
	void y_eso_calc_attack(double b, double d, double Keso);
	
	Stru_Para_y_eso para_y_eso_takeoff;//起飞段ESO
	Stru_Para_y_eso para_y_eso_cruise;//巡飞段ESO
	Stru_Para_y_eso para_y_eso_attack;//末制导段ESO
	
	
	double m_k5h;	//正
	double m_knph;	//负
	double m_knih;	//负
	double m_kwih;	//正
	

	double m_uh_adrc;
	double m_urh_record;
	
	double m_u5h;	//阻尼
	double m_unwih;//未用到
	double m_unph;	//过载控制
	double m_unih;	//过载积分
	double m_ubh;	//伪攻角项
	double m_ubh_record;
	double m_urh_zd;//制导指令
	double m_urh_record_separate_booster;//助推到巡航
	double m_urh_record_begin_combat;//巡航到制导
	

	//double m_time_record_separate_booster;//时刻
	//double m_time_record_begin_combat;
	
	double m_time_separate_booster;
	double m_time_missile_takeoff;
	double m_time_combat_status;
	double m_time_combat_delay;
	double m_wx;
	double m_wy;
	double m_wy_record;
	double m_wy_command;
	double m_wy_command_comp;
	double m_wy_command_comp_record;
	double m_distance_target;
	double m_nz_command;
	double m_nz_command_guidance;
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
	bool m_flag_v30;
	int m_count_v30;//空速大于30m/s计数
	double m_time_v30;
	
	double m_gama_command_compensate;
};

#endif
