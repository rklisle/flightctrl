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
const double ROLL_RATE_COMMAND	= 25.0; //deg per second 滚转角速度
const double ROLL_COMMAND_STATIC_LIMIT = 30.0;//deg 静态滚转极值，非机动过程
const double ROLL_COMMAND_DYNMIC_LIMIT = 60.0;//deg 动态滚转极值，BTT转弯等机动过程

typedef struct  _Stru_Roll_Control_Input
{
	double gama;//滚转角、角速度
	double wx;
	double sz;
	double mass;//质量估计
	double hz;	//组合高度
	double v;	//空速
	double vnz;//侧向速度，地速
	double g;	//重力常数
	double dqh;//航向视线角速度
	double gama_command_guidance;//末制导滚转指令
	double angle_zw;//转弯角度
	double radius_zw;//转弯半径
	double target_velocity;//目标速度
	double gama_turn_nominal;//典型滚转角
	
	double time_control;//启控时间
	double time_separate_booster;
	double time_engine_start;
	double time_missile_takeoff;
	double time_altitude_control;//开始高度控制
	double time_launch_turn_ok;	//扇面转弯完成
	double time_combat_status;	//末制导时间
	double time_combat_dive_sidectrl;//末制导后，转侧偏控制;
	
	double time_turn_in_start;	//转弯开始、结束
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
	
	double time_combat_delay;	//进入末制导延迟时间，BTT到STT制导过渡过程
	
	double gama_command_compensate;
	double gama_command;//滚转角指令
	double wx_command;	 //滚转角速度指令
	double z2_adrc;//自抗扰状态估计量
	double uz;//侧偏控制滚转角指令
}Stru_Roll_Control_Output;

// 滚转角速度观测器结构体
typedef struct  _Stru_Para_wx_eso
{
    double z1;       // 当前观测状态1：角速度
    double z2;       // 当前观测状态2：总扰动
    double z1_pre;   // 上一时刻 z1
    double z2_pre;   // 上一时刻 z2
    double ux;  		// 【输入】（上一时刻）总舵控量（控制器计算值）
    double wx;       // 【输入】当前实测滚转角速度（传感器值）
    double wx_pre;   // 【输入】上一时刻实测滚转角速度
    double Ts;       // 采样时间
}Stru_Para_wx_eso;

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
	//void Monitor_Data();
	void Calc_Control_Gain();
	void Calc_Control_Commond();

	void wx_eso_init_takeoff();
	void wx_eso_calc_takeoff(double K0, double T0, double Keso);
	void wx_eso_init_cruise();
	void wx_eso_calc_cruise(double K0, double T0, double Keso);
	
	Stru_Para_wx_eso para_wx_eso_takeoff;//起飞段ESO
	Stru_Para_wx_eso para_wx_eso_cruise;//巡飞段及末制导段ESO
	
	double m_k2g;//内环比例
	double m_k5g;//内环微分
	double m_k4g;//内环积分
	double m_k3g;//外环侧偏比例
	double m_k7g;//外环侧偏微分
	double m_k6g;//外环侧偏积分

	double m_w0;//自抗扰内部控制量
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
	
	double m_mass;	//估计质量
	double m_v;		//空速
	double m_hz;
	double m_sz;	//侧向位移
	double m_vnz;
	double m_dqh;	//航向视线角速度，大干扰BTT末制导
	double m_g;
	
	double m_gama;
	double m_gama_record1;//助推器分离时刻滚转角
	double m_gama_record2;//入转弯准备后，滚转角
	double m_gama_record3;//进入末制导时刻滚转角
	double m_gama_command;
	double m_gama_command_compensate;
	double m_gama_command_compensate_out;
	double m_gama_command_guidance;
	double m_wx;
	double m_wx_command;
	
	//侧向机动，装订航迹解算
	double m_gama_turn_nominal;
	double m_turn_angle;
	double m_turn_radius;
	double m_target_velocity;
	
	double m_time_control;
	double m_time_separate_booster;
	double m_time_engine_start;
	double m_time_missile_takeoff;
	double m_time_altitude_control;
	double m_time_launch_turn_ok;
	double m_time_combat_status;
	double m_time_combat_delay;	//末制导滚转回零时间，用于滚转角末制导过渡
	double m_time_combat_dive_sidectrl;
	double m_time_BTT_guidance_in;//进入末制导时刻
	double m_time_start;	//未用到
	
	double m_time_turn_in_start;
	double m_time_turn_in_end;
	double m_time_turn_out_start;
	double m_time_turn_out_end;
	
	double m_time_v70;
	double m_time_v120;
	double m_time_v60;
	double m_time_v100;
	double m_time_v140;

	bool m_flag_record;	//未用到
	bool m_flag_changing;	//未用到
	bool m_flag_turning;//侧向机动标识
	bool m_flag_launch_turn;//扇面转弯标识
	bool m_flag_distance_overlimit;//航迹转弯，侧偏超限标识
};

#endif