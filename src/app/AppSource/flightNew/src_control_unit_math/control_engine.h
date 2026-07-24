#ifndef _CONTROL_ENGINE_H_
#define _CONTROL_ENGINE_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 发动机控制模块
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "../../debug_monitor.h"

//油门控制与转速控制选择:  油门(定义了宏)，未定义认为转速模式
#define __VEL__CONTROL__MODE__KC__	 0

//说明1：当地速控制时，空速过大或过小时，强制空速控制，指令速度限幅
//说明2：
const double VEL_COMMAND_MAX_LIMIT = 60.0;	//m per sencond 
const double VEL_COMMAND_CRUISE = 50.0;		//m per sencond
const double VEL_COMMAND_MIN_LIMIT = 40.0;	//m per sencond

const double RPM_COMMAND_MAX_LIMIT = 7000;//r per min
const double RPM_COMMAND_CRUISE = 5380;		// r per min
const double RPM_COMMAND_MIN_LIMIT = 2500;//r per min 

const double KC_COMMAND_MAX_LIMIT = 100;//油门百分比
const double KC_COMMAND_CRUISE = 70;	   //油门百分比
const double KC_COMMAND_MIN_LIMIT = 30;	//油门百分比

//#define __ENGINE_BSFC__	450 //发动机有效耗油率g/(kW.h)
//#define __ENGINE_AFC__		450 //发动机实际耗油率g/(kW.h)


typedef struct  _Stru_Engine_Control_Input
{
	double h_ini;
	double h;
	double mass;//质量估计
	double mach;//马赫数，与空速对应
	double sonic_speed;//空速
	double target_velocity;//指令空速
	double target_time;	//到达时间，相对发射时刻
	double temperature_ground;//地面温度，用于计算声速
	double missile_average_velocity;//平均地速
	double radius_zw;//转弯半径，用于计算转弯补偿油门或转速
	double velocity_command;

	bool flag_launch_turn;	//扇面转弯过程，转弯结束后清除
	bool flag_waypoint_turn;	//航迹转弯过程
	bool flag_alltitude_change;	//高度机动
	bool flag_alltitude_climb;	//爬升
	bool flag_alltitude_decline;	//下降
	
	bool flag_flightime_ctrl;		//到达时间控制标识，半开环补偿速度指令 1引入到达时间补偿；0不引入；
	bool flag_velocity_control;	//地速控制标识

	//飞行时序相关标识
	bool flag_engine_start;//助推器分离后，油门由怠速加速到保护油门
	bool flag_missile_takeoff;//起飞完成，开始速度控制
	bool flag_altitude_control;//首次开始高度控制
	bool flag_engine_stop;//发动机关机，指令油门为零
	bool flag_combat_status;//战斗指令
	bool flag_combat_dive_pullup;//俯冲拉起过程
}Stru_Engine_Control_Input;

typedef struct  _Stru_Engine_Control_Output
{
	double control_rpm;
	double control_Kc;
	//double mass_calc;
}Stru_Engine_Control_Output;

class CMathControlEngine
{
public:
	double flight_time;
	int time_tick;
	CMathControlEngine();
	//Stru_Debug_Monitor				* p_st_debug_monitor;
	Stru_Engine_Control_Input		* p_st_engine_control_input;
	Stru_Engine_Control_Output		* p_st_engine_control_output;
	void Run();
	void Initial();
private:
	void Get_Data();
#ifdef __VEL__CONTROL__MODE__KC__
	void Calc_Data_Kc();
#else
	void Calc_Data_Rpm();
#endif
	void Send_Data();
	//void Monitor_Data();
	double m_p;
	double m_p0;
	double m_p1;
	double m_p2;
	double m_p3;

	double m_dltVg_time;
	double m_dltKc_time;//半开环控制,5s周期补偿
	
#ifdef __VEL__CONTROL__MODE__KC__
	double m_Kc0;//基准油门
	double m_Kc1;//转弯机动油门，补偿
	double m_Kc1_record;
	double m_Kc2;//高度机动油门，补偿
	double m_Kc2_record;
	double m_Kc3;//时间控制油门，补偿
	double m_Kc4;//PI控制油门补偿
#else
	double m_n0;//基准转速
	double m_n1;//转弯机动转速，补偿
	double m_n2;//高度机动转速，补偿
	double m_n3;//时间控制转速，补偿
#endif
	double m_mach;	 //马赫数，空速换算
	double m_Vsonic;//声速，计算值
	double m_air_velocity;//马赫数计算空速
	double m_target_velocity;//指令空速
	double m_missile_velocity;//输入，地速(发动机输入)
	double m_target_time;//到达时间，与飞行时间计算，速度补偿
	int m_count_vel_pidctrl;
	
	double m_missile_height;
	double m_missile_height_initial;
	//double m_temperature_ground;//地面温度
	double m_fuel_comsumped;//消耗燃料
	double m_mass_calc;		//质量估计
	double m_turn_radius;	//转弯半径

#ifdef __VEL__CONTROL__MODE__KC__
	double m_control_Kc;	//控制油门
#else
	double m_control_rpm;	//控制转速，输出
#endif

	//double m_state_rpm;		//状态转速，输出
	
	double m_velocity_integrator;//速度积分
	double m_velocity_ground;	//地速
	bool m_flag_velocity_control;//速度控制标识，1引入地速控制，未用到
	
	bool m_flag_flightime_ctrl;	//到达时间标识，1引入时间控制，即到达时间补偿

	//起飞过程时序
	bool m_flag_engine_start;//助推器分离后，油门由怠速加速到保护油门
	bool m_flag_missile_takeoff;//起飞完成，开始速度控制
	bool m_flag_altitude_control;//首次开始高度控制
	bool m_flag_engine_stop;//发动机关机，指令油门为零
	bool m_flag_combat_status;//战斗指令
	bool m_flag_combat_dive_pullup;//俯冲拉起指令
	
	//机动标识
	bool m_flag_launch_turn;//发射(扇面)转弯
	bool m_flag_alltitude_change;//高度机动
	bool m_flag_waypoint_turn;	//航迹转弯机动
	bool m_flag_alltitude_climb;//爬升
	bool m_flag_alltitude_decline;//下降
};

#endif

//待补充，地速控制，用于编队控制；