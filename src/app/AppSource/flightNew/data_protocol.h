#ifndef _DATA_PROTOCOL_H_
#define _DATA_PROTOCOL_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 定义仿真所需各模块间通讯数据内容
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "..\load_data.h"
//#include "..\datalink_sim_main.h"
//#include "..\target_sim_main.h"
#include "port/flightPort.h"

//***************** 外部仿真控制给导弹输入数据 ********************//
typedef struct _Stru_Jamming_Data_In
{
	double temperature_environment;
	int    flag_wind;
	double velocity_wind;
	double psi_wind;
	double theta_wind;
	double lp_pitch;
	double lp_yaw;
	double lp_roll;
	double lp_lift;
	double lp_drag;
	double lp_side;
	double lp_wx;
	double lp_wy;
	double lp_wz;
	double lp_rotary_inertia;
	int    flag_jggr;
	double det_mass;
	double det_x_centroid;
	double det_y_centroid;
	double det_z_centroid;
	double gama0;	//初始姿态偏差
	double zeta0;
	double psi0;
	double wxerr;	//角速度偏差
	double wyerr;
	double wzerr;
	double lp_dx;//舵效拉偏
	double lp_dy;
	double lp_dz;

	double Lp_trust_det;//推力拉偏
	double Lp_eng_flowvol;//耗油率或流量拉偏
	double trust_det_pos;
	double trust_det_angle;
	double trust_det_alpha;
	double trust_det_gama;
		
}Stru_Jamming_Data_In;	//导弹飞行干扰条件数据

typedef struct _Stru_Mission_Data_In
{
	Stru_Jamming_Data_In st_jamming_data_in;
	Stru_Initial_Data *	p_st_initial_data;
	Stru_Route_Data	  *	p_st_route_data;
}Stru_Mission_Data_In;	//任务数据

//****************** 弹体内部各设备间通讯数据 *********************//

//空速管给综控机数据包
typedef struct _Stru_Data_Baro_To_Controller  
{
	double static_pressure; //静压传感器输出
	double   total_pressure;	 //总压传感器输出
}Stru_Data_Baro_To_Controller;	//空速管给综控机数据包	

//无线电高度表给综控机数据包
typedef struct _Stru_Data_RadioAlt_To_Controller  
{
	int radioalt_status; //工作状态
	double radioalt_hight;//相对高度，也称真高度
}Stru_Data_RadioAlt_To_Controller;	//无线电高度表给综控机数据包	

typedef struct _Stru_Data_Seeker_To_Controller  
{
	bool   flag_combat_status;//稳定跟踪且锁定目标，标识
	bool   flag_seize_stable;	//光轴稳定，标识
	double pitch_LOS_rate;		//角速率
	double yaw_LOS_rate;	
	double pitch_gimbal_angle;//框架角
	double yaw_gimbal_angle;
	double longitude_target;	//目标位置
	double latitude_target;
	double distance_target;
	double pitch_LOS_angle;	//失准角
	double yaw_LOS_angle;
}Stru_Data_Seeker_To_Controller;	//导引头给综控机数据包

typedef struct _Stru_Data_INS_To_Controller  
{
	double gama;//正欧拉角：北天东地理系，经过231转序，前上右弹体系
	double psi;
	double zeta;
	double gamas;//反欧拉角：321转序
	double psis;
	double zetas;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;//北天东地理系
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
	int    GPS_status;
}Stru_Data_INS_To_Controller;	//惯导给综控机数据包

typedef struct _Stru_Data_Datalink_To_Controller  
{
	//在线航迹装订参数
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];
	//编队控制临弹状态参数
	Stru_Missile_State_Data		st_missile_state_data[MAX_CONNECT_NUMBER];
	//态势构建目标参数
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];
}Stru_Data_Datalink_To_Controller;	//数据链给综控机数据包 内容应与Stru_Data_Send_To_Missile保持一致，同步更新

typedef struct _Stru_Data_Engine_To_Controller  
{
	double rpm_engine;
	int ECU_work_status;	//工作状态
	int engine_start_result;//启动状态0x55启动过程中，0xAA启动完成，0xFF启动异常
}Stru_Data_Engine_To_Controller;	//发动机给综控机数据包

typedef struct _Stru_Data_Controller_To_Seeker  
{
	bool flag_seeker_on;		//导引头开机
	bool flag_lock_on_permit;//目标锁定允许
	double pitch_gimbal_angle_calc;	//俯仰框架角指令
	double yaw_gimbal_angle_calc;	//航向框架角指令
}Stru_Data_Controller_To_Seeker;	//综控机给导引头数据包

typedef struct _Stru_Data_Controller_To_Datalink  
{
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];//集群状态，数据链
	Stru_Missile_State_Data		st_missile_state_data;//导弹状态
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];//目标状态
}Stru_Data_Controller_To_Datalink;	//综控机给数据链数据包

typedef struct _Stru_Data_Controller_To_Actuator  
{
	double control_voltage_I;
	double control_voltage_II;
	double control_voltage_III;
	double control_voltage_IV;
	double control_voltage_V;
	double control_voltage_VI;
}Stru_Data_Controller_To_Actuator;	//综控机给舵数据包

typedef struct _Stru_Data_Controller_To_Engine  
{
	double control_Kc;	//指令油门，二选一
	double control_rpm;//指令转速，二选一
	int ECU_work_cmd;	//控制指令0x00 无指令，0x11 自检，0x22 启动，0x33 转速控制,  0x44 关机
}Stru_Data_Controller_To_Engine;	//综控机给发动机数据包

typedef struct _Stru_Data_Controller_To_Switch_Output  
{
	bool flag_separate_booster;		//助推器分离(爆炸螺栓或切割锁点火)
	bool flag_launch_missile_wing;	//弹翼展开(爆炸螺栓点火)
	bool flag_engine_start;			//主发动机点火(空中炮起点火)
}Stru_Data_Controller_To_Switch_Output;	//综控机开关量指令，未用到


//******************* 弹体与环境间交换数据 ************************//
typedef struct _Stru_Rudder_Reflection
{
	double missile_rudder_delta_I;
	double missile_rudder_delta_II;
	double missile_rudder_delta_III;
	double missile_rudder_delta_IV;
	double missile_rudder_delta_V;//航向舵
	double missile_rudder_delta_VI;
}Stru_Rudder_Reflection;	//舵偏输出

typedef struct _Stru_Data_Missile_To_Environment 
{
	double rpm_engine;//发动机转速
	Stru_Rudder_Reflection st_missile_rudder_reflection;//舵偏输出
	Stru_Data_Controller_To_Switch_Output st_missile_status_switch;//开关量输出
}Stru_Data_Missile_To_Environment;	//导弹给环境输出数据

typedef struct _Stru_INS_Data_In
{
	double gama;//正欧拉 231转序
	double psi;
	double zeta;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
}Stru_INS_Data_In;	//环境给惯导输入数据

typedef struct _Stru_RadioAlt_Data_In
{
	double radioalt_height;
}Stru_RadioAlt_Data_In;	//环境给无线电高度表输入数据

typedef struct _Stru_Baro_Data_In
{
	double static_pressure;
	double total_pressure;
}Stru_Baro_Data_In;	//环境给空速管输入数据

typedef struct _Stru_Seeker_Data_In
{
	int target_ID;
	double pitch_LOS_angle;
	double yaw_LOS_angle;
	double pitch_LOS_rate;
	double yaw_LOS_rate;
	double distance_target;
	double longitude_target;
	double latitude_target;
}Stru_Seeker_Data_In;	//导引头输入数据

typedef struct _Stru_Data_Environment_To_Missile
{
	//给惯导数据
	Stru_INS_Data_In st_data_environment_to_ins;
	//给高度表
	Stru_RadioAlt_Data_In st_data_environment_to_radioalt;
	//给空速管
	Stru_Baro_Data_In st_data_environment_to_baro;
	//给导引头数据
	Stru_Seeker_Data_In st_data_environment_to_seeker[MAX_TARGET_NUMBER];
}Stru_Data_Environment_To_Missile;	//环境给导弹输入数据

//****************** 环境内部各模块间交换数据 *********************//
typedef struct _Stru_Data_Earth_Model_Out 
{
	double gravitational_acceleration;
	double air_density;
	double air_temperature;
	double air_pressure;
	double sonic_speed;
	double velocity_wind;
	double psi_wind;
	double theta_wind;	
}Stru_Data_Earth_Model_Out;	//地球模型输出数据

typedef struct _Stru_Data_Earth_Model_In 
{
	double missile_height;
	double missile_longitude;
	double missile_latitude;
}Stru_Data_Earth_Model_In;	//地球模型输入数据

typedef struct _Stru_Data_Missile_Inertia 
{
	double mass;
	double x_centroid;
	double y_centroid;
	double z_centroid;
	double x_moment_of_inertia;
	double y_moment_of_inertia;
	double z_moment_of_inertia;
}Stru_Data_Missile_Inertia;	//导弹惯性数据

typedef struct _Stru_Data_Aerodynamic_Force 
{
	double lift_force;
	double drag_force;
	double side_force;
	double pitch_moment;
	double yaw_moment;
	double roll_moment;
}Stru_Data_Aerodynamic_Force;	//气动力数据

typedef struct _Stru_Data_Aerodynamic_Force_Coefficient
{
	double mxwx;
	double mywy;
	double mzwz;
	double dcy_dalfa;
	double dmz_dalfa;
	double dcz_dbeta;
	double dmy_dbeta;
	double dcy_ddeltaz;
	double dmz_ddeltaz;
	double dmx_ddeltax;
	double dcz_ddeltay;
	double dmy_ddeltay;
}Stru_Data_Aerodynamic_Force_Coefficient;	//气动力系数偏导数据

typedef struct _Stru_Data_Missile_Force 
{
	double gravity;
	double thrust;
	Stru_Data_Aerodynamic_Force st_aerodynamic_force;
}Stru_Data_Missile_Force;	//导弹受力数据

typedef struct _Stru_Data_Function_To_Force_Calc
{
	double mach;
	double air_speed;
	double sonic_speed;
	double air_density;
	double air_temperature;
	double height;
	double wx;
	double wy;
	double wz;
	double angle_of_attack;
	double angle_of_side_slip;
	double gravitational_acceleration;
	double aby;
}Stru_Data_Function_To_Force_Calc;	//导弹受力模型输入

typedef struct _Stru_Data_Function_Solved_Out
{
	double mach;
	double air_speed;
	double angle_of_attack;
	double angle_of_side_slip;
	bool flag_leaving_launcher;
	Stru_INS_Data_In * p_st_data_ins_related;
	Stru_Baro_Data_In * p_st_data_baro_related;
	Stru_RadioAlt_Data_In * p_st_data_radioalt_related;
	Stru_Data_Function_To_Force_Calc * p_st_data_function_to_force_calc;
}Stru_Data_Function_Solved_Out;	//方程解算输出

#endif
