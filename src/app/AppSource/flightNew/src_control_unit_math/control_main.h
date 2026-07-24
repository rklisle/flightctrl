#ifndef _CONTROL_MAIN_H_
#define _CONTROL_MAIN_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 数学仿真控制部分总控
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
#include "control_flight_basic.h"
#include "control_roll.h"
#include "control_yaw.h"
#include "control_pitch.h"
#include "control_engine.h"
#include "control_out.h"

typedef enum _MODULE_TYPE
{
	ENUM_FLIGHT_BASIC				= 0,
	ENUM_CONTROL_ROLL				= 1,
	ENUM_CONTROL_YAW				= 2,
	ENUM_CONTROL_PITCH			= 3,
	ENUM_CONTROL_ENGINE			= 4,
	ENUM_CONTROL_OUT				= 5
}MODULE_TYPE;

class CMathControlMain
{
public:
	CMathControlMain();
	void Initial();
	void Run();
	void Choose_Target_ID();
	double flight_time;
	int time_tick;
	int missile_ID;
	//Stru_Debug_Monitor				* p_st_debug_monitor;//调试/监控接口
	//输入
	Stru_Data_Seeker_To_Controller		* p_st_data_seeker_to_controller;//协议接口数据
	Stru_Data_INS_To_Controller			* p_st_data_ins_to_controller;
	//Stru_Data_Datalink_To_Controller	* p_st_data_datalink_to_controller;
	Stru_Data_Datalink_To_ControllerSig * p_st_data_datalink_to_controllersig;
	Stru_Data_Engine_To_Controller		* p_st_data_engine_to_controller;
	Stru_Data_Baro_To_Controller		* p_st_data_baro_to_controller;
	Stru_Data_RadioAlt_To_Controller	* p_st_data_radioalt_to_controller;
	//输出
	Stru_Data_Controller_To_Seeker		* p_st_data_controller_to_seeker;
	//Stru_Data_Controller_To_Datalink	* p_st_data_controller_to_datalink;
	Stru_Data_Controller_To_DatalinkTel	* p_st_data_controller_to_datalinktel;//数据链用于遥测
	Stru_Data_Controller_To_Engine		* p_st_data_controller_to_engine;//发动机
	Stru_Data_Controller_To_Actuator	* p_st_data_controller_to_actuator;//舵机
	Stru_Data_Controller_To_Switch_Output	* p_st_data_controller_to_switch_output;//输出三个标识
	
	Stru_Route_Data                       	* p_st_route_data_preflight;	//航迹装订，发射前预先装订
	Stru_Initial_Data                     	* p_st_initial_data;			//初始数据，一些列变量
private:
	void Update_Input_Data(MODULE_TYPE MODULE_NAME);
	void Update_Output_Data();
	CMathControlFlightBasic				m_math_control_flight_basic;
	CMathControlRoll					m_math_control_roll;
	CMathControlYaw						m_math_control_yaw;
	CMathControlPitch					m_math_control_pitch;
	CMathControlEngine					m_math_control_engine;
	CMathControlOut						m_math_control_out;
	
	Stru_Flight_Basic_Input				m_st_flight_basic_input;
	Stru_Flight_Basic_Output			m_st_flight_basic_output;
	Stru_Roll_Control_Input				m_st_roll_control_input;
	Stru_Roll_Control_Output			m_st_roll_control_output;
	Stru_Yaw_Control_Input				m_st_yaw_control_input;
	Stru_Yaw_Control_Output				m_st_yaw_control_output;
	Stru_Pitch_Control_Input			m_st_pitch_control_input;
	Stru_Pitch_Control_Output			m_st_pitch_control_output;
	Stru_Control_Out_Input				m_st_control_out_input;
	Stru_Control_Out_Output				m_st_control_out_output;
	Stru_Engine_Control_Input			m_st_engine_control_input;
	Stru_Engine_Control_Output			m_st_engine_control_output;
	int m_target_attack_ID;
};
#endif