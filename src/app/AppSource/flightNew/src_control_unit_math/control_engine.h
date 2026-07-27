#ifndef _CONTROL_ENGINE_H_
#define _CONTROL_ENGINE_H_
//#include "../../debug_monitor.h"

#define __VEL__CONTROL__MODE__KC__	 0

const double VEL_COMMAND_MAX_LIMIT = 60.0;
const double VEL_COMMAND_CRUISE = 50.0;
const double VEL_COMMAND_MIN_LIMIT = 40.0;

const double RPM_COMMAND_MAX_LIMIT = 7000;
const double RPM_COMMAND_CRUISE = 5380;
const double RPM_COMMAND_MIN_LIMIT = 2500;

const double KC_COMMAND_MAX_LIMIT = 100;
const double KC_COMMAND_CRUISE = 90;
const double KC_COMMAND_MIN_LIMIT = 30;

typedef struct  _Stru_Engine_Control_Input
{
	double h_ini;
	double h;
	double mass;
	double mach;
	double sonic_speed;
	double target_velocity;
	double target_time;
	double temperature_ground;
	double missile_average_velocity;
	double radius_zw;
	double velocity_command;

	bool flag_launch_turn;
	bool flag_waypoint_turn;
	bool flag_alltitude_change;
	bool flag_alltitude_climb;
	bool flag_alltitude_decline;
	
	bool flag_flightime_ctrl;
	bool flag_velocity_control;

	bool flag_engine_start;
	bool flag_missile_takeoff;
	bool flag_altitude_control;
	bool flag_engine_stop;
	bool flag_combat_status;
	bool flag_combat_dive_pullup;
}Stru_Engine_Control_Input;

typedef struct  _Stru_Engine_Control_Output
{
	double control_rpm;
	double control_Kc;
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
	void Calc_Data_Kc();

	void Send_Data();
	//void Monitor_Data();
	double m_p;
	double m_p0;
	double m_p1;
	double m_p2;
	double m_p3;

	double m_dltVg_time;
	double m_dltKc_time;
	
	double m_Kc0;
	double m_Kc1;
	double m_Kc1_record;
	double m_Kc2;
	double m_Kc2_record;
	double m_Kc3;
	double m_Kc4;

	double m_mach;
	double m_Vsonic;
	double m_air_velocity;
	double m_target_velocity;
	double m_missile_velocity;
	double m_target_time;
	int m_count_vel_pidctrl;
	
	double m_missile_height;
	double m_missile_height_initial;
	double m_fuel_comsumped;
	double m_mass_calc;
	double m_turn_radius;

	double m_control_Kc;

	double m_velocity_integrator;
	double m_velocity_ground;
	bool m_flag_velocity_control;
	
	bool m_flag_flightime_ctrl;

	bool m_flag_engine_start;
	bool m_flag_missile_takeoff;
	bool m_flag_altitude_control;
	bool m_flag_engine_stop;
	bool m_flag_combat_status;
	bool m_flag_combat_dive_pullup;
	
	bool m_flag_launch_turn;
	bool m_flag_alltitude_change;
	bool m_flag_waypoint_turn;
	bool m_flag_alltitude_climb;
	bool m_flag_alltitude_decline;
};

#endif