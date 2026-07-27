#ifndef _CONTROL_FLIGHT_BASIC_H_
#define _CONTROL_FLIGHT_BASIC_H_
#include "control_roll.h"
#include "control_engine.h"
#include "../data_protocol.h"

typedef struct  _Stru_Command
{
	bool flag_separate_booster;
	bool flag_launch_missile_wing;
	bool flag_engine_start;
	bool flag_seeker_on;
	bool flag_lock_on_permit;
	bool flag_target_lock;

	bool flag_missile_takeoff;
	bool flag_altitude_control;
	bool flag_combat_status;
	bool flag_fuze_unlock;
	bool flag_combat_dive_pullup;
	bool flag_combat_dive_sidectrl;
	bool flag_engine_shutdown;
	bool flag_open_umbrella;
}Stru_Command;

#define BIT_flag_t0                    (1U << 0)
#define BIT_flag_control_on            (1U << 1)
#define BIT_flag_separate_booster      (1U << 2)
#define BIT_flag_engine_start          (1U << 3)
#define BIT_flag_seeker_on             (1U << 4)
#define BIT_flag_initial_hz            (1U << 5)
#define BIT_flag_missile_takeoff       (1U << 6)
#define BIT_flag_launch_turn           (1U << 7)
#define BIT_flag_altitude_control      (1U << 8)
#define BIT_flag_lock_on_permit        (1U << 9)
#define BIT_flag_cooperative_attack    (1U << 10)
#define BIT_flag_combat_status         (1U << 11)
#define BIT_flag_fuze_unlock           (1U << 12)
#define BIT_flag_combat_dive_ok_set    (1U << 13)
#define BIT_flag_combat_dive_sidectrl  (1U << 14)
#define BIT_flag_combat_dive_pullup    (1U << 15)
#define BIT_flag_engine_shutdown       (1U << 16)
#define BIT_flag_open_umbrella         (1U << 17)
#define BIT_flag_alltitude_change      (1U << 18)
#define BIT_flag_alltitude_climb       (1U << 19)
#define BIT_flag_alltitude_decline     (1U << 20)
#define BIT_flag_waypoint_turn         (1U << 21)
#define BIT_flag_turn_out_set          (1U << 22)
#define BIT_flag_flightime_ctrl        (1U << 23)
#define BIT_flag_velocity_control      (1U << 24)
#define BIT_flag_heading_hold          (1U << 25)
#define BIT_flag_turndir_set           (1U << 26)
#define BIT_flag_prepare_hover         (1U << 27)
#define BIT_flag_relativehigh_ctrl     (1U << 28)
#define BIT_flag_attackangle_ctrl      (1U << 29)

typedef struct  _Stru_Control_Time
{
	double time_control;
	double time_separate_booster;
	double time_launch_missile_wing;
	double time_seeker_on;
	double time_engine_start;
	double time_engine_start_finish;
	double time_missile_takeoff;
	double time_altitude_control;
	double time_launch_turn;
	double time_launch_turn_ok;
	double time_cooperative_attack;
	double time_combat_status;

	double time_combat_dive_ok;
	double time_combat_dive_sidectrl;
	double time_combat_dive_pullup;
	
	double time_altitude_change_start;
	double time_altitude_change_end;
	double time_turn_in_start;
	double time_turn_in_end;
	double time_turn_out_start;
	double time_turn_out_end;
	
	double time_turn_in_minimum;
	double time_arrive_minimum;

	double time_engine_shutdown;
	double time_open_umbrella;
	double time_fuze_unlock;
	double time_touch_ground;
}Stru_Control_Time;

typedef struct  _Stru_Control_flag
{
	bool flag_t0;
	
	bool flag_control_set;
	bool flag_control_on;
	bool flag_separate_booster_set;
	bool flag_separate_booster;
	bool flag_launch_missile_wing_set;
	bool flag_launch_missile_wing;
	bool flag_engine_start_set;
	bool flag_engine_start;
	
	bool flag_engine_start_finish_set;
	
	bool flag_launch_turn_set;
	bool flag_launch_turn;
	bool flag_altitude_control_set;
	bool flag_altitude_control;
	
	bool flag_seeker_on_set;		
	bool flag_seeker_on;
	bool flag_lock_on_permit;
	bool flag_cooperative_attack_set;
	bool flag_cooperative_attack;
	bool flag_combat_status_set;
	bool flag_combat_status;
	bool flag_target_lock;

	bool flag_combat_dive_ok_set;
	bool flag_combat_dive_sidectrl;
	bool flag_combat_dive_pullup;

	bool flag_alltitude_change;
	bool flag_alltitude_climb;
	bool flag_alltitude_decline;
	
	bool flag_waypoint_turn;
	bool flag_turn_out_set;
	bool flag_flightime_ctrl;
	bool flag_velocity_control;

	bool flag_heading_hold;
	bool flag_turndir_set;
	bool flag_prepare_hover;
	
	bool flag_relativehigh_ctrl;
	bool flag_attackangle_ctrl;				
	
	bool flag_initial_hz;
	bool flag_missile_takeoff;
	bool flag_engine_shutdown;
	bool flag_open_umbrella;
	bool flag_fuze_unlock;
}Stru_Control_flag;

typedef struct  _Stru_Flight_Basic_Input
{
	int missile_ID;
	double engine_cmd_rpm;
	double engine_cmd_Kc;
	
	Stru_Data_RadioAlt_To_Controller st_radioalt_data;
	Stru_Data_Baro_To_Controller st_baro_data;
	Stru_Data_Engine_To_Controller st_engine_data;
	Stru_Data_INS_To_Controller	st_ins_data;
	Stru_Data_Seeker_To_Controller	st_seeker_data;
	Stru_Data_Datalink_To_ControllerSig	st_datalink_datasig;
}Stru_Flight_Basic_Input;

typedef struct  _Stru_Flight_Basic_Output
{
	double mass_calc;
	double hz;
	double h_ini;
	double nby;
	double nbz;
	double v;
	double vs;
	double vnx;
	double vnz;
	double mach;
	double sonic_speed;
	double sz;
	double zeta;
	double gama;
	double wx;
	double wy;
	double wz;
	double g;
	double dqf;
	double dqh;	
	double qf;
	double qh;
	double time_to_go;
	double phif;
	double phih;	
	int target_num__choosen;
	double gama_command;
	double ny_command;
	double nz_command;

	double angle_zw;
	double radius_zw;
	double target_velocity;
	double target_time;
	double target_height;
	double dynamic_pressure;
	int num_way_point_target;
	double target_long;
	double target_lat;
	double target_distance;
	unsigned int flight_control_state;
	unsigned int rpmState;
	
	double sz_circle;
	double azimuth;
	double psicn;
	double theta;
	double alpha_vg;
	double beita_vg;

	double ground_temperature;
	double ktheta_lauch_enc;
	double ktheta_climb_enc;
	double ktheta_hight_enc;

	double longitude;
	double latitude;
	double Rmt_n[3];
	double distance_target;
	double distance_target_t_combat;
	double gama_turn_nominal;
	double velocity_average_10s;
	int ECU_work_cmd;
	int	count_altitude_change;
	
	bool flag_altitude_change;
	bool flag_altitude_climb;
	bool flag_altitude_decline;
	bool flag_launch_turn;
	bool flag_waypoint_turn;
	
	bool flag_flightime_ctrl;
	bool flag_velocity_control;

	bool flag_heading_hold;
	bool flag_turndir_set;
	bool flag_prepare_hover;
	
	bool flag_relativehigh_ctrl;
	bool flag_attackangle_ctrl;	

	bool flag_fire_distribution;
	Stru_Command st_command;
	Stru_Control_Time st_control_time;
}Stru_Flight_Basic_Output;

typedef struct _Stru_Tustin_FirstIO_Filter
{
	double inputdata[2];
	double outputdata[2];
	double T;
	double Ts;
} Stru_Tustin_FirstIO_Filter;

class CMathControlFlightBasic
{
public:
	CMathControlFlightBasic(); 
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor					* p_st_debug_monitor;
	Stru_Route_Data						* p_st_route_data_preflight;
	Stru_Initial_Data					* p_st_initial_data;
	Stru_Flight_Basic_Input				* p_st_flight_basic_input;
	Stru_Flight_Basic_Output			* p_st_flight_basic_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	//void Monitor_Data();
	void Calc_Command();	
	void PackControlFlag(const Stru_Control_flag* pSt);
	void Calc_Flight_Data();
	void Calc_Mass_Data();
	void Calc_LOS_Rate();
	void Calc_Hz();
	void Calc_BaroHigh();
	void Calc_BaroSpd();
	void Control_Turn();
	void Control_Altitude_Change();
	void Calc_Dualplane_Guidance();
	void Coord_Rebuild();
	void Update_Task_Info();
	void Change_Task_Info_Online();
	bool Judge_Turn_Error();
	
	int m_missile_ID;
	int m_missile_flight_mode;
	double m_ground_temperature;
	double m_ktheta_lauch_enc;
	double m_ktheta_climb_enc;
	double m_ktheta_hight_enc;
	
	int m_num_way_point;
	int m_num_way_point_target;
	Stru_Way_Point m_st_way_point[MAX_ROUTE_NUMBER];
	Stru_Way_Point m_st_target;
	
	int valid_count;
	int count_qk;
	int count_v_5;
	int count_fl;
	int count_qd;
	int count_v_50;
	int count_takeoff;
	int count_tg;
	double dlt_time_tg;
	int count_cooperative_attack;
	int count_virtual;
	int count_combat_dive_ok;
	int count_distance_recycle;
	int count_v50_recycle;
	int count_h300_v54_recycle;
	int count_distance_out_recycle;
	int count_v54_recycle;
	int count_h250_recycle;
	int count_distance_out600_recycle;
	int count_h10_recycle;
	int count_h5_ny2_recycle;

	int count_altitude_change;
	int count_altitude_change_enable;
	int count_altitude_change_lauch_enable;
	int count_altitude_change_energy_enable;
	int count_altitude_change_end;
	int count_away;
	int count_sd_in;
	int count_turn_out;
	int count_turn_error;
	int count_update;

	int step_open_umbrella;
	int step_altitude_change_lauch;
	int step_dualplane_guidance;
	
	Stru_Control_flag m_st_control_flag;
	Stru_Control_Time m_st_control_time;
	unsigned int m_flight_control_state;
	
	double m_hgps;
	double m_longitude;
	double m_latitude;
	double m_ax;
	double m_ay;
	double m_az;
	double m_wx;
	double m_wy;
	double m_wz;
	double m_vtx;
	double m_vty;
	double m_vtz;
	double m_zeta;
	double m_gama;
	double m_psit;
	
	int count_ax_tyz;
	int count_ay_tyz;
	int count_az_tyz;
	int count_wx_tyz;
	int count_wy_tyz;
	int count_wz_tyz;
	double m_axtyz;
	double m_aytyz;
	double m_aztyz;
	double m_wxtyz;
	double m_wytyz;
	double m_wztyz;	
	Stru_Tustin_FirstIO_Filter m_nav_data_filter[6];
	double m_axflt;
	double m_ayflt;
	double m_azflt;
	double m_wxflt;
	double m_wyflt;
	double m_wzflt;	
	double m_A;
	double m_psin;
	double m_psicn;
	double m_theta;
	double m_Cbn[9];
	double m_Cnb[9];
	double m_vx;
	double m_sx;
	double m_v;
	double m_vs;
	double m_hz;
	double m_vnx;
	double m_vnz;
	double m_vnz_record1;
	double m_vnz_record2;
	double m_ny;
	double m_nz;
	double m_g;	
	double m_anx;
	double m_au;
	double m_anz;
	
	double m_static_pressure_raw;
	double m_total_pressure_raw;
	int count_static_pressure_tyz;
	int count_total_pressure_tyz;
	double m_static_pressure;
	double m_total_pressure;
	double m_static_pressure_flt;
	double m_total_pressure_flt;
	Stru_Tustin_FirstIO_Filter m_baro_data_filter[2];
	int m_baroalt_status;
	double m_hbaro;
	double m_Vbaro;
	double m_v_air;
	double m_dynamic_pressure;
	double m_mach;
	double m_v_average_1s;
	double m_v_average_10s;
	double m_v_record_100ms[10];
	double m_v_record_1s[10];
	double m_air_temperature;
	double m_air_density;
	double m_air_pressure;
	double m_sonic_speed;
	
	double m_total_energy;
	double m_total_energy_pre;

	double m_radioalt_hight;
	int m_radioalt_status;

	int m_engine_state;
	double m_engine_state_rpm;
	int m_ECU_work_cmd;
	double m_cmd_Kc;
	double m_cmd_rpm;
	double m_state_rpm;
	double m_fuel_comsumped;
	double m_mass_calc;
	double m_left_flight_time_calc;
	double m_left_flight_dist_calc;

	double m_seeker_cmdpara_targettype;
	double m_seeker_cmdpara_dltheight;
	double m_seeker_cmdpara_ktheta;
	double m_seeker_cmdpara_psi;
	double m_seeker_cmdpara_gama;
	double m_seeker_cmdpara_phif;
	double m_seeker_cmdpara_phih;

	double m_seeker_state_track;
	double m_seeker_dqf;
	double m_seeker_dqh;
	double m_seeker_phif;
	double m_seeker_phih;
	double m_seeker_qf;
	double m_seeker_qh;
	int m_seeker_pixelf;
	int m_seeker_pixelh;
	double m_seeker_distance_target;

	int m_target_num__choosen;
	double m_dqf;
	double m_dqh;
	double m_dqf_flt;
	double m_dqh_flt;
	Stru_Tustin_FirstIO_Filter m_guide_data_filter[2];
	double m_phif;
	double m_phih;
	double m_Qf;
	double m_Qh;
	double m_Qn;
	double m_time_to_go;
	double m_ny_command;
	double m_nz_command;
	double m_gama_command;
	double m_gama_command_record;
	
	double m_alpha_target;
	double m_distance_target;
	double m_slant_distance_target;
	double m_Rmt_n[3];
	
	double m_distance_target_t_combat;
	double m_gama_target_t_combat;
	
	double m_total_distance;
	double m_alpha_AB;
	double m_longitude_A;
	double m_latitude_A;
	double m_longitude_B;
	double m_latitude_B;
	double m_longitude_C;
	double m_latitude_C;
	double m_distance_BP;
	double m_distance_BP_projection;
	double m_distance_BP_500;
	double m_distance_BP_500pre;
	
	double m_sz;
	double m_sz_record1;
	double m_sz_record2;
	double m_sz_radius;
	
	int m_route_mode;
	int m_formation_mode;
	double m_turn_angle;
	double m_turn_radius;
	double m_accept_radius;
	double m_target_velocity;
	double m_target_height;
	double m_gama_turn_nominal;
	double m_x_coordinate_turn;
	double m_z_coordinate_turn;
	double m_distance_turn_in_compensate;
	double m_distance_turn_in;
	double m_psit_t_turn_in;

	double m_outtrack_angle;
	int m_hover_round;
	double m_attack_angle;
	double m_target_height_ground;
};

#endif