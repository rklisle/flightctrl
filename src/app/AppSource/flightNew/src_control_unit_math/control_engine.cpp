#include <cstring>
#include "control_engine.h"
#include "../data_protocol.h"
#include "../global_function.h"
//#include "../../timer.h"

CMathControlEngine::CMathControlEngine()
{
	//p_st_debug_monitor = NULL;
	p_st_engine_control_input = NULL;
	p_st_engine_control_output = NULL;
	m_p = 0.0;
	m_p0 = 0.0;
	m_p1 = 0.0;
	m_p2 = 0.0;
	m_p3 = 0.0;
	m_dltKc_time = 0.0;
	m_dltVg_time = 0.0;
	m_Kc0 = KC_COMMAND_MIN_LIMIT;
	m_Kc1 = 0;
	m_Kc1_record = 0.0;
	m_Kc2 = 0;
	m_Kc2_record = 0.0;
	m_Kc3 = 0;
	m_Kc4 = 0;
	m_control_Kc = KC_COMMAND_MIN_LIMIT;
	m_count_vel_pidctrl = 0;

	//m_state_rpm = 0.0;
	m_fuel_comsumped = 0.0;
	m_mass_calc = 165.0;
	m_target_velocity = 0.0;
	m_missile_velocity = 0.0;
	m_missile_height = 0.0;
	m_missile_height_initial = 0.0;
	m_Vsonic = 340.0;
	m_target_time = 0.0;
	//m_temperature_ground = 0.0;
	m_turn_radius = 0.0;
	m_velocity_integrator = 0.0;
	m_flag_velocity_control = false;
	m_flag_flightime_ctrl = false;
	m_flag_launch_turn = false;
	m_flag_alltitude_change = false;
	m_flag_waypoint_turn = false;
	m_flag_alltitude_climb = false;
	m_flag_alltitude_decline = false;
	m_flag_combat_status = false;
	m_flag_combat_dive_pullup = false;

	m_flag_engine_start = false;//ɵټٵ
	m_flag_missile_takeoff = false;//ɣʼٶȿ
	m_flag_engine_stop = false;//ػָΪ
}
void CMathControlEngine::Initial()
{
	
}

void CMathControlEngine::Get_Data()
{
	m_mass_calc = p_st_engine_control_input->mass;
	
	m_missile_height = p_st_engine_control_input->h;	//и߶
	m_missile_height_initial = p_st_engine_control_input->h_ini;
	//m_temperature_ground = p_st_engine_control_input->temperature_ground;
	
	m_target_velocity = p_st_engine_control_input->target_velocity;//Ŀٶ
	m_target_time = p_st_engine_control_input->target_time;	//ʱ
	
	m_Vsonic = p_st_engine_control_input->sonic_speed;//
	m_mach = p_st_engine_control_input->mach;//
	m_air_velocity = m_mach*m_Vsonic;//

	m_turn_radius = p_st_engine_control_input->radius_zw;	
	m_flag_launch_turn = p_st_engine_control_input->flag_launch_turn;		//ת
	m_flag_waypoint_turn = p_st_engine_control_input->flag_waypoint_turn;	//ת
	m_flag_alltitude_change = p_st_engine_control_input->flag_alltitude_change;//߶ 
	m_flag_alltitude_climb = p_st_engine_control_input->flag_alltitude_climb;	//
	m_flag_alltitude_decline = p_st_engine_control_input->flag_alltitude_decline;//½
	
	m_flag_velocity_control = p_st_engine_control_input->flag_velocity_control;//ٶȿƱʶ1ٿƣδõ
	m_missile_velocity = p_st_engine_control_input->missile_average_velocity;//
	m_flag_flightime_ctrl = p_st_engine_control_input->flag_flightime_ctrl;	//ʱʶ
	
	m_flag_engine_start = p_st_engine_control_input->flag_engine_start;
	m_flag_engine_stop = p_st_engine_control_input->flag_engine_stop;
	m_flag_missile_takeoff = p_st_engine_control_input->flag_missile_takeoff;
	m_flag_altitude_control = p_st_engine_control_input->flag_altitude_control;
	m_flag_combat_status = p_st_engine_control_input->flag_combat_status;
	m_flag_combat_dive_pullup = p_st_engine_control_input->flag_combat_dive_pullup;

}

void CMathControlEngine::Run()
{
	Get_Data();
	

	//1sԼ3.0s
	if(!m_flag_engine_start)
	{
		m_control_Kc = KC_COMMAND_MIN_LIMIT;
	}
	//1.0s(Լ10.0s)->70%
	//˵10degǣ50m/sٶȣԤʣṦԼ27kW70%Դڼ״̬
	else if(!m_flag_missile_takeoff)
	{
		m_control_Kc = KC_COMMAND_CRUISE;
	}
	//ȶĩƵػתٿƣҷſ
	else if(!m_flag_engine_stop)
	{
		//20260718 ޸ģȥ֪
		
		//̣ͻת
		if(m_flag_combat_status == true)
		//if(m_missile_velocity > 60.0)
		{
			m_control_Kc = KC_COMMAND_MIN_LIMIT + 20.0;
		}
		//˳̣ת
		else
		{
			Calc_Data_Kc();
		}		
	}
	//ػָΪ
	else
	{
		m_control_Kc = 0.0;
	}
	
	Send_Data();

	//Monitor_Data();
}




//ٶȿƣ
void CMathControlEngine::Calc_Data_Kc()
{
	double target_velocity = VEL_COMMAND_CRUISE;//ֵ
	double dlt_air_velocity_comp = 0.0;
	double dlt_velocity = 0.0;
	double dlt_velocity_cmd = 0.0;
	double target_velocity_cmd = VEL_COMMAND_CRUISE;
	
	//תƼ
	double kp = 8.0;//10.0;///2.0;
	double ki = 0.8;///1.0;///0.2;
	double m_array[3] = {100.0, 133.0, 165.0};//{100.0, 125.0, 150.0}
	double V_array[4] = {40.0, 50.0, 60.0, 70.0};
	double h_array[6] = {0, 1000, 2000, 3000, 4000, 5000};
	double Kc0_matrix[3][6] = {{71.6599, 72.3680, 73.0528, 74.0607, 75.2552, 77.1943},
						{73.3871, 74.2679, 75.5284, 77.4114, 79.2842, 82.0228},
						{75.4301, 77.0857, 78.8006, 81.2730, 86.1428, 93.9886}};

	//ݵǰ߶ȣֵ׼
	m_Kc0 = CFlightGlobalFun::LAQL2(3,  6,  m_array,  h_array, &Kc0_matrix[0][0], m_mass_calc, m_missile_height);
	m_Kc0 = m_Kc0 - 5.0;
	
	//ת䲹
	double gama_array[5] = {10, 20, 30, 45, 60};
	double Kc1_matrix[4][5] = {{0.374960938, 1.597629353, 4.019899216, 12.05905532, 36.17264967},    
								{0.239975,     1.022482786, 2.572735498, 7.717795407, 23.15049579},    
								{0.166649306,  0.71005749,  1.786621874, 5.359580144, 16.07673319},    
								{0.122436225,  0.521674891, 1.312620152, 3.937650718, 11.81147744}};
	//ת뾶Ϳ٣Ʊƹת
	double temp_gama = 20.0;
	double temmp_az = m_air_velocity*m_air_velocity/CFlightGlobalFun::Nozero_FUN(m_turn_radius);
	temp_gama = RTOA*atan(temmp_az/9.8);//ȡֵΧ-90~90deg
	temp_gama = fabs(temp_gama);
	//תСٶвֵóŲ
	double temp_Kc1 = 10.0;
	temp_Kc1 = CFlightGlobalFun::LAQL2(3,  6,  V_array,  gama_array, &Kc1_matrix[0][0], m_air_velocity, temp_gama);
	
	//ݸ߶Ȼȡܶȣһ㲹ϵ
	double rho_array[6] = {1.2252, 1.1119, 1.0067, 0.9095, 0.8195, 0.7366};
	double temp_Kc_rho_factor = 1.0;
	double temp_rho = 1.0;
	temp_rho = CFlightGlobalFun::LAQL1(6,  h_array,  rho_array, m_missile_height);
	temp_Kc_rho_factor = 1/temp_rho;
	temp_Kc_rho_factor = CFlightGlobalFun::Range2(temp_Kc_rho_factor, 0.8, 1.36);

	//ݹ㣬㲹ϵ
	double temp_Kc_mass_factor = 1.0;
	temp_Kc_mass_factor = (m_mass_calc/133.0)*(m_mass_calc/133.0);
	temp_Kc_mass_factor = CFlightGlobalFun::Range2(temp_Kc_mass_factor, 0.59, 1.54);

	//ܶϵ߶ȶӦϵŲ
	temp_Kc1 = temp_Kc1*temp_Kc_rho_factor*temp_Kc_mass_factor;
	
	//5.7degǣŲƵ30%
	double temp_Kc2 = 25.0;
	
	//ǰֻпٿ
	//ǰָٴٶȣٶ
	//ǰָССٶȣСٶ
	if(m_target_velocity > VEL_COMMAND_MAX_LIMIT)
	{
		target_velocity = VEL_COMMAND_MAX_LIMIT;
	}
	else if(m_target_velocity < VEL_COMMAND_MIN_LIMIT)
	{
		target_velocity = VEL_COMMAND_MIN_LIMIT;
	}
	else
	{
		target_velocity = m_target_velocity;
	}

	//ת̲ת
	if (m_flag_waypoint_turn || m_flag_launch_turn)
	{
		m_Kc1 = temp_Kc1;//20.0;
	} 
	else
	{
		m_Kc1 = 0.0;
	}

	//߶Ȼת
	if(!m_flag_altitude_control)
	{
		//߶ȿǰڳʼ	
		m_Kc2 = temp_Kc2;
	}
	else if (m_flag_alltitude_change)
	{
		//
		if (m_flag_alltitude_climb)
		{
			m_Kc2 = temp_Kc2;//30;
		}
		//»
		if (m_flag_alltitude_decline)
		{
			m_Kc2 = 0.0;
		}
	} 
	//޸߶Ȼ
	else
	{
		m_Kc2 = 0.0;
	}

	//ֵС2.5m/sʱָ Ӧֵ
	if(m_flag_flightime_ctrl == 1)
	{
		//һ10sжһΣ
		//˵ĿĿٶȲ٣ʹõĿٶ
		if(flight_time > m_dltVg_time + 10.0)
		{
			m_dltVg_time = flight_time;

			//ٲ٣ﵽʱĿ
			dlt_air_velocity_comp = target_velocity - m_missile_velocity;
		}
		//ʱƣ = ɾ/ʱ䣬Уʱ = (ʱ - ǰʱ)
		if(target_velocity + dlt_air_velocity_comp < 40.0)
		{
			dlt_velocity = 40.0 - m_air_velocity;//ټƫ
		}
		else if(target_velocity + dlt_air_velocity_comp > 60.0)
		{
			dlt_velocity = 60.0- m_air_velocity;//ټƫ
		}
		else
		{
			dlt_velocity = target_velocity - m_air_velocity + dlt_air_velocity_comp;//ټƫ
		}
		
		//ٿ
		//dlt_velocity = target_velocity - m_missile_velocity;
		//޷
	}
	else
	{
		dlt_velocity = target_velocity - m_air_velocity;
	}

	if(fabs(m_Kc1 - m_Kc1_record) > 10.0 || fabs(m_Kc2 - m_Kc2_record) > 10.0)
	{
		m_Kc3 = 0.0;//Χ¹
		m_count_vel_pidctrl = 0;

		//һʱٽСٶȿơ
		m_dltKc_time = flight_time + 2.5;
	}
	//ǰһָ֡
	m_Kc1_record = m_Kc1;
	m_Kc2_record = m_Kc2;
	
	if(flight_time > m_dltKc_time + 2.5)
	{
		m_dltKc_time = flight_time;

		if(dlt_velocity > 5.0)
		{
			m_Kc3 = m_Kc3 + 5.0;

			//PI
			m_count_vel_pidctrl = 0;
		}
		else if(dlt_velocity < -5.0)
		{
			m_Kc3 = m_Kc3 - 5.0;

			//PI
			m_count_vel_pidctrl = 0;
		}
		else
		{
			//״㣬1%ţٶ1m/sʣ֮ڽPI
			//ŻΪٶȿƿ
			if(m_count_vel_pidctrl == 0)
			{
				m_Kc3 += dlt_velocity*1.0;
			}
			//m_Kc3֮ǰֲֵ
			m_count_vel_pidctrl++;
		}
	}
	
	//˵»ǷԲPI
	if(m_count_vel_pidctrl > 2)
	{
		//ջPI
		m_velocity_integrator +=dlt_velocity * STEP_5ms;
		m_velocity_integrator = CFlightGlobalFun::Range(m_velocity_integrator, 10.0/ki);
		m_Kc4 = kp * dlt_velocity + ki * m_velocity_integrator;

		m_Kc4 = CFlightGlobalFun::Range(m_Kc4, 20.0);
	}
	else
	{
		//PIΪ㣬Ϊ
		m_velocity_integrator = 0.0;
		m_Kc4 = 0.0;
	}	
	
	//ת
	m_control_Kc = m_Kc0 + m_Kc1 + m_Kc2 + m_Kc3 + m_Kc4;

	//תٿ
	if(m_control_Kc < KC_COMMAND_MIN_LIMIT)
		m_control_Kc = KC_COMMAND_MIN_LIMIT; 
	if(m_control_Kc > KC_COMMAND_MAX_LIMIT)
		m_control_Kc = KC_COMMAND_MAX_LIMIT; 
}


void CMathControlEngine::Send_Data()
{
	p_st_engine_control_output->control_Kc = m_control_Kc;
}

//
/*
void CMathControlEngine::Monitor_Data()
{
	extern CSimMonitor sim_monitor;
	
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_target_velocity,"Vcx",ENUM_FILE_CONTROL1);	//ٶָ
		sim_monitor.Get_Variable(m_air_velocity,"Vr_air",ENUM_FILE_CONTROL1);	//
		sim_monitor.Get_Variable(m_missile_velocity,"dhcx",ENUM_FILE_CONTROL1);//
		sim_monitor.Get_Variable(m_control_Kc,"Kc",ENUM_FILE_CONTROL1);		//
	}
}
*/
