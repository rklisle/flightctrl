#include <cstring>
#include "control_engine.h"
#include "../global_function.h"
//#include "../../timer.h"

CMathControlEngine::CMathControlEngine()
{
	// p_st_debug_monitor = NULL;
	p_st_engine_control_input = NULL;
	p_st_engine_control_output = NULL;
	m_p = 0.0;
	m_p0 = 0.0;
	m_p1 = 0.0;
	m_p2 = 0.0;
	m_p3 = 0.0;
	//m_n0 = 0.0;
	//m_n1 = 0.0;
	//m_n2 = 0.0;
	//m_n3 = 0.0;
	m_dltKc_time = 0.0;
#ifdef __VEL__CONTROL__MODE__KC__
	m_Kc0 = 70.0;//满载150kg，起飞状态；
	m_Kc1 = 0;
	m_Kc2 = 0;
	m_Kc3 = 0;
	m_control_Kc = 70.0;
#else
	m_n0 = 5500;//满载150kg，起飞状态；
	m_n1 = 0;
	m_n2 = 0;
	m_n3 = 0;
	m_control_rpm = 5500;
#endif
	//m_state_rpm = 0.0;
	m_fuel_comsumped = 0.0;
	m_mass_calc = 150.0;
	m_target_velocity = 0.0;
	m_missile_velocity = 0.0;
	m_missile_height = 0.0;
	m_missile_height_initial = 0.0;
	m_Vsonic = 340.0;
	m_target_time = 0.0;
	m_temperature_ground = 0.0;
	m_turn_radius = 0.0;
	m_velocity_integrator = 0.0;
	m_flag_velocity_control = false;
	m_flag_launch_turn = false;
	m_flag_alltitude_change = false;
	m_flag_waypoint_turn = false;
	m_flag_alltitude_climb = false;
	m_flag_alltitude_decline = false;
}
void CMathControlEngine::Initial()
{

}
void CMathControlEngine::Run()
{
	Get_Data();
#ifdef __VEL__CONTROL__MODE__KC__
	Calc_Data_Kc();
#else
	Calc_Data_Rpm();
#endif
	//根据质量估计油门
	
	//Calc_Data();
	Calc_Data_Kc();
	Send_Data();
}

void CMathControlEngine::Get_Data()
{
	//根据高度计算空气密度比，一维插值
	double hight_array[7] = {0, 1000, 2000, 3000, 4000, 4500,5000};
	double Vsonic_array[7] = {340.3, 336.4, 332.5, 328.6, 324.6, 322.6 ,320.5};//高度，空气密度比

	m_mass_calc = p_st_engine_control_input->mass;
	m_mach = p_st_engine_control_input->mach;
	m_missile_height = p_st_engine_control_input->h;
	m_missile_height_initial = p_st_engine_control_input->h_ini;
	m_target_velocity = p_st_engine_control_input->target_velocity;
	m_missile_velocity = p_st_engine_control_input->missile_average_velocity;
	m_target_time = p_st_engine_control_input->target_time;
	m_temperature_ground = p_st_engine_control_input->temperature_ground;
	m_turn_radius = p_st_engine_control_input->radius_zw;
	
	m_flag_launch_turn = p_st_engine_control_input->flag_launch_turn;		//扇面转弯
	m_flag_alltitude_change = p_st_engine_control_input->flag_alltitude_change;//高度 机动
	m_flag_waypoint_turn = p_st_engine_control_input->flag_waypoint_turn;	//航迹转弯
	m_flag_alltitude_climb = p_st_engine_control_input->flag_alltitude_climb;	//爬升
	m_flag_alltitude_decline = p_st_engine_control_input->flag_alltitude_decline;//下降

	//根据高度插值计算声速
	m_Vsonic = CFlightGlobalFun::LAQL1(7,  hight_array,  Vsonic_array, m_missile_height);
	m_air_velocity = m_mach*m_Vsonic;//空速
}

#ifdef __VEL__CONTROL__MODE__KC__
void CMathControlEngine::Calc_Data_Kc()
{
	double target_velocity = VEL_COMMAND_CRUISE;//初值
	double dlt_velocity = 0.0;
	double dlt_velocity_cmd = 0.0;
	double target_velocity_cmd = VEL_COMMAND_CRUISE;
	
	//转速限制及推力当量
	double kp = 10;
	double ki = 1.3;
	double m_array[3] = {100.0, 125.0, 150.0};
	double h_array[6] = {0, 1000, 2000, 3000, 4000, 5000};
	double Kc0_matrix[3][6] = {{71.6599, 72.3680, 73.0528, 74.0607, 75.2552, 77.1943},
						{73.3871, 74.2679, 75.5284, 77.4114, 79.2842, 82.0228},
						{75.4301, 77.0857, 78.8006, 81.2730, 86.1428, 93.9886}};
	double kp_matrix[3][6] = {{2.4034, 3.0302, 3.7191, 4.4933, 5.2596, 6.0561},
							{3.9243, 4.6577, 5.3846, 6.1504, 6.9307, 7.7314},
							{5.2932, 5.9977, 6.7500, 7.5242, 8.5148, 9.7978}};
	double ki_matrix[3][6] = {{0.5465, 0.5939, 0.6455, 0.7047, 0.7624, 0.8237},
						{0.6616, 0.7171, 0.7719, 0.8309, 0.8899, 0.9518},
						{0.7655, 0.8192, 0.8761, 0.9359, 1.0223, 1.1421}};

	//当前只有空速控制
	//如果当前指令空速大于最大速度，等于最大速度
	//如果当前指令空速小于最小速度，等于最小速度
	if(m_target_velocity > VEL_COMMAND_MAX_LIMIT)
	{
		target_velocity = VEL_COMMAND_MAX_LIMIT;
	}
	else if(m_target_velocity > VEL_COMMAND_MIN_LIMIT)
	{
		target_velocity = VEL_COMMAND_MIN_LIMIT;
	}
	else
	{
		target_velocity = m_target_velocity;
	}
	dlt_velocity = target_velocity - m_missile_velocity;
	
	//根据当前质量，高度，插值计算基准油门
	m_Kc0 = CFlightGlobalFun::LAQL2(3,  6,  m_array,  h_array, &Kc0_matrix[0][0], m_mass_calc, m_missile_height);

	//转弯过程补偿转速
	if (m_flag_waypoint_turn || m_flag_launch_turn)
	{
		m_Kc1 = 20;
	} 
	else
	{
		m_Kc1 = 0.0;
	}

	//求高度机动补偿转速
	if (m_flag_alltitude_change)
	{
		//爬升
		if (m_flag_alltitude_climb)
		{
			m_Kc2 = 30;
		}
		//下滑
		if (m_flag_alltitude_decline)
		{
			m_Kc2 = 0.0;
		}
	} 
	//无高度机动
	else
	{
		m_Kc2 = 0.0;
	}

	//时间控制，增加指令速度补偿，即增加空速，待增加
	//实际地速 与 期望地速度=剩余距离/(到达时间 - 飞行时间)比较
	//大于2.5m/s时，指令空速 减小2.5m/s
	//小于-2.5m/s时，指令空速 增大2.5m/s
	//绝对值小于2.5m/s时，指令空速 补偿对应值
	if(m_flag_velocity_control == 1)
	{
		if(fabs((m_target_time - flight_time)) > 5.0)
		{
			target_velocity_cmd = (m_target_time - flight_time);

			
		}
		//保持前一次速度不变
		//else
		//{
		//	target_velocity_cmd = 
		//}
		
		
		//dlt_velocity_cmd = () ???...
	}
	

	//方法一：半开环控制，每5s，观察空速与指令速度之间关系
	//如果空速大于指令空速2.5m/s，减小5%油门；
	//如果空速小于指令空速2.5m/s，增大5%油门；
	//如果空速与指令空速小于2.5m/s，保持当前油门不变；
	if(flight_time > m_dltKc_time + 2.5)
	{
		m_dltKc_time = flight_time;
		
		if(dlt_velocity > 2.5)
		{
			m_Kc3 = m_Kc3 + 5;
		}
		else if(dlt_velocity < -2.5)
		{
			m_Kc3 = m_Kc3 - 5;
		}
		else
		{
			//m_Kc3 = m_Kc3;
		}
	}
	//方法二：闭环PI控制
	/*if(m_flag_velocity_control_init == 1)//初始化
	{
		m_flag_velocity_control_init = 2;
		m_Kc3 = 0.0;
		m_velocity_integrator = 0.0;
	}
	else
	{
		dlt_velocity = target_velocity - m_missile_velocity;
		m_velocity_integrator +=dlt_velocity * STEP_5ms;
		m_velocity_integrator = CFlightGlobalFun::Range(m_velocity_integrator, 10.0/ki);
		m_Kc3 = kp*dlt_velocity + ki * m_velocity_integrator;
	}
	*/

	//合转速
	m_control_Kc = m_Kc0 + m_Kc1 + m_Kc2 + m_Kc3;

	//转速控制
	if(m_control_Kc < 20)
		m_control_Kc = 20; 
	if(m_control_Kc > 100)
		m_control_Kc = 100; 
}
#else
void CMathControlEngine::Calc_Data_Rpm()
{
	double target_velocity = VEL_COMMAND_CRUISE;//初值
	
	//转速限制及推力当量
	double k0 = 700;
	double kp = 1.0;
	double ki = 0.13;

	//指令速度限幅
	if(m_target_velocity > VEL_COMMAND_MAX_LIMIT)
	{
		target_velocity = VEL_COMMAND_MAX_LIMIT;
	}
	else if(m_target_velocity > VEL_COMMAND_MIN_LIMIT)
	{
		target_velocity = VEL_COMMAND_MIN_LIMIT;
	}
	else
	{
		target_velocity = m_target_velocity;
	}
	
	//求基准装订转速
	m_n0 = RPM_COMMAND_CRUISE + (m_target_velocity - VEL_COMMAND_CRUISE)*14.2;

	//转弯过程补偿转速
	if (m_flag_waypoint_turn
		|| m_flag_launch_turn)
	{
		m_n1 = 300;
	} 
	else
	{
		m_n1 = 0.0;
	}

	//求高度机动补偿转速
	if (m_flag_alltitude_change)
	{
		//爬升
		if (m_flag_alltitude_climb)
		{
			m_n2 = 500;
		}
		//下滑
		if (m_flag_alltitude_decline)
		{
			m_n2 = 0.0;
		}
	} 
	//无高度机动
	else
	{
		m_n2 = 0.0;
	}

	//时间控制，即地速跟踪，PI控制
	double dlt_velocity = 0.0;
	if (m_flag_velocity_control)
	{
		dlt_velocity = target_velocity - m_missile_velocity;
		m_velocity_integrator +=dlt_velocity * STEP_5ms;
		m_velocity_integrator = CFlightGlobalFun::Range(m_velocity_integrator, (-100.0/(k0*ki)));
		m_n3 = k0*(kp*dlt_velocity + ki * m_velocity_integrator);
	}

	//合转速
	m_control_rpm = m_n0 + m_n1 + m_n2 + m_n3;

	//转速控制
	if(m_control_rpm < RPM_COMMAND_MIN_LIMIT)
		m_control_rpm = RPM_COMMAND_MIN_LIMIT; 
	if(m_control_rpm > RPM_COMMAND_MAX_LIMIT)
		m_control_rpm = RPM_COMMAND_MAX_LIMIT; 

	//指令转速估计状态转速
	//m_state_rpm = m_control_rpm;
}
#endif

void CMathControlEngine::Send_Data()
{
#ifdef __VEL__CONTROL__MODE__KC__
	p_st_engine_control_output->control_Kc = m_control_Kc;
#else
	p_st_engine_control_output->control_rpm = m_control_rpm;
#endif
	//p_st_engine_control_output->mass_calc = m_mass_calc;
}
