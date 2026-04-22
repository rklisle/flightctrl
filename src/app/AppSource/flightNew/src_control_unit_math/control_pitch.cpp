#include <cstring>
#include "control_pitch.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
#include "../port/flightPort.h"

CMathControlPitch::CMathControlPitch()
{
	p_st_pitch_control_input = NULL;
	p_st_pitch_control_output = NULL;
	// p_st_debug_monitor = NULL;
	m_k2f = 0.0;
	m_k5f = 0.0;
	m_k3f = 0.0;
	m_k7f = 0.0;
	m_k6f = 0.0;
	m_knif = 0.0;
	m_kwif = 0.0;
	m_u2f = 0.0;
	m_u5f = 0.0;
	m_u3f = 0.0;
	m_u7f = 0.0;
	m_u6f = 0.0;
	m_ugf = 0.0;
	m_unwif = 0.0;
	m_urf_zd = 0.0;

	m_mass = 0.0;
	m_v = 0.0;
	m_vs = 0.0;
	m_vs_t_altitude_control = 0.0;
	m_g = 0.0;
	m_zeta = 0.0;
	m_zeta_command = 0.0;
	m_zeta_command_record1 = 0.0;
	m_zeta_command_record2 = 0.0;
	m_wz = 0.0;
	m_hz = 0.0;
	m_hz_t_altitude_control = 0.0;
	m_h_command = 0.0;
	m_h_command_t_change = 0.0;
	m_h_command_t_change_record = 0.0;
	m_h_rate_command = 0.0;
	m_h_command1 = 0.0;
	m_h_rate_command1 = 0.0;
	m_h_command2 = 0.0;
	m_h_rate_command2 = 0.0;
	m_nby = 0.0;
	m_ny_command = 0.0;
	m_dqf = 0.0;
	m_h_target_in = 0.0;
	m_h_target = 0.0;
	m_h_target_record = 0.0;
	m_altitude_change_gain = 0.0;
	m_altitude_change_gain_seg1 = 0.0;
	m_altitude_change_gain_record = 0.0;
	m_distance_target = 0.0;
	m_distance_target_t_combat = 99999.9;
	m_gama_command_compensate = 0.0;
	m_count_altitude_change = 0;
	m_count_altitude_change_record = 0;
	m_unif = 0.0;
	m_uaf = 0.0;
	m_uaf_record = 0.0;
	m_wz_record = 0.0;
	m_time_control = MAX_TIME;
	m_time_altitude_change_start = MAX_TIME;
	m_time_altitude_change = MAX_TIME;
	m_time_altitude_change_record = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_altitude_control = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
	m_time_v70 = MAX_TIME;
	m_time_v120 = MAX_TIME;
	m_time_v60 = MAX_TIME;
	m_time_v100 = MAX_TIME;
	m_time_v140 = MAX_TIME;
	m_flag_altitude_integral_set = false;
}
void CMathControlPitch::Initial()
{
}
void CMathControlPitch::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	// Monitor_Data();
}
void CMathControlPitch::Get_Data()
{
	m_mass = p_st_pitch_control_input->mass;
	m_v = p_st_pitch_control_input->v;
	m_vs = p_st_pitch_control_input->vs;//垂速
	m_g = p_st_pitch_control_input->g;
	m_zeta = p_st_pitch_control_input->zeta;
	m_wz = p_st_pitch_control_input->wz;
	m_hz = p_st_pitch_control_input->hz;
	m_nby = p_st_pitch_control_input->nby;
	m_dqf = p_st_pitch_control_input->dqf;
	m_time_control = p_st_pitch_control_input->time_control;
	m_time_separate_booster = p_st_pitch_control_input->time_separate_booster;
	m_time_altitude_control = p_st_pitch_control_input->time_altitude_control;
	m_time_altitude_change_start = p_st_pitch_control_input->time_altitude_change_start;
	m_time_combat_status = p_st_pitch_control_input->time_combat_status;
	m_time_combat_delay = p_st_pitch_control_input->time_combat_delay;
	m_h_target_in = p_st_pitch_control_input->target_height;
	m_distance_target = p_st_pitch_control_input->distance_target;
	m_distance_target_t_combat = p_st_pitch_control_input->distance_target_t_combat;
	m_gama_command_compensate = p_st_pitch_control_input->gama_command_compensate;
	m_count_altitude_change = p_st_pitch_control_input->count_altitude_change;
}
void CMathControlPitch::Send_Data()
{
	p_st_pitch_control_output->u2f = m_u2f;
	p_st_pitch_control_output->u5f = m_u5f;
	p_st_pitch_control_output->ugf = m_ugf;	//高度控制
	p_st_pitch_control_output->urf_zd = m_urf_zd;
	p_st_pitch_control_output->h_command = m_h_command;
	p_st_pitch_control_output->zeta_command = m_zeta_command;
	p_st_pitch_control_output->ny_command = m_ny_command;
}
void CMathControlPitch::Calc_Data()
{
	Calc_Control_Gain();
	Calc_Control_Commond();
	
	m_u2f = m_k2f * (m_zeta - m_zeta_command);
	if(flight_time < m_time_separate_booster)
	{
		m_u2f = CFlightGlobalFun::Range(m_u2f, 12.0);
	}
	else
	{
		m_u2f = m_u2f / cos(m_gama_command_compensate / RTOA);//滚转解耦
		m_u2f = CFlightGlobalFun::Range(m_u2f, 11.0);		
	}	

	m_u5f = m_k5f * m_wz;

	//高度跟踪
	m_u3f = m_k3f * (m_hz - m_h_command);
	m_u3f = CFlightGlobalFun::Range(m_u3f, 10.0);	//高度机动计算，临时取消

	m_u7f = m_k7f * (m_vs - m_h_rate_command);

	//高度积分控制
	if((!m_flag_altitude_integral_set)
		&&(((flight_time > m_time_altitude_control) 
		&& (fabs(m_hz - m_h_command) <= 20.0))
		||(flight_time >= (m_time_altitude_control + 20.0))))
	{
		m_flag_altitude_integral_set = true;
	}
	if (m_flag_altitude_integral_set)
	{
		m_u6f += m_k6f * (m_hz - m_h_command) * STEP_5ms;
		m_u6f = CFlightGlobalFun::Range(m_u6f, 9.0);
	}
	
	//爬升完成前，无高度控制
	if(flight_time <= m_time_altitude_control)
	{
		m_ugf = 0.0;
	}
	//高度控制引入过渡
	else if(flight_time <= (m_time_altitude_control + 10.0))
	{
		m_ugf = (m_u3f + m_u7f + m_u6f)  * (flight_time - m_time_altitude_control) /10.0;
		m_ugf = CFlightGlobalFun::Range(m_ugf,10.0);	//高度机动计算，临时取消	
	}
	//高度控制过程
	else
	{
		m_ugf = m_u3f + m_u7f + m_u6f;
		m_ugf = CFlightGlobalFun::Range(m_ugf,10.0);	//高度机动计算，临时取消	
	}

	//目标头锁定前，即导引前
	if (flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		m_unwif = m_u2f + m_u5f + m_ugf;//内外回路控制叠加
	}
	//制导后
	else
	{
		//测试弹道阶跃响应
 		//m_ny_command = 1.0;
 		//if (flight_time > (m_time_combat_status + 10.0)) m_ny_command = 0.5;	

		//标准三回路过载控制
 		//m_unwif += (RTOA * m_knif * (m_nby - m_ny_command) + m_kwif * m_wz) * STEP_5ms;
 		//m_unwif = CFlightGlobalFun::Range(m_unwif, 5.0);
 		//m_urf_zd = m_unwif + m_u5f;

		//伪攻角三回路过载控制 
		double k1 = 0.004981;
		double k2 = 0.9925;		//伪攻角反馈回路传函1/(s+a4)离散化系数，a4=1.5
		m_unif += RTOA * m_knif * (m_nby - m_ny_command) * STEP_5ms;
		m_unif = CFlightGlobalFun::Range(m_unif, 5.0);
		m_uaf = k2 * m_uaf_record + m_kwif * k1 * m_wz_record;
		m_uaf_record = m_uaf;
		m_wz_record = m_wz;
		m_urf_zd = m_unif + m_uaf + m_u5f;
	}
}

void CMathControlPitch::Calc_Control_Gain()
{
	double kfp_stage1 = 0.3;
	double vel_stage1_array[3] = {30.0, 50.0, 70.0};
	double kfd_stage1_array[3] = {0.223, 0.312, 0.520};
	double kfd_stage1 = 0.5;
	
	
	double kfp_stage2 = 0.3;
	double vel_stage2_array[3] = {30.0, 50.0, 70.0};
	double mass_stage2_array[3] = {100.0, 125.0, 150.0};
	double kfd_stage2_matrix[9] = {0.398, 0.462, 0.520, 0.239, 0.277, 0.312,  0.170, 0.198, 0.223};
	double kfd_stage2 = 0.5;

	double temp_v = 50.0;
	double temp_mass = 125;


	if(flight_time < m_time_separate_booster)
	{	
		temp_v = CFlightGlobalFun::Range2(m_v, vel_stage1_array[2], vel_stage1_array[0]);
		kfd_stage1 = CFlightGlobalFun::LAQL1(3,  vel_stage1_array, kfd_stage1_array, temp_v);
		m_k2f = kfp_stage1;
		m_k5f = kfd_stage1;
	}
	else if(flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		temp_v = CFlightGlobalFun::Range2(m_v, vel_stage2_array[2], vel_stage2_array[0]);
		temp_mass = CFlightGlobalFun::Range2(m_mass, mass_stage2_array[2], mass_stage2_array[0]);
		kfd_stage2 = CFlightGlobalFun::LAQL2(3,  3,  mass_stage2_array,  vel_stage2_array, kfd_stage2_matrix, temp_mass, temp_v);

		m_k2f = kfp_stage2;
		m_k5f = kfd_stage2;
	}
	else
	{
		m_knif = 0.07;
		m_kwif = 0.9;
		m_k5f  = 0.14;
	}

	m_k3f = 0.45;
	m_k7f = 0.90;
	m_k6f = 0.025;
}
void CMathControlPitch::Calc_Control_Commond()
{
	//计算俯仰程序角，滚转角补偿
	double delta_zeta_command = (7.0 / (1.0 - cos(57.0 / RTOA))) * (1.0 - cos(m_gama_command_compensate / RTOA));
	delta_zeta_command = CFlightGlobalFun::Range(delta_zeta_command, 3.5);	

	//启控前，无控
	if (flight_time < m_time_control)
	{
		m_zeta_command = m_zeta;
		m_zeta_command_record1 = m_zeta_command;
	}
	//助推器分离前, 俯仰角过度到10deg，爬升
	else if(flight_time < m_time_separate_booster)
	{
		m_zeta_command = (m_zeta_command_record1 - 10.0) * exp(-(flight_time - m_time_control) * (flight_time - m_time_control) / 15.0) + 10.0;
		m_zeta_command_record2 = m_zeta_command;
// 		m_zeta_command = (m_zeta_command_record1 - 15.0) * exp(-(flight_time - m_time_control) / 2.0) + 15.0;
// 		m_zeta_command_record2 = m_zeta;
	}
	//引入高度控制前，俯仰角过度到2deg，过度到平衡攻角，便于转定高
	else if(flight_time < m_time_altitude_control)
	{
		m_zeta_command = (m_zeta_command_record2 - 2.0) * exp(-(flight_time - m_time_separate_booster) / 5.0) + 2.0;//过渡时间15s
		m_zeta_command_record1 = m_zeta_command;
		m_zeta_command = m_zeta_command + delta_zeta_command;
	}
// 	else if(flight_time < m_time_altitude_control)
// 	{
// 		m_zeta_command = (m_zeta_command_record1 - 2.0) * exp(-(flight_time - (m_time_separate_booster + 7.0)) / 2.0) + 2.0;
// 		m_zeta_command_record2 = m_zeta_command;
// 		m_zeta_command = m_zeta_command + delta_zeta_command;
// 	}
	//进入导引前，高度控制过程中
	else if(flight_time < (m_time_combat_status + 5.0))
	{
		m_zeta_command = (m_zeta_command_record1 - 2.0)*exp(-(flight_time - m_time_altitude_control) / 10.0) + 2.0; 
		m_zeta_command = m_zeta_command + delta_zeta_command;
	}

	//满足高度机动条件，初始化，计算程序高度、高度变化率
	if((m_count_altitude_change != m_count_altitude_change_record)	//受flight_basic综合流程调度控制，满足以下条件开始高度机动
		&& (flight_time >= m_time_altitude_change_start))			///1：高度机动指令次数改变		2：飞行时间大于机动开始时间
	{
		//只进入一次
		m_count_altitude_change_record = m_count_altitude_change;
		
		//记录本次及上一次高度机动开始时刻的程序高度
		m_h_command_t_change_record = m_h_command_t_change;
		m_h_command_t_change = m_h_command;				
		//记录本次及上一次高度机动开始时间
		m_time_altitude_change_record = m_time_altitude_change;
		m_time_altitude_change = m_time_altitude_change_start;		

		m_h_target_record = m_h_target;
		m_h_target = m_h_target_in;

		m_altitude_change_gain_record = m_altitude_change_gain;		//计算并记录上一次高度机动速度控制增益	
		//m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / (2.0 * 40.0);	//高度机动计算，临时更改
		m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / (2.0 * 10.0);
		//if(m_altitude_change_gain <= 5.0)	m_altitude_change_gain = 5.0;
		//if(m_altitude_change_gain >= 100.0)	m_altitude_change_gain = 100.0;	//高度机动计算，临时更改
		//if(m_altitude_change_gain >= 200.0)	m_altitude_change_gain = 200.0;
	}

	//高度控制前，更新
	if (flight_time <= m_time_altitude_control)
	{
		m_h_command = m_hz;
		m_h_rate_command = 0.0;
		m_hz_t_altitude_control = m_hz;
		m_vs_t_altitude_control = m_vs;
		m_h_target = m_h_target_in;
	}
	//首次高度机动流程，用于发射转平飞高度控制
	else if(flight_time < m_time_altitude_change)
	{
		if (fabs(m_vs_t_altitude_control) > 0.00001)
			m_altitude_change_gain_seg1 = fabs((m_hz_t_altitude_control - m_h_target)/m_vs_t_altitude_control);
		else
			m_altitude_change_gain_seg1 = 50.0;
		if (m_altitude_change_gain_seg1<=5.0)	m_altitude_change_gain_seg1 = 5.0;
		if (m_altitude_change_gain_seg1>=50.0)	m_altitude_change_gain_seg1 = 50.0;
		
		m_h_command = (m_hz_t_altitude_control - m_h_target) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
			+ m_h_target;		     
		m_h_rate_command = -((m_hz_t_altitude_control - m_h_target) / m_altitude_change_gain_seg1) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1);
	}
	//巡航飞行过程中高度机动流程，包含连续机动平滑过渡
	else													
	{
		if(1 == m_count_altitude_change)
		{			
			m_h_command1 = (m_hz_t_altitude_control - m_h_target_record)
				* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
				+ m_h_target_record;
			m_h_rate_command1 = -1.0 / m_altitude_change_gain_seg1 * (m_h_command1 - m_h_target_record);
		}
		else
		{
			double temp = (flight_time - m_time_altitude_change_record) / m_altitude_change_gain_record;
			m_h_command1 = (m_h_command_t_change_record - m_h_target_record) * exp(- temp * temp) + m_h_target_record;
			m_h_rate_command1 = -2.0 * (flight_time - m_time_altitude_change_record) * (m_h_command1 - m_h_target_record)
				/ (m_altitude_change_gain_record * m_altitude_change_gain_record);
		}
		m_h_command2 = (m_h_command_t_change - m_h_target) 
			* exp(-((flight_time - m_time_altitude_change) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change) / m_altitude_change_gain))
			+ m_h_target;
		m_h_rate_command2 = -2.0 * (flight_time - m_time_altitude_change) * (m_h_command2 - m_h_target) 
			/ (m_altitude_change_gain * m_altitude_change_gain);

		//过渡至目标高度
		if(flight_time <= (m_time_altitude_change + 8.0))
		{
			double temp = PI / 8.0 * (flight_time - m_time_altitude_change);
			m_h_command = m_h_command1 + 0.5 * ( m_h_command2 - m_h_command1) * (1.0 - cos(temp));
			m_h_rate_command = m_h_rate_command1 
				+ 0.5 * ( m_h_rate_command2 - m_h_rate_command1) * (1.0 - cos(temp)) 
				+ PI / 16.0 * ( m_h_command2 - m_h_command1) * sin(temp);
		}
		else
		{
			double temp = (flight_time - m_time_altitude_change) / m_altitude_change_gain;
			m_h_command = (m_h_command_t_change - m_h_target) * exp(- temp * temp) + m_h_target;
			m_h_rate_command = -2.0 * (flight_time - m_time_altitude_change) * (m_h_command - m_h_target)
				/ (m_altitude_change_gain * m_altitude_change_gain);
		}
	}

	//计算程序过载，导引规律
// 	if(m_distance_target > 3500.0)
// 	{
// 		m_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.05;
// 	}
// 	else if(m_distance_target > 3000.0)
// 	{
// 		m_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.12 
// 			- 0.07 * (m_distance_target - 3000.0) / 500.0;
// 	}
// 	else
// 	{
// 		m_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.12;
// 	}
// 
// 	if(m_distance_target_t_combat < 2900.0)
// 	{		    
// 		m_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.05;		
// 	}

	//重力补偿比例导引
	m_ny_command = 3.0 * m_v * m_dqf / m_g / RTOA + 1.05;	
	m_ny_command = CFlightGlobalFun::Range(m_ny_command, 4.0);
}

// void CMathControlPitch::Monitor_Data()
// {
// 	extern CSimMonitor sim_monitor;
	
// 	if (sim_monitor.flag_monitor2_valid)
// 	{
// 		sim_monitor.Get_Variable(m_zeta_command,"zetacx",ENUM_FILE_CONTROL1);	//助推段:姿态角指令
// 		sim_monitor.Get_Variable(m_h_command,"hcx",ENUM_FILE_CONTROL1);		//巡航段:高度指令
// 		sim_monitor.Get_Variable(m_h_rate_command,"dhcx",ENUM_FILE_CONTROL1);	//巡航段:垂速指令
// 		sim_monitor.Get_Variable(m_ny_command,"nyc",ENUM_FILE_CONTROL1);		//导引段:过载指令
// 	}
	
// 	if (sim_monitor.flag_monitor3_valid)
// 	{
// 		sim_monitor.Get_Variable(m_k2f,"k2f",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k5f,"k5f",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k3f,"k3f",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k7f,"k7f",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k6f,"k6f",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_knif,"knif",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_kwif,"kwif",ENUM_FILE_AERO1);
// 	}
// }
