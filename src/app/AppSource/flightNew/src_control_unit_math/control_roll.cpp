#include <cstring>
#include "control_roll.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
#include "../port/flightPort.h"

CMathControlRoll::CMathControlRoll()
{
	p_st_roll_control_input = NULL;
	p_st_roll_control_output = NULL;
	// p_st_debug_monitor = NULL;
	m_k2g = 0.0;
	m_k5g = 0.0;
	m_k4g = 0.0;
	m_k3g = 0.0;
	m_k7g = 0.0;
	m_k6g = 0.0;
	m_w0 = 0.0;
	m_k = 0.0;
	m_z1 = 0.0;
	m_z2 = 0.0;
	m_last_k2g = 0.0;
	m_last_k5g = 0.0;
	m_last_k4g = 0.0;
	m_tar_k2g = 0.0;
	m_tar_k5g = 0.0;
	m_tar_k4g = 0.0; 
	m_u2g = 0.0;
	m_u5g = 0.0;
	m_u4g = 0.0;
	m_ug_adrc = 0.0;
	m_u3g = 0.0;
	m_u7g = 0.0;
	m_u6g = 0.0;
	m_u25g = 0.0;
	m_urg_record = 0.0;
	m_uz = 0.0;
	m_uz1_record = 0.0;
	m_sz = 0.0;
	m_mass = 0.0;
	m_v = 0.0;
	m_vnz = 0.0;
	m_g = 0.0;
	m_gama = 0.0;
	m_gama_record1 = 0.0;
	m_gama_record2 = 0.0;
	m_gama_command = 0.0;
	m_gama_command_compensate = 0.0;
	m_gama_command_compensate_out = 0.0;
	m_gama_turn_nominal = 0.0;
	m_wx = 0.0;
	m_wx_command = 0.0;
	m_turn_angle = 0.0;
	m_turn_radius = 0.0;
	m_target_velocity = 0.0;
	m_time_start = 0.0;
	m_dqh = 0.0;
	m_flag_record = false;
	m_flag_changing = false;
	m_flag_turning = false;
	m_flag_distance_overlimit = false;
	m_flag_launch_turn = false;
	m_time_control = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_engine_start = MAX_TIME;
	m_time_missile_takeoff = MAX_TIME;
	m_time_altitude_control = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_turn_in_start = MAX_TIME;
	m_time_turn_in_end = MAX_TIME;
	m_time_turn_out_start = MAX_TIME;
	m_time_turn_out_end = MAX_TIME;
	m_time_v70 = MAX_TIME;
	m_time_v120 = MAX_TIME;
	m_time_v60 = MAX_TIME;
	m_time_v100 = MAX_TIME;
	m_time_v140 = MAX_TIME;
	m_time_BTT_guidance_in = MAX_TIME;
}

void CMathControlRoll::Initial()
{
	wx_eso_init_takeoff();
	wx_eso_init_cruise();
}

void CMathControlRoll::wx_eso_init_takeoff()
{
	para_wx_eso_takeoff.z1 = 0.0;
    para_wx_eso_takeoff.z2 = 0.0;
    para_wx_eso_takeoff.z1_pre = 0.0;
    para_wx_eso_takeoff.z2_pre = 0.0;
    para_wx_eso_takeoff.ux = 0.0;
    para_wx_eso_takeoff.wx = 0.0;
    para_wx_eso_takeoff.wx_pre = 0.0;
    para_wx_eso_takeoff.Ts = 0.005;
}

void CMathControlRoll::wx_eso_calc_takeoff(double K0, double T0, double Keso)
{
	double Ts       = para_wx_eso_takeoff.Ts;
    double z1_pre   = para_wx_eso_takeoff.z1_pre;
    double z2_pre   = para_wx_eso_takeoff.z2_pre;
    double wx_meas  = para_wx_eso_takeoff.wx;
    double delta_u  = para_wx_eso_takeoff.ux;

    double w0 = 1.0f / T0;
    double w_eso = Keso * w0;

    double a1 = (2.0f * T0 - Ts) / (2.0f * T0 + Ts);
    double b0 = (K0 * Ts) / (2.0f * T0 + Ts);
    double b1 = b0;

    double e = z1_pre - wx_meas;

    double z1 = a1 * z1_pre
             + b0 * delta_u
             + b1 * delta_u
             + z2_pre * Ts
             - w_eso * Ts * e;

    double z2 = z2_pre - w_eso * w_eso * Ts * e;

    //double wx_obs = z1;
    //double dis_obs = z2;

    para_wx_eso_takeoff.z1_pre = para_wx_eso_takeoff.z1;
    para_wx_eso_takeoff.z2_pre = para_wx_eso_takeoff.z2;

    para_wx_eso_takeoff.z1 = z1;
    para_wx_eso_takeoff.z2 = z2;


    para_wx_eso_takeoff.wx_pre = para_wx_eso_takeoff.wx;
}
void CMathControlRoll::wx_eso_init_cruise()
{
	para_wx_eso_cruise.z1 = 0.0;
    para_wx_eso_cruise.z2 = 0.0;
    para_wx_eso_cruise.z1_pre = 0.0;
    para_wx_eso_cruise.z2_pre = 0.0;
    para_wx_eso_cruise.ux = 0.0;
    para_wx_eso_cruise.wx = 0.0;
    para_wx_eso_cruise.wx_pre = 0.0;
    para_wx_eso_cruise.Ts = 0.005;
}
void CMathControlRoll::wx_eso_calc_cruise(double K0, double T0, double Keso)
{
	double Ts       = para_wx_eso_cruise.Ts;
    double z1_pre   = para_wx_eso_cruise.z1_pre;
    double z2_pre   = para_wx_eso_cruise.z2_pre;
    double wx_meas  = para_wx_eso_cruise.wx; 
    double delta_u  = para_wx_eso_cruise.ux;

    double w0 = 1.0f / T0;
    double w_eso = Keso * w0;

    double a1 = (2.0f * T0 - Ts) / (2.0f * T0 + Ts);
    double b0 = (K0 * Ts) / (2.0f * T0 + Ts);
    double b1 = b0;

    double e = z1_pre - wx_meas;

    double z1 = a1 * z1_pre
             + b0 * delta_u
             + b1 * delta_u
             + z2_pre * Ts
             - w_eso * Ts * e;

    double z2 = z2_pre - w_eso * w_eso * Ts * e;

    double wx_obs = z1;
    double dis_obs = z2;


    para_wx_eso_cruise.z1_pre = para_wx_eso_cruise.z1;
    para_wx_eso_cruise.z2_pre = para_wx_eso_cruise.z2;

    para_wx_eso_cruise.z1 = z1;
    para_wx_eso_cruise.z2 = z2;

    para_wx_eso_cruise.wx_pre = para_wx_eso_cruise.wx;
}
	
void CMathControlRoll::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	// Monitor_Data();
}
void CMathControlRoll::Get_Data()
{
	m_sz = p_st_roll_control_input->sz;
	m_mass = p_st_roll_control_input->mass;
	m_v = p_st_roll_control_input->v;
	m_vnz = p_st_roll_control_input->vnz;
	m_g = p_st_roll_control_input->g;
	m_gama = p_st_roll_control_input->gama;
	m_wx = p_st_roll_control_input->wx;
	m_dqh = p_st_roll_control_input->dqh;
	m_turn_angle = p_st_roll_control_input->angle_zw;
	m_turn_radius = p_st_roll_control_input->radius_zw;
	m_gama_turn_nominal = p_st_roll_control_input->gama_turn_nominal;
	m_target_velocity = p_st_roll_control_input->target_velocity;
	m_time_control = p_st_roll_control_input->time_control;
	m_time_separate_booster = p_st_roll_control_input->time_separate_booster;
	m_time_engine_start = p_st_roll_control_input->time_engine_start;
	m_time_missile_takeoff = p_st_roll_control_input->time_missile_takeoff;
	m_time_altitude_control = p_st_roll_control_input->time_altitude_control;
	m_time_combat_status = p_st_roll_control_input->time_combat_status;
	m_time_turn_in_start = p_st_roll_control_input->time_turn_in_start;
	m_time_turn_in_end = p_st_roll_control_input->time_turn_in_end;
	m_time_turn_out_start = p_st_roll_control_input->time_turn_out_start;
	m_time_turn_out_end = p_st_roll_control_input->time_turn_out_end;
	m_flag_launch_turn = p_st_roll_control_input->flag_launch_turn;
}
void CMathControlRoll::Send_Data()
{
	p_st_roll_control_output->u25g = m_u2g + m_u5g;//PD
	p_st_roll_control_output->u4g = m_u4g;			//I
	p_st_roll_control_output->ug_adrc = m_ug_adrc;	//ADRC
	p_st_roll_control_output->time_combat_delay = m_time_combat_delay;
	p_st_roll_control_output->gama_command_compensate = m_gama_command_compensate_out;
	p_st_roll_control_output->gama_command = m_gama_command;
	p_st_roll_control_output->wx_command = m_wx_command;
}
void CMathControlRoll::Calc_Data()
{
	Calc_Control_Gain();
	Calc_Control_Commond();

	double delta_sz = 0.0;
	double delta_vz = 0.0;

	//侧偏及侧向速度计算
	//if (flight_time <= m_time_separate_booster) //20260521--wym
	if (flight_time <= m_time_missile_takeoff)
	{
		delta_sz = 0.0;
		delta_vz = 0.0;	
	}
	else
	{
		//转弯过程
		if (flight_time > m_time_turn_in_start 
			&& flight_time <= m_time_turn_out_end)
		{
			//扇面转弯
			if (m_flag_launch_turn)
			{
				delta_sz = 0.0;
				delta_vz = 0.0;
			}
			//航迹转弯
			else
			{
				delta_sz = - (m_sz - m_turn_radius) * CFlightGlobalFun::FSign(m_turn_angle);		
				if(fabs(delta_sz) > (5.0 * m_k7g / m_k3g))
				{
					m_flag_distance_overlimit = true;
				}
				delta_sz = CFlightGlobalFun::Range(delta_sz, (10.0 * m_k7g / m_k3g));
				delta_vz = m_vnz; 
			}
		}
		//直航
		else
		{	
			delta_sz = m_sz;
			delta_vz = m_vnz;
		}
	}

	//侧偏积分I
	m_u6g += m_k6g * delta_sz * STEP_5ms;
	m_u6g = CFlightGlobalFun::Range(m_u6g, 4.0);
	if(m_flag_distance_overlimit
		|| (fabs(delta_sz) > (45.0 * m_k7g / m_k3g))
		|| (m_flag_launch_turn)
		|| ((flight_time > m_time_turn_in_start)&&(flight_time <= m_time_turn_in_end))
		|| ((flight_time > m_time_turn_out_start)&&(flight_time <= m_time_turn_out_end)))
	{
		m_u6g = 0.0;
	}
	//侧偏PD控制
	m_u3g = m_k3g * delta_sz;
	m_u3g = CFlightGlobalFun::Range(m_u3g,(45.0 * m_k7g));
	m_u7g = m_k7g * delta_vz;

	//侧偏PID控制->m_uz1
	double m_uz1 = m_u3g + m_u7g + m_u6g;	
	
	//限幅->m_uz1
	m_uz1 = CFlightGlobalFun::Range(m_uz1, 10.0);
	if (flight_time > m_time_turn_in_start
		&& flight_time <= m_time_turn_out_end)
	{
		m_uz1 = CFlightGlobalFun::Range(m_uz1, 6.0);
	}

	//侧向机动过渡m_uz1->uz_temp
	double uz_temp = 0.0;
	if(m_flag_launch_turn)
	{
		//扇面转弯过渡
		if (flight_time <= m_time_turn_out_end)
		{
			uz_temp = 0.0;
		}
		else if(flight_time < (m_time_turn_out_end + 1.0))
		{
			uz_temp = m_uz1 * (flight_time - m_time_turn_out_end);
		}
		else
		{
			uz_temp = m_uz1;
		}
	}
	else
	{
		//直航
		if (flight_time <= m_time_turn_in_start)
		{
			m_uz1_record = m_uz1;
			uz_temp = m_uz1;
		}
		//直航到航迹转弯
		else if (flight_time < (m_time_turn_in_start + 1.0))
		{
			uz_temp = m_uz1_record 
				+ (m_uz1 - m_uz1_record) * (flight_time - m_time_turn_in_start);
		}
		//航迹转弯
		else if (flight_time <= m_time_turn_out_end)
		{
			uz_temp = m_uz1;
			m_uz1_record = m_uz1;
		}
		//航迹转弯到直航
		else if (flight_time < (m_time_turn_out_end + 1.0))
		{
			uz_temp = m_uz1_record 
				+ (m_uz1 - m_uz1_record) * (flight_time - m_time_turn_out_end);
		}
		//直航
		else
		{
			uz_temp = m_uz1;
		}
	}
	
	//高度控制过渡uz_temp->m_uz
	if (flight_time < m_time_altitude_control)
	{
		m_uz = 0.0;
	}
	else if (flight_time < (m_time_altitude_control + 3.0))
	{
		//m_uz = uz_temp * (flight_time - (m_time_engine_start + 7.0)) / 3.0;
		m_uz = uz_temp * (flight_time - m_time_altitude_control) / 3.0;
	}
	else
	{
		m_uz = uz_temp;	
	}

	//滚转角跟踪PD控制
	m_u2g = m_k2g * (m_gama - m_gama_command);
	m_u5g = m_k5g * (m_wx - m_wx_command);
	
	//原积分
	//if(flight_time >= m_time_separate_booster + 1.0) 
	//{
	//	m_u4g += m_k4g * (m_gama - m_gama_command) * STEP_5ms;
	//	m_u4g = CFlightGlobalFun::Range(m_u4g, 12.0);
	//}
	//原ADRC
	//if(flight_time >= m_time_combat_status)
	//{
	//	m_z2 += m_w0 * m_w0 * (m_gama - m_z1) * STEP_5ms;
	//	m_z1 += (m_z2 + m_k * m_urg_record + 2.0 * m_w0 * (m_gama - m_z1)) * STEP_5ms;
	//	m_ug_adrc = - m_z2 / m_k;
	//	m_urg_record = m_u2g + m_u5g + m_ug_adrc;
	//}

	//内回路，ADRC控制
	//启控前
	if(flight_time < m_time_control)
	{
		m_ug_adrc = 0.0;
		m_u2g = 0.0;
		m_u5g = 0.0;
		m_uz = 0.0;
		m_urg_record = 0.0;

		para_wx_eso_takeoff.z1 = m_wx/RTOA;
		para_wx_eso_takeoff.z1_pre = m_wx/RTOA;
		para_wx_eso_takeoff.z2 = 0.0;
		para_wx_eso_takeoff.z2_pre= 0.0;
		para_wx_eso_takeoff.wx = m_wx/RTOA;
		para_wx_eso_takeoff.wx_pre = m_wx/RTOA;
	}
	//起飞段
	//else if(flight_time < m_time_separate_booster + 1.0) 
	else if(flight_time < m_time_missile_takeoff) 	 //20260521--wym
	{
		para_wx_eso_takeoff.wx = m_wx/RTOA;
		para_wx_eso_takeoff.ux = m_urg_record/RTOA;
		wx_eso_calc_takeoff(44.0, 0.5, 3.0);
		m_ug_adrc = RTOA*0.5*para_wx_eso_takeoff.z2/ 44.0;

		para_wx_eso_cruise.z1 = m_wx/RTOA;
		para_wx_eso_cruise.z1_pre = m_wx/RTOA;
		para_wx_eso_cruise.z2 = m_ug_adrc/RTOA*55.0/0.4;
		para_wx_eso_cruise.z2_pre= para_wx_eso_cruise.z2;
		para_wx_eso_cruise.wx = m_wx/RTOA;
		para_wx_eso_cruise.wx_pre = m_wx/RTOA;
	}
	//巡航及打击段
	else
	{
		//大干扰，采用BTT末制导，滚转角指令跟踪
		//小干扰，采用STT末制导，滚转角为零解耦
		para_wx_eso_cruise.wx = m_wx/RTOA;
		para_wx_eso_cruise.ux = m_urg_record/RTOA;
		wx_eso_calc_takeoff(55.0, 0.4, 3.0);
		m_ug_adrc = RTOA*0.4*para_wx_eso_cruise.z2/ 55.0;
	}

	//正舵产生负力矩，则自抗扰舵取相反数
	m_urg_record = -(m_ug_adrc + m_u2g + m_u5g + m_uz);
	//m_urg_record = m_ug_adrc + m_u2g + m_u5g + m_uz;
}

void CMathControlRoll::Calc_Control_Gain()
{
	//助推段参数
	if(flight_time < m_time_separate_booster)
	{
		//temp_v = CFlightGlobalFun::Range2(m_v, vel_stage1_array[2], vel_stage1_array[0]);
		//kgd_stage1 = CFlightGlobalFun::LAQL1(3,  vel_stage1_array,  kgd_stage1_array, temp_v);
		//m_k2g = kgp_stage1;
		//m_k5g = kgd_stage1;
		m_k2g = 0.1818;
		m_k5g = 0.0682;

	}
	//巡航段参数
	else if(flight_time < m_time_combat_status)
	{
		//temp_v = CFlightGlobalFun::Range2(m_v, vel_stage2_array[2], vel_stage2_array[0]);
		//temp_mass = CFlightGlobalFun::Range2(m_mass, mass_stage2_array[2], mass_stage2_array[0]);
		//kgd_stage2 = CFlightGlobalFun::LAQL2(3,  3,  mass_stage2_array,  vel_stage2_array, kgd_stage2_matrix, temp_mass, temp_v);
		//m_k2g = kgp_stage2;
		//m_k5g = kgd_stage2;
		m_k2g = 0.1653;
		m_k5g = 0.0436;
		//ADRC控制，略

		//侧偏控制
		m_k3g = 0.06;
		m_k7g = 0.65;
		m_k6g = 0.0;

		//滚转角补偿
		if(fabs(m_gama) > 30.0) // error less than 2.5%
		{
			double k_temp = fabs(m_gama / RTOA / tan(m_gama / RTOA));
			m_k3g = 0.06 * k_temp;
			m_k7g = 0.65 * k_temp;
		}
	}
	//末制导段
	else
	{
		//kgd_stage2 = CFlightGlobalFun::LAQL2(3,  3,  mass_stage2_array,  vel_stage2_array, kgd_stage2_matrix, temp_mass, temp_v);
		//m_k2g = kgp_stage2;
		//m_k5g = kgd_stage2;
		m_k2g = 0.1653;
		m_k5g = 0.0436;

		//原ADRC参数
		//m_w0 = 8.0;
		//m_k = 30.0;
	}
}
void CMathControlRoll::Calc_Control_Commond()
{
	//启控前 rx_cmd
	if(flight_time <= m_time_control)
	{
		m_gama_command = m_gama;	
		m_gama_command_compensate = 0.0;
	}
	//助推器分离前，不引入引入侧偏控制
	//else if(flight_time <= m_time_separate_booster)
	//20260521--wym
	//起飞稳定前，不引入引入侧偏控制
	else if(flight_time <= m_time_missile_takeoff)
	{
		m_gama_command = 0.0;
		m_gama_command_compensate = 0.0;
		m_gama_record1 = m_gama;
	}
	//巡航段，引入侧偏控制及指令过渡
	else if(flight_time <= m_time_combat_status)
	{
		//助推器分离到直航侧偏控制过渡
		if (flight_time <= m_time_turn_in_start)	
		{    
			m_gama_command = m_gama_record1 * exp(-(flight_time - m_time_separate_booster) * (flight_time - m_time_separate_booster) / 16.0) - m_uz / m_k2g;
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
			m_gama_record2 = m_gama_command;
		}
		//入弯滚转角过渡
		else if (flight_time <= m_time_turn_in_end)
		{
			m_gama_command = m_gama_record2 + ROLL_RATE_COMMAND * (flight_time - m_time_turn_in_start) * CFlightGlobalFun::FSign(m_turn_angle);	  
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 5.0);
			m_gama_command_compensate = m_gama_command;
		}
		//大滚转角转弯过程中
		else if (flight_time <= m_time_turn_out_start)
		{
			m_gama_command_compensate = m_gama_turn_nominal * CFlightGlobalFun::FSign(m_turn_angle);
			m_gama_command = m_gama_turn_nominal * CFlightGlobalFun::FSign(m_turn_angle) - m_uz / m_k2g;	
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 5.0);
		}
		//出弯滚转角过渡
		else if (flight_time <= m_time_turn_out_end)
		{
			m_gama_command = (m_gama_turn_nominal - ROLL_RATE_COMMAND * (flight_time - m_time_turn_out_start)) * CFlightGlobalFun::FSign(m_turn_angle);	
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 5.0);
			m_gama_command_compensate = m_gama_command;
		}
		//转弯 过渡到 直航
		else if(flight_time <= m_time_turn_out_end + 1.0)
		{
			m_gama_command = - m_uz / m_k2g * (flight_time - m_time_turn_out_end) * (flight_time - m_time_turn_out_end);
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
		}
		//直航
		else
		{
			m_gama_command = - m_uz / m_k2g;
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
		}

		//进入末制导前，滚转角收敛到零，便于STT制导
		m_time_combat_delay = fabs(m_gama) / ROLL_RATE_COMMAND;
		if(m_time_combat_delay > 2.0) m_time_combat_delay = 2.0;
		m_time_BTT_guidance_in = flight_time;
	}
	//分析后续优化，侧滑角提供的过载是否足够抑制侧向扰动
	else
	{	
// 		double knz = 3.0;	
//
//		double nz_command = knz * m_v * m_dqh / RTOA / m_g;
// 		nz_command = CFlightGlobalFun::Range(nz_command, 0.8);
// 
//		//BTT
// 		if (fabs(nz_command) >= 0.15)
// 		{
// 			m_gama_command = -atan((fabs(nz_command) - 0.15) * CFlightGlobalFun::FSign(nz_command)) * RTOA;			
// 			m_gama_command = CFlightGlobalFun::Range(m_gama_command, 25.0);
// 			if (flight_time <= (m_time_BTT_guidance_in + 1.0))
// 			{
// 				m_gama_command = (flight_time - m_time_BTT_guidance_in) * m_gama_command;
// 			}
// 		}
// 		else
//		{
			//STT
			m_gama_command = 0.0;
			m_time_BTT_guidance_in = flight_time;
//		}
	}


	//限幅角度，质量越轻滚转角越小
 	double temp_gama_command_range = 35.0 + (ROLL_COMMAND_DYNMIC_LIMIT - 35.0)/(6*3600.0) * flight_time;	
	if(temp_gama_command_range > ROLL_COMMAND_DYNMIC_LIMIT)
	{
		temp_gama_command_range = ROLL_COMMAND_DYNMIC_LIMIT;
	}
	//滚转角指令限幅
	m_gama_command = CFlightGlobalFun::Range(m_gama_command, temp_gama_command_range);
	//补偿滚转角限幅
 	m_gama_command_compensate = CFlightGlobalFun::Range(m_gama_command_compensate, temp_gama_command_range);
	if( fabs(m_gama_command_compensate) < 3.0)
	{
		m_gama_command_compensate_out = 0.0;
	}
	else if( fabs(m_gama_command_compensate) < 60.0)
	{
		m_gama_command_compensate_out = fabs(m_gama_command_compensate) - 3.0;
	}
	else
	{
		m_gama_command_compensate_out = 57.0;
	}

	//侧向机动 wx_cmd 
	if(flight_time <= m_time_combat_status)
	{
		if(flight_time < m_time_turn_in_start)
		{
			m_wx_command = 0.0;
		}
		else if(flight_time < m_time_turn_in_end)
		{
			m_wx_command = ROLL_RATE_COMMAND * CFlightGlobalFun::FSign(m_turn_angle); 
		}
		else if(flight_time < m_time_turn_out_start)
		{
			m_wx_command = 0.0;
		}
		else if(flight_time < m_time_turn_out_end)
		{
			m_wx_command = - ROLL_RATE_COMMAND * CFlightGlobalFun::FSign(m_turn_angle);
		}
		else
		{
			m_wx_command = 0.0;
		}
	}
	else
	{
		m_wx_command = 0.0;
	}
}

// void CMathControlRoll::Monitor_Data()
// {
// 	extern CSimMonitor sim_monitor;
// 	if (sim_monitor.flag_monitor2_valid)
// 	{
// 		sim_monitor.Get_Variable(m_gama_command,"gamacx",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_gama_command_compensate_out,"gamacxbc",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_wx_command,"wxcx",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_uz,"uz",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_z2,"x1",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_ug_adrc,"ug_adrc",ENUM_FILE_CONTROL1);
// 	}
// 	if (sim_monitor.flag_monitor3_valid)
// 	{
// 		sim_monitor.Get_Variable(flight_time,"time",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k2g,"k2g",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k5g,"k5g",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k4g,"k4g",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k3g,"k3g",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k7g,"k7g",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_k6g,"k6g",ENUM_FILE_AERO1);
// 	}
// }
