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
	
	m_k0f = 0.0;
	m_k2f = 0.0;
	m_k5f = 0.0;
	m_k3f = 0.0;
	m_k7f = 0.0;
	m_k6f = 0.0;
	m_knif = 0.0;
	m_kwif = 0.0;
	m_uqkf = 0.0;
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

	m_ktheta_lauch_enc = 12.0;//发射俯仰角
	m_ktheta_climb_enc = 6.0;//助推器分离后，爬升俯仰角
	m_ktheta_hight_enc = 1.0;
}
void CMathControlPitch::Initial()
{
	m_ktheta_lauch_enc = p_st_pitch_control_input->ktheta_lauch_enc;//地面装订，发射
	m_ktheta_climb_enc = p_st_pitch_control_input->ktheta_climb_enc;//地面装订，爬升
	m_ktheta_hight_enc = p_st_pitch_control_input->ktheta_hight_enc;//地面装订，定高
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
	m_mass = p_st_pitch_control_input->mass;//估计质量
	m_q = p_st_pitch_control_input->q;	//动压
	m_v = p_st_pitch_control_input->v;	//空速
	m_vs = p_st_pitch_control_input->vs;//垂速
	m_g = p_st_pitch_control_input->g;
	m_zeta = p_st_pitch_control_input->zeta;
	m_wz = p_st_pitch_control_input->wz;
	m_hz = p_st_pitch_control_input->hz;//组合高度
	m_nby = p_st_pitch_control_input->nby;
	m_dqf = p_st_pitch_control_input->dqf;
	m_time_control = p_st_pitch_control_input->time_control;
	m_time_separate_booster = p_st_pitch_control_input->time_separate_booster;
	m_time_missile_takeoff = p_st_pitch_control_input->time_missile_takeoff;
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
	p_st_pitch_control_output->uqkf = m_uqkf;
	p_st_pitch_control_output->u2f = m_u2f;
	p_st_pitch_control_output->u5f = m_u5f;
	p_st_pitch_control_output->ugf = m_ugf;	//高度控制
	p_st_pitch_control_output->urf_zd = m_urf_zd;
	p_st_pitch_control_output->h_command = m_h_command;
	p_st_pitch_control_output->zeta_command = m_zeta_command;
	p_st_pitch_control_output->ny_command = m_ny_command;
}

//说明：正逻辑舵对应负力矩，即控制系数为正，偏差为“状态 - 指令”，产生“正逻辑舵负力矩”
void CMathControlPitch::Calc_Data()
{
	Calc_Control_Gain();
	Calc_Control_Commond();
	
	//为了调试???...
	if(	flight_time > 15.0)
	{
		double temp_a = 1.0;
	}
	
	//内回路：前馈，助推器分离后逐步引入前馈
	if(flight_time < m_time_separate_booster)
	{
		m_uqkf = 0.0;//单位deg
	}
	else if(flight_time < m_time_separate_booster + 3.0)
	{
		m_uqkf = m_k0f*(flight_time - m_time_separate_booster)/3.0;
	}
	else
	{
		m_uqkf = m_k0f;//单位deg
	}
	//内回路：比例环节
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
	//内回路：微分环节，指令角速度为零
	m_u5f = m_k5f * m_wz;

	//外回路：高度跟踪
	m_u3f = m_k3f * (m_hz - m_h_command);
	m_u3f = CFlightGlobalFun::Range(m_u3f, 10.0);	//高度机动计算，临时取消
	//外回路：垂速反馈
	m_u7f = m_k7f * (m_vs - m_h_rate_command);
	//外回路:高度积分控制
	if((!m_flag_altitude_integral_set)
		&&(((flight_time > m_time_altitude_control) //开始高度控制且高度偏差较小
		&& (fabs(m_hz - m_h_command) <= 20.0))
		||(flight_time >= (m_time_altitude_control + 20.0))))//备份条件
	{
		m_flag_altitude_integral_set = true;
	}
	if (m_flag_altitude_integral_set)
	{
		m_u6f += m_k6f * (m_hz - m_h_command) * STEP_5ms;
		m_u6f = CFlightGlobalFun::Range(m_u6f, 9.0);
	}

	//根据飞行阶段，确定高度控制
	//爬升完成前，无高度控制
	if(flight_time <= m_time_altitude_control)
	{
		m_ugf = 0.0;
	}
	//高度控制引入过渡，过渡时间4s
	else if(flight_time <= (m_time_altitude_control + 4.0))
	{
		m_ugf = (m_u3f + m_u7f + m_u6f)  * (flight_time - m_time_altitude_control) /4.0;
		m_ugf = CFlightGlobalFun::Range(m_ugf,20.0);
	}
	//高度控制过程，过渡完成
	else
	{
		m_ugf = m_u3f + m_u7f + m_u6f;
		m_ugf = CFlightGlobalFun::Range(m_ugf,20.0);
	}

	//导引头锁定目标前，常规控制
	if (flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		m_unwif = m_u2f + m_u5f + m_ugf;//内外回路控制叠加
	}
	//导引头锁定目标后，进入末制导
	else
	{
		//测试弹道阶跃响应测试
 		//m_ny_command = 1.0;
 		//if (flight_time > (m_time_combat_status + 10.0)) m_ny_command = 0.5;	

		//标准三回路过载控制
 		//m_unwif += (RTOA * m_knif * (m_nby - m_ny_command) + m_kwif * m_wz) * STEP_5ms;
 		//m_unwif = CFlightGlobalFun::Range(m_unwif, 5.0);
 		//m_urf_zd = m_unwif + m_u5f;

		//（积分形式）伪攻角三回路过载控制 ，其中，过载指令由"改进比例导引(生成过载指令)"
		//过载积分项
		m_unif += RTOA * m_knif * (m_nby - m_ny_command) * STEP_5ms;
		m_unif = CFlightGlobalFun::Range(m_unif, 10.0);
		//伪攻角项，零阶保持器线性化
		double k1 = 7.4772e-3;//0.004981;
		double k2 = 0.9925;	//伪攻角反馈回路传函1/(s+a4)离散化系数，a4=1.5
		m_uaf = k2 * m_uaf_record + m_kwif * k1 * m_wz_record;
		m_uaf_record = m_uaf;
		m_wz_record = m_wz;
		
		//叠加微分项，构成总控制量
		m_urf_zd = m_unif + m_uaf + m_u5f;//过载积分、过载控制
	}
}

void CMathControlPitch::Calc_Control_Gain()
{
	//限幅后处理变量
	double temp_velocity = 50.0;//初值 m/s
	double temp_hight = 1500.0;//初值 m
	double temp_mass = 133.0;//初值 kg
	double temp_q = 1500.0;//初值 Pa	
	
	//起飞段控制参数: 固定闭环带宽的俯仰角PD控制，高度、速度二维插值
	double k2f_stage1 = 1.211;
	double k5f_stage1 = 0.200;
	static double hight_stage1_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//高度
	static double vel_stage1_array[6] = {30, 40, 50, 60, 70, 80};//速度
	// 修正后比例增益 Kp[高度][速度]
	double K2f_stage1_matrix[4][6] = {
	    {2.9019, 1.6323, 1.0447, 0.7255, 0.5330, 0.4081},
	    {3.1977, 1.7987, 1.1512, 0.7994, 0.5873, 0.4497},
	    {3.5316, 1.9865, 1.2714, 0.8829, 0.6487, 0.4966},
	    {3.9095, 2.1991, 1.4074, 0.9774, 0.7181, 0.5498}
	};
	// 修正后微分增益 Kd[高度][速度]
	double K5f_stage1_matrix[4][6] = {
	    {0.4933, 0.2775, 0.1776, 0.1233, 0.0906, 0.0694},
	    {0.5436, 0.3058, 0.1957, 0.1359, 0.0998, 0.0764},
	    {0.6004, 0.3377, 0.2161, 0.1501, 0.1103, 0.0844},
	    {0.6646, 0.3738, 0.2393, 0.1662, 0.1221, 0.0935}
	};
	//前馈俯仰舵
	double K0f_stage1_matrix[4][6] = {
    	{17.4762, 10.9412,  7.9164,  6.2733,  5.2826,  4.6396},
    	{18.9985, 11.7975,  8.4645,  6.6539,  5.5622,  4.8537},
    	{19.0000, 12.7642,  9.0831,  7.0836,  5.8779,  5.0953},//20.7170 -> 19.0
    	{20.0000, 13.8583,  9.7834,  7.5698,  6.2351,  5.3689} //22.6622 -> 20.0
	};

	//巡航段控制参数：0.8倍开环转折频率为闭环带宽，PD控制，以弹质量、动压二维插值
	double k2f_stage2 = 1.3092;
	double k5f_stage2 = 0.1147;
	/******************************************************************
	 * 输入：质量 mass(kg)、动压 q(Pa)
	 * 质量插值点：102, 133, 165 kg
	 * 动压插值点：500, 1000, 1500, 2000, 2500, 3000 Pa
	 * 输出：Kp（比例增益）、Kd（微分增益）
	 * 数组结构：Kp[质量行][动压列]、Kd[质量行][动压列]
	 *****************************************************************/
	// 质量插值表（行索引 0,1,2）
	static double mass_stage2_array[3] = {102.0, 133.0, 165.0};
	// 动压插值表（列索引 0~5）
	static double q_stage2_array[6] = {500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0};
	// 修正后比例增益 Kp[质量][动压]
	double K2f_stage2_matrix[3][6] = {
	    {1.1419, 1.3529, 1.5149, 1.6514, 1.7716, 1.8803},   // mass = 102kg
	    {1.0232, 1.1850, 1.3092, 1.4139, 1.5061, 1.5895},   // mass = 133kg
	    {0.9474, 1.0779, 1.1780, 1.2623, 1.3367, 1.4039}    // mass = 165kg
	};
	// 修正后微分增益 Kd[质量][动压]
	double K5f_stage2_matrix[3][6] = {
	    {0.3929, 0.2129, 0.1323, 0.0838, 0.0503, 0.0255},   // mass = 102kg
	    {0.3779, 0.1960, 0.1147, 0.0660, 0.0326, 0.0100},   // mass = 133kg
	    {0.3678, 0.1845, 0.1029, 0.0540, 0.0205, 0.0100}    // mass = 165kg
	};
	// 前馈 K0[质量][动压]
	double K0f_stage2_matrix[3][6] = {
	    {12.7194,  7.6292,  5.9325,  5.0841,  4.5751,  4.2358},
	    {15.8134,  9.1762,  6.9638,  5.8576,  5.1939,  4.7515},
	    {19.0072, 10.7731,  8.0284,  6.6561,  5.8327,  5.2838}
	};
	
	//末制导段控制参数：固定带宽伪攻角三回路过载控制，即 5rad-1闭环带宽，1.5rad-1长周期转折频率；
	//过载增益 knif
	//角速度增益 kwif
	//伪侧滑角增益 k5f
	double knif_staget3 = 0.10;//转化为角度，对应4.011
	double kwif_staget3 = 0.55;//角速度积分，伪攻角控制
	double k5f_staget3  = 0.14;//角速度控制
	/******************************************************************
	 * 控制参数表规则
	 * （1）速度 < 40m/s  → 使用 40m/s 对应参数（第0列）
	 * （2）速度档位：40、50、60、70 m/s → 对应列索引 0、1、2、3
	 * （3）高度 > 3km    → 使用 3km 对应参数（第3行）
	 * （4）高度档位：0、1000、2000、3000 m → 对应行索引 0、1、2、3
	 *****************************************************************/
	static double hight_stage3_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//高度
	static double vel_stage3_array[4] = {40.0, 50.0, 60.0, 70.0};//速度
	// 过载积分增益 knif[高度行][速度列]
	double knif_stage3_matrix[4][4] = {
	    {-0.1291f,  -0.1030f,  -0.0855f,  -0.0728f},   // 高度 0m
	    {-0.1290f,  -0.1029f,  -0.0855f,  -0.0728f},   // 高度 1000m
	    {-0.1289f,  -0.1029f,  -0.0854f,  -0.0728f},   // 高度 2000m
	    {-0.1287f,  -0.1028f,  -0.0854f,  -0.0729f}    // 高度 3000m
	};

	// 侧滑角增益 kwif[高度行][速度列]
	double kwif_stage3_matrix[4][4] = {
	    {-0.8938f,  -0.3943f,  -0.1266f,   0.0326f},   // 高度 0m
	    {-1.0427f,  -0.4894f,  -0.1925f,  -0.0157f},   // 高度 1000m
	    {-1.2107f,  -0.5969f,  -0.2670f,  -0.0703f},   // 高度 2000m
	    {-1.4010f,  -0.7186f,  -0.3513f,  -0.1321f}    // 高度 3000m
	};

	// 角速度增益 k5f[高度行][速度列]
	double k5f_stage3_matrix[4][4] = {
	    {-0.8938f,  -0.3943f,  -0.1266f,   0.0326f},   // 高度 0m
	    {-1.0427f,  -0.4894f,  -0.1925f,  -0.0157f},   // 高度 1000m
	    {-1.2107f,  -0.5969f,  -0.2670f,  -0.0703f},   // 高度 2000m
	    {-1.4010f,  -0.7186f,  -0.3513f,  -0.1321f}    // 高度 3000m
	};
	// 前馈 k0f[高度行][速度列]
	double k0f_stage3_matrix[4][4] = {
	    {8.3951, 6.2869, 5.1417, 4.4512},
		{8.9919, 6.6689, 5.4070, 4.6461},
		{9.6657, 7.1001, 5.7064, 4.8661},
		{10.4282, 7.5881, 6.0454, 5.1151}
	};

	//起飞段：助推器分离前或速度下降到50m/s，俯仰角PD控制
	if(flight_time < m_time_separate_booster)
	{	
		temp_hight  = CFlightGlobalFun::Range2(m_hz, hight_stage1_array[3], hight_stage1_array[0]);
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_stage1_array[5], vel_stage1_array[0]);

		m_k0f = CFlightGlobalFun::LAQL2(4, 6, hight_stage1_array, vel_stage1_array, &K0f_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_k2f = CFlightGlobalFun::LAQL2(4, 6, hight_stage1_array, vel_stage1_array, &K2f_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_k5f = CFlightGlobalFun::LAQL2(4, 6, hight_stage1_array, vel_stage1_array, &K5f_stage1_matrix[0][0], temp_hight, temp_velocity);
	}
	//巡航段:助推器分离后，进入末制导前，俯仰角PD控制 + 高度控制(后续定义)
	else if(flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		temp_mass  = CFlightGlobalFun::Range2(m_mass, mass_stage2_array[2], mass_stage2_array[0]);//估计质量mass
		temp_q = CFlightGlobalFun::Range2(m_q, q_stage2_array[5], q_stage2_array[0]);//动压q

		m_k0f = CFlightGlobalFun::LAQL2(3, 6, mass_stage2_array, q_stage2_array, &K0f_stage2_matrix[0][0], temp_mass, temp_q);
		m_k2f = CFlightGlobalFun::LAQL2(3, 6, mass_stage2_array, q_stage2_array, &K2f_stage2_matrix[0][0], temp_mass, temp_q);
		m_k5f = CFlightGlobalFun::LAQL2(3, 6, mass_stage2_array, q_stage2_array, &K5f_stage2_matrix[0][0], temp_mass, temp_q);
	}
	//末制导过程，伪攻角三回路过载控制
	else
	{
		//高度、速度
		temp_hight  = CFlightGlobalFun::Range2(m_hz, hight_stage3_array[3], hight_stage3_array[0]);
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_stage3_array[3], vel_stage3_array[0]);

		m_knif = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &knif_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_kwif = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &kwif_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_k5f = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &k5f_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_k0f = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &k0f_stage3_matrix[0][0], temp_hight, temp_velocity);
		//m_knif = 0.07;//转化为角度，对应4.011
		//m_kwif = 0.90;//角速度积分，伪攻角控制
		//m_k5f  = 0.14;//角速度控制

		//当前通道舵，已经是负反馈
		m_knif = - m_knif;
		m_kwif = - m_kwif;
		m_k5f = - m_k5f;
	}

	//前馈电压取负
	m_k0f = - m_k0f;
	//外回路控制参数，高度PID控制
	m_k3f = 0.45;
	m_k7f = 0.90;
	m_k6f = 0.025;
}
void CMathControlPitch::Calc_Control_Commond()
{
	//计算俯仰程序角，滚转角补偿
	//double delta_zeta_command = (5.5 / (1.0 - cos(60.0 / RTOA))) * (1.0 - cos(m_gama_command_compensate / RTOA));
	double delta_zeta_command = 5.5 * (1.0 - cos(m_gama_command_compensate / RTOA)) / (1.0 + cos(m_gama_command_compensate / RTOA));
	//滚转角60deg时，升力为重量2g，即半油重量平衡攻角6.6deg = 5.5deg + 1.2deg，即1g过载约攻角增量5.5deg
	delta_zeta_command = CFlightGlobalFun::Range(delta_zeta_command, 5.5);	

	//启控前，约0.3s
	if (flight_time < m_time_control)
	{
		m_zeta_command = m_zeta;
		m_zeta_command_record1 = m_zeta_command;
	}
	//助推器分离前, 约2s，俯仰角过度到初始发射角(装订值)
	else if(flight_time < m_time_separate_booster)
	{
		m_zeta_command = (m_zeta_command_record1 - m_ktheta_lauch_enc) * exp(-(flight_time - m_time_control) * (flight_time - m_time_control) / 1.0) + m_ktheta_lauch_enc;
		m_zeta_command_record2 = m_zeta_command;
// 		m_zeta_command = (m_zeta_command_record1 - 15.0) * exp(-(flight_time - m_time_control) / 2.0) + 15.0;
// 		m_zeta_command_record2 = m_zeta;
	}
	//引入高度控制前，俯仰角过度到2deg，过度到平衡攻角，便于转定高
	else if(flight_time < m_time_altitude_control)
	{
		//过渡至装订爬升角，过渡时间约4s，之后保持常值俯仰角爬升
		m_zeta_command = (m_zeta_command_record2 - m_ktheta_climb_enc) * exp(-(flight_time - m_time_separate_booster) * (flight_time - m_time_separate_booster) / 5.0) + m_ktheta_climb_enc;
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
		//过渡至估计平衡攻角,过渡时间约5.5s
		m_zeta_command = (m_zeta_command_record1 - m_ktheta_hight_enc)*exp(-(flight_time - m_time_altitude_control) * (flight_time - m_time_altitude_control)/ 10.0) + m_ktheta_hight_enc; 
		m_zeta_command = m_zeta_command + delta_zeta_command;
	}
	//进入末制导控制，不再使用指令俯仰角
	else
	{
		m_zeta_command = m_ktheta_hight_enc + delta_zeta_command;
	}

	//满足高度机动条件，初始化，计算程序高度、高度变化率
	if((m_count_altitude_change != m_count_altitude_change_record)	//受flight_basic综合流程调度控制，满足以下条件开始高度机动
		&& (flight_time >= m_time_altitude_change_start))			///1：高度机动指令次数改变		2：飞行时间大于机动开始时间
	{
		//次数更新，只进入一次
		m_count_altitude_change_record = m_count_altitude_change;
		
		//记录本次及上一次高度机动开始时，指令高度
		m_h_command_t_change_record = m_h_command_t_change;
		m_h_command_t_change = m_h_command;
		//记录本次及上一次高度机动开始时，机动开始时刻
		m_time_altitude_change_record = m_time_altitude_change;
		m_time_altitude_change = m_time_altitude_change_start;	//开始时间	
		//记录本次及上一次高度机动开始时，目标高度
		m_h_target_record = m_h_target;
		m_h_target = m_h_target_in;
		//记录本次及上一次高度机动开始时，过渡时间估计
		m_altitude_change_gain_record = m_altitude_change_gain;	//计算并记录上一次高度机动速度控制增益	
		m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / 5.0;
		if(m_altitude_change_gain <= 5.0)	m_altitude_change_gain = 5.0;
		if(m_altitude_change_gain >= 400.0)	m_altitude_change_gain = 400.0;
	}

	//高度控制前
	if (flight_time <= m_time_altitude_control)
	{
		m_h_command = m_hz;
		m_h_rate_command = 0.0;
		
		//更新直至高度控制
		m_hz_t_altitude_control = m_hz;
		m_vs_t_altitude_control = m_vs;
		//目标高度
		m_h_target = m_h_target_in;
	}
	//高度控制开始后到高度机动流程，定速爬升过程
	else if(flight_time < m_time_altitude_change)
	{
		//根据进入时刻垂速，估计机动到目标高度时间
		if (fabs(m_vs_t_altitude_control) > 0.01)
			m_altitude_change_gain_seg1 = fabs((m_hz_t_altitude_control - m_h_target)/m_vs_t_altitude_control);
		else
			m_altitude_change_gain_seg1 = 50.0;
		if (m_altitude_change_gain_seg1<=5.0)	m_altitude_change_gain_seg1 = 5.0;
		if (m_altitude_change_gain_seg1>=50.0)	m_altitude_change_gain_seg1 = 50.0;
		
		//指令高度由高度机动时刻 高度 过渡到 目标高度(装订值)，垂速过渡至零，开始高度跟踪
		m_h_command = (m_hz_t_altitude_control - m_h_target) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
			+ m_h_target;
		m_h_rate_command = -((m_hz_t_altitude_control - m_h_target) / m_altitude_change_gain_seg1) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1);
	}
	//大于高度机动时刻，；巡航飞行过程中高度机动流程，包含连续机动平滑过渡
	else			
	{
		//如果起飞过程中，航迹切换了一个航点，即m_count_altitude_change 为1，继续过 渡至 前一次目标高度
		//高度过渡至前一航点目标高度m_hz_t_altitude_control -> m_h_target_record，m_altitude_change_gain_seg1
		if(1 == m_count_altitude_change)
		{			
			m_h_command1 = (m_hz_t_altitude_control - m_h_target_record)
				* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
				+ m_h_target_record;
			m_h_rate_command1 = -1.0 / m_altitude_change_gain_seg1 * (m_h_command1 - m_h_target_record);
		}
		//如果高度机动过程中，航迹切换了一个航点，即m_count_altitude_change 大于1，计算前一次指令高度、垂速度
		//如果高度机动过程中，未切换航点，计算前一次指令高度为前一次指令高度、垂速度，即高度为前一次装订值，垂速为零
		//高度 过渡至 前一航点目标高度m_h_command_t_change_record -> m_h_target_record，m_altitude_change_gain_record
		else
		{
			double temp = (flight_time - m_time_altitude_change_record) / m_altitude_change_gain_record;
			m_h_command1 = (m_h_command_t_change_record - m_h_target_record) * exp(- temp * temp) + m_h_target_record;
			m_h_rate_command1 = -2.0 * (flight_time - m_time_altitude_change_record) * (m_h_command1 - m_h_target_record)
				/ (m_altitude_change_gain_record * m_altitude_change_gain_record);
		}
		
		//如果高度机动过程或起飞过程中，航迹切换了一个航点，计算当前次指令高度、垂速度，双指数过渡
		//进入时刻高度 过渡至 目标高度m_h_command_t_change -> m_h_target，m_altitude_change_gain
		m_h_command2 = (m_h_command_t_change - m_h_target) 
			* exp(-((flight_time - m_time_altitude_change) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change) / m_altitude_change_gain))
			+ m_h_target;
		m_h_rate_command2 = -2.0 * (flight_time - m_time_altitude_change) * (m_h_command2 - m_h_target) 
			/ (m_altitude_change_gain * m_altitude_change_gain);

		//指令高度过渡
		if(flight_time <= (m_time_altitude_change + 8.0))
		{
			//m_h_command1->m_h_command2
			double temp = PI / 8.0 * (flight_time - m_time_altitude_change);
			m_h_command = m_h_command1 + 0.5 * ( m_h_command2 - m_h_command1) * (1.0 - cos(temp));
			m_h_rate_command = m_h_rate_command1 
				+ 0.5 * ( m_h_rate_command2 - m_h_rate_command1) * (1.0 - cos(temp)) 
				+ PI / 16.0 * ( m_h_command2 - m_h_command1) * sin(temp);
		}
		else
		{
			//保持m_h_command2
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
	m_ny_command = CFlightGlobalFun::Range(m_ny_command, 2.0);
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
