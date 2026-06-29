#include <cstring>
#include "control_yaw.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
#include "../port/flightPort.h"

CMathControlYaw::CMathControlYaw()
{
	p_st_yaw_control_input = NULL;
	p_st_yaw_control_output = NULL;
	// p_st_debug_monitor = NULL;
	m_k5h = 0.0;
	m_knih = 0.0;
	m_kwih = 0.0;
	m_u5h = 0.0;
	m_unwih = 0.0;
	m_urh_zd = 0.0;
	m_urh_record_separate_booster = 0.0;
	m_urh_record_begin_combat = 0.0;
	//m_time_record_separate_booster = 0.0;
	//m_time_record_begin_combat = 0.0;
	m_wy = 0.0;
	m_wy_command = 0.0;
	m_distance_target = 0.0;
	m_nz_command = 0.0;
	m_nbz = 0.0;
	m_nvz = 0.0;
	m_dqh = 0.0;
	m_qh = 0.0;
	m_tgo = 0.0;
	m_qh_leader = 0.0;
	m_det_qh = 0.0;
	m_mass = 0.0;
	m_v = 0.0;
	m_hz = 0.0;
	m_g = 0.0;
	m_gama = 0.0;
	m_unih = 0.0;
	m_ubh = 0.0;
	m_ubh_record = 0.0;
	m_wy_record = 0.0;
	m_gama_command_compensate = 0.0;
	m_time_v70 = MAX_TIME;
	m_time_v60 = MAX_TIME;
	m_time_v100 = MAX_TIME;
	m_time_v140 = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
}
void CMathControlYaw::Initial()
{

}
void CMathControlYaw::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	// Monitor_Data();
};
void CMathControlYaw::Get_Data()
{
	m_time_separate_booster = p_st_yaw_control_input->time_separate_booster;
	m_time_combat_status = p_st_yaw_control_input->time_combat_status;
	m_time_combat_delay = p_st_yaw_control_input->time_combat_delay;
	m_wy = p_st_yaw_control_input->wy;
	m_distance_target = p_st_yaw_control_input->distance_target;
	m_nby = p_st_yaw_control_input->nby;
	m_nbz = p_st_yaw_control_input->nbz;
	m_dqh = p_st_yaw_control_input->dqh;
	m_qh = p_st_yaw_control_input->qh;
	m_tgo = p_st_yaw_control_input->tgo;
	
	m_mass = p_st_yaw_control_input->mass;
	m_hz = p_st_yaw_control_input->hz;
	m_v = p_st_yaw_control_input->v;
	m_g = p_st_yaw_control_input->g;
	m_gama = p_st_yaw_control_input->gama;
	m_gama_command_compensate = p_st_yaw_control_input->gama_command_compensate;
	m_qh_leader = p_st_yaw_control_input->qh_leader;
	m_det_qh = p_st_yaw_control_input->det_qh;
}
void CMathControlYaw::Send_Data()
{
	p_st_yaw_control_output->u5h = m_u5h;
	p_st_yaw_control_output->urh_zd = m_urh_zd;// 包含阻尼项
}
void CMathControlYaw::Calc_Data()
{
	double k1 = 4.996e-3;
	double k2 = 0.9985;		//伪侧滑角反馈回路传函1/(s+b4)离散化系数，b4=0.3
	double temp_urh_zd = 0.0;
	
	Calc_Control_Gain();
	Calc_Control_Commond();

	//增加阻尼
	m_u5h = m_k5h * (m_wy - m_wy_command);
	if(flight_time < m_time_separate_booster)
		m_u5h = CFlightGlobalFun::Range(m_u5h, 6.0);

	//测试弹道阶跃响应测试
// 	m_nz_command = 0.0;
// 	if (flight_time > (m_time_combat_status + 3.0)) m_nz_command = 0.2;	

	//标准三回路过载控制
// 	m_unwih += (RTOA * m_knih * (m_nz_command + m_nbz) + m_kwih * (m_wy - m_wy_command)) * STEP_5ms;
// 	m_unwih = CFlightGlobalFun::Range(m_unwih, 5.0);
// 	m_urh_zd = m_unwih + m_u5h;
// 	m_urh_zd = CFlightGlobalFun::Range(m_urh_zd, 6.0);
	if(flight_time < m_time_combat_status + m_time_combat_delay + 1.0)
	{
		//阻尼器和侧滑角为零控制
		//m_nvz = -m_nby*sin(m_gama/RTOA) + m_nbz*cos(m_gama/RTOA);
		m_nvz = m_nbz;
		m_unih += RTOA * m_knih * (0.0 + m_nvz) * STEP_5ms; 
		m_unih = CFlightGlobalFun::Range(m_unih, 10.0);
	}
	else
	{
		//积分形式的伪侧滑角三回路过载控制 
		m_unih += RTOA * m_knih * (m_nz_command + m_nbz) * STEP_5ms; 
		m_unih = CFlightGlobalFun::Range(m_unih, 10.0);
	}
	
	m_ubh = k2 * m_ubh_record + m_kwih * k1 * m_wy_record;
	m_ubh_record = m_ubh;
	m_wy_record = m_wy - m_wy_command;//小偏差线性化模型

	temp_urh_zd = m_unih + m_ubh + m_u5h;

	//过载跟踪
	if(flight_time < m_time_separate_booster)
	{
		m_urh_zd = temp_urh_zd;
		m_urh_record_separate_booster = m_urh_zd;
	}
	//过渡到巡航
	else if (flight_time < m_time_separate_booster + 3.0)
	{
		m_urh_zd = m_urh_record_separate_booster * (3.0 - flight_time + m_time_separate_booster)/3.0 + temp_urh_zd *(flight_time - m_time_separate_booster)/3.0;
	}
	//巡航段
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{
		m_urh_zd = temp_urh_zd;
		m_urh_record_begin_combat = m_urh_zd;
	}
	//过渡到末制导
	else if (flight_time < m_time_combat_status + m_time_combat_delay + 1.0)
	{
		m_urh_zd = m_urh_record_begin_combat * (1.0 - flight_time + (m_time_combat_status + m_time_combat_delay))/1.0 + temp_urh_zd *(flight_time - (m_time_combat_status + m_time_combat_delay))/1.0;
	}
	else
	{
		//说明: 由m_nz_command 有比例导引视线角速度计算得到，为负即-|m_nz_command|，负舵产生正力矩、正过载，导引向目标
		m_urh_zd = temp_urh_zd;
	}
}

void CMathControlYaw::Calc_Control_Gain()
{
	//限幅后处理变量
	double temp_velocity = 50.0;//初值 m/s
	double temp_hight = 1500.0;//初值 m
	double temp_mass = 133.0;//初值 kg
	double temp_q = 1500.0;//初值 Pa	

	//起飞段控制参数：
	double knih_staget1 = -0.10;//转化为角度，对应4.011
	double kwih_staget1 = -0.55;//角速度积分，伪攻角控制
	double k5h_staget1	 = -0.14;//角速度控制
	/******************************************************************
	 * 控制参数表规则
	 * （1）速度 < 30m/s  → 使用 30m/s 对应参数（第0列）
	 * （2）速度档位：30、40、50、70 m/s → 对应列索引 0、1、2、3
	 * （3）高度 > 3km    → 使用 3km 对应参数（第3行）
	 * （4）高度档位：0、1000、2000、3000 m → 对应行索引 0、1、2、3
	 *****************************************************************/
	static double hight_stage1_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//高度
	static double vel_stage1_array[4] = {30.0, 40.0, 50.0, 70.0};//速度
	// 过载积分增益 kni[高度行][速度列]
	double knih_stage1_matrix[4][4] = {
	    {-0.4806, -0.2027, -0.1038, -0.0378},
	    {-0.5295, -0.2234, -0.1144, -0.0417},
	    {-0.5848, -0.2467, -0.1263, -0.0460},
	    {-0.6474, -0.2731, -0.1398, -0.0510}    // 高度 3000m
	};

	// 侧滑角增益 kwi[高度行][速度列]
	double kwih_stage1_matrix[4][4] = {
	    {-2.0709, -0.9505, -0.4365,  0.0055},
		{-2.3391, -1.1013, -0.5328, -0.0435},
		{-2.6418, -1.2715, -0.6417, -0.0988},
		{-2.9844, -1.4641, -0.7649, -0.1615}    // 高度 3000m
	};

	// 角速度增益 k5[高度行][速度列]
	double k5h_stage1_matrix[4][4] = {
	    {-0.1208, -0.0904, -0.0720, -0.0507},
	    {-0.1208, -0.0904, -0.0720, -0.0508},
	    {-0.1207, -0.0904, -0.0721, -0.0509},
	    {-0.1207, -0.0904, -0.0721, -0.0509}    // 高度 3000m
	};
	//积分时间常数，0.30s

	//巡飞过程中控制参数：
	double knih_staget2 = -0.10;//转化为角度，对应4.011
	double kwih_staget2 = -0.46;//角速度积分，伪攻角控制
	double k5h_staget2  = -0.083;//角速度控制
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
	//经进一步分析，动压对控制参数影响不大，采用质量作为输入，进行一维控制参数插值；
	double knih_stage2_array[3] = {-0.10, -0.10, -0.10};
	double kwih_stage2_array[3] = {-0.4363, -0.4630, -0.4799};
	double k5h_stage2_array[3] = {-0.1090, -0.0830, -0.0667};
	//积分时间常数，0.25s

	//末制导段控制参数：固定带宽伪攻角三回路过载控制，即 5rad-1闭环带宽，1.5rad-1长周期转折频率；
	//过载增益 knih
	//角速度增益 kwih
	//伪侧滑角增益 k5h
	double knih_staget3 = 0.12;//转化为角度，对应4.011
	double kwih_staget3 = 0.55;//角速度积分，伪攻角控制
	double k5h_staget3  = 0.10;//角速度控制
	/******************************************************************
	 * 控制参数表规则
	 * （1）速度 < 40m/s  → 使用 40m/s 对应参数（第0列）
	 * （2）速度档位：40、50、60、70 m/s → 对应列索引 0、1、2、3
	 * （3）高度 > 3km    → 使用 3km 对应参数（第3行）
	 * （4）高度档位：0、1000、2000、3000 m → 对应行索引 0、1、2、3
	 *****************************************************************/
	static double hight_stage3_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//高度
	static double vel_stage3_array[4] = {40.0, 50.0, 60.0, 70.0};//速度
	// 过载积分增益 kni[高度行][速度列]
	double knih_stage3_matrix[4][4] = {
	    {-0.2002, -0.1025, -0.0593, -0.0374},
	    {-0.2206, -0.1129, -0.0654, -0.0412},
	    {-0.2436, -0.1247, -0.0722, -0.0455},
	    {-0.2697, -0.1381, -0.0799, -0.0503}    // 高度 3000m
	};

	// 侧滑角增益 kwi[高度行][速度列]
	double kwih_stage3_matrix[4][4] = {
	    {-0.8938, -0.3943, -0.1266,  0.0326},
	    {-1.0427, -0.4894, -0.1925, -0.0157},
	    {-1.2107, -0.5969, -0.2670, -0.0703},
	    {-1.4010, -0.7186, -0.3513, -0.1321}   // 高度 3000m
	};
	// 角速度增益 k5[高度行][速度列]
	double k5h_stage3_matrix[4][4] = {
	    {-0.1291, -0.1030, -0.0855, -0.0728},
	    {-0.1290, -0.1029, -0.0855, -0.0728},
	    {-0.1289, -0.1029, -0.0854, -0.0728},
	    {-0.1287, -0.1028, -0.0854, -0.0720}    // 高度 3000m
	};
	//积分时间常数，0.30s
	
	//助推器分离前，微分控制
	if(flight_time < m_time_separate_booster)
	{
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_stage1_array[3], vel_stage1_array[0]);
		temp_hight = CFlightGlobalFun::Range2(m_hz, vel_stage1_array[3], vel_stage1_array[0]);

		m_knih = CFlightGlobalFun::LAQL2(4, 4, hight_stage1_array, vel_stage1_array, &knih_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_kwih = CFlightGlobalFun::LAQL2(4, 4, hight_stage1_array, vel_stage1_array, &kwih_stage1_matrix[0][0], temp_hight, temp_velocity); 
		m_k5h = CFlightGlobalFun::LAQL2(4, 4, hight_stage1_array, vel_stage1_array, &k5h_stage1_matrix[0][0], temp_hight, temp_velocity);
		
		m_k5h *= 2.0;//助推段振荡，增加阻尼
	}
	//巡航段，助推器分离后，进入末制导前
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{
		temp_q = CFlightGlobalFun::Range2(m_mass, q_stage2_array[5], q_stage2_array[0]);
		temp_mass = CFlightGlobalFun::Range2(m_mass, mass_stage2_array[2], mass_stage2_array[0]);

		m_knih = CFlightGlobalFun::LAQL1(3,  mass_stage2_array, knih_stage2_array, temp_mass);
		m_kwih = CFlightGlobalFun::LAQL1(3,  mass_stage2_array, kwih_stage2_array, temp_mass);
		m_k5h = CFlightGlobalFun::LAQL1(3,  mass_stage2_array, knih_stage2_array, temp_mass);
		
		//m_knih =  0.3;	//三通道过载回路，过载积分
		//m_kwih =  -0.8;	//三通道过载回路，角速度积分
		//m_k5h  =  0.25;	//三通道过载回路，角速度比例(增稳)
	}
	//末制导过程
	else
	{
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_stage3_array[3], vel_stage3_array[0]);
		temp_hight = CFlightGlobalFun::Range2(m_hz, vel_stage3_array[3], vel_stage3_array[0]);

		m_knih = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &knih_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_kwih = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &kwih_stage3_matrix[0][0], temp_hight, temp_velocity); 
		m_k5h = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &k5h_stage3_matrix[0][0], temp_hight, temp_velocity);
	}

	//逻辑舵已经是负反馈
	m_knih = m_knih;
	m_kwih = - m_kwih;
	m_k5h = - m_k5h;
}

void CMathControlYaw::Calc_Control_Commond()
{
	//求航向程序角速度: 助推航向稳定，BTT转弯(航向角速度、角伺服)及末端姿态稳定
	if(flight_time <= (m_time_separate_booster + 1.0))
	{
		m_wy_command = 0.0;
	}
	//巡航段，滚转角产生侧向过载，产生弹道偏角变化率，即航向角变化率指令
	else
	{
		//滚转角估计侧向过载，进一步估计弹道偏角变化率
		//m_wy_command = - m_g * RTOA * sin(m_gama / RTOA) / m_v;//弹道倾角变化率指令，忽略姿态相应时间，即未航向角速度
		m_wy_command = - m_g * RTOA * sin(m_gama_command_compensate / RTOA) / m_v;
	}

	//求航向程序过载:  进入导引前，过载指令为零，即只进行侧向增稳
	if(flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		m_nz_command = 0.0;
	}
	else
	{
// 		double knz;
// 		if(m_distance_target > 2000.0)
// 			knz = 4.5;
// 		else
// 			knz = 3.5 + cos((2000.0 - m_distance_target) * PI / 2000.0);
// 
// 		if(knz > 4.5)
// 			knz = 4.5;
// 		if(knz <= 2.5)
// 			knz = 2.5;

// 		if(flight_time < (m_time_combat_status + m_time_combat_delay + 1.0))
// 		{
// 			m_nz_command = (flight_time - (m_time_combat_status + m_time_combat_delay)) * m_nz_command;
// 		}	
// 		m_nz_command = CFlightGlobalFun::Range(m_nz_command, 0.15);

		//动目标比例导引，STT制导计算过载，待更改为BTT制导
		double knz = 3.0;	
		m_nz_command = knz * m_v * m_dqh / RTOA / m_g;
		m_nz_command = CFlightGlobalFun::Range(m_nz_command, 4.0);
	}
}

// void CMathControlYaw::Monitor_Data()
// {
// 	extern CSimMonitor sim_monitor;
// 	if (sim_monitor.flag_monitor2_valid)
// 	{
// 		sim_monitor.Get_Variable(m_wy_command,"wycx",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(-m_nz_command,"nzc",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_urh_zd,"m_urh_zd",ENUM_FILE_CONTROL1);
// 	}
// 	if (sim_monitor.flag_monitor3_valid)
// 	{
// 		sim_monitor.Get_Variable(m_k5h,"k5h",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_knih,"knih",ENUM_FILE_AERO1);
// 		sim_monitor.Get_Variable(m_kwih,"kwih",ENUM_FILE_AERO1);
// 	}
// }
