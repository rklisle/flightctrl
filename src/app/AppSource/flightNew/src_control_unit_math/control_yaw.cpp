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
	m_wy = 0.0;
	m_wy_command = 0.0;
	m_distance_target = 0.0;
	m_nz_command = 0.0;
	m_nbz = 0.0;
	m_dqh = 0.0;
	m_qh = 0.0;
	m_tgo = 0.0;
	m_qh_leader = 0.0;
	m_det_qh = 0.0;
	m_mass = 0.0;
	m_v = 0.0;
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
	m_nbz = p_st_yaw_control_input->nbz;
	m_dqh = p_st_yaw_control_input->dqh;
	m_qh = p_st_yaw_control_input->qh;
	m_tgo = p_st_yaw_control_input->tgo;
	
	m_mass = p_st_yaw_control_input->mass;
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
	Calc_Control_Gain();
	Calc_Control_Commond();

	//增加阻尼
	m_u5h = m_k5h * (m_wy - m_wy_command);
	if(flight_time < m_time_separate_booster)
		m_u5h = CFlightGlobalFun::Range(m_u5h, 3.0);

	//过载跟踪
	if(flight_time >= m_time_separate_booster)
	{
		//测试弹道阶跃响应
// 		m_nz_command = 0.0;
// 		if (flight_time > (m_time_combat_status + 3.0)) m_nz_command = 0.2;	

		//标准三回路过载控制
// 		m_unwih += (RTOA * m_knih * (m_nz_command + m_nbz) + m_kwih * (m_wy - m_wy_command)) * STEP_5ms;
// 		m_unwih = CFlightGlobalFun::Range(m_unwih, 5.0);
// 		m_urh_zd = m_unwih + m_u5h;
// 		m_urh_zd = CFlightGlobalFun::Range(m_urh_zd, 6.0);

		//伪侧滑角三回路过载控制 
		double k1 = 0.004996;
		double k2 = 0.9985;		//伪侧滑角反馈回路传函1/(s+b4)离散化系数，b4=0.3
		//说明: 由m_nz_command 有比例导引视线角速度计算得到，为负即-|m_nz_command|，负舵产生正力矩、正过载，导引向目标
		m_unih += RTOA * m_knih * (m_nz_command + m_nbz) * STEP_5ms; 
		m_unih = CFlightGlobalFun::Range(m_unih, 6.0);
		m_ubh = k2 * m_ubh_record + m_kwih * k1 * m_wy_record;
		m_ubh_record = m_ubh;
		m_wy_record = m_wy - m_wy_command;//小偏差线性化模型
		m_urh_zd = m_unih + m_ubh + m_u5h;
	}
}

void CMathControlYaw::Calc_Control_Gain()
{
	//double khp_stage1 = 0.0;
	double vel_stage1_array[3] = {30.0, 50.0, 70.0};
	double khd_stage1_array[3] = {0.568, 0.341, 0.092};
	double khd_stage1 = 0.5;
	
	//double khp_stage2 = 0.0;
	double vel_stage2_array[3] = {30.0, 50.0, 70.0};
	double mass_stage2_array[3] = {100.0, 125.0, 150.0};
	double khd_stage2_matrix[9] = {0.568, 0.514, 0.454, 0.341, 0.308, 0.272, 0.092, 0.101, 0.113};
	double khd_stage2 = 0.5;

	double temp_v = 50.0;
	double temp_mass = 125;
	
	//助推器分离前
	if(flight_time < m_time_separate_booster)
	{
		temp_v = CFlightGlobalFun::Range2(m_v, vel_stage1_array[2], vel_stage1_array[0]);
		khd_stage1 = CFlightGlobalFun::LAQL1(3,  vel_stage1_array, khd_stage1_array, m_v);
		m_k5h = khd_stage1;
	}
	//助推器分离后
	else
	{
		temp_v = CFlightGlobalFun::Range2(m_v, vel_stage2_array[2], vel_stage2_array[0]);
		temp_mass = CFlightGlobalFun::Range2(m_mass, mass_stage2_array[2], mass_stage2_array[0]);
		khd_stage2 = CFlightGlobalFun::LAQL2(3,  3,  mass_stage2_array,  vel_stage2_array, khd_stage2_matrix, m_mass, m_v);
		m_k5h = khd_stage2;
		
		m_knih = -0.3;	//三通道过载回路，过载积分
		m_kwih =  0.8;	//三通道过载回路，角速度积分
		//m_k5h  =  0.25;	//三通道过载回路，角速度比例(增稳)
	}
}

void CMathControlYaw::Calc_Control_Commond()
{
	//求航向程序角速度: 助推航向稳定，BTT转弯(航向角速度、角伺服)及末端姿态稳定
	if(flight_time <= (m_time_separate_booster + 1.0))
	{
		m_wy_command = 0.0;
	}
	else
	{
		m_wy_command = - m_g * RTOA * sin(m_gama / RTOA) / m_v;//弹道倾角变化率指令，忽略姿态相应时间，即未航向角速度
	}

	//求航向程序过载:  进入导引前为零，之后三通道过载
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

		//动目标比例导引
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
