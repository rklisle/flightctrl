#include <cstring>
#include "control_flight_basic.h"
//#include "../environment_earth_model.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"

// 标准大气常数
#define P0      101325.0f    // 海平面标准气压 (Pa)
#define T0      288.15f      // 海平面标准温度 (K)
#define L       0.0065f      // 温度梯度 (K/m)
#define G       9.80665f		//重力常数 m/s^2
#define R       8.31432f
#define M       0.0289644f
#define RHO0    1.225f      // 标准空气密度kg/m^3
// 空速管校准系数
#define ARSPD_RATIO	1.0f	//压差到空速校准系数0.5~2.0，空速 ∝ √(差压 / ARSPD_RATIO)
#define FULL_MASS		150.0	//满载质量kg
#define EMPTY_MASS		105.0	//空载（含剩油）质量kg
#define MAX_FLIGHT_TIME	8*3600	//8h对应 3600s 航时
#define MAX_FLIGHT_DIST	1440*1000	//1440km 航程

CMathControlFlightBasic::CMathControlFlightBasic()
{
	m_missile_ID = 0;
	m_missile_flight_mode = 0x55;

	m_vs = 0.0;//组合垂速
	m_hz = 0.0;//组合高度
	m_au = 0.0;//天向加速度
	m_ax = 0.0;
	m_ay = 0.0;
	m_az = 0.0;
	m_hgps = 0.0;
	m_longitude = 0.0;
	m_latitude = 0.0;
	m_longitude_A = 0.0;
	m_latitude_A = 0.0;
	m_longitude_B = 0.0;
	m_latitude_B = 0.0;
	m_longitude_C = 0.0;
	m_latitude_C = 0.0;
	m_g = 0.0;
	m_distance_BP = 0.0;
	m_distance_BP_500 = 0.0;
	m_distance_BP_500pre = 0.0;
	m_distance_BP_projection = 0.0;
	m_alpha_target = 0.0;
	m_distance_target = 0.0;
	m_distance_target_t_combat = 99999.9;
	m_distance_turn_in_compensate = 0.0;
	m_distance_turn_in = 0.0;
	m_total_distance = 0.0;
	m_alpha_AB = 0.0;
	m_sz = 0.0;
	m_A = 0.0;
	m_vx = 0.0;
	m_sx = 0.0;
	m_vtx = 0.0;
	m_vty = 0.0;
	m_vtz = 0.0;
	m_v = 0.0;//地速
	m_v_air = 0.0;//空速
	m_mach = 0.0;
	m_v_average_1s = 0.0;
	m_v_average_10s = 0.0;
	m_vnx = 0.0;
	m_vnz = 0.0;
	m_ny = 0.0;
	m_nz = 0.0;
	
	m_zeta = 0.0;
	m_gama = 0.0;
	m_psit = 0.0;
	m_psit_t_turn_in = 0.0;
	m_psin = 0.0;
	m_psicn = 0.0;

	//导引头输出指令及状态
	 //m_seeker_cmd = 0x01;//导引头控制指令，0x01 自检(上电自动)，0x02 装订目标模板， 0x03 射前检查，0x04/0x05备用，0x06飞控引导搜索(指定框架角)，0x07锁定允许，0x08闭锁，0x09修正跟踪
	m_seeker_cmdpara_targettype = 0x03;//指示目标类型，0x01 车辆，0x02 飞行器，0x03 固定建筑
	m_seeker_cmdpara_dltheight = 1000.0;//弹目高度差，用于估算目标距离m
	m_seeker_cmdpara_dltheight = 0;//弹目高度差
	m_seeker_cmdpara_ktheta = 0;//导弹俯仰角
	m_seeker_cmdpara_psi = 0;//导弹偏航角
	m_seeker_cmdpara_gama = 0;//导弹滚转角
	m_seeker_cmdpara_phif = 0;//引导框架角
	m_seeker_cmdpara_phih = 0;//引导框架角
	//m_seeker_state = 0x00;//带引头工作状态，0xAX 正常，0xFX异常，0x5X 过程中
	m_seeker_state_track = 0x01;//导引头跟踪状态，0x01电锁零位,0x02搜索（或失锁），0x03闭锁，0x04跟踪
	m_seeker_dqf = 0.0;	//视线角速度
	m_seeker_dqh = 0.0;
	m_seeker_phif = 0.0;//框架角（状态）
	m_seeker_phih = 0.0;
	m_seeker_qf = 0.0;	//视线角（状态）
	m_seeker_qh = 0.0;
	m_seeker_pixelf = 0;//俯仰像素偏差，左负右正
	m_seeker_pixelh = 0;//航向像素偏差，下负上正
	m_seeker_distance_target = 0.0;//弹目距离

	//高度表
	m_radioalt_hight = 0.0;
	m_radioalt_status = 0x00;
	//空速管
	m_static_pressure = 0.0;
	m_total_pressure = 0.0;
	m_baroalt_status = 0x00;
	m_hbaro = 0.0;//气压高度

	//虚拟目标导引 与 导引头信息综合后数据
	m_dqf = 0.0;
	m_dqh = 0.0;
	m_phif = 0.0;
	m_phih = 0.0;
	m_Qf = 0.0;
	m_Qh = 0.0;
	
	m_turn_angle = 0.0;
	m_turn_radius = 0.0;
	m_target_velocity = 0.0;//航点目标速度
	m_target_height = 0.0;	//航点目标高度
	m_gama_turn_nominal = 0.0;//BTT转弯滚转角标称值
	m_x_coordinate_turn = 0.0;//圆心坐标
	m_z_coordinate_turn = 0.0;
	
	valid_count = 0;
	count_qk = 0;	
	count_v_5 = 0;
	count_fl = 0;	
	count_qd = 0;
	count_v_50 = 0;
	count_tg = 0;
	count_virtual = 0;
	count_cooperative_attack = 0;
	count_altitude_change = 0;
	count_altitude_change_enable = 0;
	count_altitude_change_end = 0;
	count_away = 0;
	count_sd_in = 0;
	count_turn_out = 0;
	count_update = 0;
	m_num_way_point = 0;
	m_num_way_point_target = 0;
	m_engine_start_result = 0xBB;
	m_engine_state_rpm = 0;
	m_ECU_work_cmd = 0x11;//待机
	m_ground_temperature = 0.0;	
	// p_st_debug_monitor = NULL;
	p_st_initial_data = NULL;
	p_st_route_data_preflight = NULL;
	p_st_flight_basic_input = NULL;	//输入
	p_st_flight_basic_output = NULL;//输出
	memset(&m_st_target		 , 0, sizeof(Stru_Way_Point));
	memset(&m_st_way_point	 , 0, sizeof(Stru_Way_Point) * MAX_ROUTE_NUMBER);
	memset(&m_st_control_flag, 0, sizeof(Stru_Control_flag));
	memset(&m_st_control_time, 0, sizeof(Stru_Control_Time));
	memset(&m_v_record_100ms , 0, sizeof(double) * 10);
	memset(&m_v_record_1s    , 0, sizeof(double) * 10);
	m_st_control_time.time_control = MAX_TIME;
	m_st_control_time.time_separate_booster = MAX_TIME;
	m_st_control_time.time_launch_missile_wing = MAX_TIME;
	m_st_control_time.time_seeker_on = MAX_TIME;
	m_st_control_time.time_engine_start = MAX_TIME;
	m_st_control_time.time_engine_start_finish = MAX_TIME;
	m_st_control_time.time_altitude_control = MAX_TIME;
	m_st_control_time.time_combat_status = MAX_TIME;
	m_st_control_time.time_altitude_change_start = MAX_TIME;
	m_st_control_time.time_altitude_change_end = MAX_TIME;
	m_st_control_time.time_turn_in_start = MAX_TIME;
	m_st_control_time.time_turn_in_end = MAX_TIME;
	m_st_control_time.time_turn_out_start = MAX_TIME;
	m_st_control_time.time_turn_out_end = MAX_TIME;
	m_st_control_time.time_turn_in_minimum = MAX_TIME;
	m_st_control_time.time_arrive_minimum = MAX_TIME;
	m_st_control_time.time_cooperative_attack = MAX_TIME;
	m_time_to_go = MAX_TIME;

	m_fuel_comsumped = 0.0;
	m_mass_calc = FULL_MASS;
	m_left_flight_time_calc = MAX_FLIGHT_TIME;
	m_left_flight_dist_calc = MAX_FLIGHT_DIST;
}

//飞行控制载入航迹，设置初始任务
void CMathControlFlightBasic::Update_Task_Info()
{
	//载入地表环境温度
	m_ground_temperature = p_st_initial_data->initial_parameter1;
	m_missile_flight_mode = (int)p_st_initial_data->initial_parameter2;
	
	//载入射前规划航路数据
	int num_column = p_st_route_data_preflight->num_columns;
	m_num_way_point = p_st_route_data_preflight->num_rows;
	for (int count_num=0; count_num<m_num_way_point; count_num++)
	{
		//原始数据
		m_st_way_point[count_num].num	= count_num;
		m_st_way_point[count_num].longitude		= p_st_route_data_preflight->p_route_data[count_num * num_column + 1];
		m_st_way_point[count_num].latitude		= p_st_route_data_preflight->p_route_data[count_num * num_column + 2];
		m_st_way_point[count_num].height		= p_st_route_data_preflight->p_route_data[count_num * num_column + 3];
		m_st_way_point[count_num].route_mode	= (int)p_st_route_data_preflight->p_route_data[count_num * num_column + 4];
		m_st_way_point[count_num].formation_mode= (int)p_st_route_data_preflight->p_route_data[count_num * num_column + 5];
		m_st_way_point[count_num].dltTime		= p_st_route_data_preflight->p_route_data[count_num * num_column + 6];
		m_st_way_point[count_num].turn_angle	= p_st_route_data_preflight->p_route_data[count_num * num_column + 7];
		m_st_way_point[count_num].turn_radius	= p_st_route_data_preflight->p_route_data[count_num * num_column + 8];
		m_st_way_point[count_num].velocity		= p_st_route_data_preflight->p_route_data[count_num * num_column + 9];
		m_st_way_point[count_num].accept_radius	= p_st_route_data_preflight->p_route_data[count_num * num_column + 10];
		//处理后数据:标识
		/*[到达时间标识；1有效，0无效；]*/		
		if(m_st_way_point[count_num].dltTime > 0)
			m_st_way_point[count_num].if_flightime_ctrl = 1;
		else
			m_st_way_point[count_num].if_flightime_ctrl = 0;	
		/*[相对高度（或真高度）控制标识：1有效，0无效]*/     		
		if(m_st_way_point[count_num].height < 0)
			m_st_way_point[count_num].if_relativehigh_ctrl = 1;
		else
			m_st_way_point[count_num].if_relativehigh_ctrl = 0;	
		/*[指点飞行标识：1有效，0无效；]*/
		if(m_st_way_point[count_num].route_mode == 2)
		{
			m_st_way_point[count_num].if_heading_hold = 1;
			m_st_way_point[count_num].outtrack_angle = m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_heading_hold = 0;	
			m_st_way_point[count_num].outtrack_angle = 0.0;
		}
		/*[地速控制标识：1地速，0空速；]*/
		if(m_st_way_point[count_num].velocity < 0)
			m_st_way_point[count_num].if_groundspeed_ctrl = 1;
		else
			m_st_way_point[count_num].if_groundspeed_ctrl = 0;	
		/*[打击落角标识：1指定落角，0无约束；]*/	
		if(((m_st_way_point[count_num].route_mode == 4) || (m_st_way_point[count_num].route_mode == 5))&&(m_st_way_point[count_num].turn_angle < 0.0))
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 1;//复用为打击落角
			m_st_way_point[count_num].attack_angle = - m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 0;	
			m_st_way_point[count_num].attack_angle = 0.0;
		}
		
		if(m_st_way_point[count_num].route_mode == 3)
		{	
			/*[预盘旋标识：1有效，0无效；]*/
			m_st_way_point[count_num].if_prepare_hover = 1;
			m_st_way_point[count_num].hover_round = (int)m_st_way_point[count_num].turn_angle;
			/*[盘旋转弯方向标识：左转1、右转0；]*/	
			if(m_st_way_point[count_num].hover_round > 0)
				m_st_way_point[count_num].if_turndir_set = 1;
			else
				m_st_way_point[count_num].if_turndir_set = 0;	
		}
		else
		{
			m_st_way_point[count_num].if_prepare_hover = 0;
			m_st_way_point[count_num].hover_round = 0;
		}
	}
	
	//载入目标点数据
	m_st_target.longitude	= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 1];
	m_st_target.latitude	= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 2];
	m_st_target.height		= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 3];

	//计算转弯角及总航程
	double distance_BC = 0.0;
	double distance_delta = 0.0;
	double alpha_BC = 0.0;
	CFlightGlobalFun::Tomas(p_st_initial_data->longitude_launch, p_st_initial_data->latitude_launch,
		m_st_way_point[0].longitude, m_st_way_point[0].latitude,
		&m_total_distance, &m_alpha_AB);

	for(int i=0; i<(m_num_way_point - 1); i++)
	{
		CFlightGlobalFun::Tomas(m_st_way_point[i].longitude, m_st_way_point[i].latitude,
			m_st_way_point[i + 1].longitude, m_st_way_point[i + 1].latitude,
			&distance_BC, &alpha_BC);		
		m_st_way_point[i].turn_angle = alpha_BC - m_alpha_AB;
		m_st_way_point[i].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[i].turn_angle, 180.0);
		m_alpha_AB = alpha_BC;//更新前一次航迹角
		m_total_distance += distance_BC;
		distance_delta = 2 * m_st_way_point[i].turn_radius 
						* (PI * fabs(m_st_way_point[i].turn_angle) / 360.0 
						- tan((fabs(m_st_way_point[i].turn_angle) / 2.0) / RTOA));
		m_total_distance += distance_delta;
	}
}
/*
void CMathControlFlightBasic::Change_Task_Info_Online()
{
	//在航迹点索引表后，加入数据链发送的航迹序列，覆盖原航迹、总航点更新，计算总航程
	m_num_way_point = m_num_way_point_target 
		+ p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].num_waypoint_updated;//当前航点 加 新增航点数
	for (int count_num=m_num_way_point_target; count_num<m_num_way_point; count_num++)
	{
		m_st_way_point[count_num].num = count_num;
		m_st_way_point[count_num].longitude		= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].longitude[count_num - m_num_way_point_target];
		m_st_way_point[count_num].latitude		= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].latitude[count_num - m_num_way_point_target];
		m_st_way_point[count_num].height		= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].height[count_num - m_num_way_point_target];
		m_st_way_point[count_num].route_mode	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].route_mode[count_num - m_num_way_point_target];
		m_st_way_point[count_num].formation_mode	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].formation_mode[count_num - m_num_way_point_target];
		m_st_way_point[count_num].dltTime	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].dltTime[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_angle	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].turn_angle[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_radius	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].turn_radius[count_num - m_num_way_point_target];
		m_st_way_point[count_num].velocity	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].velocity[count_num - m_num_way_point_target];
		m_st_way_point[count_num].accept_radius	= 
			p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].accept_radius[count_num - m_num_way_point_target];		
		
		//处理后数据:标识
		//[到达时间标识；1有效，0无效；]
		if(m_st_way_point[count_num].dltTime > 0)
			m_st_way_point[count_num].if_flightime_ctrl = 1;
		else
			m_st_way_point[count_num].if_flightime_ctrl = 0;	
		//[相对高度（或真高度）控制标识：1有效，0无效]
		if(m_st_way_point[count_num].height < 0)
			m_st_way_point[count_num].if_relativehigh_ctrl = 1;
		else
			m_st_way_point[count_num].if_relativehigh_ctrl = 0;	
		//[指点飞行标识：1有效，0无效；]
		if(m_st_way_point[count_num].route_mode == 2)
			m_st_way_point[count_num].if_heading_hold = 1;
		else
			m_st_way_point[count_num].if_heading_hold = 0;	
		//[地速控制标识：1地速，0空速；]
		if(m_st_way_point[count_num].velocity < 0)
			m_st_way_point[count_num].if_groundspeed_ctrl = 1;
		else
			m_st_way_point[count_num].if_groundspeed_ctrl = 0;	
		//[打击落角标识：1指定落角，0无约束；]
		if(((m_st_way_point[count_num].route_mode == 4) || (m_st_way_point[count_num].route_mode == 5))&&(m_st_way_point[count_num].turn_angle < 0.0))
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 1;//复用为打击落角
			m_st_way_point[count_num].attack_angle = -m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 0;	
			m_st_way_point[count_num].attack_angle = 0.0;
		}
		
		if(m_st_way_point[count_num].route_mode == 3)
		{	
			//[预盘旋标识：1有效，0无效；]
			m_st_way_point[count_num].if_prepare_hover = 1;
			m_st_way_point[count_num].hover_round = (int)m_st_way_point[count_num].turn_angle;
			//[盘旋转弯方向标识：左转1、右转0；]	
			if(m_st_way_point[count_num].hover_round > 0)
				m_st_way_point[count_num].if_turndir_set = 1;
			else
				m_st_way_point[count_num].if_turndir_set = 0;	
		}
		else
		{
			m_st_way_point[count_num].if_prepare_hover = 0;
			m_st_way_point[count_num].hover_round = 0;
		}
	}
	//最后航迹点高度，设置为目标点高度
	m_st_way_point[m_num_way_point - 1].height = m_st_target.height;
	//更新目标点经度、维度
	m_st_target.longitude	= m_st_way_point[m_num_way_point - 1].longitude;
	m_st_target.latitude	= m_st_way_point[m_num_way_point - 1].latitude;

	//初始航线: 发射点到第一个航迹点，距离、方位
	CFlightGlobalFun::Tomas(p_st_initial_data->longitude_launch, p_st_initial_data->latitude_launch,
		m_st_way_point[0].longitude, m_st_way_point[0].latitude,
		&m_total_distance, &m_alpha_AB);
	
	//计算转弯角及总航程
	double distance_BC = 0.0;
	double distance_delta = 0.0;
	double alpha_BC = 0.0;
	for(int i=0; i<(m_num_way_point - 1); i++)
	{
		CFlightGlobalFun::Tomas(m_st_way_point[i].longitude, m_st_way_point[i].latitude,
			m_st_way_point[i + 1].longitude, m_st_way_point[i + 1].latitude,
			&distance_BC, &alpha_BC);		
		m_st_way_point[i].turn_angle = alpha_BC - m_alpha_AB;
		m_st_way_point[i].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[i].turn_angle, 180.0);
		m_alpha_AB = alpha_BC;
		m_total_distance += distance_BC;
		distance_delta = 2 * m_st_way_point[i].turn_radius 
			* (PI * fabs(m_st_way_point[i].turn_angle) / 360.0 
			- tan((fabs(m_st_way_point[i].turn_angle) / 2.0) / RTOA));
		m_total_distance += distance_delta;
	}

	//更新当前目标点转弯角度
	////////目标点到下一点距离、方位
	CFlightGlobalFun::Tomas(m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		m_st_way_point[m_num_way_point_target + 1].longitude, m_st_way_point[m_num_way_point_target + 1].latitude,
		&distance_BC, &alpha_BC);
	////////当前点到目标点距离、方位
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		&distance_BC, &m_alpha_AB);
	////////两航线偏差，作为目标点转弯角
	m_st_way_point[m_num_way_point_target].turn_angle = alpha_BC - m_alpha_AB;
	m_st_way_point[m_num_way_point_target].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[m_num_way_point_target].turn_angle, 180.0);

	//更新当前飞行状态
	////////当前点、目标点及下一点，即数据链每次至少发送两个点
	m_longitude_A = m_longitude;
	m_latitude_A = m_latitude;
	m_longitude_B = m_st_way_point[m_num_way_point_target].longitude;
	m_latitude_B = m_st_way_point[m_num_way_point_target].latitude;
	m_longitude_C = m_st_way_point[m_num_way_point_target + 1].longitude;
	m_latitude_C = m_st_way_point[m_num_way_point_target + 1].latitude;
	////////转弯信息更新
	m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;
	m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;
	m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;
	////////重建航线，计算距离、方位角
	double distance_AB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
		m_longitude_B, m_latitude_B, 
		&distance_AB, &m_A);
	m_A = - m_A;
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);

	////////计算最小可能到达时间，转弯标识设置为无效
	double angle_PB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_longitude_B, m_latitude_B,
		&m_distance_BP, &angle_PB);
	double temp_v = m_v_average_10s + 20.0;
	if(temp_v >=  VEL_COMMAND_MAX_LIMIT)
	{
		temp_v = VEL_COMMAND_MAX_LIMIT; 
	}
	else if(temp_v <= VEL_COMMAND_MIN_LIMIT)
	{
		temp_v = VEL_COMMAND_MIN_LIMIT;
	}
	double distance_to_go = fabs(m_distance_BP - m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0));
	double min_time = distance_to_go / temp_v;
	m_st_control_time.time_arrive_minimum = flight_time + min_time;
	m_st_control_flag.flag_waypoint_turn = false;
}*/
void CMathControlFlightBasic::Change_Task_Info_Online()
{
	//在航迹点索引表后，加入数据链发送的航迹序列，覆盖原航迹、总航点更新，计算总航程
	m_num_way_point = m_num_way_point_target 
		+ p_st_flight_basic_input->st_datalink_datasig.num_waypoint_updated;//当前航点 加 新增航点数
	for (int count_num=m_num_way_point_target; count_num<m_num_way_point; count_num++)
	{
		m_st_way_point[count_num].num = count_num;
		m_st_way_point[count_num].longitude		= 
			p_st_flight_basic_input->st_datalink_datasig.longitude[count_num - m_num_way_point_target];
		m_st_way_point[count_num].latitude		= 
			p_st_flight_basic_input->st_datalink_datasig.latitude[count_num - m_num_way_point_target];
		m_st_way_point[count_num].height		= 
			p_st_flight_basic_input->st_datalink_datasig.height[count_num - m_num_way_point_target];
		m_st_way_point[count_num].route_mode	= 
			p_st_flight_basic_input->st_datalink_datasig.route_mode[count_num - m_num_way_point_target];
		m_st_way_point[count_num].formation_mode	= 
			p_st_flight_basic_input->st_datalink_datasig.formation_mode[count_num - m_num_way_point_target];
		m_st_way_point[count_num].dltTime	= 
			p_st_flight_basic_input->st_datalink_datasig.dltTime[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_angle	= 
			p_st_flight_basic_input->st_datalink_datasig.turn_angle[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_radius	= 
			p_st_flight_basic_input->st_datalink_datasig.turn_radius[count_num - m_num_way_point_target];
		m_st_way_point[count_num].velocity	= 
			p_st_flight_basic_input->st_datalink_datasig.velocity[count_num - m_num_way_point_target];
		m_st_way_point[count_num].accept_radius	= 
			p_st_flight_basic_input->st_datalink_datasig.accept_radius[count_num - m_num_way_point_target];		
		
		//处理后数据:标识
		//[到达时间标识；1有效，0无效；]
		if(m_st_way_point[count_num].dltTime > 0)
			m_st_way_point[count_num].if_flightime_ctrl = 1;
		else
			m_st_way_point[count_num].if_flightime_ctrl = 0;	
		//[相对高度（或真高度）控制标识：1有效，0无效]
		if(m_st_way_point[count_num].height < 0)
			m_st_way_point[count_num].if_relativehigh_ctrl = 1;
		else
			m_st_way_point[count_num].if_relativehigh_ctrl = 0;	
		//[指点飞行标识：1有效，0无效；]
		if(m_st_way_point[count_num].route_mode == 2)
			m_st_way_point[count_num].if_heading_hold = 1;
		else
			m_st_way_point[count_num].if_heading_hold = 0;	
		//[地速控制标识：1地速，0空速；]
		if(m_st_way_point[count_num].velocity < 0)
			m_st_way_point[count_num].if_groundspeed_ctrl = 1;
		else
			m_st_way_point[count_num].if_groundspeed_ctrl = 0;	
		//[打击落角标识：1指定落角，0无约束；]
		if(((m_st_way_point[count_num].route_mode == 4) || (m_st_way_point[count_num].route_mode == 5))&&(m_st_way_point[count_num].turn_angle < 0.0))
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 1;//复用为打击落角
			m_st_way_point[count_num].attack_angle = -m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 0;	
			m_st_way_point[count_num].attack_angle = 0.0;
		}
		
		if(m_st_way_point[count_num].route_mode == 3)
		{	
			//[预盘旋标识：1有效，0无效；]
			m_st_way_point[count_num].if_prepare_hover = 1;
			m_st_way_point[count_num].hover_round = (int)m_st_way_point[count_num].turn_angle;
			//[盘旋转弯方向标识：左转1、右转0；]	
			if(m_st_way_point[count_num].hover_round > 0)
				m_st_way_point[count_num].if_turndir_set = 1;
			else
				m_st_way_point[count_num].if_turndir_set = 0;	
		}
		else
		{
			m_st_way_point[count_num].if_prepare_hover = 0;
			m_st_way_point[count_num].hover_round = 0;
		}
	}
	//最后航迹点高度，设置为目标点高度
	m_st_way_point[m_num_way_point - 1].height = m_st_target.height;
	//更新目标点经度、维度
	m_st_target.longitude	= m_st_way_point[m_num_way_point - 1].longitude;
	m_st_target.latitude	= m_st_way_point[m_num_way_point - 1].latitude;

	//初始航线: 发射点到第一个航迹点，距离、方位
	CFlightGlobalFun::Tomas(p_st_initial_data->longitude_launch, p_st_initial_data->latitude_launch,
		m_st_way_point[0].longitude, m_st_way_point[0].latitude,
		&m_total_distance, &m_alpha_AB);
	
	//计算转弯角及总航程
	double distance_BC = 0.0;
	double distance_delta = 0.0;
	double alpha_BC = 0.0;
	for(int i=0; i<(m_num_way_point - 1); i++)
	{
		CFlightGlobalFun::Tomas(m_st_way_point[i].longitude, m_st_way_point[i].latitude,
			m_st_way_point[i + 1].longitude, m_st_way_point[i + 1].latitude,
			&distance_BC, &alpha_BC);		
		m_st_way_point[i].turn_angle = alpha_BC - m_alpha_AB;
		m_st_way_point[i].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[i].turn_angle, 180.0);
		m_alpha_AB = alpha_BC;
		m_total_distance += distance_BC;
		distance_delta = 2 * m_st_way_point[i].turn_radius 
			* (PI * fabs(m_st_way_point[i].turn_angle) / 360.0 
			- tan((fabs(m_st_way_point[i].turn_angle) / 2.0) / RTOA));
		m_total_distance += distance_delta;
	}

	//更新当前目标点转弯角度
	////////目标点到下一点距离、方位
	CFlightGlobalFun::Tomas(m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		m_st_way_point[m_num_way_point_target + 1].longitude, m_st_way_point[m_num_way_point_target + 1].latitude,
		&distance_BC, &alpha_BC);
	////////当前点到目标点距离、方位
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		&distance_BC, &m_alpha_AB);
	////////两航线偏差，作为目标点转弯角
	m_st_way_point[m_num_way_point_target].turn_angle = alpha_BC - m_alpha_AB;
	m_st_way_point[m_num_way_point_target].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[m_num_way_point_target].turn_angle, 180.0);

	//更新当前飞行状态
	////////当前点、目标点及下一点，即数据链每次至少发送两个点
	m_longitude_A = m_longitude;
	m_latitude_A = m_latitude;
	m_longitude_B = m_st_way_point[m_num_way_point_target].longitude;
	m_latitude_B = m_st_way_point[m_num_way_point_target].latitude;
	m_longitude_C = m_st_way_point[m_num_way_point_target + 1].longitude;
	m_latitude_C = m_st_way_point[m_num_way_point_target + 1].latitude;
	////////转弯信息更新
	m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;
	m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;
	m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;
	////////重建航线，计算距离、方位角
	double distance_AB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
		m_longitude_B, m_latitude_B, 
		&distance_AB, &m_A);
	m_A = - m_A;
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);

	////////计算最小可能到达时间，转弯标识设置为无效
	double angle_PB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_longitude_B, m_latitude_B,
		&m_distance_BP, &angle_PB);
	double temp_v = m_v_average_10s + 20.0;
	if(temp_v >=  VEL_COMMAND_MAX_LIMIT)
	{
		temp_v = VEL_COMMAND_MAX_LIMIT; 
	}
	else if(temp_v <= VEL_COMMAND_MIN_LIMIT)
	{
		temp_v = VEL_COMMAND_MIN_LIMIT;
	}
	double distance_to_go = fabs(m_distance_BP - m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0));
	double min_time = distance_to_go / temp_v;
	m_st_control_time.time_arrive_minimum = flight_time + min_time;
	m_st_control_flag.flag_waypoint_turn = false;
}
void CMathControlFlightBasic::Initial()
{
	//射前导弹任务参数装订
	Update_Task_Info();
	
	//初始位置、前面A\B\C点位置
	m_longitude = p_st_initial_data->longitude_launch;
	m_latitude = p_st_initial_data->latitude_launch;
	m_hz = p_st_initial_data->height_launch;
	m_longitude_A = p_st_initial_data->longitude_launch;
	m_latitude_A = p_st_initial_data->latitude_launch;
	m_longitude_B = m_st_way_point[0].longitude;
	m_latitude_B = m_st_way_point[0].latitude;
	m_longitude_C = m_st_way_point[1].longitude;
	m_latitude_C = m_st_way_point[1].latitude;
	
	//计算初始航线：方位、距离
	double distance_AB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
		m_longitude_B, m_latitude_B, 
		&distance_AB, &m_A);
	m_A = - m_A;//“北偏东”转换为“北偏西”
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);
	m_turn_angle = m_st_way_point[0].turn_angle;
	m_turn_radius = m_st_way_point[0].turn_radius;
	m_target_velocity = m_st_way_point[0].velocity;
	m_target_height = m_st_way_point[0].height;
	
	m_missile_ID = p_st_flight_basic_input->missile_ID;
	//count_update = p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count;
	count_update = p_st_flight_basic_input->st_datalink_datasig.update_count;
}

void CMathControlFlightBasic::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	// Monitor_Data();
}

void CMathControlFlightBasic::Get_Data()
{
	m_missile_ID = p_st_flight_basic_input->missile_ID;

	//高度表数据
	m_radioalt_hight = p_st_flight_basic_input->st_radioalt_data.radioalt_hight;
	m_radioalt_status = p_st_flight_basic_input->st_radioalt_data.radioalt_status;

	//空速管数据
	m_static_pressure = p_st_flight_basic_input->st_baro_data.static_pressure;
	m_total_pressure = p_st_flight_basic_input->st_baro_data.total_pressure;

	//发动机数据
	m_state_rpm = p_st_flight_basic_input->st_engine_data.rpm_engine;
	//发动机控制数据
#ifdef	__VEL__CONTROL__MODE__KC__	
	m_cmd_Kc = p_st_flight_basic_input->engine_cmd_Kc;	//油门控制模式
#else
	m_cmd_rpm = p_st_flight_basic_input->engine_cmd_rpm;	//转速控制模式
#endif
	
	m_au = p_st_flight_basic_input->st_ins_data.au;
	m_hgps = p_st_flight_basic_input->st_ins_data.height;
	m_longitude = p_st_flight_basic_input->st_ins_data.longitude;
	m_latitude = p_st_flight_basic_input->st_ins_data.latitude;
	m_vtx = p_st_flight_basic_input->st_ins_data.vtx;
	m_vty = p_st_flight_basic_input->st_ins_data.vty;
	m_vtz = p_st_flight_basic_input->st_ins_data.vtz;
	m_ax = p_st_flight_basic_input->st_ins_data.ax;
	m_ay = p_st_flight_basic_input->st_ins_data.ay;
	m_az = p_st_flight_basic_input->st_ins_data.az;
	m_gama = p_st_flight_basic_input->st_ins_data.gama;
	m_zeta = p_st_flight_basic_input->st_ins_data.zeta;
	m_psit = p_st_flight_basic_input->st_ins_data.psi;
	
	m_engine_state_rpm = p_st_flight_basic_input->engine_rpm;
	m_engine_start_result = p_st_flight_basic_input->engine_start_result;
}
void CMathControlFlightBasic::Calc_Data()
{
	//数据链在线更新航迹点、任务，有新航迹点，转弯过程中不更新
	//if ((count_update != p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count)
	if ((count_update != p_st_flight_basic_input->st_datalink_datasig.update_count)
		&&(!m_st_control_flag.flag_waypoint_turn))
	{
		Change_Task_Info_Online();
		//count_update = p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count;
		count_update = p_st_flight_basic_input->st_datalink_datasig.update_count;
	}
		
	//飞行计算
	Calc_Flight_Data();
	//估算质量
	Calc_Mass_Data();
	
	//视线角速度等计算
	Calc_LOS_Rate();	
	//指令时序
	Calc_Command();	
	//侧向机动
	Control_Turn();
	//高度机动
	Control_Altitude_Change();
}
void CMathControlFlightBasic::Send_Data()
{
	p_st_flight_basic_output->mass_calc = m_mass_calc;
	p_st_flight_basic_output->h_ini = p_st_initial_data->height_launch;
	p_st_flight_basic_output->hz = m_hz;
	p_st_flight_basic_output->nby = m_ny;
	p_st_flight_basic_output->nbz = m_nz;
	p_st_flight_basic_output->v = m_v;
	p_st_flight_basic_output->vs = m_vs;
	p_st_flight_basic_output->vnx = m_vnx;
	p_st_flight_basic_output->vnz = m_vnz;
	p_st_flight_basic_output->mach = m_mach;
	p_st_flight_basic_output->sz = m_sz;
	p_st_flight_basic_output->dynamic_pressure = m_dynamic_pressure;//新增
	p_st_flight_basic_output->zeta = m_zeta;
	p_st_flight_basic_output->gama = m_gama;
	p_st_flight_basic_output->wx = p_st_flight_basic_input->st_ins_data.wx;
	p_st_flight_basic_output->wy = p_st_flight_basic_input->st_ins_data.wy;
	p_st_flight_basic_output->wz = p_st_flight_basic_input->st_ins_data.wz;
	p_st_flight_basic_output->g = m_g;
	p_st_flight_basic_output->dqf = m_dqf;
	p_st_flight_basic_output->dqh = m_dqh;	
	p_st_flight_basic_output->phif = m_phif;
	p_st_flight_basic_output->phih = m_phih;
	p_st_flight_basic_output->qf = m_Qf;
	p_st_flight_basic_output->qh = m_Qh;
	p_st_flight_basic_output->time_to_go = m_time_to_go;
	p_st_flight_basic_output->angle_zw = m_turn_angle;
	p_st_flight_basic_output->radius_zw = m_turn_radius;
	p_st_flight_basic_output->target_velocity = m_target_velocity;
	p_st_flight_basic_output->target_height = m_target_height;	
	p_st_flight_basic_output->target_long = m_longitude_B;//新增
	p_st_flight_basic_output->target_lat = m_latitude_B;//新增
	p_st_flight_basic_output->ground_temperature = m_ground_temperature;
	p_st_flight_basic_output->count_altitude_change = count_altitude_change;//待计算	
	p_st_flight_basic_output->num_way_point_target = m_num_way_point_target;
	p_st_flight_basic_output->target_time = MAX_TIME;	//预留待完善
	p_st_flight_basic_output->gama_turn_nominal = m_gama_turn_nominal;
	p_st_flight_basic_output->distance_target = m_distance_target;
	p_st_flight_basic_output->distance_target_t_combat = m_distance_target_t_combat;
	p_st_flight_basic_output->velocity_average_10s = m_v_average_10s;
	p_st_flight_basic_output->ECU_work_cmd = m_ECU_work_cmd;

	p_st_flight_basic_output->token_long = 0x00;//纵向，未使用
	p_st_flight_basic_output->token_lat = 0x00;;//侧向，未使用
	p_st_flight_basic_output->dlt_psic = 0.0;;//航迹角偏差，未使用
	p_st_flight_basic_output->sz_circle = 0.0;//圆轨迹侧偏距，未使用
	p_st_flight_basic_output->theta = 0.0;//弹道倾角，未使用
	p_st_flight_basic_output->alpha_vg = 0.0;//地速攻角，未使用
	p_st_flight_basic_output->beita_vg = 0.0;//地速侧滑角，未使用
	
	p_st_flight_basic_output->flag_launch_turn = m_st_control_flag.flag_launch_turn;
	p_st_flight_basic_output->flag_altitude_change = m_st_control_flag.flag_alltitude_change;
	p_st_flight_basic_output->flag_waypoint_turn = m_st_control_flag.flag_waypoint_turn;
	p_st_flight_basic_output->flag_altitude_climb = m_st_control_flag.flag_alltitude_climb;
	p_st_flight_basic_output->flag_altitude_decline = m_st_control_flag.flag_alltitude_decline;
	
	p_st_flight_basic_output->st_command.flag_separate_booster = m_st_control_flag.flag_separate_booster;
	p_st_flight_basic_output->st_command.flag_launch_missile_wing = m_st_control_flag.flag_launch_missile_wing;
	p_st_flight_basic_output->st_command.flag_engine_start = m_st_control_flag.flag_engine_start;
	p_st_flight_basic_output->st_command.flag_seeker_on = m_st_control_flag.flag_seeker_on;
	p_st_flight_basic_output->st_command.flag_lock_on_permit = m_st_control_flag.flag_lock_on_permit;
	p_st_flight_basic_output->st_command.flag_combat_status = m_st_control_flag.flag_combat_status;

	//框架用
	p_st_flight_basic_output->st_command.flag_missile_takeoff = m_st_control_flag.flag_missile_takeoff;
	p_st_flight_basic_output->st_command.flag_engine_shutdown = m_st_control_flag.flag_engine_shutdown;
	p_st_flight_basic_output->st_command.flag_open_umbrella = m_st_control_flag.flag_open_umbrella;
		
	memcpy(&p_st_flight_basic_output->st_control_time,&m_st_control_time,sizeof(Stru_Control_Time));
	if (0x11 == m_st_way_point[m_num_way_point_target].route_mode)
	{
		p_st_flight_basic_output->flag_fire_distribution = true;
	}
	else
	{
		p_st_flight_basic_output->flag_fire_distribution = false;
	}
}
void CMathControlFlightBasic::Calc_Command()
{
	//计算指令时刻
	
	////1.计算启控时刻
	////1.1主要条件
	if((m_sx >= 3.31)
		&&(m_v >= 5.0))
	{
		count_qk++;
	}
	else
	{
		count_qk = 0;
	}

	if((count_qk >= 3)
		&&(!m_st_control_flag.flag_control_set))
	{
		m_st_control_flag.flag_control_set = true;
		m_st_control_time.time_control = flight_time + 0.17;
	}
	////1.2备份保险条件
	if(m_v >= 5.0)
	{
		count_v_5++;
	}
	else
	{
		count_v_5 = 0;
	}

	if(count_v_5 >= 3)
	{
		if((flight_time >= 0.8)
			&&(!m_st_control_flag.flag_control_set))
		{
			m_st_control_flag.flag_control_set = true;
			m_st_control_time.time_control = flight_time;
		}
	}

	////2.计算助推器分离、弹翼展开、发动机点火及导引头开机时刻
	////2.1主要条件
	if((flight_time >= 1.0)
		&&((m_ax - m_g * sin(m_zeta/RTOA)) <= 2.0)
		&&(m_v >= 50.0))
	{
		count_fl++;
	}
	else
	{
		count_fl = 0;
	}

	if((count_fl >= 3)
		&&(!m_st_control_flag.flag_separate_booster_set))
	{
		m_st_control_flag.flag_separate_booster_set = true;
		m_st_control_flag.flag_launch_missile_wing_set = true; 
		m_st_control_flag.flag_engine_start_set = true;
		m_st_control_flag.flag_seeker_on_set = true;//???...
		
		m_st_control_time.time_separate_booster = flight_time + 0.2;
		m_st_control_time.time_launch_missile_wing = 
			m_st_control_time.time_separate_booster + 0.2;
		m_st_control_time.time_engine_start = 
			m_st_control_time.time_separate_booster + 1.0;
		m_st_control_time.time_seeker_on = 
			m_st_control_time.time_separate_booster + 1.0;
	}
	////2.2保护条件
	if(m_v >= 50.0)
	{
		count_v_50++;
	}
	else
	{
		count_v_50 = 0;
	}

	if(count_v_50 >= 3)
	{
		if((flight_time >= 3.0)
			&&(!m_st_control_flag.flag_separate_booster_set))
		{
			m_st_control_flag.flag_separate_booster_set = true;
			m_st_control_flag.flag_launch_missile_wing_set = true;
			m_st_control_flag.flag_engine_start_set = true;
			m_st_control_flag.flag_seeker_on_set = true;
			m_st_control_time.time_separate_booster = flight_time;
			m_st_control_time.time_launch_missile_wing = 
				m_st_control_time.time_separate_booster + 0.2;
			m_st_control_time.time_engine_start = 
				m_st_control_time.time_separate_booster + 1.0;
			m_st_control_time.time_seeker_on = 
				m_st_control_time.time_separate_booster + 1.0;
		}
	}
	//计算高度控制接入时刻: 助推器分离后，垂速小于0m/s
	//主要条件
	if((m_vs < 0.0)
		&&(flight_time > m_st_control_time.time_separate_booster))
	{
		count_tg++;
	}
	else
	{
		count_tg = 0;
	}

	if((count_tg >= 3)
		&&(!m_st_control_flag.flag_altitude_control_set))
	{
		m_st_control_flag.flag_altitude_control_set = true;
		m_st_control_time.time_altitude_control = flight_time + 1.0;

		//认为起飞完成
		m_st_control_flag.flag_missile_takeoff = true;
		m_st_control_time.time_missile_takeoff = flight_time;

	}
	//保护条件
	if((flight_time >= 30.0)
		&&(!m_st_control_flag.flag_altitude_control_set))
	{
		m_st_control_flag.flag_altitude_control_set = true;
		m_st_control_time.time_altitude_control = flight_time;

		//认为起飞完成
		m_st_control_flag.flag_missile_takeoff = true;
		m_st_control_time.time_missile_takeoff = flight_time;
	}

	//计算协同攻击编队调整时刻,弹目距离小于10km
	if((m_distance_target <= 10000.0)
		&&(m_num_way_point_target == (m_num_way_point - 1)))
	{
		count_cooperative_attack++;
	}
	else
	{
		count_cooperative_attack = 0;
	}

	if((count_cooperative_attack >= 3
		&&(m_num_way_point_target == (m_num_way_point - 1)))
		&&(!m_st_control_flag.flag_cooperative_attack_set))
	{
		m_st_control_flag.flag_cooperative_attack_set = true;
		m_st_control_time.time_cooperative_attack = flight_time;
	}
	
	//计算虚拟导引时刻，弹目距离小于2km 或 导引头捕获目标
	if((m_distance_target <= 2000.0)
		&&(m_num_way_point_target == (m_num_way_point - 1)))
	{
		count_virtual++;
	}
	else
	{
		count_virtual = 0;
	}

	if((count_virtual >= 3 
		||(p_st_flight_basic_input->st_seeker_data.flag_combat_status
			&&(m_num_way_point_target == (m_num_way_point - 1))))
	    &&(!m_st_control_flag.flag_combat_status_set))
	{		
		m_st_control_flag.flag_combat_status_set = true;
		m_st_control_time.time_combat_status = flight_time;
		m_distance_target_t_combat = m_distance_target;
	}

	//发动机开机完成，30ms间隔、连续三次有效
	if(!((time_tick - 1) % 30))
	{
		if (0xAA == m_engine_start_result)
		{
			count_qd++;
		}
		else
		{
			count_qd = 0;
		}
	}
	if((count_qd >= 3)
		&&(!m_st_control_flag.flag_engine_start_finish_set))
	{		
		m_st_control_flag.flag_engine_start_finish_set = true;
		m_st_control_time.time_engine_start_finish = flight_time;
		m_ECU_work_cmd = 0x33;	//速度控制
	}

	//置指令标志位

	//置启控标志位
	if((flight_time >= m_st_control_time.time_control)
		&&(!m_st_control_flag.flag_control_on))
	{
		m_st_control_flag.flag_control_on = true;
	}

	//置助推器分离标志位
	if((flight_time >= m_st_control_time.time_separate_booster)
		&&(!m_st_control_flag.flag_separate_booster))
	{
		m_st_control_flag.flag_separate_booster = true;
	}

	//置弹翼展开标志位
	if((flight_time >= m_st_control_time.time_launch_missile_wing)
		&&(!m_st_control_flag.flag_launch_missile_wing))
	{
		m_st_control_flag.flag_launch_missile_wing = true;
	}

	//置发动机点火标志位
	if((flight_time >= m_st_control_time.time_engine_start)
		&&(!m_st_control_flag.flag_engine_start))
	{
		m_st_control_flag.flag_engine_start = true;	//炮启指令
		m_ECU_work_cmd = 0x22;				//启动指令
	}

	//置导引头开机标志位
	if((flight_time >= m_st_control_time.time_seeker_on)
		&&(!m_st_control_flag.flag_seeker_on))
	{
		m_st_control_flag.flag_seeker_on = true;
	}

	//置高度控制接入标志位
	if((flight_time >= m_st_control_time.time_altitude_control)
		&&(!m_st_control_flag.flag_altitude_control))
	{
		m_st_control_flag.flag_altitude_control = true;
	}
	//置允许截获标志位
	if (m_num_way_point_target == (m_num_way_point - 1))
	{
		m_st_control_flag.flag_lock_on_permit = true;
	}
	//置协同攻击编队调整时刻标志位
	if((flight_time >= m_st_control_time.time_cooperative_attack
		&&(m_num_way_point_target == (m_num_way_point - 1)))
		&&(!m_st_control_flag.flag_cooperative_attack))
	{
		m_st_control_flag.flag_cooperative_attack = true;
	}

	//导引头捕获目标或虚拟导引打击
	if(m_missile_flight_mode == 0xAA)
	{
		//置导引时刻标志位
		if((flight_time >= m_st_control_time.time_combat_status
			||(p_st_flight_basic_input->st_seeker_data.flag_combat_status
				&&(m_num_way_point_target == (m_num_way_point - 1))))
		   &&(!m_st_control_flag.flag_combat_status))
		{
			m_st_control_flag.flag_combat_status = true;
			m_st_control_flag.flag_engine_shutdown = true;//发动机关机
			m_ECU_work_cmd = 0x44;				//发动机关机
		}
	}
	//测试训练模式，回收过渡，最后航迹点为开伞点
	else
	{
		//高度下降到目标高度(例如500m)，速度保持不变
		//

		//发动机关机，定高减速
		//m_ECU_work_cmd = 0x44;

		//开伞及开始回收: 已经失速速度，发出开伞指令DO

		//切割伞：落地冲击、速度小于某阈值，发出切割伞指令DO


		//如果目标开伞点距离小于300m，待补充强哥新判据???...
		
		m_st_control_flag.flag_engine_shutdown = true;//发动机关机
		m_st_control_time.time_engine_shutdown = flight_time;
		m_st_control_flag.flag_open_umbrella = true;//开伞回收
		m_st_control_time.time_open_umbrella = flight_time;
	}
}
void CMathControlFlightBasic::Calc_Flight_Data()
{
	//计算组合高度
	Calc_Hz();

	//计算重力加速度
	double sinLat = sin(m_latitude / RTOA);
	m_g = 9.7803 + 0.051799 * sinLat * sinLat - 0.94114e-6 * m_hz;

	//计算导弹到目标航路点间投影距离
	double distance_BA = 0.0;
	double angle_BA = 0.0;
	double angle_BP = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_B, m_latitude_B, 
		m_longitude, m_latitude, 
		&m_distance_BP, &angle_BP);
	angle_BP = CFlightGlobalFun::Adjust(angle_BP, 180.0);
	CFlightGlobalFun::Tomas(m_longitude_B, m_latitude_B, 
		m_longitude_A, m_latitude_A, 
		&distance_BA, &angle_BA);
	angle_BA = CFlightGlobalFun::Adjust(angle_BA, 180.0);
	double alpha = angle_BA - angle_BP;
	alpha = CFlightGlobalFun::Adjust(alpha, 180.0);
	m_distance_BP_projection = m_distance_BP * cos(fabs(alpha / RTOA));

	//计算500ms间隔，当前及前一帧存储
	if(!((time_tick - 1) % 500))
	{
		m_distance_BP_500pre = m_distance_BP_500;
		m_distance_BP_500    = m_distance_BP_projection;
	}

	//计算导弹到目标航线垂直侧向距离
	m_sz = CFlightGlobalFun::CalcDist(m_longitude_A, m_latitude_A,
		m_longitude_B, m_latitude_B, 
		m_longitude, m_latitude);

	//计算导弹轴向飞行距离
	m_vx += (m_ax - m_g * sin(m_zeta / RTOA)) * STEP_5ms;
	m_sx += m_vx * STEP_5ms;

	//计算弹目距离及目标(大地)方位角，即航向视线角
	CFlightGlobalFun::Tomas(m_longitude, m_latitude, 
		m_st_target.longitude, m_st_target.latitude, 
		&m_distance_target, &m_alpha_target);
	m_alpha_target = CFlightGlobalFun::Adjust(m_alpha_target, 180.0);	//与航向角定义反号

	//计算游移方位角，也称地球表面大圆航线飞行，方位角修正，航向角（方位角）的自转速率
	double Adot = - RTOA * (m_vtz * tan(m_latitude / RTOA) / (RE / (1.0 - E_CONST * sinLat * sinLat) + m_hz));
	m_A += Adot * STEP_5ms;//北偏西为正，东向飞行时，方位角减小
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);

	//计算合速度
	m_v = sqrt(m_vtx * m_vtx + m_vs * m_vs + m_vtz * m_vtz);

	//计算气压高度、空速
	Calc_BaroHigh();
	Calc_BaroSpd();

	//计算当前时刻前1s内,空速度均值
	if(!((time_tick - 1) % 100))
	{
		m_v_average_1s = 0.0;
		for(int i=0; i<9; i++)
		{
			m_v_record_100ms[i] = m_v_record_100ms[i+1];
			m_v_average_1s += m_v_record_100ms[i];
		}
		m_v_record_100ms[9] = m_v_air;
		m_v_average_1s += m_v_record_100ms[9];
		m_v_average_1s = m_v_average_1s / 10.0;
	}

	//计算当前时刻前10s内，空速度均值
	if(!((time_tick - 1) % 1000))
	{
		m_v_average_10s = 0.0;
		for(int i=0; i<9; i++)
		{
			m_v_record_1s[i] = m_v_record_1s[i+1];
			m_v_average_10s += m_v_record_1s[i];
		}
		m_v_record_1s[9] = m_v_average_1s;
		m_v_average_10s += m_v_record_1s[9];
		m_v_average_10s = m_v_average_10s / 10.0;
	}

	//空速马赫数计算
	double air_temperature = 273.3 + m_ground_temperature 
		- 0.0065 * (m_hz - p_st_initial_data->height_launch);
	double sonic_speed = 20.0463 * sqrt(air_temperature);
	m_mach  = m_v_average_10s / CFlightGlobalFun::Nozero_FUN(sonic_speed);	

	//计算导航系速度
	double cosA = cos(m_A / RTOA);
	double sinA = sin(m_A / RTOA);
	m_vnx = m_vtx * cosA - m_vtz * sinA; 
	m_vnz = m_vtx * sinA + m_vtz * cosA;

	//计算弹体系过载
	m_ny = m_ay / m_g;
	m_nz = m_az / m_g;

	//计算导航系航向角
	m_psin = m_psit - m_A;
	m_psin = CFlightGlobalFun::Adjust(m_psin, 180.0);

	//计算航迹偏角
	double temp = m_vtx * m_vtx + m_vtz * m_vtz;
	if(fabs(temp) < 1e-10)
	{
		m_psicn = 0.0;
	}
	else
	{
		m_psicn = RTOA * acos(m_vtx / sqrt(temp));
	}
	if(m_vtz >= 0)
		m_psicn = 360.0 - m_psicn;
	m_psicn = CFlightGlobalFun::Adjust(m_psicn, 180.0);

	//计算转弯过程中滚动程序角标称值
	m_gama_turn_nominal = atan(m_v * m_v / m_turn_radius / m_g) * RTOA;
	m_gama_turn_nominal = CFlightGlobalFun::Range(m_gama_turn_nominal, ROLL_COMMAND_DYNMIC_LIMIT);

	//计算转弯过程中导弹到圆心距离、侧向速度
	if (flight_time > m_st_control_time.time_turn_in_start 
		&& flight_time <= m_st_control_time.time_turn_out_end)
	{
		m_x_coordinate_turn = m_x_coordinate_turn + m_vtx * STEP_5ms ;
		m_z_coordinate_turn = m_z_coordinate_turn + m_vtz * STEP_5ms ;
		m_sz = sqrt( m_x_coordinate_turn * m_x_coordinate_turn + m_z_coordinate_turn * m_z_coordinate_turn); 
		m_vnz = - (m_x_coordinate_turn * m_vtx + m_z_coordinate_turn * m_vtz ) / m_sz 
			* CFlightGlobalFun::FSign(m_turn_angle) ; //m_turn_angle待计算
	}

	//计算剩余飞行时间
	double vxz = sqrt(m_vtx * m_vtx + m_vtz * m_vtz);
	double det_A = fabs(m_psicn + m_alpha_target);
	det_A = CFlightGlobalFun::Adjust(det_A, 180.0);
	if((m_distance_target <= 10000.0)
		&&(m_num_way_point_target == (m_num_way_point - 1)))
	{
		m_time_to_go = m_distance_target / (vxz * cos(det_A / RTOA));		
	}
	else
	{
		m_time_to_go = MAX_TIME;
	}

}

//根据控制油门或控制转速，估计油耗，进一步估计消耗燃料、飞行器重量
void CMathControlFlightBasic::Calc_Mass_Data()
{
	//质量估计: 根据速度、转速，估计功率，空气密度比例修正；
	double vel_engine_array[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};	//速度mps
	double rpm_engine_array[9] = {2000, 2500, 3000, 4000, 5000, 5500, 6180, 6800, 7500};//转速rpm
	double Kc_engine_array[9] = {0.0,10.1374, 19.7358, 37.6508, 61.8964, 75.4171, 97.1771,100.0, 113.0};//油门%
	double power_engine_matrix[11*9] = {1115, 2168, 3735, 8811, 13311, 17699, 25044, 33311, 44622, 
		1295, 2417, 4034, 9197, 13622, 18044, 25433, 33744, 45099, 
		1092, 2484, 4448, 10255, 14666, 19177, 26688, 35099, 46511, 
		204.8, 1563, 3663, 10299, 15999, 20888, 28844, 37522, 49211, 
		0, 0, 0, 8626, 15844, 21500, 30400, 39933, 52500, 
		0, 0, 0, 5461, 13766, 19822, 29722, 40433, 54355, 
		0, 0, 0, 1502, 10122, 16333, 26711, 38077, 53200, 
		0, 0, 0, 822.3, 4897, 10922, 21677, 33366, 49255, 
		0, 0, 0, 822.3, 0, 4670, 14333, 26599, 42722, 
		0, 0, 0, 822.3, 0, 0, 6516, 16966, 33988, 
		0, 0, 0, 822.3, 0, 0, 0, 7706, 21699};// 功率W

#ifdef	__VEL__CONTROL__MODE__KC__
	//如果发动机油门控制模式，用从“发动机指令油门”获取的“状态转速”
	m_state_rpm = CFlightGlobalFun::LAQL1(9,  Kc_engine_array, rpm_engine_array, m_cmd_Kc);//根据油门估计转速
#else	
	//如果发动机转速控制模式，用“状态转速”近似等于“指令转速”
	m_state_rpm = m_cmd_rpm;
#endif
	//如果发动机转速传感器正常（转速有效），使用发动机输出转速，否则使用控制指令估计转速
	if(fabs(m_engine_state_rpm - m_state_rpm) < 1000.0)
	{
		m_state_rpm = m_engine_state_rpm;
	}

	//油耗估算方法一:
	//根据速度、转速估计功率
	double temp_power = 0.0;
	temp_power = CFlightGlobalFun::LAQL2(11,  9,  vel_engine_array,  rpm_engine_array, power_engine_matrix, m_v, m_state_rpm);
	//根据高度计算空气密度比，一维插值，用于修正功率
	double hight_engine_array[7] = {0, 1000, 2000, 3000, 4000, 4500,5000};
	double Krho_engine_array[7] = {1, 0.9078, 0.8220, 0.7420, 0.6686, 0.6343, 0.6008};//高度，空气密度比
	double temp_Krho = 1.0;
	temp_Krho = CFlightGlobalFun::LAQL1(7,  hight_engine_array,  Krho_engine_array, m_hz);//空气密度比

	//油耗估算方法二: 
	//根据转速、速度估计发动机地面推力、效率
	//根据高度计算空气密度比，修正推力
	//认为不同高度效率相同，结合推力、速度，估算效率

	//方法三: 根据平飞平衡攻角估算质量或质量偏差
	//根据当前高度估算空气密度
	//根据俯仰角，弹道倾角为零，计算攻角；
	//根据攻角插值计算标称升力系数，计算升力
	//升力与重力平衡，估算重量
	//说明，小升力系数附近飞行，导航计算误差较大0.1~0.3deg，方法误差大；
	//不是小攻角
	//改善估算，小攻角状态可以降低飞行速度，增大攻角，估算质量；
	
	//山东云豹耗油率为0.643 = 0.45kg/(kW.h) ，河南帕格耗油率高为0.779L/(W.h) 
	m_fuel_comsumped += temp_power*1e-3*temp_Krho*0.45*0.005/3600;	//装订参数
	m_mass_calc = FULL_MASS - m_fuel_comsumped;		//装订参数，起飞重量
	if(m_mass_calc < EMPTY_MASS)
	{
		m_mass_calc = EMPTY_MASS;	//剩油等不可消耗质量
	}

	//剩余飞行时间、航程
	m_left_flight_time_calc = (45 - m_fuel_comsumped)*3600/(temp_power*1e-3*temp_Krho*0.45);
	m_left_flight_dist_calc = m_left_flight_time_calc*m_v;
	
}
	
void CMathControlFlightBasic::Calc_LOS_Rate()
{
	double det_Y = 0.0;
	double dqf_t = 0.0;
	double dqh_t = 0.0;
	double Ym = m_st_target.height;

	det_Y = m_hz - Ym;
	//det_Y = m_hgps - Ym;	//test

	m_Qf = atan(-det_Y / m_distance_target) * RTOA;
	m_Qh = - m_alpha_target;

	double temp1 = sqrt(m_distance_target * m_distance_target + det_Y * det_Y);

	if(m_distance_target >= 500.0)
	{
		dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
			/CFlightGlobalFun::Nozero_FUN(temp1);

		dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
			/CFlightGlobalFun::Nozero_FUN(temp1 * temp1);	
	}
	else
	{
		if((m_hz >= (Ym + 25.0))
			&&(m_distance_target >= 25.0))
		{
			dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
				/CFlightGlobalFun::Nozero_FUN(temp1);

			dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
				/CFlightGlobalFun::Nozero_FUN(temp1 * temp1);
			valid_count = 0;
		}
		else
		{
			if(valid_count < 3)
			{
				dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
					/CFlightGlobalFun::Nozero_FUN(temp1);

				dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
					/CFlightGlobalFun::Nozero_FUN(temp1 * temp1);	
			}
			valid_count++;
		}		
	}

	//------------------------------test------------------------------------------//
// 	dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
// 		/CFlightGlobalFun::Nozero_FUN(temp1);
// 
// 	dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
// 		/CFlightGlobalFun::Nozero_FUN(temp1 * temp1);	
	//----------------------------------------------------------------------------//

	m_dqf = dqf_t*cos(m_gama/RTOA) - dqh_t*sin(m_gama/RTOA);
	m_dqh = dqf_t*sin(m_gama/RTOA) + dqh_t*cos(m_gama/RTOA);

	double Qh_n = m_Qh - m_A;
	Qh_n = CFlightGlobalFun::Adjust(Qh_n, 180.0);

	double SinPhih = -(cos(Qh_n/RTOA) * cos(m_Qf/RTOA) * (sin(m_gama/RTOA) * sin(m_zeta/RTOA) * cos(m_psin/RTOA) + cos(m_gama/RTOA) * sin(m_psin/RTOA))
		- sin(m_Qf/RTOA) * sin(m_gama/RTOA) * cos(m_zeta/RTOA)
		- sin(Qh_n/RTOA) * cos(m_Qf/RTOA) * (-sin(m_gama/RTOA) * sin(m_zeta/RTOA) * sin(m_psin/RTOA) + cos(m_gama/RTOA) * cos(m_psin/RTOA)));

	if (fabs(SinPhih) < 1.0)
	{
		m_phih = RTOA * asin(SinPhih);
	} 
	else
	{
		m_phih = RTOA * asin(CFlightGlobalFun::FSign(SinPhih));
	}

	double SinPhif = (cos(Qh_n/RTOA) * cos(m_Qf/RTOA) * (-sin(m_zeta/RTOA) * cos(m_psin/RTOA) * cos(m_gama/RTOA) + sin(m_gama/RTOA) * sin(m_psin/RTOA))
		+ sin(m_Qf/RTOA) * cos(m_zeta/RTOA) * cos(m_gama/RTOA)
		- cos(m_Qf/RTOA) * sin(Qh_n/RTOA) * (sin(m_zeta/RTOA) * cos(m_gama/RTOA) * sin(m_psin/RTOA) + sin(m_gama/RTOA) * cos(m_psin/RTOA))) 
		/ CFlightGlobalFun::Nozero_FUN(cos(m_phih/RTOA));

	if (fabs(SinPhif) < 1.0)
	{
		m_phif = RTOA * asin(SinPhif);
	} 
	else
	{
		m_phif = RTOA * asin(CFlightGlobalFun::FSign(SinPhif));
	}

	m_phif = CFlightGlobalFun::Range(m_phif, 60.0);
	m_phih = CFlightGlobalFun::Range(m_phih, 60.0);
	m_dqf = CFlightGlobalFun::Range(m_dqf, 20);
	m_dqh = CFlightGlobalFun::Range(m_dqh, 20);
}

//根据静压，计算气压高度
void CMathControlFlightBasic::Calc_BaroHigh()
{
    double exp = (R * L) / (G * M);
    double ratio = m_static_pressure/ P0;
    m_hbaro = (T0 / L) * (1.0f - pow(ratio, exp));
}
//根据静压、总压计空速，输出速度有效性
void CMathControlFlightBasic::Calc_BaroSpd()
{
	//海拔高度、空气密度比
	double temp_altitude_array[11] = {0,500,1000,1500,2000,2500,3000,3500,4000,4500,5000};
	double temp_rho_ratio_array[11] = {1.0000, 0.9421, 0.8870, 0.8345, 0.7846, 0.7371, 0.6920, 0.6491, 0.6084, 0.5698, 0.5332};
	double temp_rho_ratio = 0.0;
	double temp_dynamic_pressure = m_total_pressure - m_static_pressure;
	//根据海拔高度获得空气密度比
	temp_rho_ratio = CFlightGlobalFun::LAQL1(11, temp_altitude_array, temp_rho_ratio_array, m_hz);
	m_Vbaro = sqrt(2.0f * temp_dynamic_pressure / (RHO0*temp_rho_ratio*ARSPD_RATIO));

	//启控后，判断空速管数据有效性
	if(m_st_control_flag.flag_control_set == 1)
	{
		//出筒后速度，一般大于20m/s，考虑最大顺风10m/s，因此，真实空速一般大于10m/s，否则无效
		if(m_Vbaro < 10.0)
		{
			m_v_air = m_v;//空速管无效
		}
		//空速有效
		else
		{
			//10s内，卫星动态捕获过程中地速无效，信任空速
			if(flight_time < 10.0)
			{
				m_v_air = m_Vbaro;	
			}
			//之后，地速有效
			else
			{
				//空速和地速差大于10时，认为空速管性能变差，修正地速作为空速，限制在地速+10或地速-10
				if(fabs(m_Vbaro - m_v) > 10)
				{
					m_v_air = m_v + 10 * ((m_Vbaro - m_v) > 0?1:-1);
				}
				else
				{
					m_v_air = m_Vbaro;
				}
			}
		}
	}
	else
	{
		//起飞前，使用实际测量值
		m_v_air = m_Vbaro;
	}

	m_dynamic_pressure = 0.5 * temp_rho_ratio * 1.225 * m_v_air * m_v_air;
}


//惯性、卫星组合高度、垂速算法
void CMathControlFlightBasic::Calc_Hz()
{
	//暂未加入卫星、高度表切换策略

	//待补充 气压与惯性组合高度
	//m_baroalt_status
	//m_hbaro
	//待补充  无线电与惯性组合高度

	//惯卫星组合高度，待完善???...
	//发射初段认为卫星未定位，使用惯性计算高度
	if (flight_time <= 10.0)
	{
		m_vs += m_au * STEP_5ms;
		m_hz += m_vs * STEP_5ms;
		m_st_control_flag.flag_initial_hz = false;
	}
	//认为卫星已定位，且组合导航输出高度正常
	else
	{
		//组合高度初始化，之后使用卫星高度，计算组合高度、垂速
		if(!m_st_control_flag.flag_initial_hz)
		{
			m_st_control_flag.flag_initial_hz = true;
			m_hz = m_hgps;
		}
		else
		{
			m_vs += (m_au + 0.1 * (m_hgps - m_hz))*STEP_5ms;
			m_hz += (m_vs + 0.4 * (m_hgps - m_hz))*STEP_5ms;			
		}
	}
}


void CMathControlFlightBasic::Coord_Rebuild()
{
	//扇面转弯重建: 当前点到目标点
	if (m_st_control_flag.flag_launch_turn)		
	{
		m_st_control_flag.flag_launch_turn = false;
		
		//以当前点重建坐标系
		m_longitude_A = m_longitude;
		m_latitude_A = m_latitude;
		m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;	//转弯角度
		m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;	//转弯半径
		m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;//目标速度
		
		//重建游移方位角
		double distance_AB = 0.0;
		CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
			m_longitude_B, m_latitude_B, 
			&distance_AB, &m_A);
		m_A = - m_A;
		m_A = CFlightGlobalFun::Adjust(m_A, 180.0);
	}
	//航路转弯重建: 前一航点到当前航点建立航线
	else	
	{
		//目标航路号切换
		m_num_way_point_target++;
		//导航航路点坐标更新
		m_longitude_A = m_st_way_point[m_num_way_point_target - 1].longitude;
		m_latitude_A = m_st_way_point[m_num_way_point_target - 1].latitude;
		m_longitude_B = m_st_way_point[m_num_way_point_target].longitude;
		m_latitude_B = m_st_way_point[m_num_way_point_target].latitude;
		m_longitude_C = m_st_way_point[m_num_way_point_target + 1].longitude;
		m_latitude_C = m_st_way_point[m_num_way_point_target + 1].latitude;
		//目标转弯信息更新
		m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;
		m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;
		m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;
		//重建游移方位角
		double distance_AB = 0.0;
		CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
			m_longitude_B, m_latitude_B, 
			&distance_AB, &m_A);
		m_A = - m_A;
		m_A = CFlightGlobalFun::Adjust(m_A, 180.0);
	}
	
	//到达目标点，最小可能到达时间、时刻
	double angle_PB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_longitude_B, m_latitude_B,
		&m_distance_BP, &angle_PB);
	double temp_v = m_v_average_10s + 20.0;
	if(temp_v >= 60.0)
	{
		temp_v = 60.0;
	}
	else if(temp_v <= 40.0)
	{
		temp_v = 40.0;
	}
	double distance_to_go = fabs(m_distance_BP - m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0));
	double min_time = distance_to_go / temp_v;
	m_st_control_time.time_arrive_minimum = flight_time + min_time;
	m_st_control_flag.flag_waypoint_turn = false;
}

void CMathControlFlightBasic::Control_Turn()
{
	//发射扇面转弯入弯判断，只进入1次：助推器分离后8s
	if(flight_time >= (m_st_control_time.time_separate_booster + 8.0) 
		&& (!m_st_control_flag.flag_launch_turn_set))
	{
		m_st_control_flag.flag_launch_turn_set = true;//进入扇面转弯，更新一次
		m_st_control_flag.flag_launch_turn = true;	//出弯时置false
		m_turn_angle = m_psicn - m_A;
		m_turn_angle = CFlightGlobalFun::Adjust(m_turn_angle, 180.0);//20260408--wym
		
		//转弯角度大于5deg，开始扇面转弯
		if(fabs(m_turn_angle) > 5.0)		//去掉发射方位与目标夹角大于30°的条件，冗余
		{
			m_st_control_time.time_turn_in_start = flight_time;
			m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start 
				+ m_gama_turn_nominal / ROLL_RATE_COMMAND;	
			m_st_control_time.time_turn_out_start = MAX_TIME;
			m_st_control_time.time_turn_out_end = MAX_TIME;
			m_st_control_flag.flag_turn_out_set = false;
			m_psit_t_turn_in = m_psit;
		}
		//不进行扇面转弯，建立向目标飞行航线
		else
		{
			Coord_Rebuild();//不再扇面转弯
		}
		m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
	}
	
	//航路转弯入弯判断，目标点不是最后一个航点，未进入末制导，未处于转弯过程中，助推器分离后8s以上
	if ((m_num_way_point_target < (m_num_way_point - 1)) 
		&& (!m_st_control_flag.flag_combat_status) 
		&& (flight_time > m_st_control_time.time_turn_in_minimum)
		&& (flight_time > (m_st_control_time.time_separate_booster + 8.0))
		&& (!m_st_control_flag.flag_waypoint_turn))
	{
		//500ms判断一次，连续三帧弹目距离增大
		if(!((time_tick - 1) % 500))
		{
			if(m_distance_BP_projection > m_distance_BP_500pre) 
			{
				count_away++;
			}
			else
			{
				count_away = 0;
			}
		}

		//最小到达时间开始判断
		if(flight_time < m_st_control_time.time_arrive_minimum)
		{
			count_away = 0;
		}

		//入转弯提前距离，角度过渡补偿
		m_distance_turn_in_compensate = 1.25 * fabs(m_gama_turn_nominal * m_v / ROLL_RATE_COMMAND);
		m_distance_turn_in_compensate = CFlightGlobalFun::Range(m_distance_turn_in_compensate, 450.0);
		//入转弯提前距离，几何补偿+  角度过渡补偿
		m_distance_turn_in =  m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0) 
			+ m_distance_turn_in_compensate;

		//20ms判断一次，连续三次小于转弯提前距离
		if(!((time_tick - 1) % 20))
		{
			if(m_distance_BP_projection <= m_distance_turn_in)
			{
				count_sd_in++;
			}
			else
			{
				count_sd_in = 0;
			}
		}

		//满足连续三次小于转弯提前距离，或过目标点
		if ((count_away>=3)	
			|| (count_sd_in>=3))			
		{
			count_away = 0;
			count_sd_in = 0;
			m_st_control_flag.flag_waypoint_turn = true;
			
			//转弯角度大于5deg，开始航迹转弯，不重新建立航线
			if (fabs(m_turn_angle) > 5.0)
			{
				m_st_control_time.time_turn_in_start = flight_time;
				m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start 
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
				m_st_control_time.time_turn_out_start = MAX_TIME;
				m_st_control_time.time_turn_out_end = MAX_TIME;
				m_st_control_flag.flag_turn_out_set = false;

				//参考点坐标
				m_x_coordinate_turn = - m_turn_radius * sin (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle) 
					- m_distance_turn_in_compensate * cos(m_psicn / RTOA);
				m_z_coordinate_turn = - m_turn_radius * cos (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle)
					+ m_distance_turn_in_compensate * sin(m_psicn / RTOA);

				m_psit_t_turn_in = m_psit;
			}
			//不进行航迹转弯，切换下一目标点，重新向目标飞行航线
			else
			{
				Coord_Rebuild(); 	//不再航迹转弯过程中	
			}
			m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
		}
	}

	//转弯出弯判断: 扇面转弯或航迹转弯，转弯过渡结束
	if ((m_st_control_flag.flag_waypoint_turn || m_st_control_flag.flag_launch_turn) 
		&& (flight_time > m_st_control_time.time_turn_in_end))
	{
		double delta_psi = m_g * log(fabs(cos(m_gama_turn_nominal / RTOA))) / (m_v * ROLL_RATE_COMMAND) 
			* RTOA * CFlightGlobalFun::FSign(m_turn_angle);

		//当前点到下一目标点方位角
		double alpha_PC = 0.0;
		double distance_PC = 0.0;
		CFlightGlobalFun::Tomas(m_longitude, m_latitude,
			m_longitude_C, m_latitude_C,
			&distance_PC, &alpha_PC);
		alpha_PC = alpha_PC / RTOA;
		alpha_PC = CFlightGlobalFun::Adjust(alpha_PC, PI);
		
		//弹道偏角(北偏西为正) + 滚转过程航迹变化量 + 下一目标点航线(北偏东为正)
		double psic2 = m_psicn / RTOA + 2.0 * delta_psi + alpha_PC; 
		psic2 = CFlightGlobalFun::Adjust(psic2, PI);
		double psid1 = - psic2 * (m_turn_angle / RTOA);

		//航迹角过航线判断
		if((psid1 >= 0)
			&&(fabs(psic2) <= (PI / 2.0)))
		{
			count_turn_out++;
		}
		else
		{
			count_turn_out = 0;
		}

		//不是转弯过程中
		if(((count_turn_out >= 3)||(Judge_Turn_Error()))
			&& (!m_st_control_flag.flag_turn_out_set))
		{
			count_turn_out = 0;

			//扇面转弯，出转弯开始、结束
			if (m_st_control_flag.flag_launch_turn)
			{
				m_st_control_time.time_turn_out_start = flight_time;
				m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
				m_st_control_flag.flag_turn_out_set = true;
			}
			//航迹转弯，目标点不是最后一个航点
			else if(m_num_way_point_target <= (m_num_way_point - 2))
			{
				//到下一个航迹点距离，可以开展一次航迹转弯，开始出转弯过程
				if (distance_PC > (m_st_way_point[m_num_way_point_target + 1].turn_radius 
					* tan(fabs(m_st_way_point[m_num_way_point_target + 1].turn_angle / RTOA) / 2.0) 
					+ 500.0)) 
				{				
					double Omega = fabs(m_g * tan(m_gama / RTOA) / m_v);
					double time_turn_out_delay = fabs(acos(1 - fabs((m_sz - m_turn_radius) * Omega / m_v)))/CFlightGlobalFun::Nozero_FUN(Omega) - 3.5;
					if((m_sz - m_turn_radius) <= 0.0)
					{
						time_turn_out_delay = 0.0;
					}
					else
					{
						if(time_turn_out_delay < 0.0)
							time_turn_out_delay = 0.0;
						if(time_turn_out_delay > 4.0)
							time_turn_out_delay = 4.0;
					}
					m_st_control_time.time_turn_out_start = flight_time + time_turn_out_delay;//航迹转弯，出转弯开始
					m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
						+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
					m_st_control_flag.flag_turn_out_set = true;
				}
				//不足以开展一次航迹转弯，直接切换下一个航点为目标点，重新航迹转弯
				else
				{
					Coord_Rebuild();
					m_st_control_flag.flag_waypoint_turn = true;
					//转弯角度与下一航点，转弯方向相反
					if((m_turn_angle * m_st_way_point[m_num_way_point_target - 1].turn_angle) < 0.0)
					{
						m_st_control_time.time_turn_in_start = flight_time;
						m_gama_turn_nominal = atan(m_v * m_v / m_turn_radius / m_g) * RTOA;
						m_gama_turn_nominal = CFlightGlobalFun::Range(m_gama_turn_nominal, 60.0);
						double delta_t = (fabs(m_gama) + fabs(m_gama_turn_nominal))/ROLL_RATE_COMMAND;
						m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start + delta_t;
						m_st_control_time.time_turn_out_start = MAX_TIME;
						m_st_control_time.time_turn_out_end = MAX_TIME;
						m_st_control_flag.flag_turn_out_set = false;

						m_x_coordinate_turn = - m_turn_radius * sin (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle); 
						m_z_coordinate_turn = - m_turn_radius * cos (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle);
					}
			
					m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
					m_psit_t_turn_in = m_psit;
				}
			}
			//航迹转弯，目标点为最后一个航点
			else
			{
				m_st_control_time.time_turn_out_start = flight_time;
				m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
				m_st_control_flag.flag_turn_out_set = true;
			}
		}
		
		//出弯完成，切换下一航点
		if (flight_time >= m_st_control_time.time_turn_out_end
			&& m_st_control_flag.flag_turn_out_set)
		{
			Coord_Rebuild();
		}
	}
}

bool CMathControlFlightBasic::Judge_Turn_Error()
{
	//转弯过程中，转弯角度过大，即大于 “期望转弯角度叠加15deg余量”
	if((flight_time < m_st_control_time.time_turn_out_start) 
		&& (flight_time > m_st_control_time.time_turn_in_end))
	{
		double delta_psit = m_psit_t_turn_in - m_psit;//方位角偏差
		delta_psit = CFlightGlobalFun::Adjust(delta_psit, 180.0);
		if(fabs(delta_psit) > (fabs(m_turn_angle) + 15.0))
		{
			count_turn_error++;
		}
		else
		{
			count_turn_error = 0;
		}

		if(count_turn_error > 20)
		{
			count_turn_error = 0;
			return true;
		}
	}
	return false;
}

void CMathControlFlightBasic::Control_Altitude_Change()
{
	//目标高度变化判断
	double target_height_update = m_st_way_point[m_num_way_point_target].height;	//若添加数据链调高指令在此增加高差
	double delta_h_target = m_target_height - target_height_update;

	if (fabs(delta_h_target) > 1.0)
	{
		count_altitude_change_enable++;
	}
	else
	{
		count_altitude_change_enable = 0;
	}

	//高度机动开始判断
	////////高度机动开始时刻计算，允许高度机动条件
	if ((m_num_way_point_target <= (m_num_way_point - 2))//目标点不是最后两个航迹点
		&& (!m_st_control_flag.flag_waypoint_turn)	//航迹转弯过程，航迹转弯过程不进行高度机动
		&& (!m_st_control_flag.flag_launch_turn)	//扇面转弯过程，转弯过程不进行高度机动
		&& (count_altitude_change_enable >= 3))	//连续三次判断，航迹高度更新
	{
		count_altitude_change_enable = 0;
		count_altitude_change++;

		//高度机动延迟时间计算
		double altitude_change_delay = 0.0;
		//高度下滑
		if(delta_h_target >= 0.0)
		{
			altitude_change_delay = 1.0;
			m_st_control_flag.flag_alltitude_climb = false;
			m_st_control_flag.flag_alltitude_decline = true;
		}
		//高度爬升
		else
		{
			//爬升高度小于10m
			if(fabs(delta_h_target) < 10.0)
			{
				altitude_change_delay = 1.0;
			}
			//爬升高度大于等于10m，小于250m
			else if(fabs(delta_h_target) < 250.0)
			{
				altitude_change_delay = 0.1 * fabs(delta_h_target);
			}
			//爬升高度大于等于250m
			else
			{
				altitude_change_delay = 25.0;
			}	
			m_st_control_flag.flag_alltitude_climb = true;
			m_st_control_flag.flag_alltitude_decline = false;
		}
		m_target_height = target_height_update;
		m_st_control_time.time_altitude_change_start = flight_time + altitude_change_delay;
		m_st_control_time.time_altitude_change_end = MAX_TIME;
		m_st_control_flag.flag_alltitude_change = true;
	}

	//高度机动结束时刻判断
	if (m_st_control_flag.flag_alltitude_change)
	{
		if ((fabs(m_hz - m_target_height) <= 20.0))
		{
			count_altitude_change_end++;
		}
		else
		{
			count_altitude_change_end = 0;
		}

		if(count_altitude_change_end >= 3)
		{
			count_altitude_change_end = 0;
			m_st_control_time.time_altitude_change_end = flight_time;
			m_st_control_flag.flag_alltitude_change = false;
		}
	}
}

// void CMathControlFlightBasic::Monitor_Data()
// {
// 	extern CSimMonitor sim_monitor;
// 	if (sim_monitor.flag_monitor2_valid)
// 	{
// 		sim_monitor.Mark_Data_In_Flag(ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(flight_time,"time",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_A,"A",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_hz,"hz",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_vs,"vs",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_ny,"ny",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_nz,"nz",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_sz,"sz",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_dqf,"dqf",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_dqh,"dqh",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_longitude_B,"LongT",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_latitude_B,"LatT",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_distance_target,"DistanceTarget",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_control,"Tqk",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_separate_booster,"Tfl",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_launch_missile_wing,"Tdyzk",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_engine_start,"Tfdh",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_combat_status,"Tzd",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_turn_in_start,"Ts0",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_turn_in_end,"Ts1",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_turn_out_start,"Ts2",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_turn_out_end,"Ts3",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_altitude_change_start,"Th0",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_time.time_altitude_change_end,"Th1",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_flag.flag_launch_turn,"FlagLaunchTurn",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_flag.flag_waypoint_turn,"FlagWaypointTurn",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_st_control_flag.flag_alltitude_change,"FlagAlltitudeChange",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(count_altitude_change,"countAltitudeChange",ENUM_FILE_CONTROL1);
// 	}
// }