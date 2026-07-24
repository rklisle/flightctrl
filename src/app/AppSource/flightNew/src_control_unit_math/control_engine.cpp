#include <cstring>
#include "control_engine.h"
#include "../data_protocol.h"
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
	m_dltVg_time = 0.0;
#ifdef __VEL__CONTROL__MODE__KC__
	m_Kc0 = 42.0;//满载165kg，起飞状态；
	m_Kc1 = 0;
	m_Kc1_record = 0.0;
	m_Kc2 = 0;
	m_Kc2_record = 0.0;
	m_Kc3 = 0;
	m_Kc4 = 0;
	m_control_Kc = 70.0;
	m_count_vel_pidctrl = 0;
#else
	m_n0 = 5500;//满载165kg，起飞状态；
	m_n1 = 0;
	m_n2 = 0;
	m_n3 = 0;
	m_control_rpm = 5500;
#endif
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

	m_flag_engine_start = false;//助推器分离后，油门由怠速加速到保护油门
	m_flag_missile_takeoff = false;//起飞完成，开始速度控制
	m_flag_engine_stop = false;//发动机关机，指令油门为零
}
void CMathControlEngine::Initial()
{
	
}

void CMathControlEngine::Get_Data()
{
	m_mass_calc = p_st_engine_control_input->mass;
	
	m_missile_height = p_st_engine_control_input->h;	//飞行高度
	m_missile_height_initial = p_st_engine_control_input->h_ini;
	//m_temperature_ground = p_st_engine_control_input->temperature_ground;
	
	m_target_velocity = p_st_engine_control_input->target_velocity;//目标速度
	m_target_time = p_st_engine_control_input->target_time;	//到达时间
	
	//方法一：根据高度计算空气密度比，一维插值
	//double hight_array[7] = {0, 1000, 2000, 3000, 4000, 4500,5000};
	//double Vsonic_array[7] = {340.3, 336.4, 332.5, 328.6, 324.6, 322.6 ,320.5};//高度，空气密度比
	//m_Vsonic = CFlightGlobalFun::LAQL1(7,  hight_array,  Vsonic_array, m_missile_height);//飞行速度
	//方法二：载入声速，避免存在偏差
	m_Vsonic = p_st_engine_control_input->sonic_speed;//声速
	m_mach = p_st_engine_control_input->mach;//马赫数
	m_air_velocity = m_mach*m_Vsonic;//空速

	m_turn_radius = p_st_engine_control_input->radius_zw;	
	m_flag_launch_turn = p_st_engine_control_input->flag_launch_turn;		//扇面转弯
	m_flag_waypoint_turn = p_st_engine_control_input->flag_waypoint_turn;	//航迹转弯
	m_flag_alltitude_change = p_st_engine_control_input->flag_alltitude_change;//高度 机动
	m_flag_alltitude_climb = p_st_engine_control_input->flag_alltitude_climb;	//爬升
	m_flag_alltitude_decline = p_st_engine_control_input->flag_alltitude_decline;//下降
	
	m_flag_velocity_control = p_st_engine_control_input->flag_velocity_control;//速度控制标识，1引入地速控制，未用到
	m_missile_velocity = p_st_engine_control_input->missile_average_velocity;//地速
	m_flag_flightime_ctrl = p_st_engine_control_input->flag_flightime_ctrl;	//到达时间标识
	
	m_flag_engine_start = p_st_engine_control_input->flag_engine_start;
	m_flag_engine_stop = p_st_engine_control_input->flag_engine_stop;
	m_flag_missile_takeoff = p_st_engine_control_input->flag_missile_takeoff;
	m_flag_altitude_control = p_st_engine_control_input->flag_altitude_control;
	m_flag_combat_status = p_st_engine_control_input->flag_combat_status;
	m_flag_combat_dive_pullup = p_st_engine_control_input->flag_combat_dive_pullup;

	/*为了调试???...
	m_mass_calc = 150.0;
	m_mach = 0.15;
	m_missile_height = 120.0;
	m_missile_height_initial = 100.0;
	m_target_velocity = 50.0;
	m_missile_velocity = 50.0;
	m_target_time = 10.0;
	m_temperature_ground = 15.0;
	m_turn_radius = 500.0;
	
	m_flag_launch_turn = 0;		//扇面转弯
	m_flag_alltitude_change = 1;//高度 机动
	m_flag_waypoint_turn = 0;	//航迹转弯
	m_flag_alltitude_climb = 0;	//爬升，无效
	m_flag_alltitude_decline = 0;//下降，无效

	m_flag_engine_start = 1;
	m_flag_engine_stop = 0;
	m_flag_missile_takeoff = 1;*/
}

void CMathControlEngine::Run()
{
	Get_Data();
	
#ifdef __VEL__CONTROL__MODE__KC__
	//起飞至助推器分离后1s（约3.0s），怠速油门
	if(!m_flag_engine_start)
	{
		m_control_Kc = KC_COMMAND_MIN_LIMIT;
	}
	//助推器分离后1.0s至起飞完成(约10.0s)，怠速油门->保护油门70%
	//说明：按照10deg弹道倾角，50m/s速度，预估阻力功率，估算轴功率约27kW；70%油门仍处于减速状态；
	else if(!m_flag_missile_takeoff)
	{
		m_control_Kc = KC_COMMAND_CRUISE;
	}
	//起飞稳定后至末制导发动机关机，转速控制，计算油门且放开保护油门
	else if(!m_flag_engine_stop)
	{
		//进入虚拟打击过程，发送机控制转怠速
		if(m_flag_combat_status == true)
		//if(m_missile_velocity > 60.0)
		{
			m_control_Kc = KC_COMMAND_MIN_LIMIT;
		}
		//退出虚拟打击流程，发动机怠速转控制
		else
		{
			Calc_Data_Kc();
		}		
	}
	//关机后，指令油门为零
	else
	{
		m_control_Kc = 0.0;
	}
#else
	Calc_Data_Rpm();
#endif
	//Calc_Data();
	//Calc_Data_Kc();
	
	Send_Data();

	//Monitor_Data();
}



#ifdef __VEL__CONTROL__MODE__KC__
//速度控制，输出油门
void CMathControlEngine::Calc_Data_Kc()
{
	double target_velocity = VEL_COMMAND_CRUISE;//初值
	double dlt_air_velocity_comp = 0.0;
	double dlt_velocity = 0.0;
	double dlt_velocity_cmd = 0.0;
	double target_velocity_cmd = VEL_COMMAND_CRUISE;
	
	//转速限制及推力当量
	double kp = 8.0;//10.0;///2.0;
	double ki = 0.8;///1.0;///0.2;
	double m_array[3] = {100.0, 133.0, 165.0};//{100.0, 125.0, 150.0}
	double V_array[4] = {40.0, 50.0, 60.0, 70.0};
	double h_array[6] = {0, 1000, 2000, 3000, 4000, 5000};
	double Kc0_matrix[3][6] = {{71.6599, 72.3680, 73.0528, 74.0607, 75.2552, 77.1943},
						{73.3871, 74.2679, 75.5284, 77.4114, 79.2842, 82.0228},
						{75.4301, 77.0857, 78.8006, 81.2730, 86.1428, 93.9886}};
	//double kp_matrix[3][6] = {{2.4034, 3.0302, 3.7191, 4.4933, 5.2596, 6.0561},
	//						{3.9243, 4.6577, 5.3846, 6.1504, 6.9307, 7.7314},
	//						{5.2932, 5.9977, 6.7500, 7.5242, 8.5148, 9.7978}};
	//double ki_matrix[3][6] = {{0.5465, 0.5939, 0.6455, 0.7047, 0.7624, 0.8237},
	//					{0.6616, 0.7171, 0.7719, 0.8309, 0.8899, 0.9518},
	//					{0.7655, 0.8192, 0.8761, 0.9359, 1.0223, 1.1421}};

	//根据当前质量，高度，插值计算基准油门
	m_Kc0 = CFlightGlobalFun::LAQL2(3,  6,  m_array,  h_array, &Kc0_matrix[0][0], m_mass_calc, m_missile_height);
	m_Kc0 = m_Kc0 - 25.0;
	
	//转弯补偿：
	double gama_array[5] = {10, 20, 30, 45, 60};
	double Kc1_matrix[4][5] = {{0.374960938, 1.597629353, 4.019899216, 12.05905532, 36.17264967},    
								{0.239975,     1.022482786, 2.572735498, 7.717795407, 23.15049579},    
								{0.166649306,  0.71005749,  1.786621874, 5.359580144, 16.07673319},    
								{0.122436225,  0.521674891, 1.312620152, 3.937650718, 11.81147744}};
	//根据转弯半径和空速，估计标称滚转角
	double temp_gama = 20.0;
	double temmp_az = m_air_velocity*m_air_velocity/CFlightGlobalFun::Nozero_FUN(m_turn_radius);
	temp_gama = RTOA*atan(temmp_az/9.8);//取值范围-90~90deg
	temp_gama = fabs(temp_gama);
	//滚转角列、速度行插值，获得初步油门补偿量
	double temp_Kc1 = 10.0;
	temp_Kc1 = CFlightGlobalFun::LAQL2(3,  6,  V_array,  gama_array, &Kc1_matrix[0][0], m_air_velocity, temp_gama);
	
	//根据高度获取空气密度，进一步计算补偿系数
	double rho_array[6] = {1.2252, 1.1119, 1.0067, 0.9095, 0.8195, 0.7366};
	double temp_Kc_rho_factor = 1.0;
	double temp_rho = 1.0;
	temp_rho = CFlightGlobalFun::LAQL1(6,  h_array,  rho_array, m_missile_height);
	temp_Kc_rho_factor = 1/temp_rho;
	temp_Kc_rho_factor = CFlightGlobalFun::Range2(temp_Kc_rho_factor, 0.8, 1.36);

	//根据估计质量计算，计算补偿系数
	double temp_Kc_mass_factor = 1.0;
	temp_Kc_mass_factor = (m_mass_calc/133.0)*(m_mass_calc/133.0);
	temp_Kc_mass_factor = CFlightGlobalFun::Range2(temp_Kc_mass_factor, 0.59, 1.54);

	//经过空气密度系数（高度对应）和质量系数补偿，获得油门补偿量
	temp_Kc1 = temp_Kc1*temp_Kc_rho_factor*temp_Kc_mass_factor;
	
	//爬升补偿：5.7deg爬升角，油门补偿量近似等于30%附近，不做处理
	double temp_Kc2 = 30.0;

	//为了测试???...
	if(flight_time > 1175.0)
	{
		double temp_a = 1.0;
	}
	
	//当前只有空速控制
	//如果当前指令空速大于最大速度，等于最大速度
	//如果当前指令空速小于最小速度，等于最小速度
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

	//转弯过程补偿转速
	if (m_flag_waypoint_turn || m_flag_launch_turn)
	{
		m_Kc1 = temp_Kc1;//20.0;
	} 
	else
	{
		m_Kc1 = 0.0;
	}

	//求高度机动补偿转速
	if(!m_flag_altitude_control)
	{
		//进入高度控制前，处于初始爬升过程	
		m_Kc2 = temp_Kc2;
	}
	else if (m_flag_alltitude_change)
	{
		//爬升
		if (m_flag_alltitude_climb)
		{
			m_Kc2 = temp_Kc2;//30;
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
	if(m_flag_flightime_ctrl == 1)
	{
		//方法一：周期10s判断一次，补偿空速
		//说明：计算地速与目标目标速度差，补偿空速，使得地速逐渐趋近于目标速度
		if(flight_time > m_dltVg_time + 10.0)
		{
			m_dltVg_time = flight_time;

			//地速补偿空速，达到到达时间的目的
			dlt_air_velocity_comp = target_velocity - m_missile_velocity;
		}
		//到达时间控制，期望地速 = 待飞距离/待飞时间，其中，待飞时间 = (当到达时间 - 当前时间)；
		if(target_velocity + dlt_air_velocity_comp < 40.0)
		{
			dlt_velocity = 40.0 - m_air_velocity;//基于速计算偏差
		}
		else if(target_velocity + dlt_air_velocity_comp > 60.0)
		{
			dlt_velocity = 60.0- m_air_velocity;//基于速计算偏差
		}
		else
		{
			dlt_velocity = target_velocity - m_air_velocity + dlt_air_velocity_comp;//基于速计算偏差
		}
		
		//方法二：地速控制
		//dlt_velocity = target_velocity - m_missile_velocity;
		//待补偿空速限幅
	}
	else
	{
		dlt_velocity = target_velocity - m_air_velocity;
	}

	//说明1：情况一、二为大范围控制；情况三为中范围控制；情况四为小范围控制；
	//说明2：情况一、二大范围控制量稳态值，由情况三中范围控制继承，速度下降很大，不可忽略；
	//说明3：情况三中范围控制量稳态值，由情况四小范围控制继承，速度下降很小，忽略；
	//说明4：（平飞或爬升）阵风能量下降时，认为出现异常，具体处理过程如下：????...
	// 1)先开环补偿爬升油门；
	// 2)2.5s周期判断能量增加情况；
	// 3）退出异常处理，即阵风结束
	// 3.1)若能量不增加（视情，改为高度不增加），半开环增加油门；
	// 3.2)若能量增加（视情，改为高度增加），不增加半开环油门，退出异常处理；
	
	//当异常处理过程 或 大范围速度控制中，不进行半开环补偿；
	//否则，进行半开环速度补偿；

	//当半开环速度控制过程中，不进行PI速度控制
	//否则，进行PI速度控制；
	
	//情况一：高度机动结束，开环m_Kc2和m_Kc3综合后，幅值给m_Kc3，避免掉速
	//if(m_Kc1_record < m_Kc1 + 10.0)
	//{
	//	m_Kc3 = m_Kc3 + m_Kc1;
	//}
	//情况二：侧向机动结束，开环m_Kc1和m_Kc3综合后，幅值给m_Kc3，避免掉速
	//if(m_Kc2_record < m_Kc2 + 10.0)
	//{
	//	m_Kc3 = m_Kc3 + m_Kc2;
	//}
	//情况一和情况二，延迟5s再进行，情况三控制；不进行情况四控制
	if(fabs(m_Kc1 - m_Kc1_record) > 10.0 || fabs(m_Kc2 - m_Kc2_record) > 10.0)
	{
		m_Kc3 = 0.0;//大范围机动后，重新新估计油门量
		m_count_vel_pidctrl = 0;

		//等一段时间后，再进行“情况三，速度控制”
		m_dltKc_time = flight_time + 2.5;
	}
	//更新前一帧指令油门
	m_Kc1_record = m_Kc1;
	m_Kc2_record = m_Kc2;
	
	//情况三：速度中范围控制，即m_Kc1和m_Kc2保持不变，半开环控制计算m_Kc3
	//每2.5s，观察空速与指令速度之间关系
	//如果空速大于指令空速2.5m/s，减小5%油门；
	//如果空速小于指令空速2.5m/s，增大5%油门；
	//如果空速与指令空速小于2.5m/s，保持当前油门不变；
	if(flight_time > m_dltKc_time + 2.5)
	{
		m_dltKc_time = flight_time;

		if(dlt_velocity > 5.0)
		{
			m_Kc3 = m_Kc3 + 5.0;

			//不进行PI控制
			m_count_vel_pidctrl = 0;
		}
		else if(dlt_velocity < -5.0)
		{
			m_Kc3 = m_Kc3 - 5.0;

			//不进行PI控制
			m_count_vel_pidctrl = 0;
		}
		else
		{
			//首次满足，按照1%油门，速度增量1m/s，补偿剩余量，之后在进入PI控制
			//后续优化为，提升速度控制快速性
			if(m_count_vel_pidctrl == 0)
			{
				m_Kc3 += dlt_velocity*1.0;
			}
			//m_Kc3保持之前数值保持不变
			m_count_vel_pidctrl++;
		}
	}

		
	//情况四：速度细化控制，即PI控制
	//连续5s时间，速度偏差小于2.5m/s，开始进行PI速度控制；
	//当速度偏差大于2.5m/s时，结束PI速度控制，控制量、积分量清零；
	
	//说明：下滑过程是否可以不进行PI控制
	if(m_count_vel_pidctrl > 2)
	{
		//闭环PI控制
		m_velocity_integrator +=dlt_velocity * STEP_5ms;
		m_velocity_integrator = CFlightGlobalFun::Range(m_velocity_integrator, 10.0/ki);
		m_Kc4 = kp * dlt_velocity + ki * m_velocity_integrator;

		m_Kc4 = CFlightGlobalFun::Range(m_Kc4, 20.0);
	}
	else
	{
		//PI控制为零，积分为零
		m_velocity_integrator = 0.0;
		m_Kc4 = 0.0;
	}	
	
	//合转速
	m_control_Kc = m_Kc0 + m_Kc1 + m_Kc2 + m_Kc3 + m_Kc4;

	//转速控制
	if(m_control_Kc < KC_COMMAND_MIN_LIMIT)
		m_control_Kc = KC_COMMAND_MIN_LIMIT; 
	if(m_control_Kc > KC_COMMAND_MAX_LIMIT)
		m_control_Kc = KC_COMMAND_MAX_LIMIT; 
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
	if (m_flag_waypoint_turn || m_flag_launch_turn)
	{
		m_n1 = 1500;
	} 
	else
	{
		m_n1 = 0.0;
	}

	//求高度机动补偿转速
	if (m_flag_alltitude_change)
	{
		m_n2 = 0.0;
		//爬升
		if (m_flag_alltitude_climb)
		{
			m_n2 = 1500.0;
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
	if (m_flag_flightime_ctrl)
	{
		dlt_velocity = target_velocity - m_missile_velocity;
		m_velocity_integrator +=dlt_velocity * STEP_5ms;
		m_velocity_integrator = CFlightGlobalFun::Range(m_velocity_integrator, (-100.0/(k0*ki)));
		m_n3 = k0*(kp*dlt_velocity + ki * m_velocity_integrator);
		
		m_n3 = CFlightGlobalFun::Range(m_n3, 1500.0);
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

/*监控
void CMathControlEngine::Monitor_Data()
{
	extern CSimMonitor sim_monitor;
	
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_target_velocity,"Vcx",ENUM_FILE_CONTROL1);	//速度指令
		sim_monitor.Get_Variable(m_air_velocity,"Vr_air",ENUM_FILE_CONTROL1);	//空速
		sim_monitor.Get_Variable(m_missile_velocity,"dhcx",ENUM_FILE_CONTROL1);//地速
		sim_monitor.Get_Variable(m_control_Kc,"Kc",ENUM_FILE_CONTROL1);		//油门
	}
}*/
