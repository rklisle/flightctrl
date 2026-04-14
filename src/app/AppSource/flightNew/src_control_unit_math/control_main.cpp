#include <cstring>
#include "control_main.h"

CMathControlMain::CMathControlMain()
{
	missile_ID = 0;
	m_target_attack_ID = 0;
	p_st_data_seeker_to_controller = NULL;
	p_st_data_ins_to_controller = NULL;
	//p_st_data_datalink_to_controller = NULL;
	p_st_data_engine_to_controller = NULL;
	p_st_data_baro_to_controller = NULL;
	p_st_data_radioalt_to_controller = NULL;
	p_st_data_controller_to_seeker = NULL;
	//p_st_data_controller_to_datalink = NULL;
	p_st_data_controller_to_engine = NULL;
	p_st_data_controller_to_actuator = NULL;
	p_st_data_controller_to_switch_output = NULL;
	p_st_route_data_preflight = NULL;
	p_st_initial_data = NULL;
	// p_st_debug_monitor = NULL;


	//初始化数据缓存
	memset(&m_st_flight_basic_input  , 0, sizeof(Stru_Flight_Basic_Input));
	memset(&m_st_flight_basic_output , 0, sizeof(Stru_Flight_Basic_Output));
	memset(&m_st_roll_control_input  , 0, sizeof(Stru_Roll_Control_Input));			
	memset(&m_st_roll_control_output , 0, sizeof(Stru_Roll_Control_Output));
	memset(&m_st_yaw_control_input   , 0, sizeof(Stru_Yaw_Control_Input));				
	memset(&m_st_yaw_control_output  , 0, sizeof(Stru_Yaw_Control_Output));
	memset(&m_st_pitch_control_input , 0, sizeof(Stru_Pitch_Control_Input));
	memset(&m_st_pitch_control_output, 0, sizeof(Stru_Pitch_Control_Output));
	memset(&m_st_control_out_input   , 0, sizeof(Stru_Control_Out_Input));
	memset(&m_st_control_out_output  , 0, sizeof(Stru_Control_Out_Output));
	memset(&m_st_engine_control_input	 , 0, sizeof(Stru_Engine_Control_Input));
	memset(&m_st_engine_control_output	 , 0, sizeof(Stru_Engine_Control_Output));


	//绑定输入输出缓存
	m_math_control_flight_basic.p_st_flight_basic_input = &m_st_flight_basic_input;
	m_math_control_flight_basic.p_st_flight_basic_output = &m_st_flight_basic_output;
	m_math_control_roll.p_st_roll_control_input = &m_st_roll_control_input;
	m_math_control_roll.p_st_roll_control_output = &m_st_roll_control_output;
	m_math_control_yaw.p_st_yaw_control_input = &m_st_yaw_control_input;
	m_math_control_yaw.p_st_yaw_control_output = &m_st_yaw_control_output;
	m_math_control_pitch.p_st_pitch_control_input = &m_st_pitch_control_input;
	m_math_control_pitch.p_st_pitch_control_output = &m_st_pitch_control_output;
	m_math_control_out.p_st_control_out_input = &m_st_control_out_input;
	m_math_control_out.p_st_control_out_output = &m_st_control_out_output;
	m_math_control_engine.p_st_engine_control_input = &m_st_engine_control_input;
	m_math_control_engine.p_st_engine_control_output = &m_st_engine_control_output;
}

void CMathControlMain::Initial()
{
	// m_math_control_flight_basic.p_st_debug_monitor = p_st_debug_monitor;
	// m_math_control_roll.p_st_debug_monitor = p_st_debug_monitor;
	// m_math_control_yaw.p_st_debug_monitor = p_st_debug_monitor;
	// m_math_control_pitch.p_st_debug_monitor = p_st_debug_monitor;
	// m_math_control_engine.p_st_debug_monitor = p_st_debug_monitor;
	// m_math_control_out.p_st_debug_monitor = p_st_debug_monitor;

	m_math_control_flight_basic.p_st_initial_data = p_st_initial_data;
	m_math_control_flight_basic.p_st_route_data_preflight = p_st_route_data_preflight;
	m_math_control_flight_basic.Initial();
	
	m_math_control_roll.Initial();
	m_math_control_yaw.Initial();
	m_math_control_pitch.Initial();
	m_math_control_engine.Initial();

	m_math_control_out.Initial();
}

void CMathControlMain::Run()
{
	//航迹切换，制导回路
	Update_Input_Data(ENUM_FLIGHT_BASIC);
	m_math_control_flight_basic.Run();

	//三通道控制，俯仰、航向及滚转
	Update_Input_Data(ENUM_CONTROL_ROLL);	
	m_math_control_roll.Run();
	Update_Input_Data(ENUM_CONTROL_YAW);
	m_math_control_yaw.Run();
	Update_Input_Data(ENUM_CONTROL_PITCH);
	m_math_control_pitch.Run();
	
	//速度控制
	Update_Input_Data(ENUM_CONTROL_ENGINE);
	m_math_control_engine.Run();
	
	//舵控分配
	Update_Input_Data(ENUM_CONTROL_OUT);
	m_math_control_out.Run();
	Update_Output_Data();	
}

void CMathControlMain::Update_Input_Data(MODULE_TYPE MODULE_NAME)
{
	switch (MODULE_NAME)
	{
	case ENUM_FLIGHT_BASIC:			//更新基本信息计算模块输入
		m_st_flight_basic_input.missile_ID = missile_ID;
		//m_target_attack_ID = p_st_data_datalink_to_controller->st_mission_update_data[missile_ID].target_ID;
		m_math_control_flight_basic.flight_time = flight_time;
		m_math_control_flight_basic.time_tick = time_tick;
		
		m_st_flight_basic_input.engine_cmd_Kc = 
			m_st_engine_control_output.control_Kc;
		m_st_flight_basic_input.engine_cmd_rpm = 
			m_st_engine_control_output.control_rpm;

		memcpy(&m_st_flight_basic_input.st_baro_data,
			p_st_data_baro_to_controller,
			sizeof(Stru_Data_Baro_To_Controller));
		
		memcpy(&m_st_flight_basic_input.st_radioalt_data,
			p_st_data_radioalt_to_controller,
			sizeof(Stru_Data_RadioAlt_To_Controller));
		
		memcpy(&m_st_flight_basic_input.st_engine_data,
			p_st_data_engine_to_controller,
			sizeof(Stru_Data_Engine_To_Controller));
		
		memcpy(&m_st_flight_basic_input.st_ins_data,
			p_st_data_ins_to_controller,
			sizeof(Stru_Data_INS_To_Controller));

		memcpy(&m_st_flight_basic_input.st_seeker_data,
			&p_st_data_seeker_to_controller[m_target_attack_ID],
			sizeof(Stru_Data_Seeker_To_Controller));

		//memcpy(&m_st_flight_basic_input.st_datalink_data,
		//	p_st_data_datalink_to_controller,
		//	sizeof(Stru_Data_Datalink_To_Controller));
		break;
	case ENUM_CONTROL_ROLL:			//更新滚动控制模块输入
		m_math_control_roll.flight_time = flight_time;
		m_math_control_roll.time_tick = time_tick;
		m_st_roll_control_input.mass = m_st_flight_basic_output.mass_calc;
		m_st_roll_control_input.gama = m_st_flight_basic_output.gama;
		m_st_roll_control_input.wx	 = m_st_flight_basic_output.wx;
		m_st_roll_control_input.sz	 = m_st_flight_basic_output.sz;
		m_st_roll_control_input.v	 = m_st_flight_basic_output.v;
		m_st_roll_control_input.vnz	 = m_st_flight_basic_output.vnz;
		m_st_roll_control_input.g	 = m_st_flight_basic_output.g;
		if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
			/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		{
			m_st_roll_control_input.dqh	= p_st_data_seeker_to_controller[m_target_attack_ID].yaw_LOS_rate;
		} 
		else
		{
			m_st_roll_control_input.dqh	= m_st_flight_basic_output.dqh;
		}
		m_st_roll_control_input.angle_zw = m_st_flight_basic_output.angle_zw;
		m_st_roll_control_input.radius_zw = m_st_flight_basic_output.radius_zw;
		m_st_roll_control_input.gama_turn_nominal = m_st_flight_basic_output.gama_turn_nominal;
		m_st_roll_control_input.target_velocity = m_st_flight_basic_output.target_velocity;
		m_st_roll_control_input.flag_launch_turn =
			m_st_flight_basic_output.flag_launch_turn;
		m_st_roll_control_input.time_control =
			m_st_flight_basic_output.st_control_time.time_control;
		m_st_roll_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;
		m_st_roll_control_input.time_engine_start =
			m_st_flight_basic_output.st_control_time.time_engine_start_finish;
		m_st_roll_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;
		m_st_roll_control_input.time_turn_in_start =
			m_st_flight_basic_output.st_control_time.time_turn_in_start;
		m_st_roll_control_input.time_turn_in_end =
			m_st_flight_basic_output.st_control_time.time_turn_in_end;
		m_st_roll_control_input.time_turn_out_start =
			m_st_flight_basic_output.st_control_time.time_turn_out_start;
		m_st_roll_control_input.time_turn_out_end =
			m_st_flight_basic_output.st_control_time.time_turn_out_end;
		break;
	case ENUM_CONTROL_YAW:			//更新航向控制模块输入
		m_math_control_yaw.flight_time = flight_time;
		m_math_control_yaw.time_tick = time_tick;
		m_st_yaw_control_input.mass = m_st_flight_basic_output.mass_calc;
		m_st_yaw_control_input.gama = m_st_flight_basic_output.gama;
		m_st_yaw_control_input.wy	= m_st_flight_basic_output.wy;
		m_st_yaw_control_input.v	= m_st_flight_basic_output.v;
		m_st_yaw_control_input.g	= m_st_flight_basic_output.g;
		m_st_yaw_control_input.qh	= m_st_flight_basic_output.qh;
		m_st_yaw_control_input.tgo	= m_st_flight_basic_output.time_to_go;
		if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
			/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		{
			m_st_yaw_control_input.dqh	= p_st_data_seeker_to_controller[m_target_attack_ID].yaw_LOS_rate;
		} 
		else
		{
			m_st_yaw_control_input.dqh	= m_st_flight_basic_output.dqh;
		}
		m_st_yaw_control_input.nbz	= m_st_flight_basic_output.nbz;
		m_st_yaw_control_input.distance_target = m_st_flight_basic_output.distance_target;
		m_st_yaw_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;
		m_st_yaw_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;
		m_st_yaw_control_input.time_combat_delay = 
			m_st_roll_control_output.time_combat_delay;
		m_st_yaw_control_input.gama_command_compensate =
			m_st_roll_control_output.gama_command_compensate;
		break;
	case ENUM_CONTROL_PITCH:		//更新俯仰控制模块输入
		m_math_control_pitch.flight_time = flight_time;
		m_math_control_pitch.time_tick = time_tick;
		m_st_pitch_control_input.mass = m_st_flight_basic_output.mass_calc;
		m_st_pitch_control_input.gama = m_st_flight_basic_output.gama;
		m_st_pitch_control_input.zeta = m_st_flight_basic_output.zeta;
		m_st_pitch_control_input.wz   = m_st_flight_basic_output.wz;
		m_st_pitch_control_input.hz   = m_st_flight_basic_output.hz;
		m_st_pitch_control_input.v    = m_st_flight_basic_output.v;
		m_st_pitch_control_input.vs   = m_st_flight_basic_output.vs;
		m_st_pitch_control_input.g    = m_st_flight_basic_output.g;
		//m_st_pitch_control_input.mass = m_st_flight_basic_output.
		if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
			/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		{
			m_st_pitch_control_input.dqf = p_st_data_seeker_to_controller[m_target_attack_ID].pitch_LOS_rate;
		} 
		else
		{
			m_st_pitch_control_input.dqf = m_st_flight_basic_output.dqf;
		}
		m_st_pitch_control_input.nby  = m_st_flight_basic_output.nby;
		m_st_pitch_control_input.distance_target = m_st_flight_basic_output.distance_target;
		m_st_pitch_control_input.distance_target_t_combat = 
			m_st_flight_basic_output.distance_target_t_combat;
		m_st_pitch_control_input.target_height = 
			m_st_flight_basic_output.target_height;
		m_st_pitch_control_input.count_altitude_change = 
			m_st_flight_basic_output.count_altitude_change;
		m_st_pitch_control_input.time_control =
			m_st_flight_basic_output.st_control_time.time_control;
		m_st_pitch_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;
		m_st_pitch_control_input.time_altitude_control =
			m_st_flight_basic_output.st_control_time.time_altitude_control;
		m_st_pitch_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;
		m_st_pitch_control_input.time_altitude_change_start =
			m_st_flight_basic_output.st_control_time.time_altitude_change_start;
		m_st_pitch_control_input.time_combat_delay = 
			m_st_roll_control_output.time_combat_delay;
		m_st_pitch_control_input.gama_command_compensate = 
			m_st_roll_control_output.gama_command_compensate;
		break;
	case ENUM_CONTROL_ENGINE:		//更新发动机控制模块输入
		m_math_control_engine.flight_time = flight_time;
		m_math_control_engine.time_tick = time_tick;
		m_st_engine_control_input.mass = m_st_flight_basic_output.mass_calc;//估计质量
		m_st_engine_control_input.h = m_st_flight_basic_output.hz;//海拔高度
		m_st_engine_control_input.h_ini = m_st_flight_basic_output.h_ini;//初始高度
		m_st_engine_control_input.temperature_ground = m_st_flight_basic_output.ground_temperature;
		m_st_engine_control_input.target_time = m_st_flight_basic_output.target_time;//目标时间
		m_st_engine_control_input.target_velocity = m_st_flight_basic_output.target_velocity;//目标速度
		m_st_engine_control_input.missile_average_velocity = m_st_flight_basic_output.velocity_average_10s;//状态地速
		m_st_engine_control_input.mach = m_st_flight_basic_output.mach;//马赫数
		m_st_engine_control_input.radius_zw = m_st_flight_basic_output.radius_zw;//转弯半径
		m_st_engine_control_input.flag_launch_turn = m_st_flight_basic_output.flag_launch_turn;
		m_st_engine_control_input.flag_alltitude_change = m_st_flight_basic_output.flag_altitude_change;
		m_st_engine_control_input.flag_waypoint_turn = m_st_flight_basic_output.flag_waypoint_turn;
		m_st_engine_control_input.flag_alltitude_climb = m_st_flight_basic_output.flag_altitude_climb;
		m_st_engine_control_input.flag_alltitude_decline = m_st_flight_basic_output.flag_altitude_decline;
		m_st_engine_control_input.flag_velocity_control = m_st_flight_basic_output.flag_velocity_control;
		break;
	case ENUM_CONTROL_OUT:			//更新舵分配模块输入
		m_math_control_out.flight_time = flight_time;
		m_math_control_out.time_tick = time_tick;
		m_st_control_out_input.gama	= m_st_flight_basic_output.gama;
		m_st_control_out_input.u25g	= m_st_roll_control_output.u25g;
		m_st_control_out_input.u4g	= m_st_roll_control_output.u4g;
		m_st_control_out_input.ug_adrc = m_st_roll_control_output.ug_adrc;
		m_st_control_out_input.u2f	= m_st_pitch_control_output.u2f;
		m_st_control_out_input.u5f	= m_st_pitch_control_output.u5f;
		m_st_control_out_input.ugf	= m_st_pitch_control_output.ugf;
		m_st_control_out_input.u5h	= m_st_yaw_control_output.u5h;
		m_st_control_out_input.urf_zd = m_st_pitch_control_output.urf_zd;
		m_st_control_out_input.urh_zd = m_st_yaw_control_output.urh_zd;
		m_st_control_out_input.time_control_on = 
			m_st_flight_basic_output.st_control_time.time_control;
		m_st_control_out_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;
		m_st_control_out_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;
		m_st_control_out_input.time_combat_delay = 
			m_st_roll_control_output.time_combat_delay;
		break;
	default:
		break;
	}
}

void CMathControlMain::Update_Output_Data()
{
	//输出导引头数据
	p_st_data_controller_to_seeker->flag_seeker_on = m_st_flight_basic_output.st_command.flag_seeker_on;
	p_st_data_controller_to_seeker->flag_lock_on_permit = m_st_flight_basic_output.st_command.flag_lock_on_permit;
	p_st_data_controller_to_seeker->pitch_gimbal_angle_calc = m_st_flight_basic_output.phif;
	p_st_data_controller_to_seeker->yaw_gimbal_angle_calc = m_st_flight_basic_output.phih;
	/*输出数据链数据
	p_st_data_controller_to_datalink->st_missile_state_data.missile_ID = p_st_initial_data->missile_ID;
	p_st_data_controller_to_datalink->st_missile_state_data.flag_missile_launched = true;
	p_st_data_controller_to_datalink->st_missile_state_data.gama = p_st_data_ins_to_controller->gama;
	p_st_data_controller_to_datalink->st_missile_state_data.psi = p_st_data_ins_to_controller->psi;
	p_st_data_controller_to_datalink->st_missile_state_data.zeta = p_st_data_ins_to_controller->zeta;
	p_st_data_controller_to_datalink->st_missile_state_data.longitude = p_st_data_ins_to_controller->longitude;
	p_st_data_controller_to_datalink->st_missile_state_data.latitude = p_st_data_ins_to_controller->latitude;
	p_st_data_controller_to_datalink->st_missile_state_data.height = p_st_data_ins_to_controller->height;
	p_st_data_controller_to_datalink->st_missile_state_data.wx = p_st_data_ins_to_controller->wx;
	p_st_data_controller_to_datalink->st_missile_state_data.wy = p_st_data_ins_to_controller->wy;
	p_st_data_controller_to_datalink->st_missile_state_data.wz = p_st_data_ins_to_controller->wz;
	p_st_data_controller_to_datalink->st_missile_state_data.vtx = p_st_data_ins_to_controller->vtx;
	p_st_data_controller_to_datalink->st_missile_state_data.vty = p_st_data_ins_to_controller->vty;
	p_st_data_controller_to_datalink->st_missile_state_data.vtz = p_st_data_ins_to_controller->vtz;
	p_st_data_controller_to_datalink->st_missile_state_data.au = p_st_data_ins_to_controller->au;
	p_st_data_controller_to_datalink->st_missile_state_data.ax = p_st_data_ins_to_controller->ax;
	p_st_data_controller_to_datalink->st_missile_state_data.ay = p_st_data_ins_to_controller->ay;
	p_st_data_controller_to_datalink->st_missile_state_data.az = p_st_data_ins_to_controller->az;
	p_st_data_controller_to_datalink->st_missile_state_data.pitch_LOS = m_st_flight_basic_output.qf;
	p_st_data_controller_to_datalink->st_missile_state_data.yaw_LOS = m_st_flight_basic_output.qh;
	p_st_data_controller_to_datalink->st_missile_state_data.time_to_go = m_st_flight_basic_output.time_to_go;
	*/
	//输出发动机数据
	p_st_data_controller_to_engine->control_rpm = m_st_engine_control_output.control_rpm;
	p_st_data_controller_to_engine->ECU_work_cmd = m_st_flight_basic_output.ECU_work_cmd;

	//输出舵控数据
	p_st_data_controller_to_actuator->control_voltage_I   = m_st_control_out_output.u1;
	p_st_data_controller_to_actuator->control_voltage_II  = m_st_control_out_output.u2;
	p_st_data_controller_to_actuator->control_voltage_III = m_st_control_out_output.u3;
	p_st_data_controller_to_actuator->control_voltage_IV  = m_st_control_out_output.u4;
	
	//输出开关量数据
	p_st_data_controller_to_switch_output->flag_engine_start = m_st_flight_basic_output.st_command.flag_engine_start;//发动机开机
	p_st_data_controller_to_switch_output->flag_launch_missile_wing = m_st_flight_basic_output.st_command.flag_launch_missile_wing;//弹翼展开
	p_st_data_controller_to_switch_output->flag_separate_booster = m_st_flight_basic_output.st_command.flag_separate_booster;//助推器分离
}
