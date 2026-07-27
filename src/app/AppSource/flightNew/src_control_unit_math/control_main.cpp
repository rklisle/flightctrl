#include <cstring>
#include "control_main.h"

CMathControlMain::CMathControlMain()
{
	missile_ID = 0;
	m_target_attack_ID = 0;
	p_st_data_seeker_to_controller = NULL;
	p_st_data_ins_to_controller = NULL;
	//p_st_data_datalink_to_controller = NULL;
	p_st_data_datalink_to_controllersig = NULL;
	p_st_data_engine_to_controller = NULL;
	p_st_data_baro_to_controller = NULL;
	p_st_data_radioalt_to_controller = NULL;
	p_st_data_controller_to_seeker = NULL;
	//p_st_data_controller_to_datalink = NULL;
	p_st_data_controller_to_datalinktel = NULL;
	p_st_data_controller_to_engine = NULL;
	p_st_data_controller_to_actuator = NULL;
	p_st_data_controller_to_switch_output = NULL;
	p_st_route_data_preflight = NULL;
	p_st_initial_data = NULL;
	//p_st_debug_monitor = NULL;

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
/*	m_math_control_flight_basic.p_st_debug_monitor = p_st_debug_monitor;
	m_math_control_roll.p_st_debug_monitor = p_st_debug_monitor;
	m_math_control_yaw.p_st_debug_monitor = p_st_debug_monitor;
	m_math_control_pitch.p_st_debug_monitor = p_st_debug_monitor;
	m_math_control_engine.p_st_debug_monitor = p_st_debug_monitor;
	m_math_control_out.p_st_debug_monitor = p_st_debug_monitor;
*/
	m_math_control_flight_basic.p_st_initial_data = p_st_initial_data;
	m_math_control_flight_basic.p_st_route_data_preflight = p_st_route_data_preflight;
	m_math_control_flight_basic.Initial();
	
	m_math_control_roll.Initial();
	m_math_control_yaw.Initial();
	
	m_math_control_pitch.p_st_pitch_control_input->ktheta_lauch_enc = m_st_flight_basic_output.ktheta_lauch_enc;
	m_math_control_pitch.p_st_pitch_control_input->ktheta_climb_enc = m_st_flight_basic_output.ktheta_climb_enc;
	m_math_control_pitch.p_st_pitch_control_input->ktheta_hight_enc = m_st_flight_basic_output.ktheta_hight_enc;
	m_math_control_pitch.Initial();
	
	m_math_control_engine.Initial();

	m_math_control_out.Initial();
}

void CMathControlMain::Run()
{
	//目标选择
	Choose_Target_ID();
	
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
	
	//控制输出给个设备
	Update_Output_Data();	
}

void CMathControlMain::Choose_Target_ID()
{	
	//暂不添加，后续补充，暂时设定为第0个目标，根据目标“战斗及跟踪状态”，确定具体信息是否有效；
	m_target_attack_ID = 0;
}

void CMathControlMain::Update_Input_Data(MODULE_TYPE MODULE_NAME)
{
	switch (MODULE_NAME)
	{
	case ENUM_FLIGHT_BASIC:			//更新基本信息计算模块输入
		m_st_flight_basic_input.missile_ID = missile_ID;
		//数据链 指定 攻击目标ID，即 人在回路锁定，飞控将目标ID及特征信息，发动给导引头实现目标选择，实现稳定跟踪
		//m_target_attack_ID = p_st_data_datalink_to_controller->st_mission_update_data[missile_ID].target_ID;
		m_math_control_flight_basic.flight_time = flight_time;
		m_math_control_flight_basic.time_tick = time_tick;
		
		//发动机控制 给 基础控制参数解算
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

		//根据攻击目标ID，导入选择目标信息
		memcpy(&m_st_flight_basic_input.st_seeker_data,
			&p_st_data_seeker_to_controller[m_target_attack_ID],
			sizeof(Stru_Data_Seeker_To_Controller));
		//地面数据链处理
		memcpy(&m_st_flight_basic_input.st_datalink_datasig,
			p_st_data_datalink_to_controllersig,
			sizeof(Stru_Data_Datalink_To_ControllerSig));
		// 组网数据链数据，暂时不使用
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
		m_st_roll_control_input.gama_command_guidance = m_st_flight_basic_output.gama_command;
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
		m_st_roll_control_input.time_missile_takeoff = 
			m_st_flight_basic_output.st_control_time.time_missile_takeoff;
		m_st_roll_control_input.time_altitude_control =
			m_st_flight_basic_output.st_control_time.time_altitude_control;
		m_st_roll_control_input.time_launch_turn_ok =
			m_st_flight_basic_output.st_control_time.time_launch_turn_ok;
		m_st_roll_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;
		//m_st_roll_control_input.time_combat_delay = 
		//	m_st_flight_basic_output.st_control_time.time_combat_delay;
		m_st_roll_control_input.time_combat_dive_sidectrl = 
			m_st_flight_basic_output.st_control_time.time_combat_dive_sidectrl;//虚拟打击后，侧向控制
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
		m_st_yaw_control_input.hz = m_st_flight_basic_output.hz;
		m_st_yaw_control_input.gama = m_st_flight_basic_output.gama;
		m_st_yaw_control_input.wx	= m_st_flight_basic_output.wx;
		m_st_yaw_control_input.wy	= m_st_flight_basic_output.wy;
		m_st_yaw_control_input.v	= m_st_flight_basic_output.v;
		m_st_yaw_control_input.g	= m_st_flight_basic_output.g;
		m_st_yaw_control_input.qh	= m_st_flight_basic_output.qh;
		m_st_yaw_control_input.tgo	= m_st_flight_basic_output.time_to_go;
		m_st_yaw_control_input.nbz = m_st_flight_basic_output.nbz;
		m_st_yaw_control_input.nby = m_st_flight_basic_output.nby;
		m_st_yaw_control_input.nz_command_guidance = m_st_flight_basic_output.nz_command;
		m_st_yaw_control_input.distance_target = m_st_flight_basic_output.distance_target;
		m_st_yaw_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;
		m_st_yaw_control_input.time_missile_takeoff = 
			m_st_flight_basic_output.st_control_time.time_missile_takeoff;
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
		//m_st_pitch_control_input.ktheta_lauch_enc = m_st_flight_basic_output.ktheta_lauch_enc;
		//m_st_pitch_control_input.ktheta_climb_enc = m_st_flight_basic_output.ktheta_climb_enc;
		m_st_pitch_control_input.mass = m_st_flight_basic_output.mass_calc;
		m_st_pitch_control_input.q = m_st_flight_basic_output.dynamic_pressure;
		m_st_pitch_control_input.gama = m_st_flight_basic_output.gama;
		m_st_pitch_control_input.zeta = m_st_flight_basic_output.zeta;
		m_st_pitch_control_input.wz   = m_st_flight_basic_output.wz;
		m_st_pitch_control_input.hz   = m_st_flight_basic_output.hz;
		m_st_pitch_control_input.v    = m_st_flight_basic_output.v;
		m_st_pitch_control_input.vs   = m_st_flight_basic_output.vs;
		m_st_pitch_control_input.g    = m_st_flight_basic_output.g;
		m_st_pitch_control_input.nby  = m_st_flight_basic_output.nby;
		m_st_pitch_control_input.ny_command_guidance = m_st_flight_basic_output.ny_command;
		m_st_pitch_control_input.distance_target = m_st_flight_basic_output.distance_target;
		m_st_pitch_control_input.distance_target_t_combat = 
			m_st_flight_basic_output.distance_target_t_combat;
		m_st_pitch_control_input.target_height = 
			m_st_flight_basic_output.target_height;
		m_st_pitch_control_input.count_altitude_change = 
			m_st_flight_basic_output.count_altitude_change;
		m_st_pitch_control_input.time_control =
			m_st_flight_basic_output.st_control_time.time_control;//启控
		m_st_pitch_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;//助推器分离
		m_st_pitch_control_input.time_missile_takeoff = 
			m_st_flight_basic_output.st_control_time.time_missile_takeoff;//起飞完成
		m_st_pitch_control_input.time_altitude_control =
			m_st_flight_basic_output.st_control_time.time_altitude_control;//高度控制
		m_st_pitch_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;//末制导
		m_st_pitch_control_input.time_combat_dive_pullup = 
			m_st_flight_basic_output.st_control_time.time_combat_dive_pullup;//虚拟打击，俯冲拉起
		m_st_pitch_control_input.time_combat_delay = 
			m_st_roll_control_output.time_combat_delay;		//末制导延迟
		m_st_pitch_control_input.time_altitude_change_start =
			m_st_flight_basic_output.st_control_time.time_altitude_change_start;//高度机动开始
		m_st_pitch_control_input.time_altitude_change_end =
			m_st_flight_basic_output.st_control_time.time_altitude_change_end;//高度机动开始	
		m_st_pitch_control_input.gama_command_compensate = 
			m_st_roll_control_output.gama_command_compensate;
		break;
	case ENUM_CONTROL_ENGINE:		//更新发动机控制模块输入
		m_math_control_engine.flight_time = flight_time;
		m_math_control_engine.time_tick = time_tick;
		m_st_engine_control_input.mass = m_st_flight_basic_output.mass_calc;//估计质量
		m_st_engine_control_input.h = m_st_flight_basic_output.hz;//海拔高度
		m_st_engine_control_input.h_ini = m_st_flight_basic_output.h_ini;//初始高度
		//m_st_engine_control_input.temperature_ground = m_st_flight_basic_output.ground_temperature;
		m_st_engine_control_input.target_time = m_st_flight_basic_output.target_time;//目标时间
		m_st_engine_control_input.target_velocity = m_st_flight_basic_output.target_velocity;//目标速度
		m_st_engine_control_input.missile_average_velocity = m_st_flight_basic_output.velocity_average_10s;//状态地速
		m_st_engine_control_input.mach = m_st_flight_basic_output.mach;//空速马赫数
		m_st_engine_control_input.sonic_speed = m_st_flight_basic_output.sonic_speed;//声速
		m_st_engine_control_input.radius_zw = m_st_flight_basic_output.radius_zw;//转弯半径
		m_st_engine_control_input.flag_launch_turn = m_st_flight_basic_output.flag_launch_turn;
		m_st_engine_control_input.flag_alltitude_change = m_st_flight_basic_output.flag_altitude_change;
		m_st_engine_control_input.flag_waypoint_turn = m_st_flight_basic_output.flag_waypoint_turn;
		m_st_engine_control_input.flag_alltitude_climb = m_st_flight_basic_output.flag_altitude_climb;
		m_st_engine_control_input.flag_alltitude_decline = m_st_flight_basic_output.flag_altitude_decline;
		m_st_engine_control_input.flag_velocity_control = m_st_flight_basic_output.flag_velocity_control;

		m_st_engine_control_input.flag_engine_start = m_st_flight_basic_output.st_command.flag_engine_start;
		m_st_engine_control_input.flag_engine_stop = m_st_flight_basic_output.st_command.flag_engine_shutdown;
		m_st_engine_control_input.flag_missile_takeoff = m_st_flight_basic_output.st_command.flag_missile_takeoff;
		m_st_engine_control_input.flag_altitude_control = m_st_flight_basic_output.st_command.flag_altitude_control;
		m_st_engine_control_input.flag_combat_status = m_st_flight_basic_output.st_command.flag_combat_status;
		m_st_engine_control_input.flag_combat_dive_pullup = m_st_flight_basic_output.st_command.flag_combat_dive_pullup;
		break;
	case ENUM_CONTROL_OUT:			//更新舵分配模块输入
		m_math_control_out.flight_time = flight_time;
		m_math_control_out.time_tick = time_tick;
		m_st_control_out_input.gama	= m_st_flight_basic_output.gama;
		m_st_control_out_input.u25g	= m_st_roll_control_output.u25g;
		m_st_control_out_input.u4g	= m_st_roll_control_output.u4g;
		m_st_control_out_input.ug_adrc = m_st_roll_control_output.ug_adrc;
		m_st_control_out_input.uqkf	= m_st_pitch_control_output.uqkf;
		m_st_control_out_input.u2f	= m_st_pitch_control_output.u2f;
		m_st_control_out_input.u4f	= m_st_pitch_control_output.u4f;
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
	//输出数据链数据
	p_st_data_controller_to_datalinktel->curPtNo = m_st_flight_basic_output.num_way_point_target;//m_st_flight_basic_output;
	p_st_data_controller_to_datalinktel->curTargetLon = m_st_flight_basic_output.target_long;
	p_st_data_controller_to_datalinktel->curTargetLat = m_st_flight_basic_output.target_lat;
	p_st_data_controller_to_datalinktel->curTargetAlt = m_st_flight_basic_output.target_height;
	p_st_data_controller_to_datalinktel->rudderRollCmd = m_st_control_out_output.ug;	//通道舵副翼
	p_st_data_controller_to_datalinktel->rudderPitchCmd = m_st_control_out_output.uf;	//通道舵升降
	p_st_data_controller_to_datalinktel->rudderYawCmd = m_st_control_out_output.uh;		//通道舵航向

	p_st_data_controller_to_datalinktel->Vcmd = m_st_flight_basic_output.target_velocity;//速度指令 
	p_st_data_controller_to_datalinktel->thrustCmd = m_st_engine_control_output.control_Kc;// 推力指令
	p_st_data_controller_to_datalinktel->rpmState = m_st_flight_basic_output.rpmState;//估计转速
	
	p_st_data_controller_to_datalinktel->gamaCmd = m_st_roll_control_output.gama_command;	//滚转角指令	
	p_st_data_controller_to_datalinktel->nycCmd = m_st_pitch_control_output.ny_command;	//过载指令，升力面过载
	p_st_data_controller_to_datalinktel->varthetaCmd = m_st_pitch_control_output.zeta_command;//俯仰角指令
	p_st_data_controller_to_datalinktel->heightCmd = m_st_pitch_control_output.h_command;//高度指令
	p_st_data_controller_to_datalinktel->vyCmd = m_st_pitch_control_output.h_rate_command;//垂速指令

	
	p_st_data_controller_to_datalinktel->nyCmd_Guidance = m_st_pitch_control_output.ny_command;//纵向过载指令，与过载指令复用
	p_st_data_controller_to_datalinktel->nzCmd_Guidance = m_st_yaw_control_output.nz_command;//侧向过载指令，体轴系过载
	p_st_data_controller_to_datalinktel->wyCmd = m_st_yaw_control_output.wy_command;
	
	p_st_data_controller_to_datalinktel->ac_dL = m_st_flight_basic_output.target_distance;//待飞距，打击点距离改为航段目标点距离distance_target
	p_st_data_controller_to_datalinktel->ac_dZ = m_st_flight_basic_output.sz;// 侧边距
	p_st_data_controller_to_datalinktel->flight_control_state = m_st_flight_basic_output.flight_control_state;//飞行状态
	p_st_data_controller_to_datalinktel->ac_Vz = m_st_flight_basic_output.vnz;
	p_st_data_controller_to_datalinktel->ac_Vy = m_st_flight_basic_output.vs;
	p_st_data_controller_to_datalinktel->ac_Vx = m_st_flight_basic_output.vnx;
	p_st_data_controller_to_datalinktel->ac_dR = m_st_flight_basic_output.sz_circle;// 圆轨迹侧边距，未用到
	p_st_data_controller_to_datalinktel->cur_azimuth= m_st_flight_basic_output.azimuth;//航段方位角
	p_st_data_controller_to_datalinktel->cur_thetav = m_st_flight_basic_output.theta;//轨迹倾角
	p_st_data_controller_to_datalinktel->cur_psicv = m_st_flight_basic_output.psicn;//轨迹偏角
	
	p_st_data_controller_to_datalinktel->pitch_rate_nT_filterOut = m_st_flight_basic_output.dqf;//俯仰视线角速度滤波
	p_st_data_controller_to_datalinktel->yaw_rate_nT_filterOut = m_st_flight_basic_output.dqh;//偏航视线角速度滤波
	p_st_data_controller_to_datalinktel->deltaR = m_st_flight_basic_output.distance_target;//弹目距离
	p_st_data_controller_to_datalinktel->dRn = m_st_flight_basic_output.Rmt_n[0]; //弹目北向距离，仅用于遥测
	p_st_data_controller_to_datalinktel->dRu = m_st_flight_basic_output.Rmt_n[1]; //弹目天向距离，仅用于遥测
	p_st_data_controller_to_datalinktel->dRe = m_st_flight_basic_output.Rmt_n[2]; //弹目东向距离，仅用于遥测
	p_st_data_controller_to_datalinktel->Pitch_Preset_Angle = m_st_flight_basic_output.phif;//理论俯仰框架角
	p_st_data_controller_to_datalinktel->Yaw_Preset_Angle = m_st_flight_basic_output.phih;//理论偏航框架角
	p_st_data_controller_to_datalinktel->Dubins_stage = 0;//杜宾斯段
	p_st_data_controller_to_datalinktel->dubins_type1 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->dubins_type2 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->dubins_type3 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->Dubins_length = 0.0;//杜宾斯段航程
	p_st_data_controller_to_datalinktel->gamac_compensate = m_st_roll_control_output.gama_command_compensate;
	p_st_data_controller_to_datalinktel->uz_gamac = m_st_roll_control_output.uz;
	p_st_data_controller_to_datalinktel->mx_ESO = m_st_roll_control_output.z2_adrc;
	p_st_data_controller_to_datalinktel->fduox_ADRC = m_st_roll_control_output.ug_adrc;//ADRC舵偏
	p_st_data_controller_to_datalinktel->Qv = m_st_flight_basic_output.dynamic_pressure;//动压
	p_st_data_controller_to_datalinktel->alpha_ins = m_st_flight_basic_output.alpha_vg;	//地速攻角
	p_st_data_controller_to_datalinktel->beta_ins = m_st_flight_basic_output.beita_vg;	//地速侧滑角
	p_st_data_controller_to_datalinktel->nyflt = m_st_flight_basic_output.nby;
	p_st_data_controller_to_datalinktel->nzflt = m_st_flight_basic_output.nbz;
	p_st_data_controller_to_datalinktel->count_altitude_change = m_st_flight_basic_output.count_altitude_change;
	p_st_data_controller_to_datalinktel->mass_calc = m_st_flight_basic_output.mass_calc;
	p_st_data_controller_to_datalinktel->ugf_zetac = m_st_pitch_control_output.ugf;
	p_st_data_controller_to_datalinktel->uqkf = m_st_pitch_control_output.uqkf;
	p_st_data_controller_to_datalinktel->flight_time = flight_time;
	//补充，用于仿真显示等功能补充
	p_st_data_controller_to_datalinktel->rudder_I_cmd = m_st_control_out_output.u1;//物理舵
	p_st_data_controller_to_datalinktel->rudder_II_cmd = m_st_control_out_output.u2;
	p_st_data_controller_to_datalinktel->rudder_III_cmd = m_st_control_out_output.u3;
	p_st_data_controller_to_datalinktel->rudder_IV_cmd = m_st_control_out_output.u4;
	p_st_data_controller_to_datalinktel->rudder_V_cmd = m_st_control_out_output.u5;
	p_st_data_controller_to_datalinktel->rudder_VI_cmd = m_st_control_out_output.u6;
	//补充，用于仿真显示等功能补充
	p_st_data_controller_to_datalinktel->Kc_cmd = m_st_engine_control_output.control_Kc;//油门
	p_st_data_controller_to_datalinktel->curLon = m_st_flight_basic_output.longitude;//经度、纬度、高度
	p_st_data_controller_to_datalinktel->curLat = m_st_flight_basic_output.latitude;
	p_st_data_controller_to_datalinktel->curAlt = m_st_flight_basic_output.hz;//组合高度

	//输出给舵数据
	p_st_data_controller_to_actuator->control_voltage_I   = m_st_control_out_output.u1;
	p_st_data_controller_to_actuator->control_voltage_II  = m_st_control_out_output.u2;
	p_st_data_controller_to_actuator->control_voltage_III = m_st_control_out_output.u3;
	p_st_data_controller_to_actuator->control_voltage_IV  = m_st_control_out_output.u4;
	p_st_data_controller_to_actuator->control_voltage_V  = m_st_control_out_output.u5;
	p_st_data_controller_to_actuator->control_voltage_VI  = m_st_control_out_output.u6;
	
	//输出给发动机数据
	p_st_data_controller_to_engine->control_Kc = m_st_engine_control_output.control_Kc;//油门控制输出
	p_st_data_controller_to_engine->control_rpm = m_st_engine_control_output.control_rpm;//发动机转速指令，未使用
	p_st_data_controller_to_engine->ECU_work_cmd = m_st_flight_basic_output.ECU_work_cmd;//发动机控制指令，未使用
	
	//输出导引头数据
	p_st_data_controller_to_seeker->flag_seeker_on = m_st_flight_basic_output.st_command.flag_seeker_on;
	p_st_data_controller_to_seeker->flag_lock_on_permit = m_st_flight_basic_output.st_command.flag_lock_on_permit;
	p_st_data_controller_to_seeker->flag_target_lock = m_st_flight_basic_output.st_command.flag_target_lock;
	p_st_data_controller_to_seeker->target_num__choosen = m_st_flight_basic_output.target_num__choosen;
	p_st_data_controller_to_seeker->pitch_gimbal_angle_calc = m_st_flight_basic_output.phif;
	p_st_data_controller_to_seeker->yaw_gimbal_angle_calc = m_st_flight_basic_output.phih;
	
	//输出给电气开关量数据
	p_st_data_controller_to_switch_output->flag_launch_missile_wing = m_st_flight_basic_output.st_command.flag_launch_missile_wing;//弹翼展开，未用到(一般力及力矩模型处理)
	p_st_data_controller_to_switch_output->flag_separate_booster = m_st_flight_basic_output.st_command.flag_separate_booster;//助推器分离，力及力矩模型需要（分离后质量减轻），半实物仿真增加
	p_st_data_controller_to_switch_output->flag_engine_start = m_st_flight_basic_output.st_command.flag_engine_start;//发动机开机，发动机油门切换，无火工品，未用到
	p_st_data_controller_to_switch_output->flag_missle_takeoff = m_st_flight_basic_output.st_command.flag_missile_takeoff;//起飞，只做标识，无执行继电器，未用到
	p_st_data_controller_to_switch_output->flag_engine_shutdown = m_st_flight_basic_output.st_command.flag_engine_shutdown;//发动机关机，发动机执行，未用到
	p_st_data_controller_to_switch_output->flag_open_umbrella = m_st_flight_basic_output.st_command.flag_open_umbrella;//开伞，开伞舵机执行，后续关联气动模型
	p_st_data_controller_to_switch_output->flag_fuze_unlock = m_st_flight_basic_output.st_command.flag_fuze_unlock;//引信解锁，引信执行
}
