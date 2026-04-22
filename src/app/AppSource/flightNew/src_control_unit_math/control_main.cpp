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
	p_st_data_controller_to_datalink = NULL;
	p_st_data_controller_to_datalinktel = NULL;
	p_st_data_controller_to_engine = NULL;
	p_st_data_controller_to_actuator = NULL;
	p_st_data_controller_to_switch_output = NULL;
	p_st_route_data_preflight = NULL;
	p_st_initial_data = NULL;
//	p_st_debug_monitor = NULL;


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
		//数据链 指定 攻击目标ID，即 人在回路锁定，飞控将目标ID及特征信息，发动给导引头实现目标选择，实现稳定跟踪
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
		//根据攻击目标ID，确定锁定目标m_target_attack_ID
		memcpy(&m_st_flight_basic_input.st_seeker_data,
			&p_st_data_seeker_to_controller[m_target_attack_ID],
			sizeof(Stru_Data_Seeker_To_Controller));
		memcpy(&m_st_flight_basic_input.st_datalink_datasig,
			p_st_data_datalink_to_controllersig,
			sizeof(Stru_Data_Datalink_To_ControllerSig));
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
	//输出数据链数据
	p_st_data_controller_to_datalinktel->curPtNo = m_st_flight_basic_output.num_way_point_target;//m_st_flight_basic_output;
	p_st_data_controller_to_datalinktel->curTargetLon = m_st_flight_basic_output.target_long;
	p_st_data_controller_to_datalinktel->curTargetLat = m_st_flight_basic_output.target_lat;
	p_st_data_controller_to_datalinktel->curTargetAlt = m_st_flight_basic_output.target_height;
	p_st_data_controller_to_datalinktel->ac_dL = m_st_flight_basic_output.distance_target;//待飞距
	//通道舵
	p_st_data_controller_to_datalinktel->rudderRollCmd = m_st_control_out_output.ug;	//通道舵副翼
	p_st_data_controller_to_datalinktel->rudderPitchCmd = m_st_control_out_output.uf;	//通道舵升降
	p_st_data_controller_to_datalinktel->rudderYawCmd = m_st_control_out_output.uh;		//通道舵航向
	//遥测信息
	p_st_data_controller_to_datalinktel->gamaCmd = m_st_roll_control_output.gama_command;	//滚转角指令
	p_st_data_controller_to_datalinktel->nycCmd = m_st_pitch_control_output.ny_command;		//过载指令
	p_st_data_controller_to_datalinktel->varthetaCmd = m_st_pitch_control_output.zeta_command;//俯仰角指令
	p_st_data_controller_to_datalinktel->heightCmd = m_st_pitch_control_output.h_command;//高度指令
	p_st_data_controller_to_datalinktel->ac_dZ = m_st_flight_basic_output.sz;// 侧边距
	p_st_data_controller_to_datalinktel->token_long = m_st_flight_basic_output.token_long;// 纵向令牌
	p_st_data_controller_to_datalinktel->token_late = m_st_flight_basic_output.token_lat;// 侧向令牌
	p_st_data_controller_to_datalinktel->thrustCmd = m_st_engine_control_output.control_Kc;// 推力指令
	p_st_data_controller_to_datalinktel->ac_dPsi = m_st_flight_basic_output.dlt_psic;//航向角偏差
	p_st_data_controller_to_datalinktel->ac_dR = m_st_flight_basic_output.sz_circle;// 圆轨迹侧边距，未用到
	p_st_data_controller_to_datalinktel->cur_thetav = m_st_flight_basic_output.theta;//轨迹倾角，未用到
	p_st_data_controller_to_datalinktel->Vcmd = m_st_flight_basic_output.target_velocity;//速度指令 
	p_st_data_controller_to_datalinktel->nyCmd_Guidance = 0.0;//末制导纵向过载指令，与过载指令复用
	p_st_data_controller_to_datalinktel->nzCmd_Guidance = 0.0;//末制导侧向过载指令，与滚转角复用
	p_st_data_controller_to_datalinktel->pitch_rate_nT_filterOut = m_st_flight_basic_output.dqf;//俯仰视线角速度滤波
	p_st_data_controller_to_datalinktel->yaw_rate_nT_filterOut = m_st_flight_basic_output.dqh;//偏航视线角速度滤波
	p_st_data_controller_to_datalinktel->deltaR = m_st_flight_basic_output.distance_target;//弹目距离
	p_st_data_controller_to_datalinktel->dRn = 0.0; //弹目北向距离  未用到，预留
	p_st_data_controller_to_datalinktel->dRu = 0.0; //弹目天向距离  未用到，预留
	p_st_data_controller_to_datalinktel->dRe = 0.0; //弹目东向距离  未用到，预留
	p_st_data_controller_to_datalinktel->Pitch_Preset_Angle = m_st_flight_basic_output.phif;//理论俯仰框架角
	p_st_data_controller_to_datalinktel->Yaw_Preset_Angle = m_st_flight_basic_output.phih;//理论偏航框架角
	p_st_data_controller_to_datalinktel->Dubins_stage = 0;//杜宾斯段
	p_st_data_controller_to_datalinktel->dubins_type1 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->dubins_type2 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->dubins_type3 = 0;//杜宾斯类型
	p_st_data_controller_to_datalinktel->Dubins_length = 0.0;//杜宾斯段航程
	p_st_data_controller_to_datalinktel->test1 = 0.0;//测试
	p_st_data_controller_to_datalinktel->Min_IAS2Vel = 0.0;//最低折算速度
	p_st_data_controller_to_datalinktel->mx_ESO = 0.0;
	p_st_data_controller_to_datalinktel->fduox_ADRC = 0.0;//ADRC舵偏
	p_st_data_controller_to_datalinktel->Qv = m_st_flight_basic_output.dynamic_pressure;//动压
	p_st_data_controller_to_datalinktel->alpha_ins = m_st_flight_basic_output.alpha_vg;	//地速攻角
	p_st_data_controller_to_datalinktel->beta_ins = m_st_flight_basic_output.beita_vg;	//地速侧滑角
	p_st_data_controller_to_datalinktel->MaxRpm = 0.0; 		//最大转速
	p_st_data_controller_to_datalinktel->DFT_freq_max = 0.0;//辨识运动频率
	
	
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
	//开关量输出
	p_st_data_controller_to_switch_output->flag_missle_takeoff = m_st_flight_basic_output.st_command.flag_missile_takeoff;//起飞
	p_st_data_controller_to_switch_output->flag_engine_shutdown = m_st_flight_basic_output.st_command.flag_engine_shutdown;//发动机关机
	p_st_data_controller_to_switch_output->flag_separate_booster = m_st_flight_basic_output.st_command.flag_separate_booster;//开伞
}
