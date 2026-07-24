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

	//��ʼ�����ݻ���
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

	//�������������
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
	//Ŀ��ѡ��
	Choose_Target_ID();
	
	//�����л����Ƶ���·
	Update_Input_Data(ENUM_FLIGHT_BASIC);
	m_math_control_flight_basic.Run();

	//��ͨ�����ƣ����������򼰹�ת
	Update_Input_Data(ENUM_CONTROL_ROLL);	
	m_math_control_roll.Run();
	Update_Input_Data(ENUM_CONTROL_YAW);
	m_math_control_yaw.Run();
	Update_Input_Data(ENUM_CONTROL_PITCH);
	m_math_control_pitch.Run();
	
	//�ٶȿ���
	Update_Input_Data(ENUM_CONTROL_ENGINE);
	m_math_control_engine.Run();
	
	//��ط���
	Update_Input_Data(ENUM_CONTROL_OUT);
	m_math_control_out.Run();
	
	//������������豸
	Update_Output_Data();	
}

//Ŀ��ѡ���㷨
//���ܣ����ݵ���ͷ��Ϣ��λ�á��߶ȵ���Ϣ��ȷ������Ŀ��ID��
//���룺��ȡ����ͷĿ�����ݣ�����ͷ������ģ�⣬
//		Ŀ������(ָ��)p_st_data_seeker_to_controller+k��k=0...15
//		��಻����16����Ŀ����Ϣ���飬����0��ʾ��һ��Ŀ�꣬k��ʾ��k+1��Ŀ��
//˵���������絼��ͷ�ӳ��У�ֻ��һ��Ŀ�ֻ꣬������0Ŀ����Ч��		
//�����Ŀ��ID����Ŀ��ID == -1ʱ����ʾ����ЧĿ�꣬����Ϊѡ��Ŀ��������
void CMathControlMain::Choose_Target_ID()
{	
	//�ݲ����ӣ��������䣬��ʱ�趨Ϊ��0��Ŀ�꣬����Ŀ�ꡰս��������״̬����ȷ��������Ϣ�Ƿ���Ч��
	m_target_attack_ID = 0;
}

void CMathControlMain::Update_Input_Data(MODULE_TYPE MODULE_NAME)
{
	switch (MODULE_NAME)
	{
	case ENUM_FLIGHT_BASIC:			//���»�����Ϣ����ģ������
		m_st_flight_basic_input.missile_ID = missile_ID;
		//������ ָ�� ����Ŀ��ID���� ���ڻ�·�������ɿؽ�Ŀ��ID��������Ϣ������������ͷʵ��Ŀ��ѡ��ʵ���ȶ�����
		//m_target_attack_ID = p_st_data_datalink_to_controller->st_mission_update_data[missile_ID].target_ID;
		m_math_control_flight_basic.flight_time = flight_time;
		m_math_control_flight_basic.time_tick = time_tick;
		
		//���������� �� �������Ʋ�������
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

		//���ݹ���Ŀ��ID������ѡ��Ŀ����Ϣ
		memcpy(&m_st_flight_basic_input.st_seeker_data,
			&p_st_data_seeker_to_controller[m_target_attack_ID],
			sizeof(Stru_Data_Seeker_To_Controller));
		//��������������
		memcpy(&m_st_flight_basic_input.st_datalink_datasig,
			p_st_data_datalink_to_controllersig,
			sizeof(Stru_Data_Datalink_To_ControllerSig));
		// �������������ݣ���ʱ��ʹ��
		//memcpy(&m_st_flight_basic_input.st_datalink_data,
		//	p_st_data_datalink_to_controller,
		//	sizeof(Stru_Data_Datalink_To_Controller));
		break;
	case ENUM_CONTROL_ROLL:			//���¹�������ģ������
		m_math_control_roll.flight_time = flight_time;
		m_math_control_roll.time_tick = time_tick;
		m_st_roll_control_input.mass = m_st_flight_basic_output.mass_calc;
		m_st_roll_control_input.gama = m_st_flight_basic_output.gama;
		m_st_roll_control_input.wx	 = m_st_flight_basic_output.wx;
		m_st_roll_control_input.sz	 = m_st_flight_basic_output.sz;
		m_st_roll_control_input.v	 = m_st_flight_basic_output.v;
		m_st_roll_control_input.vnz	 = m_st_flight_basic_output.vnz;
		m_st_roll_control_input.g	 = m_st_flight_basic_output.g;
		//if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
		//	/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		//{
		//	m_st_roll_control_input.dqh	= p_st_data_seeker_to_controller[m_target_attack_ID].yaw_LOS_rate;
		//} 
		//else
		//{
		//	m_st_roll_control_input.dqh	= m_st_flight_basic_output.dqh;
		//}
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
			m_st_flight_basic_output.st_control_time.time_combat_dive_sidectrl;//�������󣬲������
		m_st_roll_control_input.time_turn_in_start =
			m_st_flight_basic_output.st_control_time.time_turn_in_start;
		m_st_roll_control_input.time_turn_in_end =
			m_st_flight_basic_output.st_control_time.time_turn_in_end;
		m_st_roll_control_input.time_turn_out_start =
			m_st_flight_basic_output.st_control_time.time_turn_out_start;
		m_st_roll_control_input.time_turn_out_end =
			m_st_flight_basic_output.st_control_time.time_turn_out_end;
		break;
	case ENUM_CONTROL_YAW:			//���º������ģ������
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
		//if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
		//	/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		//{
		//	m_st_yaw_control_input.dqh	= p_st_data_seeker_to_controller[m_target_attack_ID].yaw_LOS_rate;
		//} 
		//else
		//{
		//	m_st_yaw_control_input.dqh	= m_st_flight_basic_output.dqh;
		//}
		m_st_yaw_control_input.nbz = m_st_flight_basic_output.nbz;
		m_st_yaw_control_input.nby = m_st_flight_basic_output.nby;
		m_st_yaw_control_input.nz_command_guidance = m_st_flight_basic_output.nz_command;
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
	case ENUM_CONTROL_PITCH:		//���¸�������ģ������
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
		//���ݵ���ͷ���������ѡ�� ����ͷ �� �̶������߽��ٶ�
		//if (p_st_data_seeker_to_controller[m_target_attack_ID].flag_combat_status
		//	/*&&(m_st_flight_basic_output.num_way_point_target == (p_st_route_data_preflight->num_rows - 1))*/)
		//{
		//	m_st_pitch_control_input.dqf = p_st_data_seeker_to_controller[m_target_attack_ID].pitch_LOS_rate;
		//} 
		//else
		//{
		//	m_st_pitch_control_input.dqf = m_st_flight_basic_output.dqf;
		//}
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
			m_st_flight_basic_output.st_control_time.time_control;//����
		m_st_pitch_control_input.time_separate_booster = 
			m_st_flight_basic_output.st_control_time.time_separate_booster;//����������
		m_st_pitch_control_input.time_missile_takeoff = 
			m_st_flight_basic_output.st_control_time.time_missile_takeoff;//������
		m_st_pitch_control_input.time_altitude_control =
			m_st_flight_basic_output.st_control_time.time_altitude_control;//�߶ȿ���
		m_st_pitch_control_input.time_combat_status = 
			m_st_flight_basic_output.st_control_time.time_combat_status;//ĩ�Ƶ�
		m_st_pitch_control_input.time_combat_dive_pullup = 
			m_st_flight_basic_output.st_control_time.time_combat_dive_pullup;//����������������
		m_st_pitch_control_input.time_combat_delay = 
			m_st_roll_control_output.time_combat_delay;		//ĩ�Ƶ��ӳ�
		m_st_pitch_control_input.time_altitude_change_start =
			m_st_flight_basic_output.st_control_time.time_altitude_change_start;//�߶Ȼ�����ʼ
		m_st_pitch_control_input.time_altitude_change_end =
			m_st_flight_basic_output.st_control_time.time_altitude_change_end;//�߶Ȼ�����ʼ	
		m_st_pitch_control_input.gama_command_compensate = 
			m_st_roll_control_output.gama_command_compensate;
		break;
	case ENUM_CONTROL_ENGINE:		//���·���������ģ������
		m_math_control_engine.flight_time = flight_time;
		m_math_control_engine.time_tick = time_tick;
		m_st_engine_control_input.mass = m_st_flight_basic_output.mass_calc;//��������
		m_st_engine_control_input.h = m_st_flight_basic_output.hz;//���θ߶�
		m_st_engine_control_input.h_ini = m_st_flight_basic_output.h_ini;//��ʼ�߶�
		//m_st_engine_control_input.temperature_ground = m_st_flight_basic_output.ground_temperature;
		m_st_engine_control_input.target_time = m_st_flight_basic_output.target_time;//Ŀ��ʱ��
		m_st_engine_control_input.target_velocity = m_st_flight_basic_output.target_velocity;//Ŀ���ٶ�
		m_st_engine_control_input.missile_average_velocity = m_st_flight_basic_output.velocity_average_10s;//״̬����
		m_st_engine_control_input.mach = m_st_flight_basic_output.mach;//����������
		m_st_engine_control_input.sonic_speed = m_st_flight_basic_output.sonic_speed;//����
		m_st_engine_control_input.radius_zw = m_st_flight_basic_output.radius_zw;//ת��뾶
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
	case ENUM_CONTROL_OUT:			//���¶����ģ������
		m_math_control_out.flight_time = flight_time;
		m_math_control_out.time_tick = time_tick;
		m_st_control_out_input.gama	= m_st_flight_basic_output.gama;
		m_st_control_out_input.u25g	= m_st_roll_control_output.u25g;
		m_st_control_out_input.u4g	= m_st_roll_control_output.u4g;
		m_st_control_out_input.ug_adrc = m_st_roll_control_output.ug_adrc;
		m_st_control_out_input.uqkf	= m_st_pitch_control_output.uqkf;
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
	//�������������
	p_st_data_controller_to_datalinktel->curPtNo = m_st_flight_basic_output.num_way_point_target;//m_st_flight_basic_output;
	p_st_data_controller_to_datalinktel->curTargetLon = m_st_flight_basic_output.target_long;
	p_st_data_controller_to_datalinktel->curTargetLat = m_st_flight_basic_output.target_lat;
	p_st_data_controller_to_datalinktel->curTargetAlt = m_st_flight_basic_output.target_height;
	p_st_data_controller_to_datalinktel->rudderRollCmd = m_st_control_out_output.ug;	//ͨ���渱��
	p_st_data_controller_to_datalinktel->rudderPitchCmd = m_st_control_out_output.uf;	//ͨ��������
	p_st_data_controller_to_datalinktel->rudderYawCmd = m_st_control_out_output.uh;		//ͨ���溽��

	p_st_data_controller_to_datalinktel->Vcmd = m_st_flight_basic_output.target_velocity;//�ٶ�ָ�� 
	p_st_data_controller_to_datalinktel->thrustCmd = m_st_engine_control_output.control_Kc;// ����ָ��
	p_st_data_controller_to_datalinktel->rpmState = m_st_flight_basic_output.rpmState;//����ת��
	
	p_st_data_controller_to_datalinktel->gamaCmd = m_st_roll_control_output.gama_command;	//��ת��ָ��	
	p_st_data_controller_to_datalinktel->nycCmd = m_st_pitch_control_output.ny_command;	//����ָ����������
	p_st_data_controller_to_datalinktel->varthetaCmd = m_st_pitch_control_output.zeta_command;//������ָ��
	p_st_data_controller_to_datalinktel->heightCmd = m_st_pitch_control_output.h_command;//�߶�ָ��
	p_st_data_controller_to_datalinktel->vyCmd = m_st_pitch_control_output.h_rate_command;//����ָ��

	
	p_st_data_controller_to_datalinktel->nyCmd_Guidance = m_st_pitch_control_output.ny_command;//�������ָ������ָ���
	p_st_data_controller_to_datalinktel->nzCmd_Guidance = m_st_yaw_control_output.nz_command;//�������ָ�����ϵ����
	p_st_data_controller_to_datalinktel->wyCmd = m_st_yaw_control_output.wy_command;
	
	p_st_data_controller_to_datalinktel->ac_dL = m_st_flight_basic_output.target_distance;//���ɾ࣬���������Ϊ����Ŀ������distance_target
	p_st_data_controller_to_datalinktel->ac_dZ = m_st_flight_basic_output.sz;// ��߾�
	p_st_data_controller_to_datalinktel->flight_control_state = m_st_flight_basic_output.flight_control_state;//����״̬
	//p_st_data_controller_to_datalinktel->token_long = m_st_flight_basic_output.token_long;// ��������
	//p_st_data_controller_to_datalinktel->token_late = m_st_flight_basic_output.token_lat;// ��������
	//p_st_data_controller_to_datalinktel->ac_dPsi = m_st_flight_basic_output.dlt_psic;//�����ƫ��
	p_st_data_controller_to_datalinktel->ac_Vz = m_st_flight_basic_output.vnz;
	p_st_data_controller_to_datalinktel->ac_Vy = m_st_flight_basic_output.vs;
	p_st_data_controller_to_datalinktel->ac_Vx = m_st_flight_basic_output.vnx;
	p_st_data_controller_to_datalinktel->ac_dR = m_st_flight_basic_output.sz_circle;// Բ�켣��߾࣬δ�õ�
	p_st_data_controller_to_datalinktel->cur_azimuth= m_st_flight_basic_output.azimuth;//���η�λ��
	p_st_data_controller_to_datalinktel->cur_thetav = m_st_flight_basic_output.theta;//�켣���
	p_st_data_controller_to_datalinktel->cur_psicv = m_st_flight_basic_output.psicn;//�켣ƫ��
	
	p_st_data_controller_to_datalinktel->pitch_rate_nT_filterOut = m_st_flight_basic_output.dqf;//�������߽��ٶ��˲�
	p_st_data_controller_to_datalinktel->yaw_rate_nT_filterOut = m_st_flight_basic_output.dqh;//ƫ�����߽��ٶ��˲�
	p_st_data_controller_to_datalinktel->deltaR = m_st_flight_basic_output.distance_target;//��Ŀ����
	p_st_data_controller_to_datalinktel->dRn = m_st_flight_basic_output.Rmt_n[0]; //��Ŀ������룬������ң��
	p_st_data_controller_to_datalinktel->dRu = m_st_flight_basic_output.Rmt_n[1]; //��Ŀ������룬������ң��
	p_st_data_controller_to_datalinktel->dRe = m_st_flight_basic_output.Rmt_n[2]; //��Ŀ������룬������ң��
	p_st_data_controller_to_datalinktel->Pitch_Preset_Angle = m_st_flight_basic_output.phif;//���۸�����ܽ�
	p_st_data_controller_to_datalinktel->Yaw_Preset_Angle = m_st_flight_basic_output.phih;//����ƫ����ܽ�
	p_st_data_controller_to_datalinktel->Dubins_stage = 0;//�ű�˹��
	p_st_data_controller_to_datalinktel->dubins_type1 = 0;//�ű�˹����
	p_st_data_controller_to_datalinktel->dubins_type2 = 0;//�ű�˹����
	p_st_data_controller_to_datalinktel->dubins_type3 = 0;//�ű�˹����
	p_st_data_controller_to_datalinktel->Dubins_length = 0.0;//�ű�˹�κ���
	//p_st_data_controller_to_datalinktel->test1 = 0.0;//����
	//p_st_data_controller_to_datalinktel->Min_IAS2Vel = 0.0;//��������ٶ�
	p_st_data_controller_to_datalinktel->gamac_compensate = m_st_roll_control_output.gama_command_compensate;
	p_st_data_controller_to_datalinktel->uz_gamac = m_st_roll_control_output.uz;
	p_st_data_controller_to_datalinktel->mx_ESO = m_st_roll_control_output.z2_adrc;
	p_st_data_controller_to_datalinktel->fduox_ADRC = m_st_roll_control_output.ug_adrc;//ADRC��ƫ
	p_st_data_controller_to_datalinktel->Qv = m_st_flight_basic_output.dynamic_pressure;//��ѹ
	p_st_data_controller_to_datalinktel->alpha_ins = m_st_flight_basic_output.alpha_vg;	//���ٹ���
	p_st_data_controller_to_datalinktel->beta_ins = m_st_flight_basic_output.beita_vg;	//���ٲ໬��
	//p_st_data_controller_to_datalinktel->MaxRpm = 0.0; 		//���ת��
	//p_st_data_controller_to_datalinktel->DFT_freq_max = 0.0;//��ʶ�˶�Ƶ��
	p_st_data_controller_to_datalinktel->nyflt = m_st_flight_basic_output.nby;
	p_st_data_controller_to_datalinktel->nzflt = m_st_flight_basic_output.nbz;
	p_st_data_controller_to_datalinktel->count_altitude_change = m_st_flight_basic_output.count_altitude_change;
	p_st_data_controller_to_datalinktel->mass_calc = m_st_flight_basic_output.mass_calc;
	p_st_data_controller_to_datalinktel->ugf_zetac = m_st_pitch_control_output.ugf;
	p_st_data_controller_to_datalinktel->uqkf = m_st_pitch_control_output.uqkf;
	p_st_data_controller_to_datalinktel->flight_time = flight_time;
	//���䣬���ڷ�����ʾ�ȹ��ܲ���
	p_st_data_controller_to_datalinktel->rudder_I_cmd = m_st_control_out_output.u1;//������
	p_st_data_controller_to_datalinktel->rudder_II_cmd = m_st_control_out_output.u2;
	p_st_data_controller_to_datalinktel->rudder_III_cmd = m_st_control_out_output.u3;
	p_st_data_controller_to_datalinktel->rudder_IV_cmd = m_st_control_out_output.u4;
	p_st_data_controller_to_datalinktel->rudder_V_cmd = m_st_control_out_output.u5;
	p_st_data_controller_to_datalinktel->rudder_VI_cmd = m_st_control_out_output.u6;
	//���䣬���ڷ�����ʾ�ȹ��ܲ���
	p_st_data_controller_to_datalinktel->Kc_cmd = m_st_engine_control_output.control_Kc;//����
	p_st_data_controller_to_datalinktel->curLon = m_st_flight_basic_output.longitude;//���ȡ�γ�ȡ��߶�
	p_st_data_controller_to_datalinktel->curLat = m_st_flight_basic_output.latitude;
	p_st_data_controller_to_datalinktel->curAlt = m_st_flight_basic_output.hz;//��ϸ߶�

	//�����������
	p_st_data_controller_to_actuator->control_voltage_I   = m_st_control_out_output.u1;
	p_st_data_controller_to_actuator->control_voltage_II  = m_st_control_out_output.u2;
	p_st_data_controller_to_actuator->control_voltage_III = m_st_control_out_output.u3;
	p_st_data_controller_to_actuator->control_voltage_IV  = m_st_control_out_output.u4;
	p_st_data_controller_to_actuator->control_voltage_V  = m_st_control_out_output.u5;
	p_st_data_controller_to_actuator->control_voltage_VI  = m_st_control_out_output.u6;
	
	//���������������
	p_st_data_controller_to_engine->control_Kc = m_st_engine_control_output.control_Kc;//���ſ������
	p_st_data_controller_to_engine->control_rpm = m_st_engine_control_output.control_rpm;//������ת��ָ�δʹ��
	p_st_data_controller_to_engine->ECU_work_cmd = m_st_flight_basic_output.ECU_work_cmd;//����������ָ�δʹ��
	
	//�������ͷ����
	p_st_data_controller_to_seeker->flag_seeker_on = m_st_flight_basic_output.st_command.flag_seeker_on;
	p_st_data_controller_to_seeker->flag_lock_on_permit = m_st_flight_basic_output.st_command.flag_lock_on_permit;
	p_st_data_controller_to_seeker->flag_target_lock = m_st_flight_basic_output.st_command.flag_target_lock;
	p_st_data_controller_to_seeker->target_num__choosen = m_st_flight_basic_output.target_num__choosen;
	p_st_data_controller_to_seeker->pitch_gimbal_angle_calc = m_st_flight_basic_output.phif;
	p_st_data_controller_to_seeker->yaw_gimbal_angle_calc = m_st_flight_basic_output.phih;
	
	//�������������������
	p_st_data_controller_to_switch_output->flag_launch_missile_wing = m_st_flight_basic_output.st_command.flag_launch_missile_wing;//����չ����δ�õ�(һ����������ģ�ʹ���)
	p_st_data_controller_to_switch_output->flag_separate_booster = m_st_flight_basic_output.st_command.flag_separate_booster;//���������룬��������ģ����Ҫ��������������ᣩ����ʵ���������
	p_st_data_controller_to_switch_output->flag_engine_start = m_st_flight_basic_output.st_command.flag_engine_start;//�����������������������л����޻�Ʒ��δ�õ�
	p_st_data_controller_to_switch_output->flag_missle_takeoff = m_st_flight_basic_output.st_command.flag_missile_takeoff;//��ɣ�ֻ����ʶ����ִ�м̵�����δ�õ�
	p_st_data_controller_to_switch_output->flag_engine_shutdown = m_st_flight_basic_output.st_command.flag_engine_shutdown;//�������ػ���������ִ�У�δ�õ�
	p_st_data_controller_to_switch_output->flag_open_umbrella = m_st_flight_basic_output.st_command.flag_open_umbrella;//��ɡ����ɡ���ִ�У�������������ģ��
	p_st_data_controller_to_switch_output->flag_fuze_unlock = m_st_flight_basic_output.st_command.flag_fuze_unlock;//���Ž���������ִ��
}
