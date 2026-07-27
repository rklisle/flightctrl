#include <cstring>
#include "control_flight_basic.h"
//#include "../environment_earth_model.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"


// ��׼��������
#define P0      101325.0f    // ��ƽ���׼��ѹ (Pa)
#define T0      288.15f      // ��ƽ���׼�¶� (K)
#define L       0.0065f      // �¶��ݶ� (K/m)
#define G       9.80665f		//�������� m/s^2
#define R       8.31432f
#define M       0.0289644f
#define RHO0    1.225f      // ��׼�����ܶ�kg/m^3
// ���ٹ�У׼ϵ��
#define ARSPD_RATIO	1.0f	//ѹ�����У׼ϵ��0.5~2.0������ �� ��(��ѹ / ARSPD_RATIO)
#define FULL_MASS		165.0	//��������kg  ����Ϊ165kg
#define EMPTY_MASS		105.0	//���أ���ʣ�ͣ�����kg
#define MAX_FLIGHT_TIME	8*3600	//8h��Ӧ 3600s ��ʱ
#define MAX_FLIGHT_DIST	1440*1000	//1440km ����

CMathControlFlightBasic::CMathControlFlightBasic()
{
	m_missile_ID = 0;
	m_missile_flight_mode = 0x55;

	m_vs = 0.0;//��ϴ���
	m_hz = 0.0;//��ϸ߶�
	m_anz = 0.0;
	m_au = 9.8;//������ٶ�
	m_anz = 0.0;//������ٶ�
	//m_an[0] = 0.0;
	//m_an[1] = 9.8;
	//m_an[2] = 0.0;
	m_Cbn[0] = 1;
	m_Cbn[1] = 0;
	m_Cbn[2] = 0;
	m_Cbn[3] = 0;
	m_Cbn[4] = 1;
	m_Cbn[5] = 0;
	m_Cbn[6] = 0;
	m_Cbn[7] = 0;
	m_Cbn[8] = 1; 
	m_ax = 0.0;
	m_ay = 0.0;
	m_az = 0.0;
	m_axflt = 0.0;
	m_ayflt = 0.0;
	m_azflt = 0.0;	
	m_wx = 0.0;
	m_wy = 0.0;
	m_wz = 0.0;
	m_wxflt = 0.0;
	m_wyflt = 0.0;
	m_wzflt = 0.0;
	m_hgps = 0.0;
	m_longitude = 0.0;
	m_latitude = 0.0;
	m_longitude_A = 0.0;
	m_latitude_A = 0.0;
	m_longitude_B = 0.0;
	m_latitude_B = 0.0;
	m_longitude_C = 0.0;
	m_latitude_C = 0.0;
	m_outtrack_angle = 0.0;
	m_hover_round = 0;
	m_attack_angle = 0.0;
	m_g = 0.0;
	m_distance_BP = 0.0;
	m_distance_BP_500 = 0.0;
	m_distance_BP_500pre = 0.0;
	m_distance_BP_projection = 0.0;
	m_alpha_target = 0.0;
	m_distance_target = 0.0;
	m_slant_distance_target = 0.0;
	m_Rmt_n[0] = 99999.9;
	m_Rmt_n[1] = 99999.9;
	m_Rmt_n[2] = 99999.9;
	m_distance_target_t_combat = 99999.9;
	m_gama_target_t_combat = 0.0;
	m_distance_turn_in_compensate = 0.0;
	m_distance_turn_in = 0.0;
	m_total_distance = 0.0;
	m_alpha_AB = 0.0;
	m_sz = 0.0;
	m_sz_record1 = 0.0;
	m_sz_record2 = 0.0;
	m_sz_radius = 0.0;
	m_A = 0.0;
	m_vx = 0.0;
	m_sx = 0.0;
	m_vtx = 0.0;
	m_vty = 0.0;
	m_vtz = 0.0;
	m_v = 0.0;//����
	m_v_air = 0.0;//����
	m_mach = 0.0;
	m_air_temperature = 15.0 + 273.15;
	m_air_density = 1.225;
	m_air_pressure = 101325;
	m_sonic_speed = 340.0;
	m_v_average_1s = 0.0;
	m_v_average_10s = 0.0;
	m_vnx = 0.0;
	m_vnz = 0.0;
	m_vnz_record1 = 0.0;
	m_vnz_record2 = 0.0;
	m_ny = 0.0;
	m_nz = 0.0;
	
	m_zeta = 0.0;
	m_gama = 0.0;
	m_psit = 0.0;
	m_psit_t_turn_in = 0.0;
	m_psin = 0.0;
	m_psicn = 0.0;
	m_theta = 0.0;

	//����ͷ���ָ�״̬
	 //m_seeker_cmd = 0x01;//����ͷ����ָ�0x01 �Լ�(�ϵ��Զ�)��0x02 װ��Ŀ��ģ�壬 0x03 ��ǰ��飬0x04/0x05���ã�0x06�ɿ���������(ָ����ܽ�)��0x07����������0x08������0x09��������
	m_seeker_cmdpara_targettype = 0x03;//ָʾĿ�����ͣ�0x01 ������0x02 ��������0x03 �̶�����
	m_seeker_cmdpara_dltheight = 1000.0;//��Ŀ�߶Ȳ���ڹ���Ŀ�����m
	m_seeker_cmdpara_dltheight = 0;//��Ŀ�߶Ȳ�
	m_seeker_cmdpara_ktheta = 0;//����������
	m_seeker_cmdpara_psi = 0;//����ƫ����
	m_seeker_cmdpara_gama = 0;//������ת��
	m_seeker_cmdpara_phif = 0;//������ܽ�
	m_seeker_cmdpara_phih = 0;//������ܽ�
	//m_seeker_state = 0x00;//����ͷ����״̬��0xAX ������0xFX�쳣��0x5X ������
	m_seeker_state_track = 0x01;//����ͷ����״̬��0x01������λ,0x02��������ʧ������0x03������0x04����
	m_seeker_dqf = 0.0;	//���߽��ٶ�
	m_seeker_dqh = 0.0;
	m_seeker_phif = 0.0;//��ܽǣ�״̬��
	m_seeker_phih = 0.0;
	m_seeker_qf = 0.0;	//���߽ǣ�״̬��
	m_seeker_qh = 0.0;
	m_seeker_pixelf = 0;//��������ƫ�������
	m_seeker_pixelh = 0;//��������ƫ��¸�����
	m_seeker_distance_target = 0.0;//��Ŀ����

	//�߶ȱ�
	m_radioalt_hight = 0.0;
	m_radioalt_status = 0x00;
	//���ٹ�
	count_static_pressure_tyz = 0;
	count_total_pressure_tyz = 0;
	m_static_pressure = 101325;
	m_total_pressure = 101325;
	m_static_pressure_raw= 101325;
	m_total_pressure_raw = 101325;
	m_static_pressure_flt = 101325;
	m_total_pressure_flt = 101325;
	m_baroalt_status = 0x00;
	m_hbaro = 0.0;//��ѹ�߶�
	m_Vbaro = 0.0;//��ѹ�ٶ�

	//����Ŀ�굼�� �� ����ͷ��Ϣ�ۺϺ�����
	m_target_num__choosen = 0;
	m_dqf = 0.0;
	m_dqh = 0.0;
	m_phif = 0.0;
	m_phih = 0.0;
	m_Qf = 0.0;
	m_Qh = 0.0;
	m_Qn = 0.0;
	m_dqf_flt = 0.0;
	m_dqh_flt = 0.0;

	m_ny_command = 0.0;
	m_nz_command = 0.0;
	m_gama_command = 0.0;
	m_gama_command_record = 0.0;

	m_route_mode = 0;
	m_formation_mode = 0;
	m_turn_angle = 0.0;
	m_turn_radius = 0.0;
	m_accept_radius = 0.0;
	m_target_velocity = 0.0;//����Ŀ���ٶ�
	m_target_height = 0.0;	//����Ŀ��߶�
	m_target_height_ground = 0.0;
	m_gama_turn_nominal = 0.0;//BTTת���ת�Ǳ��ֵ
	m_x_coordinate_turn = 0.0;//Բ������
	m_z_coordinate_turn = 0.0;
	
	step_open_umbrella = 0;
	step_altitude_change_lauch = 0;
	step_dualplane_guidance = 0;
	count_distance_recycle = 0;//������հ뾶����
	count_v50_recycle = 0;//�ٶ�С��50m/s����������
	count_h300_v54_recycle = 0;//�ٶ�С��54m/s������Ը߶�С��300m����������
	count_distance_out_recycle = 0;//��Ȧ�ж�
	count_v54_recycle = 0;//��1)�ٶ�С��54m/s��(2) ��Ը߶�С��250m��(3)��Ȧ�󣬷��о������600m
	count_h250_recycle = 0;
	count_distance_out600_recycle = 0;
	count_h10_recycle = 0;//�߶�С��10m��������
	count_h5_ny2_recycle = 0;//�߶�С��5m��y����ش���2
	
	valid_count = 0;
	count_qk = 0;	
	count_v_5 = 0;
	count_fl = 0;	
	count_qd = 0;
	count_v_50 = 0;
	count_takeoff = 0;
	count_tg = 0;
	dlt_time_tg = 0.0;
	count_virtual = 0;
	count_combat_dive_ok = 0;
	count_cooperative_attack = 0;
	count_altitude_change = 0;
	count_altitude_change_enable = 0;
	count_altitude_change_lauch_enable = 0;
	count_altitude_change_energy_enable = 0;
	count_altitude_change_end = 0;
	count_away = 0;
	count_sd_in = 0;
	count_turn_out = 0;
	count_update = 0;
	m_num_way_point = 0;
	m_num_way_point_target = 1;//�������º���������㵥��װ������һ������ΪĿ��㣻
	m_engine_state = 0x00;
	m_engine_state_rpm = 0;
	m_ECU_work_cmd = 0x11;//����

	m_ground_temperature = 0.0;	//����װ��ֵ
	m_ktheta_lauch_enc = 12.0;
	m_ktheta_climb_enc = 6.0;
	
	//p_st_debug_monitor = NULL;
	p_st_initial_data = NULL;
	p_st_route_data_preflight = NULL;
	p_st_flight_basic_input = NULL;	//����
	p_st_flight_basic_output = NULL;//���
	memset(&m_st_target		 , 0, sizeof(Stru_Way_Point));
	memset(&m_st_way_point	 , 0, sizeof(Stru_Way_Point) * MAX_ROUTE_NUMBER);
	memset(&m_st_control_flag, 0, sizeof(Stru_Control_flag));
	memset(&m_st_control_time, 0, sizeof(Stru_Control_Time));
	memset(&m_v_record_100ms , 0, sizeof(double) * 10);
	memset(&m_v_record_1s    , 0, sizeof(double) * 10);
	memset(&m_nav_data_filter    , 0, sizeof(Stru_Tustin_FirstIO_Filter) * 6);
	memset(&m_guide_data_filter    , 0, sizeof(Stru_Tustin_FirstIO_Filter) * 2);
	memset(&m_baro_data_filter    , 0, sizeof(Stru_Tustin_FirstIO_Filter) * 2);
	
	m_total_energy = 0.0;
	m_total_energy_pre = 0.0;
	
	m_flight_control_state = 0;
	m_st_control_time.time_control = MAX_TIME;
	m_st_control_time.time_separate_booster = MAX_TIME;
	m_st_control_time.time_launch_missile_wing = MAX_TIME;
	m_st_control_time.time_seeker_on = MAX_TIME;
	m_st_control_time.time_engine_start = MAX_TIME;
	m_st_control_time.time_engine_start_finish = MAX_TIME;
	m_st_control_time.time_altitude_control = MAX_TIME;
	m_st_control_time.time_missile_takeoff = MAX_TIME;
	m_st_control_time.time_altitude_control = MAX_TIME;
	m_st_control_time.time_launch_turn = MAX_TIME;
	m_st_control_time.time_launch_turn_ok = MAX_TIME;
	m_st_control_time.time_combat_status = MAX_TIME;
	m_st_control_time.time_combat_dive_ok = MAX_TIME;
	m_st_control_time.time_combat_dive_sidectrl = MAX_TIME;
	m_st_control_time.time_combat_dive_pullup = MAX_TIME;
	m_st_control_time.time_cooperative_attack = MAX_TIME;
	m_st_control_time.time_touch_ground = MAX_TIME;
	
	m_st_control_time.time_altitude_change_start = MAX_TIME;
	m_st_control_time.time_altitude_change_end = MAX_TIME;
	m_st_control_time.time_turn_in_start = MAX_TIME;
	m_st_control_time.time_turn_in_end = MAX_TIME;
	m_st_control_time.time_turn_out_start = MAX_TIME;
	m_st_control_time.time_turn_out_end = MAX_TIME;

	m_st_control_time.time_engine_shutdown= MAX_TIME;
	m_st_control_time.time_open_umbrella= MAX_TIME;
	m_st_control_time.time_fuze_unlock= MAX_TIME;
	m_st_control_time.time_touch_ground= MAX_TIME;
	
	m_st_control_time.time_turn_in_minimum = MAX_TIME;
	m_st_control_time.time_arrive_minimum = MAX_TIME;
	
	m_time_to_go = MAX_TIME;

	m_fuel_comsumped = 0.0;
	m_mass_calc = FULL_MASS;
	m_left_flight_time_calc = MAX_FLIGHT_TIME;
	m_left_flight_dist_calc = MAX_FLIGHT_DIST;
}

//���п������뺽�������ó�ʼ����
void CMathControlFlightBasic::Update_Task_Info()
{	
	//������ǰ�滮��·����
	int num_column = p_st_route_data_preflight->num_columns;//ÿ�����㣬������
	m_num_way_point = p_st_route_data_preflight->num_rows;	//������
	
	//����Ŀ�������
	m_st_target.longitude	= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 1];
	m_st_target.latitude	= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 2];
	m_st_target.height		= p_st_route_data_preflight->p_route_data[(m_num_way_point - 1) * num_column + 3];

	for (int count_num=0; count_num < m_num_way_point; count_num++)
	{
		//����װ���������� ����
		m_st_way_point[count_num].num	= count_num;
		m_st_way_point[count_num].longitude		= p_st_route_data_preflight->p_route_data[count_num * num_column + 1];
		m_st_way_point[count_num].latitude		= p_st_route_data_preflight->p_route_data[count_num * num_column + 2];
		m_st_way_point[count_num].height		= p_st_route_data_preflight->p_route_data[count_num * num_column + 3];
		m_st_way_point[count_num].route_mode	= (int)p_st_route_data_preflight->p_route_data[count_num * num_column + 4];
		m_st_way_point[count_num].formation_mode= (int)p_st_route_data_preflight->p_route_data[count_num * num_column + 5];
		m_st_way_point[count_num].dltTime		= p_st_route_data_preflight->p_route_data[count_num * num_column + 6];
		m_st_way_point[count_num].velocity		= p_st_route_data_preflight->p_route_data[count_num * num_column + 7];
		m_st_way_point[count_num].turn_angle	= p_st_route_data_preflight->p_route_data[count_num * num_column + 8];
		m_st_way_point[count_num].turn_radius	= p_st_route_data_preflight->p_route_data[count_num * num_column + 9];
		m_st_way_point[count_num].accept_radius	= p_st_route_data_preflight->p_route_data[count_num * num_column + 10];
		
		//�������ݣ���ʶ
		/*[����ʱ���ʶ��1��Ч��0��Ч��]*/		
		if(m_st_way_point[count_num].dltTime > 0)
			m_st_way_point[count_num].if_flightime_ctrl = 1;
		else
			m_st_way_point[count_num].if_flightime_ctrl = 0;	
		/*[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]*/     		
		if(m_st_way_point[count_num].height < 0)
			m_st_way_point[count_num].if_relativehigh_ctrl = 1;
		else
			m_st_way_point[count_num].if_relativehigh_ctrl = 0;	
		/*[ָ����б�ʶ��1��Ч��0��Ч��]*/
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
		/*[���ٿ��Ʊ�ʶ��1���٣�0���٣�]*/
		if(m_st_way_point[count_num].velocity < 0)
			m_st_way_point[count_num].if_groundspeed_ctrl = 1;
		else
			m_st_way_point[count_num].if_groundspeed_ctrl = 0;	
		/*[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]*/	
		if(((m_st_way_point[count_num].route_mode == 4) || (m_st_way_point[count_num].route_mode == 5))&&(m_st_way_point[count_num].turn_angle < 0.0))
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 1;//����Ϊ������
			m_st_way_point[count_num].attack_angle = - m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 0;	
			m_st_way_point[count_num].attack_angle = 0.0;
		}
		
		if(m_st_way_point[count_num].route_mode == 3)
		{	
			/*[Ԥ������ʶ��1��Ч��0��Ч��]*/
			m_st_way_point[count_num].if_prepare_hover = 1;
			m_st_way_point[count_num].hover_round = (int)m_st_way_point[count_num].turn_angle;
			/*[����ת�䷽���ʶ����ת1����ת0��]*/	
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
		
		//������ɡ�������յ�
		if(m_st_way_point[count_num].route_mode == 6)
		{
			m_st_way_point[count_num].recycle_ground_hight = m_target_height_ground;//m_st_way_point[count_num].height;	//����߶�Ϊ����߶�

			//ǰһ������Ϊ�������㣬���𹥵�
			if(m_st_way_point[count_num - 1].route_mode == 5)
			{
				//m_st_way_point[count_num].height = m_st_way_point[count_num - 1].height + 100.0;//回收点前一个航点之上100m，确保末制导后，俯冲拉起
				if(m_st_way_point[count_num].height < m_st_way_point[count_num - 1].height + 100.0)
				{
					m_st_way_point[count_num].height = m_st_way_point[count_num - 1].height + 100.0;//回收点前一个航点之上100m，确保末制导后，俯冲拉起
				}
					
				//相对地面高度过低
				if(m_st_way_point[count_num].height < m_st_way_point[count_num].recycle_ground_hight + 250.0)
				{
					m_st_way_point[count_num].height = m_st_way_point[count_num].recycle_ground_hight + 250.0;//���300m	
				}
			}
			else
			{
				if(m_st_way_point[count_num].height < m_st_way_point[count_num].recycle_ground_hight + 250.0)
				{
					m_st_way_point[count_num].height = m_st_way_point[count_num].recycle_ground_hight + 250.0;//���300m
				}
				else
				{
					//��������		
				}
			}
		}
		else
		{
			//��ǰ���㲻Ϊ���յ㣬����߶� ���Ƶ��� ����߶�
			m_st_way_point[count_num].recycle_ground_hight = p_st_initial_data->height_launch;
			//m_st_way_point[count_num].height
		}
	}
	//如果最后一个目标点是回收点，前一个目标点为佯攻点，目标点设置为佯攻点
	if((m_st_way_point[m_num_way_point - 1].route_mode == 6) && (m_st_way_point[m_num_way_point - 2].route_mode == 5))
	{
		m_st_target.height = m_st_way_point[m_num_way_point - 2].height;
		m_st_target.longitude	= m_st_way_point[m_num_way_point - 2].longitude;
		m_st_target.latitude	= m_st_way_point[m_num_way_point - 2].latitude;
	}

	//计算转弯角及初段航程
	double distance_BC = 0.0;
	double distance_delta = 0.0;
	double alpha_BC = 0.0;
	CFlightGlobalFun::Tomas(p_st_initial_data->longitude_launch, p_st_initial_data->latitude_launch,
		m_st_way_point[0].longitude, m_st_way_point[0].latitude,
		&m_total_distance, &m_alpha_AB);

	//�����ܺ���
	for(int i=0; i<(m_num_way_point - 1); i++)
	{
		CFlightGlobalFun::Tomas(m_st_way_point[i].longitude, m_st_way_point[i].latitude,
			m_st_way_point[i + 1].longitude, m_st_way_point[i + 1].latitude,
			&distance_BC, &alpha_BC);		
		m_st_way_point[i].turn_angle = alpha_BC - m_alpha_AB;//ת��Ƕ����´���
		m_st_way_point[i].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[i].turn_angle, 180.0);
		m_alpha_AB = alpha_BC;//����ǰһ�κ�����
		m_total_distance += distance_BC;
		distance_delta = 2 * m_st_way_point[i].turn_radius 
						* (PI * fabs(m_st_way_point[i].turn_angle) / 360.0 
						- tan((fabs(m_st_way_point[i].turn_angle) / 2.0) / RTOA));
		m_total_distance += distance_delta;
	}
}

void CMathControlFlightBasic::Change_Task_Info_Online()
{
	//�ں������������󣬼������������͵ĺ������У�����ԭ�������ܺ�����£������ܺ���
	m_num_way_point = m_num_way_point_target 
		+ p_st_flight_basic_input->st_datalink_datasig.num_waypoint_updated;//���º��ܺ����� = ��ǰ���� �� ����������
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
		m_st_way_point[count_num].velocity	= 
			p_st_flight_basic_input->st_datalink_datasig.velocity[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_angle	= 
			p_st_flight_basic_input->st_datalink_datasig.turn_angle[count_num - m_num_way_point_target];
		m_st_way_point[count_num].turn_radius	= 
			p_st_flight_basic_input->st_datalink_datasig.turn_radius[count_num - m_num_way_point_target];
		m_st_way_point[count_num].accept_radius	= 
			p_st_flight_basic_input->st_datalink_datasig.accept_radius[count_num - m_num_way_point_target];		
		
		//����������:��ʶ
		//[����ʱ���ʶ��1��Ч��0��Ч��]
		if(m_st_way_point[count_num].dltTime > 0)
			m_st_way_point[count_num].if_flightime_ctrl = 1;
		else
			m_st_way_point[count_num].if_flightime_ctrl = 0;	
		//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
		if(m_st_way_point[count_num].height < 0)
			m_st_way_point[count_num].if_relativehigh_ctrl = 1;
		else
			m_st_way_point[count_num].if_relativehigh_ctrl = 0;	
		//[ָ����б�ʶ��1��Ч��0��Ч��]
		if(m_st_way_point[count_num].route_mode == 2)
			m_st_way_point[count_num].if_heading_hold = 1;
		else
			m_st_way_point[count_num].if_heading_hold = 0;	
		//[���ٿ��Ʊ�ʶ��1���٣�0���٣�]
		if(m_st_way_point[count_num].velocity < 0)
			m_st_way_point[count_num].if_groundspeed_ctrl = 1;
		else
			m_st_way_point[count_num].if_groundspeed_ctrl = 0;	
		//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]
		if(((m_st_way_point[count_num].route_mode == 4) || (m_st_way_point[count_num].route_mode == 5))&&(m_st_way_point[count_num].turn_angle < 0.0))
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 1;//����Ϊ������
			m_st_way_point[count_num].attack_angle = -m_st_way_point[count_num].turn_angle;
		}
		else
		{
			m_st_way_point[count_num].if_attackangle_ctrl = 0;	
			m_st_way_point[count_num].attack_angle = 0.0;
		}
		
		if(m_st_way_point[count_num].route_mode == 3)
		{	
			//[Ԥ������ʶ��1��Ч��0��Ч��]
			m_st_way_point[count_num].if_prepare_hover = 1;
			m_st_way_point[count_num].hover_round = (int)m_st_way_point[count_num].turn_angle;
			//[����ת�䷽���ʶ����ת1����ת0��]	
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

		//������ɡ�������յ�
		if(m_st_way_point[count_num].route_mode == 6)
		{
			m_st_way_point[count_num].recycle_ground_hight = m_target_height_ground;//m_st_way_point[count_num].height;	//����߶�Ϊ����߶�

			//ǰһ������Ϊ�������㣬���𹥵�
			if(m_st_way_point[count_num - 1].route_mode == 5)
			{
				m_st_way_point[count_num].height = m_st_way_point[count_num - 1].height + 100.0;//���յ�ǰһ������֮��100m��ȷ��ĩ�Ƶ��󣬸�������
				//��Ե���߶ȹ���
				if(m_st_way_point[count_num].height < m_st_way_point[count_num].recycle_ground_hight + 250.0)
				{
					m_st_way_point[count_num].height = m_st_way_point[count_num].recycle_ground_hight + 250.0;//���300m	
				}
			}
			else
			{
				if(m_st_way_point[count_num].height < m_st_way_point[count_num].recycle_ground_hight + 250.0)
				{
					m_st_way_point[count_num].height = m_st_way_point[count_num].recycle_ground_hight + 250.0;//���300m
				}
				else
				{
					//��������		
				}
			}
		}
		else
		{
			//��ǰ���㲻Ϊ���յ㣬����߶� ���Ƶ��� ����߶�
			m_st_way_point[count_num].recycle_ground_hight = p_st_initial_data->height_launch;
			//m_st_way_point[count_num].height
		}
	}
	 
	//最后航迹点为目标点，目标高度使用初始装订值，不更新
	//m_st_way_point[m_num_way_point - 1].height = m_st_target.height;
	//更新目标点经度、维度
	m_st_target.height = m_st_way_point[m_num_way_point - 1].height;
	m_st_target.longitude	= m_st_way_point[m_num_way_point - 1].longitude;
	m_st_target.latitude	= m_st_way_point[m_num_way_point - 1].latitude;
	//如果最后一个目标点是回收点，前一个目标点为佯攻点，目标点设置为佯攻点
	if((m_st_way_point[m_num_way_point - 1].route_mode == 6) && (m_st_way_point[m_num_way_point - 2].route_mode == 5))
	{
		m_st_target.height = m_st_way_point[m_num_way_point - 2].height;
		m_st_target.longitude	= m_st_way_point[m_num_way_point - 2].longitude;
		m_st_target.latitude	= m_st_way_point[m_num_way_point - 2].latitude;
	}

	
	//初始航线: 发射点到第一个航迹点，距离、方位
	CFlightGlobalFun::Tomas(p_st_initial_data->longitude_launch, p_st_initial_data->latitude_launch,
		m_st_way_point[0].longitude, m_st_way_point[0].latitude,
		&m_total_distance, &m_alpha_AB);
	//����ת��Ǽ��ܺ���
	double distance_BC = 0.0;
	double distance_delta = 0.0;
	double alpha_BC = 0.0;
	for(int i=0; i<(m_num_way_point - 1); i++)
	{
		CFlightGlobalFun::Tomas(m_st_way_point[i].longitude, m_st_way_point[i].latitude,
			m_st_way_point[i + 1].longitude, m_st_way_point[i + 1].latitude,
			&distance_BC, &alpha_BC);		
		m_st_way_point[i].turn_angle = alpha_BC - m_alpha_AB;//����ת��Ƕ�
		m_st_way_point[i].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[i].turn_angle, 180.0);
		m_alpha_AB = alpha_BC;
		m_total_distance += distance_BC;
		distance_delta = 2 * m_st_way_point[i].turn_radius 
			* (PI * fabs(m_st_way_point[i].turn_angle) / 360.0 
			- tan((fabs(m_st_way_point[i].turn_angle) / 2.0) / RTOA));
		m_total_distance += distance_delta;
	}

	//���µ�ǰ�㵽Ŀ���ת��Ƕ�
	////////Ŀ��㵽��һ����롢��λ
	CFlightGlobalFun::Tomas(m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		m_st_way_point[m_num_way_point_target + 1].longitude, m_st_way_point[m_num_way_point_target + 1].latitude,
		&distance_BC, &alpha_BC);
	////////��ǰ�㵽Ŀ�����롢��λ
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_st_way_point[m_num_way_point_target].longitude, m_st_way_point[m_num_way_point_target].latitude,
		&distance_BC, &m_alpha_AB);
	////////������ƫ���ΪĿ���ת���
	m_st_way_point[m_num_way_point_target].turn_angle = alpha_BC - m_alpha_AB;
	m_st_way_point[m_num_way_point_target].turn_angle = CFlightGlobalFun::Adjust(m_st_way_point[m_num_way_point_target].turn_angle, 180.0);

	//���µ�ǰ����״̬
	////////��ǰ�㡢Ŀ��㼰��һ�㣬��������ÿ�����ٷ���������
	m_longitude_A = m_longitude;
	m_latitude_A = m_latitude;
	m_longitude_B = m_st_way_point[m_num_way_point_target].longitude;
	m_latitude_B = m_st_way_point[m_num_way_point_target].latitude;
	m_longitude_C = m_st_way_point[m_num_way_point_target + 1].longitude;
	m_latitude_C = m_st_way_point[m_num_way_point_target + 1].latitude;
	////////ת����Ϣ����
	m_route_mode = m_st_way_point[m_num_way_point_target].route_mode;
	m_formation_mode = m_st_way_point[m_num_way_point_target].formation_mode;
	m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;
	m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;
	m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;
	m_accept_radius = m_st_way_point[m_num_way_point_target].accept_radius;
	
	//�ؽ����ߣ��������£�������롢��λ��
	double distance_AB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
		m_longitude_B, m_latitude_B, 
		&distance_AB, &m_A);
	m_A = - m_A;
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);

	////////������С���ܵ���ʱ�䣬ת���ʶ����Ϊ��Ч
	double angle_PB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_longitude_B, m_latitude_B,
		&m_distance_BP, &angle_PB);
	double temp_v = m_v_average_10s + 10.0;// + 20.0
	if(temp_v >=  VEL_COMMAND_MAX_LIMIT)
	{
		temp_v = VEL_COMMAND_MAX_LIMIT; 
	}
	else if(temp_v <= VEL_COMMAND_MIN_LIMIT)
	{
		temp_v = VEL_COMMAND_MIN_LIMIT;
	}
	double distance_to_go = fabs(m_distance_BP - m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0));
	double min_time = distance_to_go / temp_v;		//���ٶȹ���
	m_st_control_time.time_arrive_minimum = flight_time + min_time;
	m_st_control_flag.flag_waypoint_turn = false;
}
void CMathControlFlightBasic::Initial()
{
	m_missile_ID = p_st_flight_basic_input->missile_ID;
	//����ر������¶�
	m_ground_temperature = p_st_initial_data->initial_parameter1;
	m_missile_flight_mode = (int)p_st_initial_data->initial_parameter2;
	m_ktheta_lauch_enc = p_st_initial_data->lauch_pitch;//����ֵ12deg
	m_ktheta_climb_enc = p_st_initial_data->climb_ktheta_enc;//�����Ƕ�,����ֵ6deg
	m_ktheta_hight_enc = p_st_initial_data->cruise_ktheta_enc;//ƽ�ɹ��ǣ�����ֵ1deg
	
	//��ǰ�����������װ��
	Update_Task_Info();

	//����߶�
	m_target_height_ground = m_hz = p_st_initial_data->height_launch;
	//��ʼλ�á�ǰ��A\B\C��λ��
	m_longitude = p_st_initial_data->longitude_launch;//������뺽���0Ϊ��ͬ
	m_latitude = p_st_initial_data->latitude_launch;
	m_hz = p_st_initial_data->height_launch;
	m_longitude_A = p_st_initial_data->longitude_launch;
	m_latitude_A = p_st_initial_data->latitude_launch;
	m_longitude_B = m_st_way_point[1].longitude;//��һ����
	m_latitude_B = m_st_way_point[1].latitude;
	m_longitude_C = m_st_way_point[2].longitude;//�ڶ�����
	m_latitude_C = m_st_way_point[2].latitude;
	
	//��ɼ����ʼ���ߣ���λ������
	double distance_AB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
		m_longitude_B, m_latitude_B, 
		&distance_AB, &m_A);
	m_A = - m_A;//����ƫ����ת��Ϊ����ƫ����
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);
	
	//������Ϣ
	m_route_mode = m_st_way_point[1].route_mode;
	m_target_velocity = m_st_way_point[1].velocity;
	m_turn_angle = m_st_way_point[1].turn_angle;//��ƫ��Ϊ����-180~180deg
	m_turn_radius = m_st_way_point[1].turn_radius;
	m_accept_radius = m_st_way_point[1].accept_radius;
	m_target_height = m_st_way_point[1].height;	//��ʼ��Ϊ��һ����߶ȣ������Ϊ���㺽��
	//��һ����������ʱ��
	dlt_time_tg = (m_target_height - m_hz)/5.0;//����5m/s���٣����Ʊ�������ʱ��
	if(dlt_time_tg < 20.0)
	{
		dlt_time_tg = 20.0;//�����߶ȣ�����100m,�����κ��̲�����1km
	}
	
	//���������±�ʶ����ʼ��
	//count_update = p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count;
	count_update = p_st_flight_basic_input->st_datalink_datasig.update_count;

	//���淢��װ�����ݣ������
	p_st_flight_basic_output->h_ini = p_st_initial_data->height_launch;
	p_st_flight_basic_output->ground_temperature = m_ground_temperature;
	p_st_flight_basic_output->ktheta_lauch_enc = m_ktheta_lauch_enc;
	p_st_flight_basic_output->ktheta_climb_enc = m_ktheta_climb_enc;
	p_st_flight_basic_output->ktheta_hight_enc = m_ktheta_hight_enc;

	//�˲�����
	int i = 0;
	//�����˲�����
	for(i=0;i<6;i++)
	{
		m_nav_data_filter[i].T = 0.015;//��ת��Ƶ��Լ 10.6Hz
		m_nav_data_filter[i].Ts = 0.005;
	}
	//�Ƶ��˲�����
	for(i=0;i<2;i++)
	{
		m_guide_data_filter[i].T = 0.10;//��ת��Ƶ��Լ 1.60Hz��1/5���Ƶ�Ƶ��0.32Hz����2.00rad/s����Ծ��Ӧʱ��1.5s��
		//m_nav_data_filter[i].T = 0.15;//��ת��Ƶ��Լ 1.00Hz��1/5���Ƶ�Ƶ��0.20Hz����1.26rad/s����Ծ��Ӧʱ��2.4s��
		m_guide_data_filter[i].Ts = 0.005;
	}
	//���ٹ��˲�����
	for(i=0;i<2;i++)
	{
		m_baro_data_filter[i].T = 0.10;//��ת��Ƶ��Լ 1.60Hz
		m_baro_data_filter[i].Ts = 0.020;
	}
}

void CMathControlFlightBasic::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	//Monitor_Data();
}

void CMathControlFlightBasic::Get_Data()
{
	//�����
	m_missile_ID = p_st_flight_basic_input->missile_ID;

	//���ߵ�߶ȱ�����
	//m_radioalt_hight = p_st_flight_basic_input->st_radioalt_data.radioalt_hight;
	//m_radioalt_status = p_st_flight_basic_input->st_radioalt_data.radioalt_status;
	//��Ұֵ���˲��㷨������
	m_radioalt_hight = m_hz;
	m_radioalt_status = 0xFF;

	//���ٹ�
	//m_static_pressure = p_st_flight_basic_input->st_baro_data.static_pressure;
	//m_total_pressure = p_st_flight_basic_input->st_baro_data.total_pressure;
	//������Ұֵ���˲�����
	//��������20ms����Ұֵ������1��20ms�ٶ�����1m/s����ѹ���Ӳ�����150Pa��2������10m/s���߶ȱ仯0.2m����ѹ�仯2.2Pa
	//��ͨ�˲���ʱ�����ӣ�������������ж������������ͨ�˲�ʱ�䳣��Ϊ50~100ms����Ҫ����̬�����ʹ��
	if(!((time_tick - 1) % 20))
	{
		//��ȡ����������
		m_static_pressure_raw = p_st_flight_basic_input->st_baro_data.static_pressure;
		m_total_pressure_raw = p_st_flight_basic_input->st_baro_data.total_pressure;
		//��Ұֵ
		m_static_pressure = CFlightGlobalFun::Reject_outlier(m_static_pressure_raw, &m_static_pressure, &count_static_pressure_tyz, 2000.0);//��ѹ���Χ
		m_total_pressure = CFlightGlobalFun::Reject_outlier(m_total_pressure_raw, &m_total_pressure, &count_total_pressure_tyz, 2000.0);//��ѹ���Χ
		//��ͨ�˲���8Hzת��Ƶ�ʣ�����ʱ��20ms
		//��ѹ
		m_baro_data_filter[0].inputdata[1] = m_baro_data_filter[0].inputdata[0];
		m_baro_data_filter[0].inputdata[0] = m_static_pressure;
		CFlightGlobalFun::Tustin_FirstIO(0,1,m_baro_data_filter[0].T,1,
					m_baro_data_filter[0].inputdata,m_baro_data_filter[0].outputdata,m_baro_data_filter[0].Ts);
		m_baro_data_filter[0].outputdata[1] = m_baro_data_filter[0].outputdata[0];
		m_static_pressure_flt = m_baro_data_filter[0].outputdata[0];
		//��ѹ
		m_baro_data_filter[1].inputdata[1] = m_baro_data_filter[1].inputdata[0];
		m_baro_data_filter[1].inputdata[0] = m_total_pressure;
		CFlightGlobalFun::Tustin_FirstIO(0,1,m_baro_data_filter[1].T,1,
					m_baro_data_filter[1].inputdata,m_baro_data_filter[1].outputdata,m_baro_data_filter[1].Ts);
		m_baro_data_filter[1].outputdata[1] = m_baro_data_filter[1].outputdata[0];
		m_total_pressure_flt = m_baro_data_filter[1].outputdata[0];
		//Ϊ�˲���???...
		m_static_pressure_flt = m_static_pressure;
		m_total_pressure_flt = m_total_pressure;
	

		//������ѹ�߶ȡ�����
		Calc_BaroHigh();
		Calc_BaroSpd();

		//���ٹ�������Ч���ж�
		if((fabs(m_v - m_Vbaro) > 12.0) 
			|| (m_static_pressure > 101325*1.5) 
			|| (m_static_pressure < 54048*0.8) 
			|| (fabs(m_static_pressure - m_total_pressure) > 3000.0*1.5))
		{
			m_baroalt_status = 0xFF;//�쳣
		}
		else
		{
			m_baroalt_status = 0xAA;//�쳣
		}
	}

	//����������
	m_state_rpm = p_st_flight_basic_input->st_engine_data.rpm_engine;
	//��������������
#ifdef	__VEL__CONTROL__MODE__KC__	
	m_cmd_Kc = p_st_flight_basic_input->engine_cmd_Kc;	//���ſ���ģʽ
#else
	m_cmd_rpm = p_st_flight_basic_input->engine_cmd_rpm;	//ת�ٿ���ģʽ
#endif

	//��ϵ�������
	//m_au = p_st_flight_basic_input->st_ins_data.au;
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
	m_wx = p_st_flight_basic_input->st_ins_data.wx;
	m_wy = p_st_flight_basic_input->st_ins_data.wy;
	m_wz = p_st_flight_basic_input->st_ins_data.wz;

	//���ٶȡ����ٶ���Ұֵ	
	CFlightGlobalFun::Reject_outlier(m_ax, &m_axtyz, &count_ax_tyz, 10.0);//���ٶȱ仯���Χ�����Ż�
	CFlightGlobalFun::Reject_outlier(m_ay, &m_aytyz, &count_ay_tyz, 10.0);
	CFlightGlobalFun::Reject_outlier(m_az, &m_aztyz, &count_az_tyz, 10.0);
	CFlightGlobalFun::Reject_outlier(m_wx, &m_wxtyz, &count_wx_tyz, 10.0);//���ٶȱ仯���Χ�����Ż�
	CFlightGlobalFun::Reject_outlier(m_wy, &m_wytyz, &count_wy_tyz, 10.0);
	CFlightGlobalFun::Reject_outlier(m_wz, &m_wztyz, &count_wz_tyz, 10.0);
	//���ٶȡ����ٶ��˲�
	double temp_data[6];
	temp_data[0] = m_axtyz;
	temp_data[1] = m_aytyz;
	temp_data[2] = m_aztyz;
	temp_data[3] = m_wxtyz;
	temp_data[4] = m_wytyz;
	temp_data[5] = m_wztyz;
	int i = 0;
	for(i = 0;i < 6;i++)
	{
		m_nav_data_filter[i].inputdata[1] = m_nav_data_filter[i].inputdata[0];
		m_nav_data_filter[i].inputdata[0] = temp_data[i];
		CFlightGlobalFun::Tustin_FirstIO(0,1,m_nav_data_filter[i].T,1,
					m_nav_data_filter[i].inputdata,m_nav_data_filter[i].outputdata,m_nav_data_filter[i].Ts);
		m_nav_data_filter[i].outputdata[1] = m_nav_data_filter[i].outputdata[0];
	}
	m_axflt = m_nav_data_filter[0].outputdata[0];
	m_ayflt = m_nav_data_filter[1].outputdata[0];
	m_azflt = m_nav_data_filter[2].outputdata[0];
	m_wxflt = m_nav_data_filter[3].outputdata[0];
	m_wyflt = m_nav_data_filter[4].outputdata[0];
	m_wzflt = m_nav_data_filter[5].outputdata[0];
	
	//Ϊ�˲��ԣ���ʱ��������Ұֵ���˲��㷨???...
	m_wxflt = m_wx;
	m_wyflt = m_wy;
	m_wzflt = m_wz;
	m_axflt = m_ax;
	m_ayflt = m_ay;
	m_azflt = m_az;

	//����������
	//m_engine_state_rpm = p_st_flight_basic_input->engine_rpm;
	//m_engine_state = p_st_flight_basic_input->engine_start_result;
	m_engine_state_rpm = p_st_flight_basic_input->st_engine_data.rpm_engine;
	m_engine_state = p_st_flight_basic_input->st_engine_data.ECU_work_status;

	//����ͷ���ݣ���������20ms
	//�յ���������ָ�����ͷ����ĩ�Ƶ����̣�����������״̬
	if(p_st_flight_basic_input->st_seeker_data.flag_combat_status == 1)
	{
		m_seeker_state_track = 0x02;
	}
	else
	{
		m_seeker_state_track = 0x01;
	}
	//����ͷ�������٣�����������Ŀ��
	if(p_st_flight_basic_input->st_seeker_data.flag_seize_stable == 1)
	{
		m_seeker_state_track = 0x03;//��ʱ��ʧĿ�꣬�������״̬��Ϊ0x04
	}
	m_seeker_dqf = p_st_flight_basic_input->st_seeker_data.pitch_LOS_rate;
	m_seeker_dqh = p_st_flight_basic_input->st_seeker_data.yaw_LOS_rate;
	m_seeker_phif = p_st_flight_basic_input->st_seeker_data.pitch_gimbal_angle;
	m_seeker_phih = p_st_flight_basic_input->st_seeker_data.yaw_gimbal_angle;
	m_seeker_qf = p_st_flight_basic_input->st_seeker_data.pitch_LOS_angle;
	m_seeker_qh = p_st_flight_basic_input->st_seeker_data.yaw_LOS_angle;
	//����ƫ��Ϊ�㣬������Ч
	m_seeker_pixelf = 0;
	m_seeker_pixelh = 0;
	m_seeker_distance_target = 100000;//��絼��ͷ��̽�ⵯĿ������Ч��100km
	//double m_seeker_state_track;//����ͷ����״̬��0x01������λ,0x02��������ʧ������0x03������0x04����
	//int m_seeker_pixelf;//��������ƫ�������
	//int m_seeker_pixelh;//��������ƫ��¸�����
	//double m_seeker_distance_target;//����ͷ�����Ŀ���룬�״�򼤹��ֱ�������
	//									//�ɽ���������ݸ߶Ȳ���߽ǹ��㣬Ҳ�ɸ���Ŀ�����ش�С�ͽ������Ŀ����룻
}
void CMathControlFlightBasic::Calc_Data()
{
	//���������߸��º����㡢�������º����㣬ת������в�����
	//if ((count_update != p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count)
	if ((count_update != p_st_flight_basic_input->st_datalink_datasig.update_count)
		&&(!m_st_control_flag.flag_waypoint_turn))
	{
		Change_Task_Info_Online();
		//count_update = p_st_flight_basic_input->st_datalink_data.st_mission_update_data[m_missile_ID].update_count;
		count_update = p_st_flight_basic_input->st_datalink_datasig.update_count;
	}		
	//���м���
	Calc_Flight_Data();
	//��������
	Calc_Mass_Data();
	//���߽��ٶȵȼ���
	Calc_LOS_Rate();	
	//ָ��ʱ��
	Calc_Command();	
	//�������������������󣬸߶ȿ�����ɺ󣬿�ʼ����ת��
	//������Ż��������ɺ�ͬ����ʼ����ת��
	Control_Turn();
	//�߶Ȼ���
	Control_Altitude_Change();
	//�Ƶ�����
	Calc_Dualplane_Guidance();
	//���б�ʶ��״̬��װ
	PackControlFlag(&m_st_control_flag);
}
void CMathControlFlightBasic::Send_Data()
{
	//ʵʱ����
	p_st_flight_basic_output->mass_calc = m_mass_calc;
	p_st_flight_basic_output->hz = m_hz;
	p_st_flight_basic_output->nby = m_ny;
	p_st_flight_basic_output->nbz = m_nz;
	p_st_flight_basic_output->v = m_v_air;
	p_st_flight_basic_output->vs = m_vs;
	p_st_flight_basic_output->vnx = m_vnx;
	p_st_flight_basic_output->vnz = m_vnz;
	p_st_flight_basic_output->mach = m_mach;
	p_st_flight_basic_output->sonic_speed = m_sonic_speed;
	p_st_flight_basic_output->sz = m_sz;
	p_st_flight_basic_output->dynamic_pressure = m_dynamic_pressure;//����
	p_st_flight_basic_output->zeta = m_zeta;
	p_st_flight_basic_output->gama = m_gama;
	p_st_flight_basic_output->wx = m_wxflt;//p_st_flight_basic_input->st_ins_data.wx;
	p_st_flight_basic_output->wy = m_wyflt;//p_st_flight_basic_input->st_ins_data.wy;
	p_st_flight_basic_output->wz = m_wzflt;//p_st_flight_basic_input->st_ins_data.wz;
	p_st_flight_basic_output->g = m_g;
	p_st_flight_basic_output->dqf = m_dqf;
	p_st_flight_basic_output->dqh = m_dqh;	
	p_st_flight_basic_output->target_num__choosen = m_target_num__choosen;
	p_st_flight_basic_output->phif = m_phif;
	p_st_flight_basic_output->phih = m_phih;
	p_st_flight_basic_output->qf = m_Qf;
	p_st_flight_basic_output->qh = m_Qh;
	p_st_flight_basic_output->time_to_go = m_time_to_go;
	p_st_flight_basic_output->gama_command = m_gama_command;
	p_st_flight_basic_output->ny_command = m_ny_command;
	p_st_flight_basic_output->nz_command = m_nz_command;
	
	p_st_flight_basic_output->angle_zw = m_turn_angle;
	p_st_flight_basic_output->radius_zw = m_turn_radius;
	p_st_flight_basic_output->target_velocity = m_target_velocity;
	p_st_flight_basic_output->target_height = m_target_height;	
	p_st_flight_basic_output->target_long = m_longitude_B;//����
	p_st_flight_basic_output->target_lat = m_latitude_B;//����
	p_st_flight_basic_output->target_distance = m_distance_BP;
	p_st_flight_basic_output->longitude = m_longitude;
	p_st_flight_basic_output->latitude = m_latitude;	

	p_st_flight_basic_output->flight_control_state = m_flight_control_state;
	p_st_flight_basic_output->rpmState = (unsigned int)m_state_rpm;
	p_st_flight_basic_output->count_altitude_change = count_altitude_change;//������	
	p_st_flight_basic_output->num_way_point_target = m_num_way_point_target;
	p_st_flight_basic_output->target_time = MAX_TIME;	//Ԥ��������
	p_st_flight_basic_output->gama_turn_nominal = m_gama_turn_nominal;
	
	p_st_flight_basic_output->Rmt_n[0] = m_Rmt_n[0];
	p_st_flight_basic_output->Rmt_n[1] = m_Rmt_n[1];
	p_st_flight_basic_output->Rmt_n[2] = m_Rmt_n[2];
	p_st_flight_basic_output->distance_target = m_distance_target;
	p_st_flight_basic_output->distance_target_t_combat = m_distance_target_t_combat;
	p_st_flight_basic_output->velocity_average_10s = m_v_average_10s;
	p_st_flight_basic_output->ECU_work_cmd = m_ECU_work_cmd;

	//p_st_flight_basic_output->token_long = 0x00;//����δʹ��
	//p_st_flight_basic_output->token_lat = 0x00;;//����δʹ��
	//p_st_flight_basic_output->dlt_psic = 0.0;;//������ƫ�δʹ��
	p_st_flight_basic_output->sz_circle = m_sz_radius;//Բ�켣��ƫ�࣬δȥ��ת��뾶
	p_st_flight_basic_output->azimuth = m_A;//���η�λ��
	p_st_flight_basic_output->psicn = m_psicn;//����ƫ��
	p_st_flight_basic_output->theta = m_theta;//�������
	p_st_flight_basic_output->alpha_vg = 0.0;//���ٹ��ǣ�δʹ��
	p_st_flight_basic_output->beita_vg = 0.0;//���ٲ໬�ǣ�δʹ��

	//����������ر�ʶ
	p_st_flight_basic_output->flag_launch_turn = m_st_control_flag.flag_launch_turn;
	p_st_flight_basic_output->flag_waypoint_turn = m_st_control_flag.flag_waypoint_turn;
	p_st_flight_basic_output->flag_altitude_change = m_st_control_flag.flag_alltitude_change;	
	p_st_flight_basic_output->flag_altitude_climb = m_st_control_flag.flag_alltitude_climb;
	p_st_flight_basic_output->flag_altitude_decline = m_st_control_flag.flag_alltitude_decline;

	//��������ر�ʶ
	p_st_flight_basic_output->flag_velocity_control = m_st_control_flag.flag_velocity_control;	//���ٿ��Ʊ�ʶ
	p_st_flight_basic_output->flag_flightime_ctrl = m_st_control_flag.flag_flightime_ctrl;	//����ʱ����Ʊ�ʶ
	
	p_st_flight_basic_output->flag_heading_hold = m_st_control_flag.flag_heading_hold;	//[ָ����б�ʶ��1��Ч��0��Ч��]
	p_st_flight_basic_output->flag_turndir_set = m_st_control_flag.flag_turndir_set;	//[����ת�䣺��ת����ת��ʶ��]
	p_st_flight_basic_output->flag_prepare_hover = m_st_control_flag.flag_prepare_hover;//[Ԥ������ʶ��1��Ч��0��Ч��]
	
	p_st_flight_basic_output->flag_relativehigh_ctrl = m_st_control_flag.flag_relativehigh_ctrl;//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	p_st_flight_basic_output->flag_attackangle_ctrl = m_st_control_flag.flag_attackangle_ctrl;	//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]	

	//�������������
	p_st_flight_basic_output->st_command.flag_separate_booster = m_st_control_flag.flag_separate_booster;
	p_st_flight_basic_output->st_command.flag_launch_missile_wing = m_st_control_flag.flag_launch_missile_wing;//δ�õ�
	p_st_flight_basic_output->st_command.flag_engine_start = m_st_control_flag.flag_engine_start;
	p_st_flight_basic_output->st_command.flag_seeker_on = m_st_control_flag.flag_seeker_on;//������ͷ
	p_st_flight_basic_output->st_command.flag_lock_on_permit = m_st_control_flag.flag_lock_on_permit;//������ͷ
	p_st_flight_basic_output->st_command.flag_target_lock = m_st_control_flag.flag_target_lock;//������ͷ����ʱ����
	p_st_flight_basic_output->st_command.flag_combat_status = m_st_control_flag.flag_combat_status;//�����ڲ���
	//��ǰ�׶�ʹ��
	p_st_flight_basic_output->st_command.flag_missile_takeoff = m_st_control_flag.flag_missile_takeoff;
	p_st_flight_basic_output->st_command.flag_altitude_control = m_st_control_flag.flag_altitude_control;
	p_st_flight_basic_output->st_command.flag_engine_shutdown = m_st_control_flag.flag_engine_shutdown;
	p_st_flight_basic_output->st_command.flag_combat_dive_pullup = m_st_control_flag.flag_combat_dive_pullup;
	p_st_flight_basic_output->st_command.flag_combat_dive_sidectrl = m_st_control_flag.flag_combat_dive_sidectrl;
	
	p_st_flight_basic_output->st_command.flag_open_umbrella = m_st_control_flag.flag_open_umbrella;
	p_st_flight_basic_output->st_command.flag_fuze_unlock = m_st_control_flag.flag_fuze_unlock;
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
	////0.���T0
	m_st_control_flag.flag_t0 = true;
	////1.����
	////1.1��Ҫ����������λ�Ƴ���1.1�����峤�� �� ���ٶȴ���5m/s���ӳ�50ms����
	if((m_sx >= 3.036)
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
		m_st_control_time.time_control = flight_time + 0.05;
	}
	////1.2���ݱ������������ٶȴ���5m/s���ҷɿ�ʱ�����0.5s�����ӳ�����������
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
		if((flight_time >= 0.50)
			&&(!m_st_control_flag.flag_control_set))
		{
			m_st_control_flag.flag_control_set = true;
			m_st_control_time.time_control = flight_time;
		}
	}

	////2.�������������롢����չ������������𼰵���ͷ����ʱ��
	////2.1��Ҫ������������ٶ�С��2m/s2 �� �ٶȴ��� 40m/s
	if((flight_time >= 1.0)
		&&((m_axflt - m_g * sin(m_zeta/RTOA)) <= 2.0)//�����������½�
		&&(m_v >= 40.0))//�ٶȽϴ�
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
		m_st_control_flag.flag_seeker_on_set = true;
		
		m_st_control_time.time_separate_booster = flight_time + 0.2;//�����½�0.2s��ִ������������(��ը��˨���и�������)
		m_st_control_time.time_launch_missile_wing = 
			m_st_control_time.time_separate_booster + 0.2;//����չ����δʹ��
		m_st_control_time.time_engine_start = 
			m_st_control_time.time_separate_booster + 1.0;//Ԥ��0.5s��չ��λ�����������룬֮�󷢳���������������������δʹ��
		m_st_control_time.time_seeker_on = 
			m_st_control_time.time_separate_booster + 2.0;//Ԥ��2.0s������������ɣ�����ͷ����������չ
	}
	////2.2�����������ٶȴ���40m/s��2.0s
	if(m_v >= 40.0)
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
			m_st_control_time.time_separate_booster = flight_time;//��Ϊ�������Ѿ����䣬���ӳ�
			m_st_control_time.time_launch_missile_wing = 
				m_st_control_time.time_separate_booster + 0.2;
			m_st_control_time.time_engine_start = 
				m_st_control_time.time_separate_booster + 1.0;
			m_st_control_time.time_seeker_on = 
				m_st_control_time.time_separate_booster + 2.0;
		}
	}

	///3.������������ɣ���ת�ٶȿ��ƣ�30ms���������������Ч�����ӳ�Լ0.1s
	if(!((time_tick - 1) % 30))
	{
		if (0xAA == m_engine_state)
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

		//һ��Ϊ��������ʼ����������ţ����������ſ���
		m_ECU_work_cmd = 0x33;	
	}
	
	///4.����ȶ���ʶ���ٶ����½��Ҹ����ǹ��ɵ������Ƕ�(�������ȶ����٣����ٶ��½���Ѳ���ٶ�)
	///4.1��Ҫ����
	// ʱ����ƣ�
	// 12kNs������������ٶ�61.8m/s��2~2.5m/s^2������Ѳ���ٶ�52m/s��ʱ����Լ5s������������ʱ��2s��ʱ��Լ7s
	// 13kNs������������ٶ�67.0m/s��2~2.5m/s^2������Ѳ���ٶ�52m/s��ʱ����Լ8s������������ʱ��2s��ʱ��Լ10s ok
	// 14kNs������������ٶ�71.0m/s��2~2.5m/s^2������Ѳ���ٶ�52m/s��ʱ����Լ10s������������ʱ��2s��ʱ��Լ12s
	//if((m_vs < 0.0)
	if((flight_time > m_st_control_time.time_separate_booster + 4)
		&&(m_v_air < 52.0)		//���ٻ��ǵ���
		//&&(m_v < 52.0)
		&&(fabs(m_zeta - m_ktheta_lauch_enc) < 2.0) )//��̬�ǹ������������٣�Լ6s
	{
		count_takeoff++;
	}
	else
	{
		count_takeoff = 0;
	}

	if((count_takeoff >= 3)
		&&(!m_st_control_flag.flag_missile_takeoff))
	{
		//��ʼ�߶ȿ��ƣ���ʼ�ٶȿ��Ʋ��ſ���������
		//������
		m_st_control_flag.flag_missile_takeoff = true;
		m_st_control_time.time_missile_takeoff = flight_time;//�����ɣ�ȥ���ӳ�2s

		//��ʼ���������ƣ�����Ϊ����ģʽ
		step_altitude_change_lauch = 1;
	}
	///4.2��������
	if((flight_time >= 15.0)
		&&(!m_st_control_flag.flag_missile_takeoff))
	{
		//��Ϊ������
		m_st_control_flag.flag_missile_takeoff = true;
		m_st_control_time.time_missile_takeoff = flight_time;

		//��ʼ�ٶȿ��ƣ�����Ϊ����ģʽ
		step_altitude_change_lauch = 1;
	}

	///5.����߶ȿ��ƽ���ʱ��: ֮ǰ�������ǿ��ƣ�������Ԥ���߶ȣ���ʼ����ת��
	///5.1������
	if( (m_hz - m_target_height > -20.0)//�߶Ȼ������
		&&(flight_time > m_st_control_time.time_missile_takeoff + 2.0) )
	{
		count_tg++;
	}
	else
	{
		count_tg = 0;
	}

	if((count_tg >= 3)&&(!m_st_control_flag.flag_altitude_control_set))
	{
		m_st_control_flag.flag_altitude_control_set = true;
		m_st_control_time.time_altitude_control = flight_time + 0.5;
	}
	///5.2��������
	if((flight_time >= dlt_time_tg + 25.0)
		&&(!m_st_control_flag.flag_altitude_control_set))
	{
		//��Ϊ������
		m_st_control_flag.flag_altitude_control_set = true;
		m_st_control_time.time_altitude_control = flight_time;
	}

	///6.����Эͬ������ӵ���ʱ��,Ŀ���Ϊ���һ�����㣬��Ŀ����С��10km 
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
	
	//�����ر�־λ
	if((flight_time >= m_st_control_time.time_control)
		&&(!m_st_control_flag.flag_control_on))
	{
		m_st_control_flag.flag_control_on = true;
	}

	//�������������־λ
	if((flight_time >= m_st_control_time.time_separate_booster)
		&&(!m_st_control_flag.flag_separate_booster))
	{
		m_st_control_flag.flag_separate_booster = true;
	}

	//�õ���չ����־λ
	if((flight_time >= m_st_control_time.time_launch_missile_wing)
		&&(!m_st_control_flag.flag_launch_missile_wing))
	{
		m_st_control_flag.flag_launch_missile_wing = true;
	}

	//�÷�������������������־λ
	if((flight_time >= m_st_control_time.time_engine_start)
		&&(!m_st_control_flag.flag_engine_start))
	{
		m_st_control_flag.flag_engine_start = true;	//����ָ�δ�õ�
		m_ECU_work_cmd = 0x22;						//����ָ�δ�õ�
	}

	//�õ���ͷ������־λ
	if((flight_time >= m_st_control_time.time_seeker_on)
		&&(!m_st_control_flag.flag_seeker_on))
	{
		m_st_control_flag.flag_seeker_on = true;
	}

	//�ø߶ȿ��ƽ����־λ
	if((flight_time >= m_st_control_time.time_altitude_control)
		&&(!m_st_control_flag.flag_altitude_control))
	{
		m_st_control_flag.flag_altitude_control = true;

		step_altitude_change_lauch = 3;
	}
	//�������ػ��־λ
	if (m_num_way_point_target == (m_num_way_point - 1))
	{
		m_st_control_flag.flag_lock_on_permit = true;

		//���Ž���
		m_st_control_flag.flag_fuze_unlock = true;
		m_st_control_time.time_fuze_unlock = flight_time;
		
	}
	//��Эͬ������ӵ���ʱ�̱�־λ
	if((flight_time >= m_st_control_time.time_cooperative_attack	//Эͬ��ӹ���ִ��
		&&(m_num_way_point_target == (m_num_way_point - 1)))
		&&(!m_st_control_flag.flag_cooperative_attack))
	{
		m_st_control_flag.flag_cooperative_attack = true;
		m_st_control_time.time_cooperative_attack = flight_time;
	}

	//7.ĩ�Ƶ�����������
	//���һ������ͷ����Ŀ���̶������⵼�����
	if(m_missile_flight_mode == 0xAA) //ʮ����170
	{
		// 7.1����ĩ�Ƶ�(��ս��)ʱ�̣�Ŀ���Ϊ���һ�����㣬��Ŀ����С��2km �� ����ͷ����Ŀ�꣬�ӳ�50ms
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
			m_st_control_time.time_combat_status = flight_time + 0.05;
			
			m_distance_target_t_combat = m_distance_target;
		}
	
		// 7.2 ����ĩ�Ƶ�ʱ�̣��Ƶ���ʼ��: ���緢�����ػ���
		//if( (flight_time >= m_st_control_time.time_combat_status
		//		||( p_st_flight_basic_input->st_seeker_data.flag_combat_status && (m_num_way_point_target == (m_num_way_point - 1)) )  )
		//      &&(!m_st_control_flag.flag_combat_status) )
		if((flight_time >= m_st_control_time.time_combat_status)&&(!m_st_control_flag.flag_combat_status) )
		{
			m_st_control_flag.flag_combat_status = true;	//ս��ֻ�ܹ֣�����ĩ�Ƶ�

			//��������ʽ���ģʽʱ������ĩ�Ƶ��ҷ������ػ�
			//m_st_control_flag.flag_engine_shutdown = true;
			//m_st_control_time.time_engine_shutdown = flight_time;
			//m_ECU_work_cmd = 0x44;
		}
	}
	//�����������ѵ��ģʽ�����չ��ɣ���󺽼���Ϊ��ɡ��
	else
	{
		// 7.1 ����ĩ�Ƶ�(��ս��)ʱ�̣�Ŀ���Ϊ���һ�����㣬��Ŀ����С��2km �� ����ͷ����Ŀ�꣬�ӳ�50ms
		// ˵��������Ϊ�����ڶ������㣬�Һ�������Ϊ��������(�����ģʽ)�����һ������Ϊ���յ�
		if((m_distance_target <= 2000.0)
			//&&(m_num_way_point_target == (m_num_way_point - 2))	//ȥ��������Լ��
			&&(m_route_mode == 5))//�𹥵�
		{
			count_virtual++;
		}
		else
		{
			count_virtual = 0;
		}
		//��Ŀ����С��2km �� ����ͷ����Ŀ��
		if((count_virtual >= 3 
			||(p_st_flight_basic_input->st_seeker_data.flag_combat_status && (m_num_way_point_target == m_num_way_point - 2)))
		    &&(!m_st_control_flag.flag_combat_status_set)
		    &&(!m_st_control_flag.flag_combat_dive_pullup))//���븩������󣬲��ٽ���
		{		
			m_st_control_flag.flag_combat_status_set = true;
			m_st_control_time.time_combat_status = flight_time + 0.05;
			
			m_distance_target_t_combat = m_distance_target;
		}
		
		// 7.2���������ԣ�������Ҫ�������𣬿�ɡ����
		if((flight_time >= m_st_control_time.time_combat_status)&&(!m_st_control_flag.flag_combat_status) )
		{
			m_st_control_flag.flag_combat_status = true;	//ս��ָ�����ĩ�Ƶ�

			//�����ڲ���ģʽʱ������ĩ�Ƶ��ҷ�����δ�ػ������뵡��ģʽ
			//m_st_control_flag.flag_engine_shutdown = true;	
			//m_st_control_time.time_engine_shutdown = flight_time;
			//m_ECU_work_cmd = 0x11;//����������
		}

		//7.3 虚拟打击结束后，转俯冲拉起，之后定高巡航，逐步抵近目标点
		if((m_st_control_flag.flag_combat_status)&&(m_hz - m_st_target.height - 100.0 < 0.0))
		{
			count_combat_dive_ok++;
		}
		else
		{
			count_combat_dive_ok = 0;
		}			
		
		if( (count_combat_dive_ok >= 3) && (!m_st_control_flag.flag_combat_dive_ok_set) )
		{
			m_st_control_flag.flag_combat_dive_ok_set = true;
			m_st_control_time.time_combat_dive_ok = flight_time + 0.10;
			//m_ECU_work_cmd = 0x33;//发动机速度控制

			//虚拟打击转俯冲拉起，战斗指令无效
			m_st_control_flag.flag_combat_status = false;	
			m_st_control_time.time_combat_status = MAX_TIME;
		}
		
		//7.3 2��ĩ�Ƶ����� ת �������𣬽�������״̬:��ǰ��̬�� ���ɵ� ������̬�ǲ����֣�������Ŀ��߶Ⱥ󣬽��и߶ȿ���
		if((flight_time >= m_st_control_time.time_combat_dive_ok) && (!m_st_control_flag.flag_combat_dive_pullup))
		{
			m_st_control_flag.flag_combat_dive_pullup = true;
			m_st_control_time.time_combat_dive_pullup = flight_time;
		}

		// 7.4 �������Ѹ��£�Ŀ��߶ȸ���Ϊ��ǰһ�θ߶�����100m������
		//�߶Ȼ������� ĩ�Ƶ�ת������ʵ�ָ���������̣������ø߶Ȼ�����ʼ�о�
		//����ͨ�����ƣ�ʵ�ָ߶Ȼ�����ֱ��������Ŀ��߶�
		//�߶Ȼ������� ����תѲ�����߶ȿ��ƣ���ƽ����Ŀ��ֽ������ø߶Ȼ��������о�
		
		// 7.5 ���ݵ�Ŀ�����жϣ�������պ��㸽���󣬽��뿪ɡ��������
	
		//ɡ�������գ���: ������ǰװ�������߸��£����������ա���㺽���ȣ��������������յ㡢������յ�Ȳ�������
		//˵����������Ϣ�У���������߶ȡ��ٶȣ�
		//���У����㣨���Σ��߶���Ե���߶ȣ���С��300m����֤�ɿ����գ�
		//���У������ٶ�ȡֵ��ΧΪ40~50m/s�������ٿ����ҵ���Ѳ���ٶȣ�������ٻ��գ�
		//�߶��½���Ŀ��߶ȣ�����300m�����ٶ��½���Ŀ���ٶȣ�����50m/s��
		//˵��1������ɡ��800�C1500 m������ɡ��200�C500 m �����Ͻ��� 100 m ���¿���ɡ��
		//˵��2��һ���� �����߶� 5�C15 m ʱ���� / չ����ȫ���ң�С�Ͱл���5�C10 m������ / ���ٰл���10�C15 m���Ͻ��� 3 m ���²ų��������������Σ�����ʧЧ��
		if((step_open_umbrella == 0)&&(m_route_mode == 6))
		{
			step_open_umbrella = 1;
		}
		//��һ��:�жϽ������Ȧ��������հ뾶������ֵ300m����������ͣ�������߼��ٿ���
		else if(step_open_umbrella == 1)
		{
			//if(m_distance_target < m_accept_radius)
			if(m_distance_BP < m_accept_radius)
			{
				count_distance_recycle++;
			}
			else
			{
				count_distance_recycle = 0;
			}

			if(count_distance_recycle >= 3 )
			{
				step_open_umbrella = 2;
				
				m_st_control_flag.flag_engine_shutdown = true;//�������ػ�
				m_st_control_time.time_engine_shutdown = flight_time;
				m_ECU_work_cmd = 0x44;				//�������ػ�
			}
		}
		//m_ECU_work_cmd = 0x44;
		//�ڶ�������������δ��Ȧ��������֮һ����
		//��1���ٶ�С��50m/s��ִ�п�ɡ����2���ٶ�50��54m/s������Ը߶�С��300m��
		else if(step_open_umbrella == 2)
		{
			if(m_v_air < 50.0)
			{
				count_v50_recycle++;
			}
			else
			{
				count_v50_recycle = 0;
			}

			if((m_v_air < 54.0) && (m_hz - m_target_height_ground < 280.0))
			{
				count_h300_v54_recycle++;
			}
			else
			{
				count_h300_v54_recycle = 0;
			}

			
			if((count_v50_recycle > 3)|| (count_h300_v54_recycle > 3))
			{
				step_open_umbrella = 3;
				m_st_control_flag.flag_open_umbrella = true;//开伞回收
				m_st_control_time.time_open_umbrella = flight_time;				
			}
			else
			{
				//备份条件：主条件未满足，出圈后
				//if(m_accept_radius > m_distance_target + 50.0)
				if(m_distance_BP > m_accept_radius + 50.0)
				{
					count_distance_out_recycle++;
				}
				else
				{
					count_distance_out_recycle = 0;
				}

				if(count_distance_out_recycle >= 3 )
				{
					step_open_umbrella = 4;//��Ȧ
				}
			}
		}
		//������: ������δ�������жϳ�Ȧ����������֮һ����
		//��1)�ٶ�С��54m/s��(2) ��Ը߶�С��250m��(3)��Ȧ�󣬷��о������600m
		else if(step_open_umbrella == 4)
		{
			//����1��2��3
			if(m_v_air < 54.0)
			{
				count_v54_recycle ++;
			}
			else
			{
				count_v54_recycle = 0;
			}

			if(m_hz - m_target_height_ground < 250.0)
			{
				count_h250_recycle ++;
			}
			else
			{
				count_h250_recycle = 0;
			}

			//if(m_distance_target > 600.0)
			if(m_distance_BP > 600.0)
			{
				count_distance_out600_recycle ++;
			}
			else
			{
				count_distance_out600_recycle = 0;
			}

			if((count_v54_recycle > 3) || (count_h250_recycle > 3) || (count_distance_out600_recycle > 3))
			{
				step_open_umbrella = 5;

				m_st_control_flag.flag_open_umbrella = true;//��ɡ����
				m_st_control_time.time_open_umbrella = flight_time;		
			}
		
		}
		//���Ĳ�����ɡ����Ե���߶�С��10mʱ���򿪰�ȫ���� (���Ͽ�ɡʱ�䣩
		else if( (step_open_umbrella == 3) || (step_open_umbrella == 5))
		{
			//��ʱ�ȿ���
			if(m_hz - m_target_height_ground < 10.0)
			{
				count_h10_recycle++;
			}
			else
			{
				count_h10_recycle = 0;
			}

			if(count_h10_recycle > 3)
			{
				step_open_umbrella = 6;
				//�򿪰�ȫ���ң�������...
			}
		}
		//���岽���򿪰�ȫ���Һ󣬴��غ�5s���и����ɡ
		else if(step_open_umbrella == 6)
		{
			//�����оݣ���Ը߶�С��5m���� ny��1.5g(����50ms)
			////���س���������ޣ�ny��1.5��2.0g
			////�Ͽ��л� / Ӳ��½�����ң�ny��2.0g
			////���������������գ�ny��1.5g
			////�����߶��оݣ���Ը߶�С��5m
			if((m_hz - m_target_height_ground < 5.0) && (m_ny > 2.0))
			{
				count_h5_ny2_recycle++;
			}
			else
			{
				count_h5_ny2_recycle = 0;
			}

			if(count_h5_ny2_recycle > 10)
			{
				step_open_umbrella = 7;
				//û�д��ر�ʶ
				m_st_control_time.time_touch_ground = flight_time;
			}
		}
		else if(step_open_umbrella == 7)
		{
			if(flight_time > m_st_control_time.time_touch_ground + 5.0)
			{
				step_open_umbrella = 8;
				//�и��ɡ��������...
			}
		}
	}
}

// �����в�����־ѹ��Ϊ�����޷�������
void CMathControlFlightBasic::PackControlFlag(const Stru_Control_flag* pSt)
{
    unsigned int uRet = 0U;

    if (pSt->flag_t0)                    uRet |= BIT_flag_t0;
    if (pSt->flag_control_on)            uRet |= BIT_flag_control_on;
    if (pSt->flag_separate_booster)      uRet |= BIT_flag_separate_booster;
    if (pSt->flag_engine_start)          uRet |= BIT_flag_engine_start;
    if (pSt->flag_seeker_on)             uRet |= BIT_flag_seeker_on;
    if (pSt->flag_initial_hz)            uRet |= BIT_flag_initial_hz;
    if (pSt->flag_missile_takeoff)       uRet |= BIT_flag_missile_takeoff;
    if (pSt->flag_launch_turn)           uRet |= BIT_flag_launch_turn;
    if (pSt->flag_altitude_control)      uRet |= BIT_flag_altitude_control;
    if (pSt->flag_lock_on_permit)        uRet |= BIT_flag_lock_on_permit;
    if (pSt->flag_cooperative_attack)    uRet |= BIT_flag_cooperative_attack;
    if (pSt->flag_combat_status)         uRet |= BIT_flag_combat_status;
    if (pSt->flag_fuze_unlock)           uRet |= BIT_flag_fuze_unlock;
    if (pSt->flag_combat_dive_ok_set)    uRet |= BIT_flag_combat_dive_ok_set;
    if (pSt->flag_combat_dive_sidectrl)  uRet |= BIT_flag_combat_dive_sidectrl;
    if (pSt->flag_combat_dive_pullup)    uRet |= BIT_flag_combat_dive_pullup;
    if (pSt->flag_engine_shutdown)       uRet |= BIT_flag_engine_shutdown;
    if (pSt->flag_open_umbrella)         uRet |= BIT_flag_open_umbrella;
    if (pSt->flag_alltitude_change)      uRet |= BIT_flag_alltitude_change;
    if (pSt->flag_alltitude_climb)       uRet |= BIT_flag_alltitude_climb;
    if (pSt->flag_alltitude_decline)     uRet |= BIT_flag_alltitude_decline;
    if (pSt->flag_waypoint_turn)         uRet |= BIT_flag_waypoint_turn;
    if (pSt->flag_turn_out_set)          uRet |= BIT_flag_turn_out_set;
    if (pSt->flag_flightime_ctrl)        uRet |= BIT_flag_flightime_ctrl;
    if (pSt->flag_velocity_control)      uRet |= BIT_flag_velocity_control;
    if (pSt->flag_heading_hold)          uRet |= BIT_flag_heading_hold;
    if (pSt->flag_turndir_set)           uRet |= BIT_flag_turndir_set;
    if (pSt->flag_prepare_hover)         uRet |= BIT_flag_prepare_hover;
    if (pSt->flag_relativehigh_ctrl)     uRet |= BIT_flag_relativehigh_ctrl;
    if (pSt->flag_attackangle_ctrl)      uRet |= BIT_flag_attackangle_ctrl;

	m_flight_control_state = uRet;
    return;
}

void CMathControlFlightBasic::Calc_Flight_Data()
{
	//�����������ٶ�
	double sinLat = sin(m_latitude / RTOA);
	m_g = 9.7803 + 0.051799 * sinLat * sinLat - 0.94114e-6 * m_hz;
	
	//�������Ʒ�λ�ǣ�Ҳ�Ƶ�������Բ���߷��У���λ������������ǣ���λ�ǣ�����ת����
	double Adot = - RTOA * (m_vtz * tan(m_latitude / RTOA) / (RE / (1.0 - E_CONST * sinLat * sinLat) + m_hz));
	m_A += Adot * STEP_5ms;//��ƫ��Ϊ�����������ʱ����λ�Ǽ�С
	m_A = CFlightGlobalFun::Adjust(m_A, 180.0);

	//���㵼��ϵ�����
	m_psin = m_psit - m_A;
	m_psin = CFlightGlobalFun::Adjust(m_psin, 180.0);

	//����ϵ������ϵת������ Cnb���������򡢲���2-3-1 ת�򣬵õ� ǰ���� ����ϵ
	double sinR = sin(m_gama / RTOA);
	double cosR = cos(m_gama / RTOA);
	double sinY = sin(m_psin / RTOA);
	double cosY = cos(m_psin / RTOA);
	double sinP = sin(m_zeta / RTOA);
	double cosP = cos(m_zeta / RTOA);	
	m_Cnb[0] = cosY * cosP;
	m_Cnb[1] = sinP;
	m_Cnb[2] = -cosP * sinY; //��һ��
	m_Cnb[3] = -cosY * sinP * cosR + sinY * sinR;
	m_Cnb[4] = cosP * cosR;
	m_Cnb[5] = sinY * sinP * cosR + cosY * sinR;//�ڶ���
	m_Cnb[6] = cosY * sinP * sinR + sinY * cosR;
	m_Cnb[7] = -cosP * sinR;
	m_Cnb[8] = -sinY * sinP * sinR + cosY * cosR;//������
	//��任
	m_Cbn[0] = m_Cbn[0];
	m_Cbn[1] = m_Cbn[3];
	m_Cbn[2] = m_Cbn[6];
	m_Cbn[3] = m_Cbn[1];
	m_Cbn[4] = m_Cbn[4];
	m_Cbn[5] = m_Cbn[7];
	m_Cbn[6] = m_Cbn[2];
	m_Cbn[7] = m_Cbn[5];
	m_Cbn[8] = m_Cbn[8];
	
	//����ϵ���ٶ� ���ɵ� ����ϵ
	m_anx = m_Cbn[0]*m_axflt + m_Cbn[1]*m_ayflt + m_Cbn[2]*m_azflt;
	m_au = m_Cbn[3]*m_axflt + m_Cbn[4]*m_ayflt + m_Cbn[5]*m_azflt;
	m_anz = m_Cbn[6]*m_axflt + m_Cbn[7]*m_ayflt + m_Cbn[8]*m_azflt;

	//������ϸ߶ȣ���Ϊ��ϵ���Kalman�˲����������ٽ����˲���������ͻ�䣻
	Calc_Hz();
	//Ϊ�˲���???...
	m_hz = m_hgps;
	m_vs = m_vty;
	
	//���㵼����Ŀ�꺽·���ͶӰ����
	double distance_BA = 0.0;
	double angle_BA = 0.0;//��ƫ��Ϊ��
	double angle_BP = 0.0;//��ƫ��Ϊ��
	CFlightGlobalFun::Tomas(m_longitude_B, m_latitude_B, 
		m_longitude, m_latitude, 
		&m_distance_BP, &angle_BP);
	angle_BP = CFlightGlobalFun::Adjust(angle_BP, 180.0);
	CFlightGlobalFun::Tomas(m_longitude_B, m_latitude_B, 
		m_longitude_A, m_latitude_A, 
		&distance_BA, &angle_BA);
	angle_BA = CFlightGlobalFun::Adjust(angle_BA, 180.0);
	double dlt_alpha = angle_BA - angle_BP;
	dlt_alpha = CFlightGlobalFun::Adjust(dlt_alpha, 180.0);
	m_distance_BP_projection = m_distance_BP * cos(fabs(dlt_alpha / RTOA));

	//����500ms�������ǰ��ǰһ֡�洢
	if(!((time_tick - 1) % 500))
	{
		m_distance_BP_500pre = m_distance_BP_500;
		m_distance_BP_500    = m_distance_BP_projection;
	}

	//���㵼��������о���
	m_vx += (m_ax - m_g * sin(m_zeta / RTOA)) * STEP_5ms;
	m_sx += m_vx * STEP_5ms;

	//���㵯Ŀ���뼰Ŀ��(���)��λ�ǣ����������߽�
	CFlightGlobalFun::Tomas(m_longitude, m_latitude, 
		m_st_target.longitude, m_st_target.latitude, 
		&m_distance_target, &m_alpha_target);
	m_alpha_target = -m_alpha_target;
	m_alpha_target = CFlightGlobalFun::Adjust(m_alpha_target, 180.0);	//�뺽��Ƕ��巴��
	if(m_distance_target < 10.0)
	{
		m_distance_target = 10.0;
	}
	
	//������ٶȣ�����
	m_v = sqrt(m_vtx * m_vtx + m_vs * m_vs + m_vtz * m_vtz);

	//���㵱ǰʱ��ǰ1s��,���ٶȾ�ֵ
	if(!((time_tick - 1) % 100))
	{
		m_v_average_1s = 0.0;
		for(int i=0; i<9; i++)
		{
			m_v_record_100ms[i] = m_v_record_100ms[i+1];
			m_v_average_1s += m_v_record_100ms[i];
		}
		//m_v_record_100ms[9] = m_v_air;
		m_v_record_100ms[9] = m_v;	//��Ϊ����
		m_v_average_1s += m_v_record_100ms[9];
		m_v_average_1s = m_v_average_1s / 10.0;
	}

	//���㵱ǰʱ��ǰ10s�ڣ����ٶȾ�ֵ
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

	//���������������٣�����������
	m_air_temperature = 273.15 + m_ground_temperature 
		- 0.0065 * (m_hz - p_st_initial_data->height_launch);
	m_sonic_speed = 20.0449 * sqrt(m_air_temperature);
	m_mach  = m_v_air / CFlightGlobalFun::Nozero_FUN(m_sonic_speed);	
	if( m_hz > 11000  &&  m_air_temperature < 216.85)	
		m_air_temperature = 216.85;
	m_air_pressure = 101325.0 * pow( 1 - m_hz / 44330.769, 5.25588);//ѹ��
	m_air_density = m_air_pressure / (287.05287 * m_air_temperature);	//�����ܶ�
	
	//���㵼��ϵ�ٶ�
	double cosA = cos(m_A / RTOA);
	double sinA = sin(m_A / RTOA);
	m_vnx = m_vtx * cosA - m_vtz * sinA; 
	m_vnz = m_vtx * sinA + m_vtz * cosA;

	//���㵯��ϵ����
	m_ny = m_ayflt / m_g;
	m_nz = m_azflt / m_g;

	//���㺽��ƫ��
	double temp = m_vtx * m_vtx + m_vtz * m_vtz;
	if(fabs(temp) < 1e-10)
	{
		m_psicn = 0.0;
		if(fabs(m_vty) < 1e-10)
		{
			m_theta = 0.0;
		}
		else
		{
			m_theta = 90.0*CFlightGlobalFun::FSign(m_vty);
		}
	}
	else
	{
		m_psicn = RTOA * acos(m_vtx / sqrt(temp));
		//ȡֵ��Χ -90~90deg
		m_theta = RTOA * atan2(m_vty, sqrt(temp));
	}
	//��Χת��Ϊ-180deg~180deg
	if(m_vtz >= 0)
		m_psicn = 360.0 - m_psicn;
	m_psicn = CFlightGlobalFun::Adjust(m_psicn, 180.0);

	//����ת������й�������Ǳ��ֵ�����ټ���
	m_gama_turn_nominal = atan(m_v * m_v / m_turn_radius / m_g) * RTOA;
	m_gama_turn_nominal = CFlightGlobalFun::Range(m_gama_turn_nominal, ROLL_COMMAND_DYNMIC_LIMIT);
	
	//����ת������е�����Բ�ľ��롢�����ٶȣ�����ֱ�����̲�ƫ
	//if (flight_time > m_st_control_time.time_turn_in_start 
	//	&& flight_time <= m_st_control_time.time_turn_out_end)
	//˵��1��ԭ�оݣ�����ʱ�����ӣ������ƫ���л�ֱ�߲�ƫ�����л������ߣ�����������
	//˵��2����������ͳ�����̲�ƫ�޿��ƣ����ڲ�ƫ�������ٶ����䣬����1s���ɣ������Ż�����ʱ�䣬����1s->3s��
	if(m_st_control_flag.flag_waypoint_turn == true)
	{
		m_x_coordinate_turn = m_x_coordinate_turn + m_vtx * STEP_5ms ;
		m_z_coordinate_turn = m_z_coordinate_turn + m_vtz * STEP_5ms ;
		
		m_sz_radius = sqrt( m_x_coordinate_turn * m_x_coordinate_turn + m_z_coordinate_turn * m_z_coordinate_turn); 
		//����תʱ��ת��Ƕ�Ϊ����Բ�����߷���Ϊ���߷���
		//�ٶ���Ժ�����ƫΪ��
		m_vnz = - (m_x_coordinate_turn * m_vtx + m_z_coordinate_turn * m_vtz ) * CFlightGlobalFun::FSign(m_turn_angle)/ m_sz_radius;
		//m_vnz = (m_x_coordinate_turn * m_vtx + m_z_coordinate_turn * m_vtz ) * CFlightGlobalFun::FSign(m_turn_angle) / m_sz 
		//�����ں����Ҳ�Ϊ��
		m_sz = - (m_sz_radius - m_turn_radius) * CFlightGlobalFun::FSign(m_turn_angle);

		//����ת��Σ���ֱ����ת�����
		if(flight_time < m_st_control_time.time_turn_in_start + 1.0)
		{
			m_sz = m_sz*(flight_time - m_st_control_time.time_turn_in_start)
				+ m_sz_record1*(1.0 + m_st_control_time.time_turn_in_start - flight_time);
			m_vnz = m_vnz*(flight_time - m_st_control_time.time_turn_in_start)
				+ m_vnz_record1*(1.0 + m_st_control_time.time_turn_in_start - flight_time);
		}
		//����ת��β�ƫ���ٶ�
		m_sz_record2 = m_sz;
		m_vnz_record2 = m_vnz;
	}
	//ֱ����ƫ
	else
	{
		//���㵼����Ŀ�꺽�ߴ�ֱ�������
		m_sz = CFlightGlobalFun::CalcDist(m_longitude_A, m_latitude_A,
			m_longitude_B, m_latitude_B, 
			m_longitude, m_latitude);
		//m_vnz ���ڵ���ϵ�ٶ�
		//m_vnz = m_vtx * sinA + m_vtz * cosA;

		//����ֱ���Σ���ת�䵽ֱ������
		if( (flight_time < m_st_control_time.time_turn_out_end + 1.0)
			&&(flight_time > m_st_control_time.time_turn_out_end) 
			&&(flight_time > m_st_control_time.time_launch_turn_ok + 5.0))//����ת����ɺ󣬲�������
		{
			m_sz = m_sz*(flight_time - m_st_control_time.time_turn_out_end)
				+ m_sz_record2*(m_st_control_time.time_turn_out_end + 1.0 - flight_time);
			m_vnz = m_vnz*(flight_time - m_st_control_time.time_turn_out_end)
				+ m_vnz_record2*(m_st_control_time.time_turn_out_end + 1.0 - flight_time);
		}
		
		//����ֱ���β�ƫ���ٶ�
		m_sz_record1 = m_sz;
		m_vnz_record1 = m_vnz;
	}
	

	//����ʣ�����ʱ��
	double vxz = sqrt(m_vtx * m_vtx + m_vtz * m_vtz);
	double det_A = m_psicn - m_alpha_target;
	det_A = CFlightGlobalFun::Adjust(det_A, 180.0);
	det_A = fabs(det_A);
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

//���ݿ������Ż����ת�٣������ͺģ���һ����������ȼ�ϡ�����������
void CMathControlFlightBasic::Calc_Mass_Data()
{
	//��������: �����ٶȡ�ת�٣����ƹ��ʣ������ܶȱ���������
	double vel_engine_array[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};	//�ٶ�mps
	double rpm_engine_array[9] = {2000, 2500, 3000, 4000, 5000, 5500, 6180, 6800, 7500};//ת��rpm
	double Kc_engine_array[9] = {0.0,10.1374, 19.7358, 37.6508, 61.8964, 75.4171, 97.1771,100.0, 113.0};//����%
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
		0, 0, 0, 822.3, 0, 0, 0, 7706, 21699};// ����W

#ifdef	__VEL__CONTROL__MODE__KC__
	//������������ſ���ģʽ���ôӡ�������ָ�����š���ȡ�ġ�״̬ת�١�
	m_state_rpm = CFlightGlobalFun::LAQL1(9,  Kc_engine_array, rpm_engine_array, m_cmd_Kc);//�������Ź���ת��
#else	
	//���������ת�ٿ���ģʽ���á�״̬ת�١����Ƶ��ڡ�ָ��ת�١�
	m_state_rpm = m_cmd_rpm;
#endif
	//����һ���ӳ�����???...
	
	//���������ת�ٴ�����������ת����Ч����ʹ�÷��������ת�٣�����ʹ�ÿ���ָ�����ת��
	if(fabs(m_engine_state_rpm - m_state_rpm) < 1000.0)
	{
		m_state_rpm = m_engine_state_rpm;
	}

	//�ͺĹ��㷽��һ:
	//�����ٶȡ�ת�ٹ��ƹ���
	double temp_power = 0.0;
	temp_power = CFlightGlobalFun::LAQL2(11,  9,  vel_engine_array,  rpm_engine_array, power_engine_matrix, m_v_air, m_state_rpm);
	//���ݸ߶ȼ�������ܶȱȣ�һά��ֵ��������������
	double hight_engine_array[7] = {0, 1000, 2000, 3000, 4000, 4500,5000};
	double Krho_engine_array[7] = {1, 0.9078, 0.8220, 0.7420, 0.6686, 0.6343, 0.6008};//�߶ȣ������ܶȱ�
	double temp_Krho = 1.0;
	temp_Krho = CFlightGlobalFun::LAQL1(7,  hight_engine_array,  Krho_engine_array, m_hz);//�����ܶȱ�

	//�ͺĹ��㷽����: 
	//����ת�١��ٶȹ��Ʒ���������������Ч��
	//���ݸ߶ȼ�������ܶȱȣ���������
	//��Ϊ��ͬ�߶�Ч����ͬ������������ٶȣ�����Ч��

	//������: ����ƽ��ƽ�⹥�ǹ�������������ƫ��
	//���ݵ�ǰ�߶ȹ�������ܶ�
	//���ݸ����ǣ��������Ϊ�㣬���㹥�ǣ�
	//���ݹ��ǲ�ֵ����������ϵ������������
	//����������ƽ�⣬��������
	//˵����С����ϵ���������У������������ϴ�0.1~0.3deg����������
	//����С����
	//���ƹ��㣬С����״̬���Խ��ͷ����ٶȣ����󹥽ǣ�����������
	
	//ɽ���Ʊ�������Ϊ0.643 = 0.45kg/(kW.h) ��������������ʸ�Ϊ0.779L/(W.h) 
	m_fuel_comsumped += temp_power*1e-3*temp_Krho*0.45*0.005/3600;	//װ������
	m_mass_calc = FULL_MASS - m_fuel_comsumped;		//װ���������������
	if(m_mass_calc < EMPTY_MASS)
	{
		m_mass_calc = EMPTY_MASS;	//ʣ�͵Ȳ�����������
	}

	//ʣ�����ʱ�䡢����
	m_left_flight_time_calc = (45 - m_fuel_comsumped)*3600/(temp_power*1e-3*temp_Krho*0.45);
	m_left_flight_dist_calc = m_left_flight_time_calc*m_v;
}

void CMathControlFlightBasic::Calc_LOS_Rate()
{
	double dqf_t = 0.0;//deg/s
	double dqh_t = 0.0;
	double dqg_t = 0.0;

	double dqf_b = 0.0;//deg/s
	double dqh_b = 0.0;
	double dqg_b = 0.0;

	//���ݵ�Ŀˮƽ���롢�߶Ȳ���㸩�����߽ǡ��������߽�
	double det_Y = 0.0;//�߶Ȳ�
	double Ym = m_st_target.height;//Ŀ��߶�
	det_Y = m_hz - Ym;
	if(det_Y < 0.0)
		det_Y = 0.0;
	//б�����
	m_slant_distance_target = sqrt(m_distance_target * m_distance_target + det_Y * det_Y);

	//无导引头状态，为了测试???...
	m_seeker_state_track = 0x00;
	
	//���ݵ���ͷ����״̬��ѡ�����߽��ٶ���Դ
	if(m_seeker_state_track == 0x03 || m_seeker_state_track == 0x04)
	{
		//����������Ұֵ�㷨����ʱ������
		
		//ʹ�õ���ͷ�䣬����ϵ���߽��ٶȣ�ת��Ϊ����ϵ
		//m_dqh = m_seeker_dqh*cos(m_gama / RTOA) - m_seeker_dqf*sin(m_gama / RTOA);//��ת����任
		//m_dqf = m_seeker_dqh*sin(m_gama / RTOA) + m_seeker_dqf*cos(m_gama / RTOA);
		m_dqh = m_seeker_dqh*cos(m_gama / RTOA) + m_seeker_dqf*sin(m_gama / RTOA);//��ת����任
		m_dqf = -m_seeker_dqh*sin(m_gama / RTOA) + m_seeker_dqf*cos(m_gama / RTOA);
		
		//���ݵ���ͷ���߽ǡ�Ŀ��װ���߶ȣ�����Ŀ��λ��
		//˵������絼��ͷ���޷�̽�ⵯĿ���룬��Ҫ�������߽Ǻ͸߶Ȳ���㣬���λ��ʸ������һ������Ŀ��λ��
		m_Qf = m_seeker_qf;
		//m_Qn = m_seeker_qh;
		//m_Qh = m_Qn + m_A;//�溽�����߽�
		//���󣬵���ͷ������ǵ���ϵ�������߽ǣ��޸�
		m_Qf = m_seeker_qf;
		m_Qh = m_seeker_qh;
		m_Qn = m_Qh - m_A;

		//���ݸ߶Ȳ���߽ǹ��Ƶ�Ŀ���룬��һ������Ŀ��λ�ã���Ŀ�꾭�ȡ�Ŀ��ά��
		//double temp_slant_distance = det_Y/CFlightGlobalFun::Nozero_FUN(sin(m_dqf / RTOA));
		double temp_slant_distance = det_Y/CFlightGlobalFun::Nozero_FUN(sin(m_Qf / RTOA));
		double temp_distance = sqrt(temp_slant_distance*temp_slant_distance - det_Y*det_Y);
		m_slant_distance_target = temp_slant_distance;
		m_distance_target = temp_distance;

		//���ݡ���Ŀ���롱�͡���Է�λ���������߽ǡ�������Ŀ��λ��ʸ��
		m_Rmt_n[0] = m_slant_distance_target*cos(m_Qf / RTOA)*cos(m_Qn / RTOA);
		m_Rmt_n[1] = m_slant_distance_target*sin(m_Qf / RTOA);
		m_Rmt_n[2] =- m_slant_distance_target*cos(m_Qf / RTOA)*sin(m_Qn / RTOA);

		//���ݡ���Ŀ���롱�͡���Է�λ��������Ŀ��λ��
		double dLat_deg = (temp_distance * cos(m_Qh/RTOA) / R_EARTH);
		double dLon_deg = (temp_distance * sin(m_Qh/RTOA) / (R_EARTH * cos(m_latitude / RTOA)));
		//Ŀ�꾭�ȡ�γ�ȸ��£����⵼��ͷ��ʧĿ�꣬����Ŀ�꾭γ��???...
		m_st_target.longitude = dLon_deg + m_longitude;
		m_st_target.latitude = dLat_deg + m_latitude;
		//Ŀ��߶ȱ��ֲ��䣬������
	} 
	else
	{
		//�ⲿ�֣����߽ǽ������???...
		if((m_distance_target > 50.0)&&(det_Y > 10.0))
		{
			//���߽Ǽ���
			m_Qf = atan(-det_Y / m_distance_target) * RTOA;//deg
			m_Qh = m_alpha_target;//deg
			m_Qn = m_Qh - m_A;
			m_Qn = CFlightGlobalFun::Adjust(m_Qn, 180.0);

			//���߽��ٶ�
			dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
				/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target);
			dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
				/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target * m_slant_distance_target);
			dqg_t = RTOA * ((-m_vty * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
				/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target * m_slant_distance_target);	

			//ת��������ϵ���ٴӵ���ϵ��ת����
			//dqf_t -> dqf_b
			//dqh_t -> dqh_b
			//dqg_t -> dqg_b
			//���Ը����ǡ������ƫ�ֻ���й�ת����
			//���߽��ٶȣ�����ϵת��������ϵ???...

			//���߽ǹ�ת����???...������
		
			//����ϵ��ת����
			m_dqh = dqh_t*cos(m_gama/RTOA) + dqf_t*sin(m_gama/RTOA); //��ת�����任
			m_dqf = - dqh_t*sin(m_gama/RTOA) + dqf_t*cos(m_gama/RTOA) ;
			
		}
		

		//���߽��ٶȼ���
		//ˮƽ���������ֵ
		//if(m_distance_target >= 500.0)
		//{
		//	dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
		//		/CFlightGlobalFun::Nozero_FUN(temp1);
		//
		//	dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
		//		/CFlightGlobalFun::Nozero_FUN(temp1 * temp1);	
		//}
		//ˮƽ����С��500m��ֵʱ
		//else
		//{
			//�߶ȴ���Ŀ��߶�10m���ϣ���ˮƽ�������50m����
			if( (det_Y > 10.0) &&(m_distance_target >= 50.0))
			{
				//����
				valid_count = 0;
			}
			//��Ը߶Ƚ�С
			else
			{
				valid_count++;
			}		

			//�������߽Ǽ����ٶ�
			if(valid_count < 3)
			{
				//����һ�����ݵ���ϵ�ٶȡ�λ��ʸ�����������߽��ٶȣ�����ϵ�������
				/*dqh_t = RTOA * (m_vtx * sin(m_Qh/RTOA) + m_vtz * cos(m_Qh/RTOA))
					/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target);
				dqf_t = RTOA * ((-m_vtx * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
					/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target * m_slant_distance_target);
				dqg_t = RTOA * ((-m_vty * cos(m_Qh/RTOA)+ m_vtz * sin(m_Qh/RTOA)) * det_Y - m_vs * m_distance_target)
					/CFlightGlobalFun::Nozero_FUN(m_slant_distance_target * m_slant_distance_target);	
				//ת��������ϵ���ٴӵ���ϵ��ת����
				//dqf_t -> dqf_b
				//dqh_t -> dqh_b
				//dqg_t -> dqg_b
				//���Ը����ǡ������ƫ�ֻ���й�ת����
				//���߽��ٶȣ�����ϵת��������ϵ???...

				//���߽ǹ�ת����???...������
		
				//����ϵ��ת����
				m_dqf = dqf_t*cos(m_gama/RTOA) - dqh_t*sin(m_gama/RTOA);
				m_dqh = dqf_t*sin(m_gama/RTOA) + dqh_t*cos(m_gama/RTOA);
				*/

				/*
				//��������ʸ������
				//���ݵ���Ŀ��γ�ȣ��������λ��ʸ��������ϵ������ ���ݵ�Ŀ���롢�������߽ǡ��������߽�
				m_Rmt_n[0] = m_slant_distance_target*cos(m_Qf / RTOA)*cos(m_Qn / RTOA);
				m_Rmt_n[1] = m_slant_distance_target*sin(m_Qf / RTOA);
				m_Rmt_n[2] = - m_slant_distance_target*cos(m_Qf / RTOA)*sin(m_Qn / RTOA);

				//��һ��ʸ��
				double temp_Rmt_n_dir[3] = {0.0};
				temp_Rmt_n_dir[0] = cos(m_Qf / RTOA)*cos(m_Qn / RTOA);
				temp_Rmt_n_dir[1] = sin(m_Qf / RTOA);
				temp_Rmt_n_dir[2] = - cos(m_Qf / RTOA)*sin(m_Qn / RTOA);

				//����ϵ�����߽��ٶ�(δ����Ŀ���˶����������߽��ٶ�)
				double dq_n[3] = {0.0};
				dq_n[0] = (-temp_Rmt_n_dir[1] * (m_vnx - 0.0) + temp_Rmt_n_dir[2] * (m_vs - 0.0)) / m_slant_distance_target;
				dq_n[1] = (-temp_Rmt_n_dir[2] * (m_vnx - 0.0) + temp_Rmt_n_dir[0] * (m_vnz - 0.0)) / m_slant_distance_target;
				dq_n[2] = (-temp_Rmt_n_dir[0] * (m_vs - 0.0) + temp_Rmt_n_dir[1] * (m_vnx - 0.0)) / m_slant_distance_target;

				//���߽��ٶȴӵ���ϵת������ϵ
				double dq_b[3] = {0.0};
				dq_b[0] = m_Cnb[0] * dq_n[0] + m_Cnb[1] * dq_n[1] + m_Cnb[2] * dq_n[2];
				dq_b[1] = m_Cnb[3] * dq_n[0] + m_Cnb[4] * dq_n[1] + m_Cnb[5] * dq_n[2];
				dq_b[2] = m_Cnb[6] * dq_n[0] + m_Cnb[7] * dq_n[1] + m_Cnb[8] * dq_n[2];

				//��ת����
				m_dqf = dq_b[2]*cos(m_gama/RTOA) - dq_b[1]*sin(m_gama/RTOA);
				m_dqh = dq_b[2]*sin(m_gama/RTOA) + dq_b[1]*cos(m_gama/RTOA);
				*/
			}
			//����������ֵ����
			//else
			//{
			//}
		//}

		//���߽��ٶ�
		m_dqf = CFlightGlobalFun::Range(m_dqf, 20);
		m_dqh = CFlightGlobalFun::Range(m_dqh, 20);

	}

	//���ٶ��˲�
	m_guide_data_filter[0].inputdata[1] = m_guide_data_filter[0].inputdata[0];
	m_guide_data_filter[0].inputdata[0] = m_dqf;
	CFlightGlobalFun::Tustin_FirstIO(0,1,m_guide_data_filter[0].T,1,
				m_guide_data_filter[0].inputdata,m_guide_data_filter[0].outputdata,m_guide_data_filter[0].Ts);
	m_guide_data_filter[0].outputdata[1] = m_guide_data_filter[0].outputdata[0];
	m_dqf_flt = m_guide_data_filter[0].outputdata[0];

	m_guide_data_filter[1].inputdata[1] = m_guide_data_filter[1].inputdata[0];
	m_guide_data_filter[1].inputdata[0] = m_dqh;
	CFlightGlobalFun::Tustin_FirstIO(0,1,m_guide_data_filter[1].T,1,
				m_guide_data_filter[1].inputdata,m_guide_data_filter[1].outputdata,m_guide_data_filter[1].Ts);
	m_guide_data_filter[1].outputdata[1] = m_guide_data_filter[1].outputdata[0];
	m_dqf_flt = m_guide_data_filter[1].outputdata[0];

	//�����˲���Ϊ�˲���???...
	m_dqf_flt = m_dqf;
	m_dqh_flt = m_dqh;

	//�������ÿ�ܽ�ָ�����
	//��ܽǼ��㣬�⻷Ϊ�����ڻ�Ϊ������������ϵ������ϵ23ת��
	double SinPhih = -(cos(m_Qn/RTOA) * cos(m_Qf/RTOA) * (sin(m_gama/RTOA) * sin(m_zeta/RTOA) * cos(m_psin/RTOA) + cos(m_gama/RTOA) * sin(m_psin/RTOA))
		- sin(m_Qf/RTOA) * sin(m_gama/RTOA) * cos(m_zeta/RTOA)
		- sin(m_Qn/RTOA) * cos(m_Qf/RTOA) * (-sin(m_gama/RTOA) * sin(m_zeta/RTOA) * sin(m_psin/RTOA) + cos(m_gama/RTOA) * cos(m_psin/RTOA)));

	if (fabs(SinPhih) < 1.0)
	{
		m_phih = RTOA * asin(SinPhih);
	} 
	else
	{
		m_phih = RTOA * asin(CFlightGlobalFun::FSign(SinPhih));
	}
	//�������-180~180degͨ�û�����
	double SinPhif = (cos(m_Qn/RTOA) * cos(m_Qf/RTOA) * (-sin(m_zeta/RTOA) * cos(m_psin/RTOA) * cos(m_gama/RTOA) + sin(m_gama/RTOA) * sin(m_psin/RTOA))
		+ sin(m_Qf/RTOA) * cos(m_zeta/RTOA) * cos(m_gama/RTOA)
		- cos(m_Qf/RTOA) * sin(m_Qn/RTOA) * (sin(m_zeta/RTOA) * cos(m_gama/RTOA) * sin(m_psin/RTOA) + sin(m_gama/RTOA) * cos(m_psin/RTOA))) 
		/ CFlightGlobalFun::Nozero_FUN(cos(m_phih/RTOA));

	if (fabs(SinPhif) < 1.0)
	{
		m_phif = RTOA * asin(SinPhif);
	} 
	else
	{
		m_phif = RTOA * asin(CFlightGlobalFun::FSign(SinPhif));
	}
	//�������-90`90degͨ�û�����
	
	//��ܽ��޷�
	m_phif = CFlightGlobalFun::Range(m_phif, 60.0);
	m_phih = CFlightGlobalFun::Range(m_phih, 60.0);
}

//���ݾ�ѹ��������ѹ�߶�
void CMathControlFlightBasic::Calc_BaroHigh()
{
    double exp = (R * L) / (G * M);
    double ratio = m_static_pressure/ P0;
    m_hbaro = (T0 / L) * (1.0f - pow(ratio, exp));
}
//���ݾ�ѹ����ѹ�ƿ��٣�����ٶ���Ч��
void CMathControlFlightBasic::Calc_BaroSpd()
{
	//���θ߶ȡ������ܶȱ�
	double temp_altitude_array[11] = {0,500,1000,1500,2000,2500,3000,3500,4000,4500,5000};
	double temp_rho_ratio_array[11] = {1.0000, 0.9421, 0.8870, 0.8345, 0.7846, 0.7371, 0.6920, 0.6491, 0.6084, 0.5698, 0.5332};
	double temp_rho_ratio = 0.0;
	double temp_dynamic_pressure = m_total_pressure - m_static_pressure;
	//���ݺ��θ߶Ȼ�ÿ����ܶȱ�
	temp_rho_ratio = CFlightGlobalFun::LAQL1(11, temp_altitude_array, temp_rho_ratio_array, m_hz);
	m_Vbaro = sqrt(2.0f * temp_dynamic_pressure / (RHO0*temp_rho_ratio*ARSPD_RATIO));

	//���غ��жϿ��ٹ�������Ч��
	if(m_st_control_flag.flag_control_set == 1)
	{
		//��Ͳ���ٶȣ�һ�����20m/s���������˳��10m/s����ˣ���ʵ����һ�����10m/s��������Ч
		if(m_Vbaro < 10.0)
		{
			m_v_air = m_v;//���ٹ���Ч
		}
		//������Ч
		else
		{
			//10s�ڣ����Ƕ�̬��������е�����Ч�����ο���
			if(flight_time < 10.0)
			{
				m_v_air = m_Vbaro;	
			}
			//֮�󣬵�����Ч
			else
			{
				//���ٺ͵��ٲ����10ʱ����Ϊ���ٹ����ܱ�����������Ϊ���٣������ڵ���+10�����-10
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
		//���ǰ��ʹ��ʵ�ʲ���ֵ
		m_v_air = m_Vbaro;
	}

	m_dynamic_pressure = 0.5 * temp_rho_ratio * 1.225 * m_v_air * m_v_air;
}


//���ԡ�������ϸ߶ȡ������㷨
void CMathControlFlightBasic::Calc_Hz()
{
	//��δ�������ǡ��߶ȱ��л�����

	//������ ��ѹ�������ϸ߶�
	//m_baroalt_status
	//m_hbaro
	//������  ���ߵ��������ϸ߶�

	//��������ϸ߶ȣ�������???...
	//���������Ϊ����δ��λ��ʹ�ù��Լ���߶�
	if (flight_time <= 10.0)
	{
		m_vs += m_au * STEP_5ms;
		m_hz += m_vs * STEP_5ms;
		m_st_control_flag.flag_initial_hz = false;
	}
	//��Ϊ�����Ѷ�λ������ϵ�������߶�����
	else
	{
		//��ϸ߶ȳ�ʼ����֮��ʹ�����Ǹ߶ȣ�������ϸ߶ȡ�����
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
	//����ת���ؽ�: ��ǰ�㵽Ŀ���
	if (m_st_control_flag.flag_launch_turn)		
	{
		m_st_control_flag.flag_launch_turn = false;//�������ת���ʶ
		m_st_control_time.time_launch_turn_ok = flight_time;
		
		//�Ե�ǰ���ؽ�����ϵ
		m_longitude_A = m_longitude;
		m_latitude_A = m_latitude;
		m_route_mode = m_st_way_point[m_num_way_point_target].route_mode;//��������
		m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;//Ŀ���ٶ�
		m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;	//ת��Ƕ�
		m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;	//ת��뾶
		m_accept_radius = m_st_way_point[m_num_way_point_target].accept_radius;//������հ뾶
		
		//����ת����ɣ���ǰ����Ŀ��㽨���ߣ����㺽�η�λ��
		double distance_AB = 0.0;
		CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
			m_longitude_B, m_latitude_B, 
			&distance_AB, &m_A);
		m_A = - m_A;
		m_A = CFlightGlobalFun::Adjust(m_A, 180.0);
	}
	//��·ת���ؽ�: ǰһ���㵽��ǰ���㽨������
	else	
	{
		m_st_control_flag.flag_waypoint_turn = false;//������������ת���ʶ
		
		//Ŀ�꺽·���л������½�������
		m_num_way_point_target++;

		//���������ơ��л���Ŀ���Ϊ���һ�����㣬�Լ�֮���޺��㡱�����л��߼�����Ϊ���ֵ�ǰ���߼�������
		//���л���B��Ϊ���һ������ʱ��C��ȡΪAB���߷���֮��10km��ͬ�£�
		//���л���A��Ϊ���һ������ʱ��A��ȡΪ���л�ǰB�㡱��B��Ϊ���л�ǰC�㡱��C��ΪAB���߷���֮��10km��
		//���л�ǰA����Ϊ���㺽�㣬ͬ��		
		if(m_num_way_point_target >= m_num_way_point - 1)
		{
			m_num_way_point_target = m_num_way_point - 1;

			m_longitude_A = m_longitude_B;
			m_latitude_A = m_latitude_B;
			m_longitude_B = m_longitude_C;
			m_latitude_B = m_latitude_C;

			//��BΪ��ʼ�㣬����AB���߷�λ�� �� 10km���룬����C������
			//�ؽ����Ʒ�λ�ǣ����º���
			double distance_AB = 0.0;
			double azimuth_AB = 0.0;
			CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
				m_longitude_B, m_latitude_B, 
				&distance_AB, &azimuth_AB);
			//����C�����꣬��λ��ȡֵ��Χ0~360deg������Ϊ�㣬��ƫ��Ϊ��
			CFlightGlobalFun::Vincenty_Forward(m_longitude_B,m_latitude_B,
										distance_AB,azimuth_AB,
										&m_longitude_C, &m_latitude_C);
		}
		else
		{
			m_longitude_A = m_st_way_point[m_num_way_point_target - 1].longitude;
			m_latitude_A = m_st_way_point[m_num_way_point_target - 1].latitude;
			m_longitude_B = m_st_way_point[m_num_way_point_target].longitude;
			m_latitude_B = m_st_way_point[m_num_way_point_target].latitude;
		
			m_longitude_C = m_st_way_point[m_num_way_point_target + 1].longitude;
			m_latitude_C = m_st_way_point[m_num_way_point_target + 1].latitude;		
		}
		//Ŀ��ת����Ϣ����
		m_route_mode = m_st_way_point[m_num_way_point_target].route_mode;
		m_turn_angle = m_st_way_point[m_num_way_point_target].turn_angle;
		m_turn_radius = m_st_way_point[m_num_way_point_target].turn_radius;
		m_accept_radius = m_st_way_point[m_num_way_point_target].accept_radius;//������հ뾶
		m_target_velocity = m_st_way_point[m_num_way_point_target].velocity;
		//�����л������½������ߣ����㷽λ��
		double distance_AB = 0.0;
		CFlightGlobalFun::Tomas(m_longitude_A, m_latitude_A, 
			m_longitude_B, m_latitude_B, 
			&distance_AB, &m_A);
		m_A = - m_A;
		m_A = CFlightGlobalFun::Adjust(m_A, 180.0);	
	}
	
	//����Ŀ��㣬��С���ܵ���ʱ�䡢ʱ��
	double angle_PB = 0.0;
	CFlightGlobalFun::Tomas(m_longitude, m_latitude,
		m_longitude_B, m_latitude_B,
		&m_distance_BP, &angle_PB);
	double temp_v = m_v_average_10s + 10.0;
	if(temp_v >= VEL_COMMAND_MAX_LIMIT)
	{
		temp_v = VEL_COMMAND_MAX_LIMIT;
	}
	else if(temp_v <= VEL_COMMAND_MIN_LIMIT)
	{
		temp_v = VEL_COMMAND_MIN_LIMIT;
	}
	double distance_to_go = fabs(m_distance_BP - m_turn_radius * tan(fabs(m_turn_angle / RTOA) / 2.0));//���ɾ���
	double min_time = distance_to_go / temp_v;//����ʱ��
	m_st_control_time.time_arrive_minimum = flight_time + min_time;//����ת����ɺ����¼������ߣ�������һ�����㵽��ʱ��
	

	//����
	m_st_control_flag.flag_flightime_ctrl = m_st_way_point[m_num_way_point_target].if_groundspeed_ctrl;
	m_st_control_flag.flag_velocity_control = m_st_way_point[m_num_way_point_target].if_flightime_ctrl;

	m_st_control_flag.flag_heading_hold = m_st_way_point[m_num_way_point_target].if_heading_hold;  	//[ָ����б�ʶ��1��Ч��0��Ч��]
	m_st_control_flag.flag_turndir_set = m_st_way_point[m_num_way_point_target].if_turndir_set;	//[����ת�䣺��ת����ת��ʶ��]
	m_st_control_flag.flag_prepare_hover = m_st_way_point[m_num_way_point_target].if_prepare_hover;	//[Ԥ������ʶ��1��Ч��0��Ч��]

	m_st_control_flag.flag_relativehigh_ctrl = m_st_way_point[m_num_way_point_target].if_relativehigh_ctrl;//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	m_st_control_flag.flag_attackangle_ctrl = m_st_way_point[m_num_way_point_target].if_attackangle_ctrl;	//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]		
	
	if(m_st_control_flag.flag_heading_hold == true)
	{
		m_outtrack_angle = m_st_way_point[m_num_way_point_target].outtrack_angle;
	}

	if(m_st_control_flag.flag_prepare_hover == true)
	{
		m_hover_round = m_st_way_point[m_num_way_point_target].hover_round;
	}
	//�������� Ϊ ���պ���
	//if(m_route_mode == 6)
	//{
	//	m_target_height_ground = m_st_way_point[m_num_way_point_target].recycle_ground_hight;
	//}
}

void CMathControlFlightBasic::Control_Turn()
{
	// 1.��������ת�������жϣ�ֻ����1�Σ������������8s
	//if(flight_time >= (m_st_control_time.time_separate_booster + 10.0) 
	//	&& (!m_st_control_flag.flag_launch_turn_set))
	//����߶ȿ��ƺ���������ת�䣬���� ���򡢲�����ϻ���
	if((flight_time >= m_st_control_time.time_altitude_control + 5.0) 
		&& (!m_st_control_flag.flag_launch_turn_set))		
	{
		m_st_control_flag.flag_launch_turn_set = true;//��������ת�䣬����һ�Σ���ʶ�������
		m_st_control_flag.flag_launch_turn = true;  //����ת�����ʱ����false
		m_st_control_time.time_launch_turn = flight_time;
			
		m_turn_angle = m_psicn - m_A;
		m_turn_angle = CFlightGlobalFun::Adjust(m_turn_angle, 180.0);//20260408
		
		//ת��Ƕȴ���5deg����ʼ����ת�䣬ת����ɺ����½�������
		if(fabs(m_turn_angle) > 5.0)		//ȥ�����䷽λ��Ŀ��нǴ���30�������������
		{
			m_st_control_time.time_turn_in_start = flight_time;
			m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start 
				+ (m_gama_turn_nominal - m_gama) / ROLL_RATE_COMMAND;	//����  - m_gama������ָ��ͻ��
			m_st_control_time.time_turn_out_start = MAX_TIME;
			m_st_control_time.time_turn_out_end = MAX_TIME;
			m_st_control_flag.flag_turn_out_set = false;
			m_psit_t_turn_in = m_psit;
		}
		//����������ת�䣬������Ŀ����к���
		else
		{
			Coord_Rebuild();//��������ת�䣬ֱ�Ӱ����º��߷�
		}
		m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
	}
	
	// 2.��·ת�������жϣ�Ŀ��㲻�����һ�����㣬δ����ĩ�Ƶ���δ����ת������У������������8s����
	if ((m_num_way_point_target < (m_num_way_point - 1)) 
		&& (!m_st_control_flag.flag_combat_status)
		&& (flight_time > m_st_control_time.time_turn_in_minimum)	//ת�����10s��������һ�κ����л�
		&& (flight_time > (m_st_control_time.time_separate_booster + 8.0))
		&& (!m_st_control_flag.flag_waypoint_turn))//�����ں���ת�����
	{
		//����������500ms�ж�һ�Σ�������֡��Ŀ��������
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

		//��С����ʱ�俪ʼ�ж�
		if(flight_time < m_st_control_time.time_arrive_minimum)
		{
			count_away = 0;	
		}

		//��ת����ǰ���룬�Ƕȹ��ɲ���
		//m_distance_turn_in_compensate = 1.25 * fabs(m_gama_turn_nominal * m_v / ROLL_RATE_COMMAND);
		m_distance_turn_in_compensate = 1.0 * fabs(m_gama_turn_nominal * m_v / ROLL_RATE_COMMAND);
		m_distance_turn_in_compensate = CFlightGlobalFun::Range(m_distance_turn_in_compensate, 150.0);
		//��ת����ǰ���룬���β��� + �Ƕȹ��ɲ���
		double temp_tan_psi = 1.0;
		if(fabs(m_turn_angle) > 150.0)
		{
			temp_tan_psi = 3.732;//��tan(150.0/2)������ת����ǰ����̫��
		}
		else
		{
			temp_tan_psi = tan(fabs(m_turn_angle / RTOA) / 2.0);
		}
		m_distance_turn_in =  m_turn_radius * temp_tan_psi + m_distance_turn_in_compensate;

		//20ms�ж�һ�Σ���Ŀ���� ��������С�� ת����ǰ����
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

		//������������С��ת����ǰ���룬���Ŀ��㣬��ʼ����ת���ж�
		if ((count_away>=3)	
			|| (count_sd_in>=3))			
		{
			count_away = 0;
			count_sd_in = 0;
			m_st_control_flag.flag_waypoint_turn = true;//��ʼ����ת��
			
			//ת��Ƕȴ���5deg����ʼ����ת�䣬�����½�������
			if (fabs(m_turn_angle) > 5.0)
			{
				m_st_control_time.time_turn_in_start = flight_time;
				m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start 
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
				m_st_control_time.time_turn_out_start = MAX_TIME;
				m_st_control_time.time_turn_out_end = MAX_TIME;
				m_st_control_flag.flag_turn_out_set = false;

				//���Բ�Ĳο��㣬��ǰ������
				m_x_coordinate_turn = - m_turn_radius * sin (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle) 
					- m_distance_turn_in_compensate * cos(m_psicn / RTOA);
				m_z_coordinate_turn = - m_turn_radius * cos (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle)
					+ m_distance_turn_in_compensate * sin(m_psicn / RTOA);
	
				m_psit_t_turn_in = m_psit;
			}
			//�����к���ת�䣬�л���һĿ��㣬������Ŀ����к���
			else
			{
				Coord_Rebuild(); 	//���ٺ���ת�������	
			}
			m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
		}
	}

	// 3.ת������ж�: ��������ɽ�������ת���ѹ��ɵ�Ŀ��ֵ��ʱ������ת��򺽼�ת�䣬��ʼ�жϳ���
	if ((m_st_control_flag.flag_waypoint_turn || m_st_control_flag.flag_launch_turn) 
		&& (flight_time > m_st_control_time.time_turn_in_end))
	{
		//������̺��������
		double delta_psi = m_g * log(fabs(cos(m_gama_turn_nominal / RTOA))) / (m_v * ROLL_RATE_COMMAND) 
			* RTOA * CFlightGlobalFun::FSign(m_turn_angle);

		//��ǰ�㵽��һĿ��㷽λ�ǣ���ƫ��Ϊ������Χ-PI~PI rad
		double alpha_PC = 0.0;
		double distance_PC = 0.0;
		CFlightGlobalFun::Tomas(m_longitude, m_latitude,
			m_longitude_C, m_latitude_C,
			&distance_PC, &alpha_PC);
		alpha_PC = alpha_PC / RTOA;
		alpha_PC = CFlightGlobalFun::Adjust(alpha_PC, PI);
		
		//����ƫ��(��ƫ��Ϊ��) + ������̺����仯��(��ƫ��Ϊ��) + ��һĿ��㺽��(��ƫ��Ϊ��)
		double psic2 = m_psicn / RTOA + 2.0 * delta_psi + alpha_PC; 
		psic2 = CFlightGlobalFun::Adjust(psic2, PI);
		double psid1 = - psic2 * (m_turn_angle / RTOA);//�����෴�����ֵΪ��

		//�����ǹ������ж�
		if((psid1 >= 0)
			&&(fabs(psic2) <= (PI / 2.0)))
		{
			count_turn_out++;
		}
		else
		{
			count_turn_out = 0;
		}

		//ת����ɴ���
		if(((count_turn_out >= 3)||(Judge_Turn_Error()))
			&& (!m_st_control_flag.flag_turn_out_set))
		{
			count_turn_out = 0;

			//����ת�䣬��ת�俪ʼ������ʱ��
			if (m_st_control_flag.flag_launch_turn)
			{
				m_st_control_time.time_turn_out_start = flight_time;	//��ת�俪ʼʱ��
				m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;		//��ת�����ʱ��
				m_st_control_flag.flag_turn_out_set = true;
			}
			//����ת�䣬Ŀ��㲻�����һ������
			else if(m_num_way_point_target <= (m_num_way_point - 2))
			{
				//����һ����������룬���Կ�չһ�κ���ת�䣬��ʼ��ת�����
				if (distance_PC > (m_st_way_point[m_num_way_point_target + 1].turn_radius 
					* tan(fabs(m_st_way_point[m_num_way_point_target + 1].turn_angle / RTOA) / 2.0) 
					+ 120.0)) 
				{				
					//��ת�Ƕ�Ӧ��ת����ٶ�
					double Omega = fabs(m_g * tan(m_gama / RTOA) / m_v);
					//�ӳ�ʱ�䣬���㷽���޸�
					//double time_turn_out_delay = fabs(acos(1 - fabs((m_sz - m_turn_radius) * Omega / m_v)))/CFlightGlobalFun::Nozero_FUN(Omega) - 3.5;
					double time_turn_out_delay = fabs(acos(1 - fabs((m_sz) * Omega / m_v)))/CFlightGlobalFun::Nozero_FUN(Omega) - 2.0;
					//ʵ��ת��뾶ƫС����ǰ����ת�䣬������ʼ��ƫ����
					//if((m_sz - m_turn_radius) <= 0.0)
					if( m_sz <= 0.0)
					{
						time_turn_out_delay = 0.0;
					}
					//ʵ��ת��뾶ƫ����Ҫ��תһ��������ܵ�Ŀ�꺽��
					else
					{
						if(time_turn_out_delay < 0.0)
							time_turn_out_delay = 0.0;
						if(time_turn_out_delay > 4.0)
							time_turn_out_delay = 4.0;
					}
					m_st_control_time.time_turn_out_start = flight_time + time_turn_out_delay;//����ת�䣬��ת�俪ʼ
					m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
						+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
					m_st_control_flag.flag_turn_out_set = true;
				}
				//�����Կ�չһ�κ���ת�䣬ֱ���л���һ������ΪĿ��㣬���º���ת��
				else
				{
					Coord_Rebuild();
					m_st_control_flag.flag_waypoint_turn = true;
					
					//����һ����ת��Ƕȡ��롰��ǰ�����ת��Ƕȡ������෴������ת��Բ�ģ����򱣳�ǰһ��ת��Բ�ģ�ֻ����ת��Ƕȡ�ת��뾶
					if((m_turn_angle * m_st_way_point[m_num_way_point_target - 1].turn_angle) < 0.0)
					{
						m_st_control_time.time_turn_in_start = flight_time;
						m_gama_turn_nominal = atan(m_v * m_v / m_turn_radius / m_g) * RTOA;
						m_gama_turn_nominal = CFlightGlobalFun::Range(m_gama_turn_nominal, 60.0);
						double delta_t = (fabs(m_gama) + fabs(m_gama_turn_nominal))/ROLL_RATE_COMMAND;
						m_st_control_time.time_turn_in_end = m_st_control_time.time_turn_in_start + delta_t;
						m_st_control_time.time_turn_out_start = MAX_TIME;
						m_st_control_time.time_turn_out_end = MAX_TIME;
						m_st_control_flag.flag_turn_out_set = false;//��������ʶ

						//�Ե�ǰ��Ϊת����뿪ʼ�㣬�ٶȷ�����Ϊ��BC����(�����º��AB)������ͬ����ʼ��CD����(�����º��BC)���ɣ���ʼ��һת�����
						//�������ù��ط�����ת��뾶��ȷ��ת��Բ��Ϊ����ο��㣬���춫Ϊ�ᣬ���㵱ǰ��λ������
						m_x_coordinate_turn = - m_turn_radius * sin (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle); 
						m_z_coordinate_turn = - m_turn_radius * cos (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle);
						//m_x_coordinate_turn = m_turn_radius * sin (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle); 
						//m_z_coordinate_turn = m_turn_radius * cos (m_psicn / RTOA) * CFlightGlobalFun::FSign(m_turn_angle);
					}

					//���ݹ�ת�ǹ���ת����ٶȣ��������ʱ�䣬�Ľ�10.0sʱ��

					//��Ϊ10s�ڴ���ת����̣���ʱ���ڣ������������ж�
					m_st_control_time.time_turn_in_minimum = flight_time + 10.0;
					m_psit_t_turn_in = m_psit;
				}
			}
			//����ת�䣬Ŀ���Ϊ���һ������
			else
			{
				m_st_control_time.time_turn_out_start = flight_time;
				m_st_control_time.time_turn_out_end = m_st_control_time.time_turn_out_start
					+ m_gama_turn_nominal / ROLL_RATE_COMMAND;
				m_st_control_flag.flag_turn_out_set = true;
			}
		}
		
		//������ɣ��������ת�䡢����ת���ʶ���л���һ����
		if (flight_time >= m_st_control_time.time_turn_out_end
			&& m_st_control_flag.flag_turn_out_set)
		{
			Coord_Rebuild();
		}
	}

	// 4.���������Թ��̽��������½������ߣ���ʼ��ƫ���ƣ�
	// ˵����Ŀ�������Ϊ�������㣬�Ƚ������⵼�����߶Ȳ�����������������������𣬸߶Ȳ�С��0m֮��0.1s������Ѳ�������̽�����
	if ((flight_time > m_st_control_time.time_combat_dive_ok) && (!m_st_control_flag.flag_combat_dive_sidectrl))
	{
		m_st_control_flag.flag_combat_dive_sidectrl = true;
		m_st_control_time.time_combat_dive_sidectrl = flight_time;
		
		//�����к���ת�䣬�л���һĿ��㣬������Ŀ����к���
		//�����滮Լ���������λ�� ��ǰһ��·��(�����׼����)�� �� �����յ㡱 ����֮��
		Coord_Rebuild();
	}
}

//˫ƽ���Ƶ�
void CMathControlFlightBasic::Calc_Dualplane_Guidance()
{
	//�Ƶ�����ѡ����BTT�Ƶ���STT�Ƶ��л�
	if((flight_time > m_st_control_time.time_launch_turn_ok)&&(step_dualplane_guidance == 0))
	{
		//����ת����ɣ����½������ߣ����롰Ѳ���� BTT�Ƶ���
		step_dualplane_guidance = 1;
	}
	if((flight_time > m_st_control_time.time_combat_status)&&(step_dualplane_guidance == 1))
	{
		//����ĩ�Ƶ����̣����롰ĩ�Ƶ� BTT�Ƶ���
		step_dualplane_guidance = 2;
		//��¼����ĩ�Ƶ�ʱ�̣���ת��
		m_gama_target_t_combat = m_gama;
	}
	if((fabs(m_gama_target_t_combat) < 8.0)&&(step_dualplane_guidance == 2))
	{
		//ĩ�Ƶ�ʱ����ת�Ǻ�С����������ؽ�С��ת��Ϊ��ĩ�Ƶ� STT�Ƶ���
		step_dualplane_guidance = 3;

		//˵��������̬���Խϲ�����ͣ���ĩ�Ƶ� STT�Ƶ���Ч���ܲҲ����BTT�Ƶ�
		step_dualplane_guidance = 2;
	}
	if((flight_time > m_st_control_time.time_combat_dive_ok)
		&& ( (step_dualplane_guidance == 2) || (step_dualplane_guidance == 3) ))
	{
		//��ĩ�Ƶ��׶ν���������������̣����롰�������Ǹ��٣��������Ϊ�㡱
		//˵�����Ѿ�ת"Ѳ�� BTT�Ƶ�"
		step_dualplane_guidance = 4;
		//step_dualplane_guidance = 2;
	}
	if((flight_time > m_st_control_time.time_launch_turn_ok + 20.0)&&(step_dualplane_guidance == 4))
	{
		//��ĩ�Ƶ��������������������תѲ�� BTT�Ƶ���ͬ��Ѳ���� BTT�Ƶ���
		//step_dualplane_guidance = 5;
		step_dualplane_guidance = 2;
	}

	//�Ƶ�ϵ��������
	double temp_ny_command = 0.0;
	temp_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.00*cos(m_zeta / RTOA);
	temp_ny_command = CFlightGlobalFun::Range2(temp_ny_command, 2.0, -0.5);
	double temp_nz_command = 0.0;
	temp_nz_command = -3.0 * m_v * m_dqh / RTOA / m_g;
	temp_nz_command = CFlightGlobalFun::Range(temp_nz_command, 1.5);
	double temp_nyz_command = 0.0;
	temp_nyz_command = sqrt(temp_ny_command*temp_ny_command + temp_nz_command*temp_nz_command) * CFlightGlobalFun::FSign(temp_ny_command);
	temp_nyz_command = CFlightGlobalFun::Range2(temp_nyz_command, 2.0, -0.5);
		
	//�����Ƶ�ָ��
	if(step_dualplane_guidance == 2)
	{
		//��ĩ�Ƶ� BTT�Ƶ���
		if(temp_nyz_command < 0.01)
		{
			//��Y���Z����ؾ���С����ת�Ǳ���ǰһֵ֡��������
			m_gama_command = m_gama_command_record;
		}
		else
		{
			//�����ת��
			m_gama_command = RTOA*asin(temp_nz_command / temp_nyz_command);

			//��ת���޷�����
			if(fabs(m_gama_command) > ROLL_COMMAND_DYNMIC_LIMIT)
			{
				m_gama_command = ROLL_COMMAND_DYNMIC_LIMIT * CFlightGlobalFun::FSign(m_gama_command);
				
				//��֤ �������ָ����˼��㡰���������ָ�WQ
				temp_nyz_command = temp_ny_command/cos(m_gama_command / RTOA);
				temp_nyz_command = CFlightGlobalFun::Range2(temp_nyz_command, 2.0, -0.5);
			}
			//������ع��㴦��
			if(fabs(temp_ny_command) < 0.25)
			{
				m_gama_command = CFlightGlobalFun::Range2(m_gama_command, 5.0, -5.0);
			}
		}

		m_ny_command = temp_nyz_command;
		m_nz_command = 0.0;
	}
	else if(step_dualplane_guidance == 3)
	{
		//��ĩ�Ƶ� STT�Ƶ���
		m_gama_command = 0.0;
		m_ny_command = temp_ny_command;
		m_nz_command = temp_nz_command;
	}
	else
	{
		m_ny_command = 1.0;
		m_nz_command = 0.0;
		m_gama_command = 0.0;
	}
	m_gama_command_record = m_gama_command;
	
}

bool CMathControlFlightBasic::Judge_Turn_Error()
{
	//ת������У�ת��Ƕȹ��󣬼����� ������ת��Ƕȵ���15deg������
	if((flight_time < m_st_control_time.time_turn_out_start) 
		&& (flight_time > m_st_control_time.time_turn_in_end))
	{
		double delta_psit = m_psit_t_turn_in - m_psit;//��λ��ƫ��
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

// ����߶Ȼ��� �� �˳��߶Ȼ�������
void CMathControlFlightBasic::Control_Altitude_Change()
{
	//�߶Ȼ����ӳ�ʱ����㣬�ȴ������������Ԥ��5m/s�ٶ�
	double altitude_change_delay = 0.0;
	
	//���һ��ָ��߶ȱ仯��ǰһ��Ŀ��߶� �� ��Ŀ��߶� �����1m
	double target_height_update = m_st_way_point[m_num_way_point_target].height;	//����������������ָ���ڴ����Ӹ߲�
	double delta_h_target = target_height_update - m_target_height;
	if (fabs(delta_h_target) > 1.0)
	{
		count_altitude_change_enable++;
	}
	else
	{
		count_altitude_change_enable = 0;
	}
	
	//���������ʼ���������� �� δ�����ʼ�߶ȿ��ƣ���Ϊ��ʼ����
	//��Ϊ step_altitude_change_lauch
	//if((flight_time >  m_st_control_time.time_missile_takeoff)&&(flight_time < m_st_control_time.time_altitude_control))
	//{
	//	count_altitude_change_lauch_enable++;
	//}
	//else
	//{
	//	count_altitude_change_lauch_enable = 0;
	//}
	
	//����������ʹ�ôﲻ���ﲻ������ �� ����Ѳ����Ŀ��:ʹ�� �߶��½��ٶ����ӣ������½����߶��½���ʹ�ø�������ݸ���������������������������
	//�о�: ����Ѳ��������ģʽʱ����������С�쳣��ֱ�ӽ��������߶Ȼ���
	if( (m_st_control_flag.flag_missile_takeoff == true)
		&&((m_st_control_flag.flag_alltitude_climb == true) || (m_st_control_flag.flag_alltitude_change == false)))
	{
		if(!((time_tick - 1) % 1000))
		{
			m_total_energy = 0.5*m_v_air*m_v_air + 9.8*m_hz;
			//����С��-1.0m/s����1sʱ���߶Ȳ�С��-1m��������С��10 J
			if(m_total_energy < m_total_energy_pre - 10.0)
			{
				count_altitude_change_energy_enable++;
			}
			else
			{
				count_altitude_change_energy_enable = 0;
			}
		}
	}

	//����ģ��������󸩳����𣬸��ݵ�ǰ�����ǣ������ǣ� ���ɵ� ���������ǣ�����ʱ��ϳ�
	//�����һ�ϲ����Զ�ʶ��Ϊ���������Ǻ����滮Լ������������֮��Ϊ���յ㣬�һ��յ�߶ȴ�����������߶�100.0m���ϣ�
	//m_st_control_flag.flag_combat_dive_pullup
	
	//�߶Ȼ�����ʼ�жϣ�ת������� �� �������һ�����㣬�����߶Ȼ���
	if(( ((m_num_way_point_target <= m_num_way_point - 2)&&(m_missile_flight_mode == 0xAA)) 	//��ʽ����ģʽ��Ŀ��㲻�����һ�����㣬��δ����ĩ�Ƶ�
		    ||  ((m_num_way_point_target != m_num_way_point - 2)&&(m_missile_flight_mode != 0xAA)) )//���Է���ģʽ��Ŀ��㲻�Ǻ����ڶ������㣬��δ���롰�𹥡�ĩ�Ƶ�
		&& (!m_st_control_flag.flag_waypoint_turn)	//����ת����̣�����ת����̲����и߶Ȼ���
		&& (!m_st_control_flag.flag_launch_turn)	//����ת����̣�ת����̲����и߶Ȼ���
		&& ( (count_altitude_change_enable >= 3)  || (count_altitude_change_energy_enable >= 5)  ) )// || (step_altitude_change_lauch == 1)	//���������жϣ������߶ȸ���
	{
		//�����ʶ�����»����߶�
		if(count_altitude_change_enable >= 3)
		{
			count_altitude_change_enable = 0;
			//ָ��߶Ȳ��ָ��߶� �� ǰһ��ָ��߶�
			m_target_height = target_height_update;//�������£���ָ��߶ȴ���ǰһָ��߶�
		}
		
		//�����ʶ����������½�
		if(count_altitude_change_energy_enable >= 3)
		{
			count_altitude_change_energy_enable = 0;
			delta_h_target = m_target_height - m_hz;
		}

		//�߶��»�
		//else if(delta_h_target > 0.0)
		if(delta_h_target < 0.0)
		{
			altitude_change_delay = 2.0;
			m_st_control_flag.flag_alltitude_climb = false;
			m_st_control_flag.flag_alltitude_decline = true;//�»�
		}
		//�߶�����
		else
		{
			altitude_change_delay = 2.0;
			m_st_control_flag.flag_alltitude_climb = true;//����
			m_st_control_flag.flag_alltitude_decline = false;
		}
		
		//�����л����߶Ȼ���
		count_altitude_change++;
		//���¸߶Ȼ�����ʶ
		m_st_control_flag.flag_alltitude_change = true;//�߶Ȼ���
		//�߶Ȼ�����ʼʱ��
		m_st_control_time.time_altitude_change_start = flight_time + altitude_change_delay;//�߶Ȼ�����ʼʱ��
		m_st_control_time.time_altitude_change_end = MAX_TIME;
	}

	//�߶Ȼ�������ʱ���ж�
	//if((m_st_control_flag.flag_alltitude_change)&&(flight_time > m_st_control_time.time_altitude_change_start))
	if(m_st_control_flag.flag_alltitude_change)
	{
		//���ﵽĿ��߶�
		if( ((m_st_control_flag.flag_alltitude_climb == true)&&(m_hz - m_target_height > -20.0))
			|| ((m_st_control_flag.flag_alltitude_decline == true)&&(m_hz - m_target_height < 20.0)) )
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
			m_st_control_time.time_altitude_change_end = flight_time + altitude_change_delay;

			//ȡ���߶Ȼ���
			m_st_control_flag.flag_alltitude_change = false;
			m_st_control_flag.flag_alltitude_climb = false;
			m_st_control_flag.flag_alltitude_decline = false;
		}
	}
}
/*
void CMathControlFlightBasic::Monitor_Data()
{
	extern CSimMonitor sim_monitor;
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Mark_Data_In_Flag(ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(flight_time,"time",ENUM_FILE_CONTROL1);//ʱ�� 1
		sim_monitor.Get_Variable(m_A,"A",ENUM_FILE_CONTROL1);//��λ�ǣ���������ʱ���£�ת��ǶȺ�С �� ת�����
		sim_monitor.Get_Variable(m_hz,"hz",ENUM_FILE_CONTROL1);//��ϸ߶�
		sim_monitor.Get_Variable(m_vs,"vs",ENUM_FILE_CONTROL1);//��ϴ���
		//sim_monitor.Get_Variable(m_ny,"ny",ENUM_FILE_CONTROL1);//����ϵY������� 5
		sim_monitor.Get_Variable(m_au,"ny",ENUM_FILE_CONTROL1);//����ϵY������� 5
		//sim_monitor.Get_Variable(m_nz,"nz",ENUM_FILE_CONTROL1);//����ϵZ�������
		sim_monitor.Get_Variable(m_anz,"nz",ENUM_FILE_CONTROL1);//����ϵZ�������
		sim_monitor.Get_Variable(m_sz,"sz",ENUM_FILE_CONTROL1);//��ƫ
		sim_monitor.Get_Variable(m_vnz,"vz",ENUM_FILE_CONTROL1);//�����ٶ�
		sim_monitor.Get_Variable(m_dqf,"dqf",ENUM_FILE_CONTROL1);//�������߽��ٶ�
		sim_monitor.Get_Variable(m_dqh,"dqh",ENUM_FILE_CONTROL1);//�������߽��ٶ� 10
		
		sim_monitor.Get_Variable(m_longitude_B,"LongT",ENUM_FILE_CONTROL1);//��ǰ���Σ�Ŀ��㾭�ȡ�γ�ȣ�11
		sim_monitor.Get_Variable(m_latitude_B,"LatT",ENUM_FILE_CONTROL1);//
		//sim_monitor.Get_Variable(m_distance_target,"DistanceTarget",ENUM_FILE_CONTROL1);//��������
		sim_monitor.Get_Variable(m_distance_BP,"DistanceTarget",ENUM_FILE_CONTROL1);//��·Ŀ������
		sim_monitor.Get_Variable(m_st_control_time.time_control,"Tqk",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_st_control_time.time_separate_booster,"Tfl",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_st_control_time.time_missile_takeoff,"Tdyzk",ENUM_FILE_CONTROL1);//�滻time_launch_missile_wing
		sim_monitor.Get_Variable(m_st_control_time.time_altitude_control,"Tfdh",ENUM_FILE_CONTROL1);//�滻time_engine_start
		sim_monitor.Get_Variable(m_st_control_time.time_combat_status,"Tzd",ENUM_FILE_CONTROL1);///18 ĩ�Ƶ�ʱ��
		
		sim_monitor.Get_Variable(m_st_control_time.time_turn_in_start,"Ts0",ENUM_FILE_CONTROL1);//19 ����������룬��ʼʱ��
		sim_monitor.Get_Variable(m_st_control_time.time_turn_in_end,"Ts1",ENUM_FILE_CONTROL1);///20  ����������룬����ʱ��
		sim_monitor.Get_Variable(m_st_control_time.time_turn_out_start,"Ts2",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_st_control_time.time_turn_out_end,"Ts3",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_st_control_time.time_altitude_change_start,"Th0",ENUM_FILE_CONTROL1);//23 �߶Ȼ�����ʼʱ��
		sim_monitor.Get_Variable(m_st_control_time.time_altitude_change_end,"Th1",ENUM_FILE_CONTROL1);///24 �߶Ȼ�������ʱ��
		
		sim_monitor.Get_Variable(m_st_control_flag.flag_launch_turn,"FlagLaunchTurn",ENUM_FILE_CONTROL1);//������ 1
		sim_monitor.Get_Variable(m_st_control_flag.flag_waypoint_turn,"FlagWaypointTurn",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_st_control_flag.flag_alltitude_change,"FlagAlltitudeChange",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(count_altitude_change,"countAltitudeChange",ENUM_FILE_CONTROL1);///������ 4
		sim_monitor.Get_Variable((int)m_state_rpm,"state_rpm",ENUM_FILE_CONTROL1);//������״̬ת�� rpm
	}
}*/
