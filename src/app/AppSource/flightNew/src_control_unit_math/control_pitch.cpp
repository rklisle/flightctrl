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
	//p_st_debug_monitor = NULL;
	
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
	m_ugf_record = 0.0;
	m_unwif = 0.0;
	m_urf_zd = 0.0;

	m_mass = 0.0;
	m_v = 0.0;
	m_vs = 0.0;
	m_vs_t_altitude_control = 0.0;
	m_g = 0.0;
	m_zeta = 0.0;
	m_zeta_command = 0.0;
	m_zeta_t_change = 0.0;
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
	m_ny_command_guidance = 0.0;
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
 
	m_time_altitude_change = MAX_TIME;
	m_time_altitude_change_record = MAX_TIME;
	m_time_zeta_change = MAX_TIME;
	m_time_zeta_change_record = MAX_TIME;
	m_time_altitude_change_tocruise = MAX_TIME;
	
	m_time_control = MAX_TIME;
	m_time_altitude_change_start = MAX_TIME;
	m_time_altitude_change_end = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_altitude_control = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
	m_time_combat_dive_pullup = MAX_TIME;
	m_time_v70 = MAX_TIME;
	m_time_v120 = MAX_TIME;
	m_time_v60 = MAX_TIME;
	m_time_v100 = MAX_TIME;
	m_time_v140 = MAX_TIME;
	m_flag_altitude_integral_set = false;

	m_ktheta_lauch_enc = 12.0;//���丩����
	m_ktheta_climb_enc = 8.0;//���������������������
	m_ktheta_hight_enc = 2.0;
	m_ktheta_decline_enc = -4.0;//�»���
	m_alpha_b = 2.0;
}
void CMathControlPitch::Initial()
{
	m_ktheta_lauch_enc = p_st_pitch_control_input->ktheta_lauch_enc;//����װ��������
	m_ktheta_climb_enc = p_st_pitch_control_input->ktheta_climb_enc;//����װ��������
	m_ktheta_hight_enc = p_st_pitch_control_input->ktheta_hight_enc;//����װ��������
	m_ktheta_decline_enc = -4.0;//�»���̬�ǣ�����Լ2deg���������-8deg

	m_high_maneuver_state = 0;//��ʼ��
}
void CMathControlPitch::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
}
void CMathControlPitch::Get_Data()
{
	m_mass = p_st_pitch_control_input->mass;//��������
	m_q = p_st_pitch_control_input->q;	//��ѹ
	m_v = p_st_pitch_control_input->v;	//����
	m_vs = p_st_pitch_control_input->vs;//����
	m_g = p_st_pitch_control_input->g;
	m_zeta = p_st_pitch_control_input->zeta;
	m_wz = p_st_pitch_control_input->wz;
	m_hz = p_st_pitch_control_input->hz;//��ϸ߶�
	m_nby = p_st_pitch_control_input->nby;
	m_ny_command_guidance = p_st_pitch_control_input->ny_command_guidance;
	m_dqf = p_st_pitch_control_input->dqf;
	m_time_control = p_st_pitch_control_input->time_control;
	m_time_separate_booster = p_st_pitch_control_input->time_separate_booster;
	m_time_missile_takeoff = p_st_pitch_control_input->time_missile_takeoff;
	m_time_altitude_control = p_st_pitch_control_input->time_altitude_control;
	m_time_altitude_change_start = p_st_pitch_control_input->time_altitude_change_start;
	m_time_altitude_change_end = p_st_pitch_control_input->time_altitude_change_end;
	m_time_combat_status = p_st_pitch_control_input->time_combat_status;
	m_time_combat_delay = p_st_pitch_control_input->time_combat_delay;
	m_time_combat_dive_pullup = p_st_pitch_control_input->time_combat_dive_pullup;
	m_h_target_in = p_st_pitch_control_input->target_height;
	m_distance_target = p_st_pitch_control_input->distance_target;
	m_distance_target_t_combat = p_st_pitch_control_input->distance_target_t_combat;
	m_gama_command_compensate = p_st_pitch_control_input->gama_command_compensate;
	m_count_altitude_change = p_st_pitch_control_input->count_altitude_change;
}
void CMathControlPitch::Send_Data()
{
	p_st_pitch_control_output->uqkf = m_uqkf;//�ڻ�·ǰ��
	p_st_pitch_control_output->u2f = m_u2f;	//�ڻ�·kp
	p_st_pitch_control_output->u5f = m_u5f;	//�ڻ�·kd
	p_st_pitch_control_output->ugf = m_ugf;//m_ugf;	//���·���߶ȿ���
	p_st_pitch_control_output->urf_zd = m_urf_zd;//���·���Ƶ�
	
	p_st_pitch_control_output->h_command = m_h_command;
	p_st_pitch_control_output->h_rate_command = m_h_rate_command;
	p_st_pitch_control_output->zeta_command = m_zeta_command;
	p_st_pitch_control_output->ny_command = m_ny_command;
}

//˵�������߼����Ӧ�����أ�������ϵ��Ϊ����ƫ��Ϊ��״̬ - ָ������������߼��渺���ء�
void CMathControlPitch::Calc_Data()
{
	Calc_Control_Gain();
	Calc_Control_Commond();

	//�ڻ�·��ǰ���������������������ǰ��
	if (flight_time < m_time_control)
	{
		m_uqkf = 0.0;
	}
	else if(flight_time < m_time_separate_booster)
	{
		m_uqkf = 2.0;//��λdeg
	}
	else if(flight_time < m_time_separate_booster + 2.0)
	{
		m_uqkf = m_k0f*(flight_time - m_time_separate_booster)/2.0 + 2.0*(m_time_separate_booster + 2.0 - flight_time)/2.0;//��λdeg
	}
	//else if(flight_time < m_time_missile_takeoff + 1.0)
	//{
	//	m_uqkf = -2.0;//��λdeg
	//}
	//else if(flight_time < m_time_missile_takeoff + 3.0)
	//{
	//	m_uqkf = m_k0f*(flight_time - m_time_missile_takeoff - 1.0)/2.0 - 2.0*(m_time_missile_takeoff + 3.0 - flight_time)/2.0;
	//}
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{
		m_uqkf = m_k0f;//��λdeg
	}
	//����ĩ�Ƶ��л��أ������ع��ǲ���
	else
	{
		m_uqkf = -6.5;//��λdeg
	}
	//ǰ������̧ͷ���ز�������ɼ�����
	//m_uqkf = m_k0f;//��λdeg
	
	//�ڻ�·����������
	m_u2f = m_k2f * (m_zeta - m_zeta_command);
	if (flight_time < m_time_control)
	{
		m_u2f = 0.0;
	}
	else if(flight_time < m_time_separate_booster + 1.0)
	{
		m_u2f = CFlightGlobalFun::Range(m_u2f, 12.0);
	}
	else
	{
		m_u2f = m_u2f / cos(m_gama_command_compensate / RTOA);//��ת����
		m_u2f = CFlightGlobalFun::Range(m_u2f, 10.0);		
	}	
	//�ڻ�·��΢�ֻ��ڣ�ָ����ٶ�Ϊ��
	m_u5f = m_k5f * m_wz;
	if (flight_time < m_time_control)
	{
		m_u5f = 0.0;
	}

	//���·���߶ȸ���
	m_u3f = m_k3f * (m_hz - m_h_command);
	m_u3f = CFlightGlobalFun::Range(m_u3f, 10.0);	//�߶Ȼ������㣬��ʱȡ��
	//���·�����ٷ���
	m_u7f = m_k7f * (m_vs - m_h_rate_command);
	//���·:�߶Ȼ��ֿ���
	//if((!m_flag_altitude_integral_set)
	//	&&(((flight_time > m_time_altitude_control) && (fabs(m_hz - m_h_command) <= 20.0))//��ʼ�߶ȿ����Ҹ߶�ƫ���С
	//	    ||(flight_time >= (m_time_altitude_control + 20.0))))//��������
	//{
	//	m_flag_altitude_integral_set = true;
	//}
	if (m_flag_altitude_integral_set)
	{
		m_u6f += m_k6f * (m_hz - m_h_command) * STEP_5ms;
		m_u6f = CFlightGlobalFun::Range(m_u6f, 6.0);
	}
	else
	{
		m_u6f = 0.0;
	}

	//���ݷ��н׶Σ�ȷ���߶ȿ���
	//�������ǰ���޸߶ȿ���
	if(flight_time <= m_time_altitude_control)
	{
		m_ugf = 0.0;
	}
	//��ʼ�߶ȿ���������ɣ�����ʱ��4s
	else if(flight_time <= (m_time_altitude_control + 4.0))
	{
		m_ugf = (m_u3f + m_u7f + m_u6f)  * (flight_time - m_time_altitude_control) /4.0;
		
	}
	//�߶ȿ��ƹ��̣��������
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{
		//�����߶Ȼ������̣��������·
		if(2 == m_high_maneuver_state)
		{
			m_ugf = m_u3f + m_u7f + m_u6f;
			m_ugf_record = m_ugf;
		}
		//�߶Ȼ������̣�ֻ���и�����ָ����٣������·
		//(3 == m_high_maneuver_state)&&(4 == m_high_maneuver_state)
		else
		{
			m_ugf = 0.0;
		}
	}
	//else if(flight_time < m_time_combat_status + m_time_combat_delay + 1.0)
	//{
	//	//����ĩ�Ƶ��󣬲����и߶ȿ���
	//	//(5 == m_high_maneuver_state)
	//	//˵�����𹥲���ʱ��ĩ�Ƶ������󣬸���������̣��Ƶ�ʱ�丳ֵΪ��ֵ������Ѳ��״̬����
	//	
	//	//��m_ugf_record ���ɵ�0������ʱ��1s
	//	m_ugf = m_ugf_record * (m_time_combat_status + m_time_combat_delay + 1.0 - flight_time);
	//}
	else
	{
		m_ugf = 0.0;
	}
	m_ugf = CFlightGlobalFun::Range(m_ugf,6.5);//���ȱ�1��1deg���Ƕ�Ӧ0.15g���ع���
	//��Ϊ�ڻ�·���룬��������ָ��
	//m_ugf = m_ugf/m_k2f;

	//����ͷ����Ŀ��ǰ���������
	if(flight_time < m_time_combat_status + m_time_combat_delay)
	//if(flight_time < m_time_combat_status)
	{
		//�������ݼ���
		m_unwif = m_uqkf + m_u2f + m_u5f;// + m_ugf;//���ڻ�ͼ����
	}
	//���䣬ĩ�Ƶ�������̹��ɣ�ս�����ӳ�ʱ��
	//else if(flight_time < (m_time_combat_status + m_time_combat_delay))
	//{
	//	//����ĩ�Ƶ�ʱ�̣�Ѳ�ɶ�ص�ѹ ���ɵ� ĩ�Ƶ���ѹ
	//	//m_unwif -> m_urf_zd
	//}
	//����ͷ����Ŀ��󣬽���ĩ�Ƶ�
	else
	{
		//���Ե�����Ծ��Ӧ����
 		//m_ny_command = 1.0;
 		//if (flight_time > (m_time_combat_status + 10.0)) m_ny_command = 0.5;	

		//��׼����·���ؿ���
 		//m_unwif += (RTOA * m_knif * (m_nby - m_ny_command) + m_kwif * m_wz) * STEP_5ms;
 		//m_unwif = CFlightGlobalFun::Range(m_unwif, 5.0);
 		//m_urf_zd = m_unwif + m_u5f;

		//��������ʽ��α��������·���ؿ��� �����У�����ָ����"�Ľ���������(���ɹ���ָ��)"
		//���ػ�����
		m_unif += RTOA * m_knif * (m_nby - m_ny_command) * STEP_5ms;
		m_unif = CFlightGlobalFun::Range(m_unif, 10.0);
		//α�������ױ��������Ի�
		double k1 = 7.4772e-3;//0.004981;
		double k2 = 0.9925;	//α���Ƿ�����·����1/(s+a4)��ɢ��ϵ����a4=1.5
		m_uaf = k2 * m_uaf_record + m_kwif * k1 * m_wz_record;
		m_uaf_record = m_uaf;
		m_wz_record = m_wz;
		
		//����΢��������ܿ�����
		if(m_distance_target > 10.0)
			m_urf_zd = m_unif + m_uaf + m_u5f;//���ػ��֡����ؿ���
		//����С��10mʱ�����ֲ���

		//�������ݼ���
		m_unwif = m_uqkf + m_urf_zd;
	}
}

void CMathControlPitch::Calc_Control_Gain()
{
	//�޷���������
	double temp_velocity = 50.0;//��ֵ m/s
	double temp_hight = 1500.0;//��ֵ m
	double temp_mass = 133.0;//��ֵ kg
	double temp_q = 1500.0;//��ֵ Pa	
	
	//��ɶο��Ʋ���: 1.2Hz�̶��ջ������ĸ�����PD���ƣ��߶ȡ��ٶȶ�ά��ֵ
	double k2f_stage1 = 1.211;
	double k5f_stage1 = 0.200;
	static double hight_uf_stage1_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//�߶�
	static double vel_uf_stage1_array[6] = {30, 40, 50, 60, 70, 80};//�ٶ�
	// ������������� Kp[�߶�][�ٶ�]
	double K2f_stage1_matrix[4][6] = {
	    {5.0347, 2.8320, 1.8125, 1.2587, 0.9247, 0.7080},
	    {5.5478, 3.1206, 1.9972, 1.3869, 1.0190, 0.7802},
	    {6.1270, 3.4465, 2.2057, 1.5318, 1.1254, 0.8616},
	    {6.7827, 3.8153, 2.4418, 1.6957, 1.2458, 0.9538}
	};
	// ������΢������ Kd[�߶�][�ٶ�]
	double K5f_stage1_matrix[4][6] = {
	    {1.4206, 0.8089, 0.5221, 0.3643, 0.2679, 0.2046},
	    {1.5568, 0.8855, 0.5711, 0.3983, 0.2929, 0.2237},
	    {1.7105, 0.9720, 0.6265, 0.4368, 0.3212, 0.2453},
	    {1.8845, 1.0699, 0.6891, 0.4803, 0.3531, 0.2698}
	};

	//Ѳ���ο��Ʋ�����0.8������ת��Ƶ��Ϊ�ջ�������PD���ƣ��Ե���������ѹ��ά��ֵ
	double k2f_stage2 = 1.3092;
	double k5f_stage2 = 0.1147;
	/******************************************************************
	 * ���룺���� mass(kg)����ѹ q(Pa)
	 * ������ֵ�㣺102, 133, 165 kg
	 * ��ѹ��ֵ�㣺500, 1000, 1500, 2000, 2500, 3000 Pa
	 * �����Kp���������棩��Kd��΢�����棩
	 * ����ṹ��Kp[������][��ѹ��]��Kd[������][��ѹ��]
	 *****************************************************************/
	// ������ֵ���������� 0,1,2��
	static double mass_uf_stage2_array[3] = {102.0, 133.0, 165.0};
	// ��ѹ��ֵ���������� 0~5��
	static double q_uf_stage2_array[6] = {500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0};
	// ������������� Kp[����][��ѹ]
	double K2f_stage2_matrix[3][6] = {
	    {1.4863, 1.5120, 1.5318, 1.5484, 1.5631, 1.5763},   // mass = 102kg
	    {1.4718, 1.4916, 1.5067, 1.5195, 1.5307, 1.5409},   // mass = 133kg
	    {1.4626, 1.4785, 1.4907, 1.5010, 1.5100, 1.5182}    // mass = 165kg
	};
	// ������΢������ Kd[����][��ѹ]
	double K5f_stage2_matrix[3][6] = {
	    {0.6086, 0.4274, 0.3470, 0.2992, 0.2665, 0.2423},   // mass = 102kg
	    {0.6127, 0.4293, 0.3481, 0.2997, 0.2666, 0.2422},   // mass = 133kg
	    {0.6153, 0.4306, 0.3488, 0.3000, 0.2667, 0.2421}    // mass = 165kg
	};
	//ǰ�� K0[����][��ѹ]
	//double K0f_stage2_matrix[3][6] = {
	//   {11.9249,  7.1517,  5.5607,  4.7651,  4.2878,  3.9696},
    //    {14.8263,  8.6024,  6.5278,  5.4905,  4.8681,  4.4532},
    //    {17.8212, 10.0999,  7.5261,  6.2392,  5.4671,  4.9523}
	//};
	
	//12.3051    7.4145    5.7843    4.9692    4.4802    4.1541
	//15.2778    8.9009    6.7752    5.7124    5.0747    4.6496
	//18.3464   10.4352    7.7981    6.4796    5.6884    5.1610
	//��ͬ������ͬ��ѹ��ƽ�⹥�ǣ���Ӧƽ�⸩����
	double alpha_b_stage2_matrix[3][6] = {
	    {6.6295, 1.4113, -0.3281, -1.1978, -1.7197, -2.0675},
    	{9.8014, 2.9972,  0.7292, -0.4049, -1.0853, -1.5389},
    	{13.0756,4.6343,  1.8206,  0.4137, -0.4304, -0.9932}
	};
	
	//ĩ�Ƶ��ο��Ʋ������̶�����α��������·���ؿ��ƣ��� 5rad-1�ջ�������1.5rad-1������ת��Ƶ�ʣ�
	//�������� knif
	//���ٶ����� kwif
	//α�໬������ k5f
	double knif_staget3 = 0.10;//ת��Ϊ�Ƕȣ���Ӧ4.011
	double kwif_staget3 = 0.55;//���ٶȻ��֣�α���ǿ���
	double k5f_staget3  = 0.14;//���ٶȿ���
	/******************************************************************
	 * ���Ʋ���������
	 * ��1���ٶ� < 40m/s  �� ʹ�� 40m/s ��Ӧ��������0�У�
	 * ��2���ٶȵ�λ��40��50��60��70 m/s �� ��Ӧ������ 0��1��2��3
	 * ��3���߶� > 3km    �� ʹ�� 3km ��Ӧ��������3�У�
	 * ��4���߶ȵ�λ��0��1000��2000��3000 m �� ��Ӧ������ 0��1��2��3
	 *****************************************************************/
	static double hight_uf_stage3_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//�߶�
	static double vel_uf_stage3_array[4] = {40.0, 50.0, 60.0, 70.0};//�ٶ�
	// ���ػ������� knif[�߶���][�ٶ���]
	double knif_stage3_matrix[4][4] = {
		{0.1291,  0.1030,  0.0855,  0.0728},   // �߶� 0m
		{0.1290,  0.1029,  0.0855,  0.0728},   // �߶� 1000m
		{0.1289,  0.1029,  0.0854,  0.0728},   // �߶� 2000m
		{0.1287,  0.1028,  0.0854,  0.0729}    // �߶� 3000m
	};

	// �໬������ kwif[�߶���][�ٶ���]
	double kwif_stage3_matrix[4][4] = {
		{0.8938,  0.3943,  0.1266,  0.0326},   // �߶� 0m
		{1.0427,  0.4894,  0.1925,  0.0157},   // �߶� 1000m
		{1.2107,  0.5969,  0.2670,  0.0703},   // �߶� 2000m
		{1.4010,  0.7186,  0.3513,  0.1321}    // �߶� 3000m
	};

	// ���ٶ����� k5f[�߶���][�ٶ���]
	double k5f_stage3_matrix[4][4] = {
		{0.8938,  0.3943,  0.1266,  0.0326},   // �߶� 0m
		{1.0427,  0.4894,  0.1925,  0.0157},   // �߶� 1000m
		{1.2107,  0.5969,  0.2670,  0.0703},   // �߶� 2000m
		{1.4010,  0.7186,  0.3513,  0.1321}    // �߶� 3000m
	};

	/*Ka = 

	   -0.1859   -0.0762   -0.0367   -0.0198
	   -0.2258   -0.0925   -0.0446   -0.0241
	   -0.2754   -0.1128   -0.0544   -0.0294
	   -0.3374   -0.1382   -0.0667   -0.0360


	K0 =

	    0.0254    0.1661    0.1955    0.1924
	   -0.0516    0.1456    0.1960    0.2015
	   -0.1586    0.1121    0.1906    0.2081
	   -0.3056    0.0609    0.1769    0.2106


	Kg =

	   -0.4094   -0.2357   -0.1440   -0.0904
	   -0.4649   -0.2712   -0.1687   -0.1085
	   -0.5275   -0.3112   -0.1965   -0.1290
	   -0.5984   -0.3566   -0.2280   -0.1521
  */
	// ǰ�� k0f[�߶���][�ٶ���]
	/*double k0f_stage3_matrix[4][4] = {
	    {8.3951, 6.2869, 5.1417, 4.4512},
		{8.9919, 6.6689, 5.4070, 4.6461},
		{9.6657, 7.1001, 5.7064, 4.8661},
		{10.4282, 7.5881, 6.0454, 5.1151}
	};*/

	//��ɶΣ�����������ǰ���ٶ��½���50m/s��������PD����
	if(flight_time < m_time_separate_booster)
	{	
		temp_hight  = CFlightGlobalFun::Range2(m_hz, hight_uf_stage1_array[3], hight_uf_stage1_array[0]);
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_uf_stage1_array[5], vel_uf_stage1_array[0]);

		//m_k0f = CFlightGlobalFun::LAQL2(4, 6, hight_stage1_array, vel_stage1_array, &K0f_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_k2f = CFlightGlobalFun::LAQL2(4, 6, hight_uf_stage1_array, vel_uf_stage1_array, &K2f_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_k5f = CFlightGlobalFun::LAQL2(4, 6, hight_uf_stage1_array, vel_uf_stage1_array, &K5f_stage1_matrix[0][0], temp_hight, temp_velocity);
	}
	//Ѳ����:����������󣬽���ĩ�Ƶ�ǰ��������PD���� + �߶ȿ���(��������)
	else if(flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		temp_mass  = CFlightGlobalFun::Range2(m_mass, mass_uf_stage2_array[2], mass_uf_stage2_array[0]);//��������mass
		temp_q = CFlightGlobalFun::Range2(m_q, q_uf_stage2_array[5], q_uf_stage2_array[0]);//��ѹq

		//m_k0f = CFlightGlobalFun::LAQL2(3, 6, mass_stage2_array, q_stage2_array, &K0f_stage2_matrix[0][0], temp_mass, temp_q);
		m_k2f = CFlightGlobalFun::LAQL2(3, 6, mass_uf_stage2_array, q_uf_stage2_array, &K2f_stage2_matrix[0][0], temp_mass, temp_q);
		m_k5f = CFlightGlobalFun::LAQL2(3, 6, mass_uf_stage2_array, q_uf_stage2_array, &K5f_stage2_matrix[0][0], temp_mass, temp_q);
		m_k5f *= 2.0;//???...
	}
	//ĩ�Ƶ����̣�α��������·���ؿ���
	else
	{
		//�߶ȡ��ٶ�
		temp_hight  = CFlightGlobalFun::Range2(m_hz, hight_uf_stage3_array[3], hight_uf_stage3_array[0]);
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_uf_stage3_array[3], vel_uf_stage3_array[0]);

		m_knif = CFlightGlobalFun::LAQL2(4, 4, hight_uf_stage3_array, vel_uf_stage3_array, &knif_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_kwif = CFlightGlobalFun::LAQL2(4, 4, hight_uf_stage3_array, vel_uf_stage3_array, &kwif_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_k5f = CFlightGlobalFun::LAQL2(4, 4, hight_uf_stage3_array, vel_uf_stage3_array, &k5f_stage3_matrix[0][0], temp_hight, temp_velocity);
		//m_k0f = CFlightGlobalFun::LAQL2(4, 4, hight_stage3_array, vel_stage3_array, &k0f_stage3_matrix[0][0], temp_hight, temp_velocity);
		//m_knif = 0.07;//ת��Ϊ�Ƕȣ���Ӧ4.011
		//m_kwif = 0.90;//���ٶȻ��֣�α���ǿ���
		//m_k5f  = 0.14;//���ٶȿ���

		//��ǰͨ���棬�Ѿ��Ǹ�����
		//m_knif = - m_knif;
		//m_kwif = - m_kwif;
		//m_k5f = - m_k5f;
	}

	//���������Ͷ�ѹ������ǰ����ѹ
	temp_mass  = CFlightGlobalFun::Range2(m_mass, mass_uf_stage2_array[2], mass_uf_stage2_array[0]);//��������mass
	temp_q = CFlightGlobalFun::Range2(m_q, q_uf_stage2_array[5], q_uf_stage2_array[0]);//��ѹq
	//���������Ͷ�ѹ������ƽ�⹥��
	m_alpha_b = CFlightGlobalFun::LAQL2(3, 6, mass_uf_stage2_array, q_uf_stage2_array, &alpha_b_stage2_matrix[0][0], temp_mass, temp_q);
	m_alpha_b = CFlightGlobalFun::Range2(m_alpha_b, 8.0, -2.0);

	//m_k0f = CFlightGlobalFun::LAQL2(3, 6, mass_uf_stage2_array, q_uf_stage2_array, &K0f_stage2_matrix[0][0], temp_mass, temp_q);
	//m_k0f = - m_k0f;
	//m_k0f = CFlightGlobalFun::Range2(m_k0f, 0.0, -10.0);
	//���������ع���-7.0deg�Ͳ��ȱ�1.067����ǰ����ѹ��������Ť�Ҿ��ȣ�ƽ����Ϊ��������̧ͷ����
	m_k0f = -(m_alpha_b  + 6.0)/1.067;//-6.56;
	m_k0f = CFlightGlobalFun::Range2(m_k0f, 0.0, -10.0);
	
	//���·���Ʋ������߶�PID����
	m_k3f = 0.245;//0.45;
	m_k7f = 0.900;//0.90;
	m_k6f = 0.027;//0.025;
}
void CMathControlPitch::Calc_Control_Commond()
{
	//���㸩����ָ���ת�ǲ���
	//double delta_zeta_command = (5.5 / (1.0 - cos(60.0 / RTOA))) * (1.0 - cos(m_gama_command_compensate / RTOA));
	double temp_dltzeta_factor = m_mass /24.48;
	temp_dltzeta_factor = CFlightGlobalFun::Range2(temp_dltzeta_factor, 6.74, 4.17);
	double delta_zeta_command = temp_dltzeta_factor * (1/cos(m_gama_command_compensate / RTOA) - 1);
	//��ת��60degʱ������Ϊ����2g������������ƽ�⹥��6.6deg = 5.5deg + 1.2deg����1g����Լ��������5.5deg
	delta_zeta_command = CFlightGlobalFun::Range(delta_zeta_command, 5.5);	

	//���㸩����ָ��߶ȿ��Ʋ��������߶ȿ��Ƹ�����Ϊ5.0deg
	double temp_hightctrl_zeta_command = -m_ugf/m_k2f;
	temp_hightctrl_zeta_command = CFlightGlobalFun::Range(temp_hightctrl_zeta_command, 3.0);

	//ʵʱ����߶Ȳ�
	//double dlt_hight = m_hz - m_h_target;

	//��һ���֣�����߶Ȼ���״̬����
	// 1.��ʼת�������
	if((m_high_maneuver_state == 0)&&(flight_time > m_time_missile_takeoff))
	{
		m_high_maneuver_state_record = m_high_maneuver_state;
		m_high_maneuver_state = 1;//��ʼ����
	}
	// 2.�������ת���θ߶ȿ���
	if((m_high_maneuver_state == 1)&&(flight_time > m_time_altitude_control))
	{
		m_high_maneuver_state_record = m_high_maneuver_state;
		m_high_maneuver_state = 2;//��ʼ�߶ȿ���

		//ָ����ǣ�˫ָ�����ɣ�����ƽ��ָ����ǲ���
		//��¼��ʼ������
		m_zeta_t_change = m_zeta;
		//m_zeta_t_change = m_ktheta_hight_enc;
		//���ɵ�Ŀ�긩����
		//m_ktheta_hight_enc
		//����ʱ�䳣��Ϊ��ֵ����

		//�߶Ȼ�����ʶ����
		m_flag_altitude_integral_set = 1;
	}
	// 3.���ݡ������л�����������쳣�����������߶Ȼ�����������Ŀ��߶ȡ������߶Ȼ���������2������3���»�4��Ѳ��״̬ת��
	//˵������������ɹ��̣���������ɹ��̣����º������л�
	if((m_high_maneuver_state > 1)&&(m_high_maneuver_state < 5))
	{
		//���һ�������л�������ת����ɺ󣬿�ʼ�߶Ȼ���
		//�������ͻ����޷�ƽ�ɻ������߶ȿ��ƣ���ʼ�߶Ȼ���
		// 3.1 ����߶Ȼ�����������ʼ�������ǣ��������߶ȡ��߶ȱ仯��
		if((m_count_altitude_change != m_count_altitude_change_record)	//��flight_basic�ۺ����̵��ȿ��ƣ���������������ʼ�߶Ȼ���
			&& (flight_time > m_time_altitude_change_start))			///1���߶Ȼ���ָ������ı�		2������ʱ����ڻ�����ʼʱ��
		{
			//�������£�ֻ����һ��
			m_count_altitude_change_record = m_count_altitude_change;

			// 1.ָ��߶Ȳ��С�����и߶ȹ��ɸ��٣���ִ�ж����������� �� �»�
			if(fabs(m_hz - m_h_target_in) < 20.0)
			{
				m_high_maneuver_state_record = m_high_maneuver_state;
				m_high_maneuver_state = 2;//�߶ȿ��ƣ����ͣ����»�������->�߶ȿ���

				//���ָ߶ȿ��ƣ���¼תѲ��ʱ��
				m_time_altitude_change_tocruise = flight_time;
				
				//����Bָ��߶ȣ�˫ָ�����ɵ�������Cָ��߶�
				//��ʼָ��߶ȣ�m_h_command_t_change = m_h_command;
				m_h_command_t_change_record = m_h_command_t_change;
				m_h_command_t_change = m_h_command;
				//Ŀ���ָ��߶ȣ�m_h_target
				//����ʱ�䳣��
				m_altitude_change_gain_record = m_altitude_change_gain;
				m_altitude_change_gain = 4.0;

				//ǰ����Ǽ̳������������⴦��
				
				//�߶Ȼ��ֿ��ƶ�Ǽ̳У������⴦����������
				m_flag_altitude_integral_set = 1;
			}
			// 2.�߶Ȼ���
			else 
			{
				// 2.1 ��������
				if( (m_hz - m_h_target_in) < - 20.0 )
				{
					m_high_maneuver_state_record = m_high_maneuver_state;
					m_high_maneuver_state = 3;//Ѳ�������ͣ����»�->����

					//ǰ����Ǽ̳У���ǰһ�׶Ρ��߶ȿ���ƽ���ء��̳�
					
					//�����ǻ������㣬��ǰ�����

					//�߶Ȼ�����ʶ����
					m_flag_altitude_integral_set = 0;
				}
				// 2.2 �»�����
				else
				{
					m_high_maneuver_state_record = m_high_maneuver_state;
					m_high_maneuver_state = 4;//Ѳ�������ͣ�������->�»�
					
					//ǰ����Ǽ̳У���ǰһ�׶Ρ��߶ȿ���ƽ���ء��̳�
					
					//�����ǻ������㣬��ǰ�����

					//�߶Ȼ�����ʶ����
					m_flag_altitude_integral_set = 0;
				}

				//ָ����ǣ�˫ָ�����ɣ�����ƽ��ָ����ǲ���
				//��¼��ʼ�����Ǽ�ʱ��
				//m_zeta_t_change = m_zeta;
				m_zeta_t_change = m_zeta_command;
				m_time_zeta_change = flight_time;
				//���ɵ�Ŀ�긩����
				///2->2 �״�Ϊm_ktheta_hight_enc�����״�Ϊƽ�⹥��m_alpha_b
				///2->3 m_ktheta_climb_enc
				///2->4 m_ktheta_decline_enc
				//����ʱ�䳣��Ϊ��ֵ����

				//����Bָ��߶ȣ�˫ָ�����ɵ�������Cָ��߶�
				//��ʼָ��߶ȣ�m_h_command_t_change = m_h_command;
				//m_h_command_t_change_record = m_h_command_t_change;
				//m_h_command_t_change = m_h_command;
				//Ŀ���ָ��߶ȣ�m_h_target

				//��¼���μ���һ�θ߶Ȼ�����ʼʱ������ʱ�����
				//m_altitude_change_gain_record = m_altitude_change_gain;	//���㲢��¼��һ�θ߶Ȼ����ٶȿ�������	
				//m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / (2.5*2.0);
				//���մ���2.5m/s��4sʱ�������߶�Լ10m������2s�ӳٵ�5m/s����10m
				//if(m_altitude_change_gain <= 4.0)	m_altitude_change_gain = 4.0;
				//���մ���2.5m/s��100sʱ�������߶�Լ250m
				//if(m_altitude_change_gain >= 100.0) m_altitude_change_gain = 100.0;
			}

			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��ָ��߶�
			//m_h_command_t_change_record = m_h_command_t_change;
			//m_h_command_t_change = m_h_command;
			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��������ʼʱ��
			m_time_altitude_change_record = m_time_altitude_change;
			m_time_altitude_change = m_time_altitude_change_start;	//�߶Ȼ�����ʼʱ��
			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��Ŀ��߶ȸ���
			m_h_target_record = m_h_target;
			m_h_target = m_h_target_in;
		}

		//�������������Ŀ��߶ȡ������߶Ȼ��������������»�ת�߶ȿ��ƣ����˻���ʱ��
		// 3.2 �߶Ȼ���������ת�߶ȿ���
		if( ((m_high_maneuver_state == 3) || (m_high_maneuver_state == 4)) && (flight_time > m_time_altitude_change_end) )
		{
			m_high_maneuver_state_record = m_high_maneuver_state;
			m_high_maneuver_state = 2;

			//��¼תѲ��ʱ��
			m_time_altitude_change_tocruise = flight_time;
			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��������ʼʱ��
			//m_time_altitude_change_record = m_time_altitude_change;
			//m_time_altitude_change = m_time_altitude_change_end;	//�߶Ȼ�����ʼʱ�� tocruise
			
			//��¼����߶ȿ���ʱ�̵ĸ߶ȣ���ָ������ �� Ŀ��߶�
			//��ʼָ��߶ȣ�m_h_command_t_change = m_hz;
			m_h_command_t_change_record = m_h_command_t_change;
			m_h_command_t_change = m_hz;
			//Ŀ��ָ��߶ȣ�m_h_target
			//�߶ȹ���ʱ�䳣�����߶Ȳ�20m����ʼ�ٶ�5m/s��ĩ���ٶ�0m/s��ƽ���ٶ�Լδ2.5m/s�����ƹ���ʱ��8s
			m_altitude_change_gain_record = m_altitude_change_gain;
			m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / (5.0*2.0);
			if(m_altitude_change_gain <= 4.0)	m_altitude_change_gain = 4.0;//���մ���5m/s��4sʱ�������߶�Լ20m
			if(m_altitude_change_gain >= 450.0) m_altitude_change_gain = 450.0;//���մ���5m/s��900sʱ�������߶�Լ4500m

			//ǰ����Ǽ̳У���ǰһ�׶Ρ������Ǹ��ٿ��ƣ���̬ƽ���ء��̳�
			
			//ָ����ǹ��ɼ�ʱ��
			//��ʼָ�����m_ktheta_decline_enc �� m_ktheta_climb_enc
			m_zeta_t_change = m_zeta_command;///3��4->2 �� m_zeta
			m_time_zeta_change = flight_time;
			//Ŀ��ָ�����m_ktheta_hight_enc
			//����ʱ�䳣�����̶�

			//�߶Ȼ�����ʶ����
			m_flag_altitude_integral_set = 1;
		}
		
		// 4.Ѳ��״̬תĩ�Ƶ���һ��ΪѲ��״̬תĩ�Ƶ���Ҳ����Ϊ�������»��߶Ȼ�����תĩ�Ƶ�
		//��ʽ���ģʽ������Ҫ��������
		//�𹥴��ģʽ��ĩ�Ƶ������󣬽��븩������֮���ٽ���ĩ�Ƶ�
		if( (flight_time > m_time_combat_status + m_time_combat_delay)&&(flight_time < m_time_combat_dive_pullup) )
		{
			m_high_maneuver_state_record = m_high_maneuver_state;
			m_high_maneuver_state = 5;

			//�߶ȿ��� ת ĩ�Ƶ� ���ɣ���˼��???...
			
			//�ڻ�·����������·���ؿ��ƣ���ʹ�� ָ�����
			//ָ����� ���� ƽ�ɸ����ǣ�������

			//���·�����øĽ�������������ʹ��ָ��߶�
			//ָ��߶� ���ɵ� Ŀ��߶�
			//��ʼָ��߶ȣ�m_h_command_t_change = m_h_command;
			//m_h_command_t_change_record = m_h_command_t_change;
			//m_h_command_t_change = m_hz;
			//Ŀ���ָ��߶ȣ�m_h_target
			//����ʱ�䳣��
			//m_altitude_change_gain_record = m_altitude_change_gain;
			//m_altitude_change_gain = fabs(m_h_command_t_change - m_h_target) / (5.0*2.0);
			//if(m_altitude_change_gain <= 4.0)	m_altitude_change_gain = 4.0;//���մ���5m/s��4sʱ�������߶�Լ20m
			//if(m_altitude_change_gain >= 450.0) m_altitude_change_gain = 450.0;//���մ���5m/s��900sʱ�������߶�Լ4500m

			//��¼ĩ�Ƶ��߶Ȼ���ʱ��
			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��������ʼʱ��
			m_time_altitude_change_record = m_time_altitude_change;
			m_time_altitude_change = flight_time;	//�߶Ȼ�����ʼʱ��
			//��¼���μ���һ�θ߶Ȼ�����ʼʱ��Ŀ��߶ȸ���
			m_h_target_record = m_h_target;
			m_h_target = m_h_target_in;
		}

		// 5.Ѳ��״̬��ת����״̬���������㷨��������ط���ʱ����ֱ�ӻ��㼴�ɣ�
		//m_high_maneuver_state = 8;
	}

	// 6.ĩ�Ƶ����������𣬼�Ѳ��״̬ �� ����
	// 7.ĩ�Ƶ��󸩳�������̣���Ѳ����״̬���ϣ������⴦����6��7״̬��ִ��
	if(m_high_maneuver_state == 5)
	{
		if(flight_time > m_time_combat_dive_pullup)
		{
			//��ĩ�Ƶ�ת��Ѳ������������
			m_high_maneuver_state_record = m_high_maneuver_state;
			m_high_maneuver_state = 3;

			//��¼����ʱ�̸����ǣ����ڹ��ɵ������Ƕ�
			m_zeta_t_change = m_zeta_command;///5->3 �� m_zeta
			//ʱ��Ϊ��������ʱ��
			//m_time_combat_dive_pullup
			m_time_zeta_change = flight_time;

			//��¼�߶Ȼ���ʱ��
			m_time_altitude_change_record = m_time_altitude_change;
			m_time_altitude_change = flight_time;	//�߶Ȼ�����ʼʱ��
			//����Ŀ���߶�
			m_h_target_record = m_h_target;
			m_h_target = m_h_target_in;
			
			//�߶Ȼ��������У����ֱ�ʶ����
			m_flag_altitude_integral_set = 0;
		}
	}

	// 8.���������Ѳ��״̬ ת ����״̬������Ѳ��״̬��̬�ȶ���8״̬��ִ��
	//˵����ֱ������ɡ����ɡ�������ع��󣬲������޷�ά��ƽ�⣬��ط���ʱ�����Ϊ��

	//�ڶ����֣�������ָ��
	// 1.����ǰ��Լ0.3s
	if (flight_time < m_time_control)
	{
		m_zeta_command = m_zeta;
		m_zeta_command_record1 = m_zeta_command;
	}
	// 2.��غ���������ǰ, Լ2s��9.5s������ʱ��Լ1.7s
	//else if(flight_time < m_time_separate_booster)
	else if(flight_time < m_time_missile_takeoff)
	{
		//�����ǹ��ȵ���ʼ�����(װ��ֵ)��֮�󱣳ַ����
		m_zeta_command = (m_zeta_command_record1 - m_ktheta_lauch_enc) * exp(-(flight_time - m_time_control) * (flight_time - m_time_control) / 1.0) + m_ktheta_lauch_enc;
		m_zeta_command_record2 = m_zeta_command;
	}
	// 3.�����ɺ��״θ߶ȿ���ǰ�������ǹ��ɵ��������ǣ�����ʱ��4s�����ڸ߶ȿ���
	else if(flight_time < m_time_altitude_control)
	{
		//������װ�������ǣ�֮�󱣳ֳ�ֵ����������
		m_zeta_command = (m_zeta_command_record2 - m_ktheta_climb_enc) * exp(-(flight_time - m_time_missile_takeoff) * (flight_time - m_time_missile_takeoff) / 5.0) + m_ktheta_climb_enc;
		m_zeta_command_record1 = m_zeta_command;
		m_zeta_command = m_zeta_command + delta_zeta_command;
	}
	// 4.���뵼��ǰ�������ǹ���ʱ��5.5s
	else if(flight_time < (m_time_combat_status + m_time_combat_delay))
	{
		//������ƽ����̬�ǹ��ɵ�������̬�ǣ���2deg��8deg
		if(m_high_maneuver_state == 3)
		{
			m_zeta_command = (m_zeta_t_change - m_ktheta_climb_enc)*exp(-(flight_time - m_time_zeta_change) * (flight_time - m_time_zeta_change)/ 10.0) + m_ktheta_climb_enc; 
		}
		//�½���ƽ����̬�ǹ��ɵ��»���̬�ǣ���2deg��-6deg
		if(m_high_maneuver_state == 4)
		{
			m_zeta_command = (m_zeta_t_change - m_ktheta_decline_enc)*exp(-(flight_time - m_time_zeta_change) * (flight_time - m_time_zeta_change)/ 10.0) + m_ktheta_decline_enc; 
		}
		//ƽ�ɸ߶ȿ��ƣ���̬��ָ����ɵ�ƽ����̬�ǣ�����ǰ��̬�ǹ��ɵ�0deg
		if(m_high_maneuver_state == 2)
		{
			//�״θ߶ȿ��ƣ�����ʱ��ָ���(����������) ���ɵ� �״θ߶ȿ��Ƹ����ǣ�װ��ֵ��
			//if(0 == m_count_altitude_change)
			//��Ϊ��Ӧ��ָ���ٴ���������������£����� �������״̬������δ����
			if(0 == m_count_altitude_change_record)
			{
				m_zeta_command = (m_zeta_command_record1 - m_ktheta_hight_enc)*exp(-(flight_time - m_time_altitude_control) * (flight_time - m_time_altitude_control)/ 10.0) + m_ktheta_hight_enc; 	
			}
			//���״θ߶ȿ��ƣ�����ʱ�̸�����(״̬�� ���ɵ� ƽ�ɸ����ǣ�ƽ�⹥�ǣ�
			//�»�ת�߶ȿ��ƣ���ʼʱ��Ϊ�»�����ʱ��
			//����ת�߶ȿ��ƣ���ʼʱ��Ϊ��������ʱ��
			//�߶ȿ���ת�߶ȿ��ƣ���ʼʱ��Ϊ �߶ȹ��ɿ�ʼʱ��
			else
			{
				//m_zeta_command = (m_zeta_t_change - m_ktheta_hight_enc)*exp(-(flight_time - m_time_altitude_change_end) * (flight_time - m_time_altitude_change_end)/ 10.0) + m_ktheta_hight_enc; 
				m_zeta_command = (m_zeta_t_change - m_alpha_b)*exp(-(flight_time - m_time_zeta_change) * (flight_time - m_time_zeta_change)/ 10.0) + m_alpha_b; 
			}
			
			//�߶ȿ��Ƹ�����ָ���
			m_zeta_command = m_zeta_command + temp_hightctrl_zeta_command;
		}

		//��ת���Ƹ�����ָ���
		m_zeta_command = m_zeta_command + delta_zeta_command;
	}
	// 5.����ĩ�Ƶ����ƣ�����ʹ��ָ����ǣ�Ϊ�˱������������ԣ����ָ߶ȿ��Ƽ���ֵ
	else
	{
		//m_zeta_command = m_ktheta_hight_enc + delta_zeta_command;
		m_zeta_command = m_alpha_b + delta_zeta_command + temp_hightctrl_zeta_command;
	}
	// 6.�ܸ�����ָ���޷�
	// ˵��������2degƽ�⹥�ǹ��ƣ��������ԼΪ-10deg~10deg
	m_zeta_command = CFlightGlobalFun::Range2(m_zeta_command, 12.0, -8.0);

	
	//�������֣��߶�ָ�����
	// 1.�״θ߶ȿ���ǰ
	if (flight_time <= m_time_altitude_control)
	{
		//ָ��߶ȡ����٣������·
		m_h_command = m_hz;
		m_h_rate_command = m_vs;
		
		//����ֱ���߶ȿ���
		m_hz_t_altitude_control = m_hz;
		m_vs_t_altitude_control = m_vs;
		//Ŀ��߶�
		m_h_target = m_h_target_in;
	}
	// 2.�״θ߶ȿ��ƿ�ʼ�� �� �߶Ȼ���ǰ����ʼ���������д��٣����ɵ�ƽ��
	else if(flight_time < m_time_altitude_change)
	{
		//���ݽ���ʱ�̴��٣����ƻ�����Ŀ��߶�ʱ��
		if (fabs(m_vs_t_altitude_control) > 0.5)
			m_altitude_change_gain_seg1 = 2.0*fabs((m_hz_t_altitude_control - m_h_target)/m_vs_t_altitude_control);
		else
			m_altitude_change_gain_seg1 = 100.0;
		if (m_altitude_change_gain_seg1<=4.0)	m_altitude_change_gain_seg1 = 4.0;//��������20m
		if (m_altitude_change_gain_seg1>=100.0)	m_altitude_change_gain_seg1 = 100.0;//��������250m
		
		//ָ��߶��ɸ߶Ȼ���ʱ�� �߶� ���ɵ� Ŀ��߶�(װ��ֵ)�����ٹ������㣬��ʼ�߶ȸ���
		m_h_command = (m_hz_t_altitude_control - m_h_target) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
			+ m_h_target;
		m_h_rate_command = -((m_hz_t_altitude_control - m_h_target) / m_altitude_change_gain_seg1) 
			* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1);
	}
	// 3.��ʼ�߶Ȼ�����Ѳ�����й����и߶Ȼ������̣�������������ƽ������
	else
	{	
		//����壺Ѳ���޸߶Ȼ����������һ�������ϣ���Ϊ�ǹ�����ɺ����̬
		if(m_high_maneuver_state == 2)
		{
			//���һ: �������»���ɣ�state = 2��ָ�����ɵ�Ŀ��߶ȣ������֣����и߶ȿ���
			//�л�ʱ����Ŀ�꣬�߶Ȳ�С����ʼʱ�䣻����ʱ�䣬���ڽ���ʱ��
			if ((m_high_maneuver_state_record == 3)||(m_high_maneuver_state_record == 4))
			{
				m_h_command = (m_h_command_t_change - m_h_target) 
								* exp(- (flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain)
								//* exp(- (flight_time - m_time_altitude_change) / m_altitude_change_gain)
								+ m_h_target;
				m_h_rate_command = -((m_h_command_t_change - m_h_target) / m_altitude_change_gain) 
								* exp(- (flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain);
								//* exp(- (flight_time - m_time_altitude_change) / m_altitude_change_gain);
			}
			//��������߶Ȼ�����Χ��С��state = 2��˫ָ�����ɵ�Ŀ��߶ȣ������֣��������������»�
			//�л�ʱ����Ŀ�꣬�߶Ȳ����ʼʱ�䣻����ʱ�䣬�տ�ʼС�ڽ���ʱ�䣬�������ڽ���ʱ��
			else
			{
				m_h_command = (m_h_command_t_change - m_h_target) 
					* exp(-((flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain))
					//* exp(-((flight_time - m_time_altitude_change) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change) / m_altitude_change_gain))
					+ m_h_target;
				m_h_rate_command = -2.0 * (flight_time - m_time_altitude_change_tocruise) * (m_h_command_t_change - m_h_target) / (m_altitude_change_gain * m_altitude_change_gain)
					* exp(-((flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change_tocruise) / m_altitude_change_gain));
					//* exp(-((flight_time - m_time_altitude_change) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change) / m_altitude_change_gain));
			}
		}
		//��������������̣�state = 3
		//����ģ��»����̣�state = 4
		//����壺�Ƶ�������̣�state = 5��
		//�߶Ȼ�����ʼ����δ�����������������»����̣�ָ��߶� ���� ��ǰ�߶�
		else if( (m_high_maneuver_state == 3) || (m_high_maneuver_state == 4) || (m_high_maneuver_state == 5) )
		{
			//�߶�ָ�����
			m_h_command = m_hz;
			m_h_rate_command = m_vs;
		}
		//���������ָ��߶ȵ���Ŀ��߶ȣ�ָ���ٶ�Ϊ��̬�ٶ�Ϊ��
		else
		{
			m_h_command = m_h_target;
			m_h_rate_command = 0.0;
		}
		
		/* �����Ǹ߶Ȼ��������У������л��������Ż�???...
		
		// 1.�����л�ǰ�ĺ��Σ�ָ��߶ȡ����ٶȣ�ָ��߶ȡ�ָ��ٶȼ���
		// 1.1�����ɹ����У������л���һ�����㣬��m_count_altitude_change Ϊ1�������� ���� ǰһ��Ŀ��߶�
		//�߶ȹ�����ǰһ����Ŀ��߶�m_hz_t_altitude_control -> m_h_target_record��m_altitude_change_gain_seg1
		if(1 == m_count_altitude_change)
		{			
			m_h_command1 = (m_hz_t_altitude_control - m_h_target_record)
				* exp(- (flight_time - m_time_altitude_control) / m_altitude_change_gain_seg1) 
				+ m_h_target_record;
			m_h_rate_command1 = -1.0 / m_altitude_change_gain_seg1 * (m_h_command1 - m_h_target_record);
		}
		// 1.2����߶Ȼ��������У������л���һ�����㣬��m_count_altitude_change ����1������ǰһ��ָ��߶ȡ����ٶ�
		//����߶Ȼ��������У�δ�л����㣬����ǰһ��ָ��߶�Ϊǰһ��ָ��߶ȡ����ٶȣ����߶�Ϊǰһ��װ��ֵ������Ϊ��
		//�߶� ������ ǰһ����Ŀ��߶�m_h_command_t_change_record -> m_h_target_record��m_altitude_change_gain_record
		else
		{
			double temp = (flight_time - m_time_altitude_change_record) / m_altitude_change_gain_record;
			m_h_command1 = (m_h_command_t_change_record - m_h_target_record) * exp(- temp * temp) + m_h_target_record;
			m_h_rate_command1 = -2.0 * (flight_time - m_time_altitude_change_record) * (m_h_command1 - m_h_target_record)
				/ (m_altitude_change_gain_record * m_altitude_change_gain_record);
		}
		
		// 2.����߶Ȼ������̻���ɹ����У������л���һ�����㣬���㵱ǰ��ָ��߶ȡ����ٶȣ�˫ָ������
		//����ʱ�̸߶� ������ Ŀ��߶�m_h_command_t_change -> m_h_target��m_altitude_change_gain
		m_h_command2 = (m_h_command_t_change - m_h_target) 
			* exp(-((flight_time - m_time_altitude_change) / m_altitude_change_gain) * ((flight_time - m_time_altitude_change) / m_altitude_change_gain))
			+ m_h_target;
		m_h_rate_command2 = -2.0 * (flight_time - m_time_altitude_change) * (m_h_command2 - m_h_target) 
			/ (m_altitude_change_gain * m_altitude_change_gain);

		// 3.ָ��߶ȹ���
		// 3.1 ָ��߶ȹ��ɹ�����
		if(flight_time <= (m_time_altitude_change + 8.0))
		{
			//m_h_command1->m_h_command2
			double temp = PI / 8.0 * (flight_time - m_time_altitude_change);
			m_h_command = m_h_command1 + 0.5 * ( m_h_command2 - m_h_command1) * (1.0 - cos(temp));
			m_h_rate_command = m_h_rate_command1 
				+ 0.5 * ( m_h_rate_command2 - m_h_rate_command1) * (1.0 - cos(temp)) 
				+ PI / 16.0 * ( m_h_command2 - m_h_command1) * sin(temp);
		}
		// 3.2 ָ��߶ȹ������
		else
		{
			//����m_h_command2
			double temp = (flight_time - m_time_altitude_change) / m_altitude_change_gain;
			m_h_command = (m_h_command_t_change - m_h_target) * exp(- temp * temp) + m_h_target;
			m_h_rate_command = -2.0 * (flight_time - m_time_altitude_change) * (m_h_command - m_h_target)
				/ (m_altitude_change_gain * m_altitude_change_gain);
		}*/
	}


	//���Ĳ��֣����������ָ��
	
	//ĩ�˵����ɣ����Ż�???...
	//���������أ���������
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
	//����������������
	//m_ny_command = 4.0 * m_v * m_dqf / m_g / RTOA + 1.00*cos(m_zeta / RTOA);
	//m_ny_command = CFlightGlobalFun::Range(m_ny_command, 2.0);
	
	//�ⲿ�����Ƶ�ָ��
	if(flight_time < m_time_combat_status + m_time_combat_delay)
	{
		m_ny_command = 0.0;
	}
	else
	{
		m_ny_command = m_ny_command_guidance;
	}
	
}

/*
void CMathControlPitch::Monitor_Data()
{
	extern CSimMonitor sim_monitor;

	//����ָ��
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_zeta_command,"zetacx",ENUM_FILE_CONTROL1);	//���ƶ�:��̬��ָ��
		sim_monitor.Get_Variable(m_h_command,"hcx",ENUM_FILE_CONTROL1);		//Ѳ����:�߶�ָ��
		sim_monitor.Get_Variable(m_h_rate_command,"dhcx",ENUM_FILE_CONTROL1);	//Ѳ����:����ָ��
		sim_monitor.Get_Variable(m_ny_command,"nyc",ENUM_FILE_CONTROL1);		//������:����ָ��
		
		//����
		sim_monitor.Get_Variable(m_zeta,"zeta",ENUM_FILE_CONTROL1);//������
		sim_monitor.Get_Variable(m_hz,"hight",ENUM_FILE_CONTROL1);//�߶�
		sim_monitor.Get_Variable(m_vs,"vy",ENUM_FILE_CONTROL1);//����
		sim_monitor.Get_Variable(m_nby,"ny",ENUM_FILE_CONTROL1);//����ϵ����

		//����1��2
		sim_monitor.Get_Variable(m_unwif,"unwif",ENUM_FILE_CONTROL1); //�Ƶ�ǰ���
		sim_monitor.Get_Variable(m_urf_zd,"urf_zd",ENUM_FILE_CONTROL1); //�Ƶ�������
	}

	//���Ʋ������������Է���
	if (sim_monitor.flag_monitor3_valid)
	{
		sim_monitor.Get_Variable(m_k2f,"k2f",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k5f,"k5f",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k3f,"k3f",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k7f,"k7f",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k6f,"k6f",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_knif,"knif",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_kwif,"kwif",ENUM_FILE_AERO1);
	}
}
*/