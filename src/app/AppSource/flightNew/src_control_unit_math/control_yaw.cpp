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
	//p_st_debug_monitor = NULL;
	m_k5h = 0.0;
	m_uh_adrc = 0.0;
	m_urh_record = 0.0;
	m_knih = 0.0;
	m_knph = 0.0;
	m_kwih = 0.0;
	m_u5h = 0.0;
	m_unwih = 0.0;
	m_urh_zd = 0.0;
	m_urh_record_separate_booster = 0.0;
	m_urh_record_begin_combat = 0.0;
	//m_time_record_separate_booster = 0.0;
	//m_time_record_begin_combat = 0.0;
	m_wx = 0.0;
	m_wy = 0.0;
	m_wy_command = 0.0;
	m_wy_command_comp = 0.0;
	m_wy_command_comp_record = 0.0;
	m_distance_target = 0.0;
	m_nz_command = 0.0;
	m_nz_command_guidance = 0.0;
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
	m_unph = 0.0;
	m_ubh = 0.0;
	m_ubh_record = 0.0;
	m_wy_record = 0.0;
	m_gama_command_compensate = 0.0;

	m_flag_v30 = false;
	m_count_v30 = 0;
	m_time_v30 = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_missile_takeoff = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
}
void CMathControlYaw::Initial()
{

	y_eso_init_takeoff();
	y_eso_init_cruise();
	y_eso_init_attack();
}

void CMathControlYaw::y_eso_init_takeoff()
{
	para_y_eso_takeoff.z1 = 0.0;
    para_y_eso_takeoff.z2 = 0.0;
    para_y_eso_takeoff.z1_pre = 0.0;
    para_y_eso_takeoff.z2_pre = 0.0;
    para_y_eso_takeoff.uy = 0.0;
    para_y_eso_takeoff.y = 0.0;
    para_y_eso_takeoff.y_pre = 0.0;
	para_y_eso_takeoff.k = 5.0;
    para_y_eso_takeoff.Ts = 0.005;
}
void CMathControlYaw::y_eso_calc_takeoff(double b0, double d0, double Keso)
{
	double Ts       = para_y_eso_takeoff.Ts;
    double z1_pre   = para_y_eso_takeoff.z1_pre;
    double z2_pre   = para_y_eso_takeoff.z2_pre;
    double y_meas  = para_y_eso_takeoff.y;
    double uy = para_y_eso_takeoff.uy;

    double w0 = 2.5;
    double w_eso = Keso * w0;

	double l1 = 2*1.0*w_eso - b0;
	double l2 = w_eso*w_eso;

    double z1 = (1 - Ts*b0 - Ts*l1)*z1_pre + Ts*z2_pre + Ts*d0*uy + Ts*l1*y_meas;
    double z2 = z2_pre - Ts*l2*z1_pre + Ts*l2*y_meas;


    para_y_eso_takeoff.z1_pre = para_y_eso_takeoff.z1;
    para_y_eso_takeoff.z2_pre = para_y_eso_takeoff.z2;

    para_y_eso_takeoff.z1 = z1;
    para_y_eso_takeoff.z2 = z2;

    para_y_eso_takeoff.y_pre = para_y_eso_takeoff.y;
}

void CMathControlYaw::y_eso_init_cruise()
{
	para_y_eso_cruise.z1 = 0.0;
    para_y_eso_cruise.z2 = 0.0;
    para_y_eso_cruise.z1_pre = 0.0;
    para_y_eso_cruise.z2_pre = 0.0;
    para_y_eso_cruise.uy = 0.0;
    para_y_eso_cruise.y = 0.0;
    para_y_eso_cruise.y_pre = 0.0;
	para_y_eso_cruise.k = 5.0;
    para_y_eso_cruise.Ts = 0.005;
}
void CMathControlYaw::y_eso_calc_cruise(double b0, double d0, double Keso)
{
	double Ts       = para_y_eso_cruise.Ts;
    double z1_pre   = para_y_eso_cruise.z1_pre;
    double z2_pre   = para_y_eso_cruise.z2_pre;
    double y_meas  = para_y_eso_cruise.y;
    double uy = para_y_eso_cruise.uy;

    double w0 = 2.5;
    double w_eso = Keso * w0;

	double l1 = 2*1.0*w_eso - b0;
	double l2 = w_eso*w_eso;

    double z1 = (1 - Ts*b0 - Ts*l1)*z1_pre + Ts*z2_pre + Ts*d0*uy + Ts*l1*y_meas;
    double z2 = z2_pre - Ts*l2*z1_pre + Ts*l2*y_meas;


    para_y_eso_cruise.z1_pre = para_y_eso_cruise.z1;
    para_y_eso_cruise.z2_pre = para_y_eso_cruise.z2;

    para_y_eso_cruise.z1 = z1;
    para_y_eso_cruise.z2 = z2;

    para_y_eso_cruise.y_pre = para_y_eso_cruise.y;
}

void CMathControlYaw::y_eso_init_attack()
{
	para_y_eso_attack.z1 = 0.0;
    para_y_eso_attack.z2 = 0.0;
    para_y_eso_attack.z1_pre = 0.0;
    para_y_eso_attack.z2_pre = 0.0;
    para_y_eso_attack.uy = 0.0;
    para_y_eso_attack.y = 0.0;
    para_y_eso_attack.y_pre = 0.0;
	para_y_eso_attack.k = 5.0;
    para_y_eso_attack.Ts = 0.005;
}
void CMathControlYaw::y_eso_calc_attack(double b0, double d0, double Keso)
{
	double Ts       = para_y_eso_attack.Ts;
    double z1_pre   = para_y_eso_attack.z1_pre;
    double z2_pre   = para_y_eso_attack.z2_pre;
    double y_meas  = para_y_eso_attack.y;
    double uy = para_y_eso_attack.uy;

    double w0 = 2.5;
    double w_eso = Keso * w0;

	double l1 = 2*1.0*w_eso - b0;
	double l2 = w_eso*w_eso;

    double z1 = (1 - Ts*b0 - Ts*l1)*z1_pre + Ts*z2_pre + Ts*d0*uy + Ts*l1*y_meas;
    double z2 = z2_pre - Ts*l2*z1_pre + Ts*l2*y_meas;


    para_y_eso_attack.z1_pre = para_y_eso_attack.z1;
    para_y_eso_attack.z2_pre = para_y_eso_attack.z2;

    para_y_eso_attack.z1 = z1;
    para_y_eso_attack.z2 = z2;

    para_y_eso_attack.y_pre = para_y_eso_attack.y;
}

void CMathControlYaw::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	//Monitor_Data();
};
void CMathControlYaw::Get_Data()
{
	m_time_separate_booster = p_st_yaw_control_input->time_separate_booster;
	m_time_missile_takeoff = p_st_yaw_control_input->time_missile_takeoff;
	m_time_combat_status = p_st_yaw_control_input->time_combat_status;
	m_time_combat_delay = p_st_yaw_control_input->time_combat_delay;
	m_wx = p_st_yaw_control_input->wx;
	m_wy = p_st_yaw_control_input->wy;
	m_distance_target = p_st_yaw_control_input->distance_target;
	m_nby = p_st_yaw_control_input->nby;
	m_nbz = p_st_yaw_control_input->nbz;
	m_nz_command_guidance = p_st_yaw_control_input->nz_command_guidance;
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

	if(m_v > 30.0)
	{
		m_count_v30++;
	}
	else
	{
		m_count_v30 = 0;
	}
	
	if((m_count_v30 >=3) && (!m_flag_v30))
	{
		m_flag_v30 = 1;
		m_time_v30 = flight_time;
	}
	
}
void CMathControlYaw::Send_Data()
{
	p_st_yaw_control_output->u5h = m_u5h;
	p_st_yaw_control_output->urh_zd = m_urh_zd;// ����������

	p_st_yaw_control_output->wy_command = m_wy_command;
	p_st_yaw_control_output->nz_command = m_nz_command;// ����������
}
void CMathControlYaw::Calc_Data()
{
	double k1 = 4.996e-3;
	double k2 = 0.9985;		//α�໬�Ƿ�����·����1/(s+b4)��ɢ��ϵ����b4=0.3
	double temp_urh_zd = 0.0;
	
	Calc_Control_Gain();
	Calc_Control_Commond();

	//��������
	m_u5h = m_k5h * (m_wy - m_wy_command);
	

	//������: PI + ADRC �Կ��Ź��ؿ���
	if(m_v < m_time_v30)
	{
		//ADRC��ʼ��
		m_uh_adrc = 0.0;
		para_y_eso_takeoff.z1 = (m_nbz + para_y_eso_takeoff.k*m_wy/RTOA);
		para_y_eso_takeoff.z1_pre = (m_nbz + para_y_eso_takeoff.k*m_wy/RTOA);
		para_y_eso_takeoff.z2 = m_urh_record/RTOA*16.4631;
		para_y_eso_takeoff.z2_pre= para_y_eso_takeoff.z2;
		para_y_eso_takeoff.y = (m_nbz + para_y_eso_takeoff.k*m_wy/RTOA);
		para_y_eso_takeoff.y_pre = (m_nbz + para_y_eso_takeoff.k*m_wy/RTOA);

		//ֻ��΢�ֿ��ƣ���ADRC���ƣ��޹��ؿ���
		m_k5h = 0.6982;
		m_u5h = m_k5h * (m_wy - m_wy_command);
		m_unih = 0.0;
		m_unph = 0.0;
	}
	else if(flight_time < m_time_missile_takeoff)
	{
		//ADRC ����
		para_y_eso_takeoff.y = (m_nbz + para_y_eso_takeoff.k*m_wy/RTOA);
		para_y_eso_takeoff.uy = m_urh_record/RTOA;
		y_eso_calc_takeoff(3.4611, 16.4631, 5.0);
		
		m_uh_adrc = RTOA*para_y_eso_takeoff.z2/ 16.4631;

		//���ģ�͹��ؿ���
		// [kp ki kw] = 8.0015   11.2768    0.6982
		m_knph = -8.0015;
		m_knih = -11.2768;
		m_k5h = 0.6982;

		m_unph = m_knph*(m_nbz - m_nz_command);
		m_unih = m_knih*(m_nbz - m_nz_command)*STEP_5ms; 
		m_u5h = m_k5h * (m_wy - m_wy_command);
		//�����޷�����������

		//Ѳ��״̬��ֵ
		para_y_eso_cruise.z1 = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.z1_pre = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.z2 = m_urh_record/RTOA*16.4631;
		para_y_eso_cruise.z2_pre= para_y_eso_cruise.z2;
		para_y_eso_cruise.y = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.y_pre = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
	}
	//Ѳ��״̬
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{	
		//ADRC ����
		para_y_eso_cruise.y = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.uy = m_urh_record/RTOA;
		y_eso_calc_cruise(3.4611, 16.4631, 5.0);
		
		m_uh_adrc = RTOA*para_y_eso_cruise.z2/ 16.4631;

		//���ģ�͹��ؿ���
		// [kp ki kw] = 8.0015   11.2768    0.6982
		m_knph = -8.0015;
		m_knih = -11.2768;
		m_k5h = 0.6982;

		m_unih = m_knih*(m_nbz - m_nz_command)*STEP_5ms; 
		m_unph = m_knph*(m_nbz - m_nz_command);
		m_u5h = m_k5h * (m_wy - m_wy_command);
		//�����޷�����������

		//���״̬��ֵ
		para_y_eso_attack.z1 = (m_nbz + para_y_eso_attack.k*m_wy/RTOA);
		para_y_eso_attack.z1_pre = (m_nbz + para_y_eso_attack.k*m_wy/RTOA);
		para_y_eso_attack.z2 = m_urh_record/RTOA*16.4631;
		para_y_eso_attack.z2_pre= para_y_eso_attack.z2;
		para_y_eso_attack.y = (m_nbz + para_y_eso_attack.k*m_wy/RTOA);
		para_y_eso_attack.y_pre = (m_nbz + para_y_eso_attack.k*m_wy/RTOA);
	}
	//ĩ�Ƶ�״̬
	else
	{
		//ADRC ����
		para_y_eso_attack.y = (m_nbz + para_y_eso_attack.k*m_wy/RTOA);
		para_y_eso_attack.uy = m_urh_record/RTOA;
		y_eso_calc_attack(3.4611, 16.4631, 5.0);
		
		m_uh_adrc = RTOA*para_y_eso_attack.z2/ 16.4631;

		//���ģ�͹��ؿ���
		// [kp ki kw] = 8.0015   11.2768    0.6982
		m_knph = -8.0015;
		m_knih = -11.2768;
		m_k5h = 0.6982;

		m_unih = m_knih*(m_nbz - m_nz_command)*STEP_5ms; 
		m_unph = m_knph*(m_nbz - m_nz_command);
		m_u5h = m_k5h * (m_wy - m_wy_command);
		//�����޷�����������

		//���������תѲ��״̬��׼��
		para_y_eso_cruise.z1 = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.z1_pre = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.z2 = m_urh_record/RTOA*16.4631;
		para_y_eso_cruise.z2_pre= para_y_eso_cruise.z2;
		para_y_eso_cruise.y = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
		para_y_eso_cruise.y_pre = (m_nbz + para_y_eso_cruise.k*m_wy/RTOA);
	}
	//���������������𣬴���
	m_uh_adrc = CFlightGlobalFun::Range(m_uh_adrc, 6.0);
	if(flight_time < m_time_separate_booster)
		m_u5h = CFlightGlobalFun::Range(m_u5h, 6.0);
	m_unih = CFlightGlobalFun::Range(m_unih, 5.0);
	m_unph = CFlightGlobalFun::Range(m_unph, 10.0);

	//�����ص�ѹ
	m_urh_zd = m_unih + m_unph + m_u5h + m_uh_adrc;
	//���ƶ�ֻ����΢�ֿ��� m_u5h;
	m_urh_zd = CFlightGlobalFun::Range(m_urh_zd, 15.0);
	//Ϊ�˲���
	//m_urh_zd = m_u5h;
	
	//��¼���Ƶ�ѹ
	m_urh_record = m_unih + m_unph + m_u5h + m_uh_adrc;
	m_urh_record = CFlightGlobalFun::Range(m_urh_record, 15.0);
}
void CMathControlYaw::Calc_Control_Gain_ADRC()
{

	return;
}

void CMathControlYaw::Calc_Control_Gain()
{
	//�޷���������
	double temp_velocity = 50.0;//��ֵ m/s
	double temp_hight = 1500.0;//��ֵ m
	double temp_mass = 133.0;//��ֵ kg
	double temp_q = 1500.0;//��ֵ Pa	

	//��ɶο��Ʋ�����
	double knih_staget1 = -0.10;//ת��Ϊ�Ƕȣ���Ӧ4.011
	double kwih_staget1 = -0.55;//���ٶȻ��֣�α���ǿ���
	double k5h_staget1	 = -0.14;//���ٶȿ���
	/******************************************************************
	 * ���Ʋ���������
	 * ��1���ٶ� < 30m/s  �� ʹ�� 30m/s ��Ӧ��������0�У�
	 * ��2���ٶȵ�λ��30��40��50��70 m/s �� ��Ӧ������ 0��1��2��3
	 * ��3���߶� > 3km    �� ʹ�� 3km ��Ӧ��������3�У�
	 * ��4���߶ȵ�λ��0��1000��2000��3000 m �� ��Ӧ������ 0��1��2��3
	 *****************************************************************/
	double hight_uh_stage1_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//�߶�
	double vel_uh_stage1_array[4] = {30.0, 40.0, 50.0, 70.0};//�ٶ�

	double knih_stage1_matrix[4][4] = {
	    {-0.3470, -0.1236, -0.0537, -0.0148},
    	{-0.4107, -0.1476, -0.0645, -0.0178},
    	{-0.4870, -0.1768, -0.0777, -0.0216},
    	{-0.5787, -0.2124, -0.0940, -0.0263}    // �߶� 3000m
	};

	// �໬������ kwi[�߶���][�ٶ���]
	double kwih_stage1_matrix[4][4] = {
	    {-2.2938,  0.2142,  1.0220,  1.3157},
    	{-3.1392, -0.0936,  0.9353,  1.3721},
    	{-4.1899, -0.5005,  0.8003,  1.4194},
    	{-5.4922, -1.0334,  0.6017,  1.4527}    // �߶� 3000m
	};

	// ���ٶ����� k5[�߶���][�ٶ���]
	double k5h_stage1_matrix[4][4] = {
	    {-0.6896, -0.3825, -0.2411, -0.1191},
    	{-0.7633, -0.4240, -0.2677, -0.1327},
    	{-0.8465, -0.4709, -0.2977, -0.1480},
    	{-0.9407, -0.5240, -0.3317, -0.1654}    // �߶� 3000m
	};
	//����ʱ�䳣����0.30s
	

	//Ѳ�ɹ����п��Ʋ�����
	double knih_staget2 = -0.10;//ת��Ϊ�Ƕȣ���Ӧ4.011
	double kwih_staget2 = -0.46;//���ٶȻ��֣�α���ǿ���
	double k5h_staget2  = -0.083;//���ٶȿ���
	/******************************************************************
	 * ���룺���� mass(kg)����ѹ q(Pa)
	 * ������ֵ�㣺102, 133, 165 kg
	 * ��ѹ��ֵ�㣺500, 1000, 1500, 2000, 2500, 3000 Pa
	 * �����Kp���������棩��Kd��΢�����棩
	 * ����ṹ��Kp[������][��ѹ��]��Kd[������][��ѹ��]
	 *****************************************************************/
	// ������ֵ���������� 0,1,2��
	double mass_uh_stage2_array[3] = {102.0, 133.0, 165.0};
	// ��ѹ��ֵ���������� 0~5��
	double q_uh_stage2_array[6] = {500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0};
	//����һ����������ѹ�Կ��Ʋ���Ӱ�첻�󣬲���������Ϊ���룬����һά���Ʋ�����ֵ��
	//double knih_stage2_array[3] = {-0.10, -0.10, -0.10};
	//double kwih_stage2_array[3] = {-0.4363, -0.4630, -0.4799};
	//double k5h_stage2_array[3] = {-0.1090, -0.0830, -0.0667};
	//����ʱ�䳣����0.25s
	// ���ػ������� [����][��ѹ]
	double knih_stage2_matrix[3][6] = {
	    {-0.1249f, -0.0977f, -0.0828f, -0.0731f, -0.0662f, -0.0609f},
    	{-0.1538f, -0.1233f, -0.1054f, -0.0935f, -0.0849f, -0.0783f},
    	{-0.1805f, -0.1480f, -0.1277f, -0.1139f, -0.1037f, -0.0959f}    // mass = 165kg
	};
	// �໬������ [����][��ѹ]
	double kwih_stage2_matrix[3][6] = {
	    {-0.0000f, -0.0114f, -0.0231f, -0.0270f, -0.0283f, -0.0287f},
    	{-0.0763f, -0.1163f, -0.1162f, -0.1112f, -0.1057f, -0.1006f},
    	{-0.1865f, -0.2194f, -0.2094f, -0.1963f, -0.1843f, -0.1740f}    // mass = 165kg
	};
	// ���ٶ����� [����][��ѹ]
	double k5h_stage2_matrix[3][6] = {
	    {-0.5722f, -0.4038f, -0.3284f, -0.2834f, -0.2525f, -0.2297f},
    	{-0.5780f, -0.4079f, -0.3317f, -0.2862f, -0.2550f, -0.2320f},
    	{-0.5818f, -0.4105f, -0.3338f, -0.2880f, -0.2566f, -0.2334f}    // mass = 165kg
	};

	//ĩ�Ƶ��ο��Ʋ������̶�����α��������·���ؿ��ƣ��� 5rad-1�ջ�������1.5rad-1������ת��Ƶ�ʣ�
	//�������� knih
	//���ٶ����� kwih
	//α�໬������ k5h
	double knih_staget3 = 0.12;//ת��Ϊ�Ƕȣ���Ӧ4.011
	double kwih_staget3 = 0.55;//���ٶȻ��֣�α���ǿ���
	double k5h_staget3  = 0.10;//���ٶȿ���
	/******************************************************************
	 * ���Ʋ���������
	 * ��1���ٶ� < 40m/s  �� ʹ�� 40m/s ��Ӧ��������0�У�
	 * ��2���ٶȵ�λ��40��50��60��70 m/s �� ��Ӧ������ 0��1��2��3
	 * ��3���߶� > 3km    �� ʹ�� 3km ��Ӧ��������3�У�
	 * ��4���߶ȵ�λ��0��1000��2000��3000 m �� ��Ӧ������ 0��1��2��3
	 *****************************************************************/
	double hight_uh_stage3_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//�߶�
	double vel_uh_stage3_array[4] = {40.0, 50.0, 60.0, 70.0};//�ٶ�
	// ���ػ������� kni[�߶���][�ٶ���]
	double knih_stage3_matrix[4][4] = {
        {-0.1102f, -0.0920f, -0.0786f, -0.0684f},
		{-0.1142f, -0.0958f, -0.0820f, -0.0714f},
		{-0.1184f, -0.0997f, -0.0855f, -0.0746f},
		{-0.1226f, -0.1038f, -0.0893f, -0.0781f} // �߶� 3000m
	};
	// �໬������ kwi[�߶���][�ٶ���]
	double kwih_stage3_matrix[4][4] = {
	    {-0.0179f, -0.0285f, -0.0306f, -0.0299f},
    	{-0.0337f, -0.0438f, -0.0448f, -0.0429f},
    	{-0.0516f, -0.0615f, -0.0612f, -0.0579f},
    	{-0.0718f, -0.0817f, -0.0801f, -0.0753f}   // �߶� 3000m
	};
	// ���ٶ����� k5[�߶���][�ٶ���]
	double k5h_stage3_matrix[4][4] = {
	    {-0.4012f, -0.3216f, -0.2683f, -0.2301f},
    	{-0.4225f, -0.3387f, -0.2826f, -0.2424f},
    	{-0.4453f, -0.3570f, -0.2979f, -0.2555f},
    	{-0.4698f, -0.3767f, -0.3143f, -0.2696f}    // �߶� 3000m
	};
	//����ʱ�䳣����0.30s
	
	//����������ǰ��΢�ֿ���
	if(flight_time < m_time_separate_booster)
	{
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_uh_stage1_array[3], vel_uh_stage1_array[0]);
		temp_hight = CFlightGlobalFun::Range2(m_hz, hight_uh_stage1_array[3], hight_uh_stage1_array[0]);

		m_knih = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage1_array, vel_uh_stage1_array, &knih_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_kwih = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage1_array, vel_uh_stage1_array, &kwih_stage1_matrix[0][0], temp_hight, temp_velocity); 
		m_k5h = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage1_array, vel_uh_stage1_array, &k5h_stage1_matrix[0][0], temp_hight, temp_velocity);
		
		m_k5h *= 2.0;//���ƶ��񵴣���������
	}
	//Ѳ���Σ�����������󣬽���ĩ�Ƶ�ǰ
	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	//else if(flight_time < m_time_combat_status)
	{
		temp_q = CFlightGlobalFun::Range2(m_mass, q_uh_stage2_array[5], q_uh_stage2_array[0]);
		temp_mass = CFlightGlobalFun::Range2(m_mass, mass_uh_stage2_array[2], mass_uh_stage2_array[0]);

		m_knih = CFlightGlobalFun::LAQL2(3, 6, mass_uh_stage2_array, q_uh_stage2_array, &knih_stage3_matrix[0][0], temp_mass, temp_q);
		m_kwih = CFlightGlobalFun::LAQL2(3, 6, mass_uh_stage2_array, q_uh_stage2_array, &kwih_stage3_matrix[0][0], temp_mass, temp_q);
		m_k5h = CFlightGlobalFun::LAQL2(3, 6, mass_uh_stage2_array, q_uh_stage2_array, &k5h_stage3_matrix[0][0], temp_mass, temp_q);
		
		//m_knih =  0.3;	//��ͨ�����ػ�·�����ػ���
		//m_kwih =  -0.8;	//��ͨ�����ػ�·�����ٶȻ���
		//m_k5h  =  0.25;	//��ͨ�����ػ�·�����ٶȱ���(����)
	}
	//ĩ�Ƶ�����
	else
	{
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_uh_stage3_array[3], vel_uh_stage3_array[0]);
		temp_hight = CFlightGlobalFun::Range2(m_hz, vel_uh_stage3_array[3], vel_uh_stage3_array[0]);

		m_knih = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage3_array, vel_uh_stage3_array, &knih_stage3_matrix[0][0], temp_hight, temp_velocity);
		m_kwih = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage3_array, vel_uh_stage3_array, &kwih_stage3_matrix[0][0], temp_hight, temp_velocity); 
		m_k5h = CFlightGlobalFun::LAQL2(4, 4, hight_uh_stage3_array, vel_uh_stage3_array, &k5h_stage3_matrix[0][0], temp_hight, temp_velocity);
	}

	//�߼����Ѿ��Ǹ�����
	m_knih = m_knih;	//��
	m_kwih = - m_kwih;	//��
	m_k5h = - m_k5h;	//��
}

void CMathControlYaw::Calc_Control_Commond()
{
	//���������ٶ�: ���ƺ����ȶ���BTTת��(������ٶȡ����ŷ�)��ĩ����̬�ȶ�
	if(flight_time <= (m_time_separate_booster + 1.0))
	{
		m_wy_command = 0.0;
	}
	//Ѳ���Σ���ת�ǲ���������أ���������ƫ�Ǳ仯�ʣ�������Ǳ仯��ָ��
	else
	{
		//��ת�ǹ��Ʋ�����أ���һ�����Ƶ���ƫ�Ǳ仯��
		//m_wy_command = - m_g * RTOA * sin(m_gama / RTOA) / m_v;//������Ǳ仯��ָ�������̬��Ӧʱ�䣬��δ������ٶ�
		m_wy_command = - m_g * RTOA * sin(m_gama_command_compensate / RTOA) / m_v;
		
		//�����ת���ٶ�
		//˵�������������߷��в������ʱ������һ��������alpha_b
		//BTTˮƽ����ʱ��1)�˶�ѧ��ϲ����Ĳ໬�ǣ�����beita_dot = wx*alpha
		//               2)����ѧ��ϲ����Ĳ໬�ǣ�����beita_dot = Kwx_alpha*wx*alpha;
		double temp_Kwx_comp = -0.1;
		if(fabs(m_wx) > 5.0)
		{	
			m_wy_command_comp = temp_Kwx_comp * m_wx;
		}
		else
		{
			m_wy_command_comp = 0.0;
		}
		//ͻ��ƽ����5ms���ٶȱ仯������0.05deg/s����10deg/s^2
		if( m_wy_command_comp - m_wy_command_comp_record < -0.05)
		{
			m_wy_command_comp_record -= 0.05;
		}
		else if( m_wy_command_comp - m_wy_command_comp_record > 0.05)
		{
			m_wy_command_comp_record += 0.05;
		}
		else
		{
			m_wy_command_comp_record = m_wy_command_comp;
		}
		
		//Ϊ�˲��ԣ������й�ת����������ſ�???...
		m_wy_command_comp_record = 0.0;
		
		m_wy_command = m_wy_command + m_wy_command_comp_record; 
		//���ٶȽ�Сʱ��������
	}

	//����������:  ���뵼��ǰ������ָ��Ϊ�㣬��ֻ���в�������
	if(flight_time < (m_time_combat_status + m_time_combat_delay))
	//if(flight_time < m_time_combat_status)
	{
		m_nz_command = 0.0;
	}
	else
	{
		m_nz_command = m_nz_command_guidance;
	}
}
/*
void CMathControlYaw::Monitor_Data()
{
	extern CSimMonitor sim_monitor;
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_wy_command,"wycx",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(-m_nz_command,"nzc",ENUM_FILE_CONTROL1);
		sim_monitor.Get_Variable(m_urh_zd,"m_urh_zd",ENUM_FILE_CONTROL1);

		//�������
		sim_monitor.Get_Variable(m_wy,"wy",ENUM_FILE_CONTROL1);	//������ٶ�
		sim_monitor.Get_Variable(m_nbz,"nzb",ENUM_FILE_CONTROL1);	//������أ�ĩ�Ƶ����ظ���
	}
	if (sim_monitor.flag_monitor3_valid)
	{
		sim_monitor.Get_Variable(m_k5h,"k5h",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_knih,"knih",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_kwih,"kwih",ENUM_FILE_AERO1);
	}
}*/
