#include <cstring>
#include "control_roll.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
#include "../port/flightPort.h"

CMathControlRoll::CMathControlRoll()
{
	p_st_roll_control_input = NULL;
	p_st_roll_control_output = NULL;
	//p_st_debug_monitor = NULL;
	m_k2g = 0.0;
	m_k5g = 0.0;
	m_k4g = 0.0;
	m_k3g = 0.0;
	m_k7g = 0.0;
	m_k6g = 0.0;
	m_w0 = 0.0;
	m_k = 0.0;
	m_z1 = 0.0;
	m_z2 = 0.0;
	m_last_k2g = 0.0;
	m_last_k5g = 0.0;
	m_last_k4g = 0.0;
	m_tar_k2g = 0.0;
	m_tar_k5g = 0.0;
	m_tar_k4g = 0.0; 
	m_u2g = 0.0;
	m_u5g = 0.0;
	m_u4g = 0.0;
	m_ug_adrc = 0.0;
	m_u3g = 0.0;
	m_u7g = 0.0;
	m_u6g = 0.0;
	m_u25g = 0.0;
	m_urg_record = 0.0;
	m_uz = 0.0;
	m_uz1_record = 0.0;
	m_uz2_record = 0.0;
	m_sz = 0.0;
	m_mass = 0.0;
	m_v = 0.0;
	m_hz = 0.0;
	m_vnz = 0.0;
	m_g = 0.0;
	m_gama = 0.0;
	m_gama_record1 = 0.0;
	m_gama_record2 = 0.0;
	m_gama_record3 = 0.0;
	m_gama_command = 0.0;
	m_gama_command_compensate = 0.0;
	m_gama_command_compensate_out = 0.0;
	m_gama_command_guidance = 0.0;
	m_gama_turn_nominal = 0.0;
	m_wx = 0.0;
	m_wx_command = 0.0;
	m_turn_angle = 0.0;
	m_turn_radius = 0.0;
	m_target_velocity = 0.0;
	m_time_start = 0.0;
	m_dqh = 0.0;
	m_flag_record = false;
	m_flag_changing = false;
	m_flag_turning = false;
	m_flag_distance_overlimit = false;
	m_flag_launch_turn = false;
	m_time_control = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_engine_start = MAX_TIME;
	m_time_missile_takeoff = MAX_TIME;
	m_time_altitude_control = MAX_TIME;
	m_time_launch_turn_ok = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
	m_time_combat_dive_sidectrl = MAX_TIME;
	m_time_turn_in_start = MAX_TIME;
	m_time_turn_in_end = MAX_TIME;
	m_time_turn_out_start = MAX_TIME;
	m_time_turn_out_end = MAX_TIME;
	m_time_v70 = MAX_TIME;
	m_time_v120 = MAX_TIME;
	m_time_v60 = MAX_TIME;
	m_time_v100 = MAX_TIME;
	m_time_v140 = MAX_TIME;
	m_time_BTT_guidance_in = MAX_TIME;
}

void CMathControlRoll::Initial()
{
	wx_eso_init_takeoff();
	wx_eso_init_cruise();
}

void CMathControlRoll::wx_eso_init_takeoff()
{
	para_wx_eso_takeoff.z1 = 0.0;
    para_wx_eso_takeoff.z2 = 0.0;
    para_wx_eso_takeoff.z1_pre = 0.0;
    para_wx_eso_takeoff.z2_pre = 0.0;
    para_wx_eso_takeoff.ux = 0.0;
    para_wx_eso_takeoff.wx = 0.0;
    para_wx_eso_takeoff.wx_pre = 0.0;
    para_wx_eso_takeoff.Ts = 0.005;
}

void CMathControlRoll::wx_eso_calc_takeoff(double K0, double T0, double Keso)
{
	double Ts       = para_wx_eso_takeoff.Ts;
    double z1_pre   = para_wx_eso_takeoff.z1_pre;
    double z2_pre   = para_wx_eso_takeoff.z2_pre;
    double wx_meas  = para_wx_eso_takeoff.wx;
    double delta_u  = para_wx_eso_takeoff.ux;

    double w0 = 1.0f / T0;
    double w_eso = Keso * w0;

    double a1 = (2.0f * T0 - Ts) / (2.0f * T0 + Ts);
    double b0 = (K0 * Ts) / (2.0f * T0 + Ts);
    double b1 = b0;

    double e = z1_pre - wx_meas;

    double z1 = a1 * z1_pre
             + b0 * delta_u
             + b1 * delta_u
             + z2_pre * Ts
             - w_eso * Ts * e;

    double z2 = z2_pre - w_eso * w_eso * Ts * e;

    //double wx_obs = z1;
    //double dis_obs = z2;

    para_wx_eso_takeoff.z1_pre = para_wx_eso_takeoff.z1;
    para_wx_eso_takeoff.z2_pre = para_wx_eso_takeoff.z2;

    para_wx_eso_takeoff.z1 = z1;
    para_wx_eso_takeoff.z2 = z2;


    para_wx_eso_takeoff.wx_pre = para_wx_eso_takeoff.wx;
}
void CMathControlRoll::wx_eso_init_cruise()
{
	para_wx_eso_cruise.z1 = 0.0;
    para_wx_eso_cruise.z2 = 0.0;
    para_wx_eso_cruise.z1_pre = 0.0;
    para_wx_eso_cruise.z2_pre = 0.0;
    para_wx_eso_cruise.ux = 0.0;
    para_wx_eso_cruise.wx = 0.0;
    para_wx_eso_cruise.wx_pre = 0.0;
    para_wx_eso_cruise.Ts = 0.005;
}
void CMathControlRoll::wx_eso_calc_cruise(double K0, double T0, double Keso)
{
	double Ts       = para_wx_eso_cruise.Ts;
    double z1_pre   = para_wx_eso_cruise.z1_pre;
    double z2_pre   = para_wx_eso_cruise.z2_pre;
    double wx_meas  = para_wx_eso_cruise.wx; 
    double delta_u  = para_wx_eso_cruise.ux;

    double w0 = 1.0f / T0;
    double w_eso = Keso * w0;

    double a1 = (2.0f * T0 - Ts) / (2.0f * T0 + Ts);
    double b0 = (K0 * Ts) / (2.0f * T0 + Ts);
    double b1 = b0;

    double e = z1_pre - wx_meas;

    double z1 = a1 * z1_pre
             + b0 * delta_u
             + b1 * delta_u
             + z2_pre * Ts
             - w_eso * Ts * e;

    double z2 = z2_pre - w_eso * w_eso * Ts * e;

    double wx_obs = z1;
    double dis_obs = z2;


    para_wx_eso_cruise.z1_pre = para_wx_eso_cruise.z1;
    para_wx_eso_cruise.z2_pre = para_wx_eso_cruise.z2;

    para_wx_eso_cruise.z1 = z1;
    para_wx_eso_cruise.z2 = z2;

    para_wx_eso_cruise.wx_pre = para_wx_eso_cruise.wx;
}
	
void CMathControlRoll::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	//Monitor_Data();
}
void CMathControlRoll::Get_Data()
{
	m_sz = p_st_roll_control_input->sz;
	m_mass = p_st_roll_control_input->mass;
	m_v = p_st_roll_control_input->v;
	m_hz = p_st_roll_control_input->hz;
	m_vnz = p_st_roll_control_input->vnz;
	m_g = p_st_roll_control_input->g;
	m_gama = p_st_roll_control_input->gama;
	m_wx = p_st_roll_control_input->wx;
	m_dqh = p_st_roll_control_input->dqh;
	m_gama_command_guidance = p_st_roll_control_input->gama_command_guidance;
	m_turn_angle = p_st_roll_control_input->angle_zw;
	m_turn_radius = p_st_roll_control_input->radius_zw;
	m_gama_turn_nominal = p_st_roll_control_input->gama_turn_nominal;
	m_target_velocity = p_st_roll_control_input->target_velocity;
	m_time_control = p_st_roll_control_input->time_control;
	m_time_separate_booster = p_st_roll_control_input->time_separate_booster;
	m_time_engine_start = p_st_roll_control_input->time_engine_start;
	m_time_missile_takeoff = p_st_roll_control_input->time_missile_takeoff;
	m_time_altitude_control = p_st_roll_control_input->time_altitude_control;
	m_time_launch_turn_ok = p_st_roll_control_input->time_launch_turn_ok;
	m_time_combat_status = p_st_roll_control_input->time_combat_status;
	m_time_turn_in_start = p_st_roll_control_input->time_turn_in_start;
	m_time_turn_in_end = p_st_roll_control_input->time_turn_in_end;
	m_time_turn_out_start = p_st_roll_control_input->time_turn_out_start;
	m_time_turn_out_end = p_st_roll_control_input->time_turn_out_end;
	m_flag_launch_turn = p_st_roll_control_input->flag_launch_turn;
}
void CMathControlRoll::Send_Data()
{
	p_st_roll_control_output->u25g = m_u2g + m_u5g;//PD
	p_st_roll_control_output->u4g = m_u4g;			//I
	p_st_roll_control_output->ug_adrc = m_ug_adrc;	//ADRC
	p_st_roll_control_output->z2_adrc = m_z2;
	p_st_roll_control_output->uz = m_uz;
	p_st_roll_control_output->time_combat_delay = m_time_combat_delay;
	p_st_roll_control_output->gama_command_compensate = m_gama_command_compensate_out;
	p_st_roll_control_output->gama_command = m_gama_command;
	p_st_roll_control_output->wx_command = m_wx_command;
}
void CMathControlRoll::Calc_Data()
{
	// 1.��������
	Calc_Control_Gain(); 
	
	// 2.��ƫ�������ٶȼ���
	double delta_sz = 0.0;
	double delta_vz = 0.0;
	// 2.1 ����ȶ�ǰ�������в�ƫ����
	//20260521 ����������ǰ����Ϊ����ȶ�ǰ
	//if (flight_time <= m_time_separate_booster) 
	if (flight_time <= m_time_missile_takeoff)
	{
		delta_sz = 0.0;
		delta_vz = 0.0;	
	}
	//����ȶ���
	//˵��:Ѳ����ĩ�Ƶ����̣�����ֱ�����㣬�����⴦��
	else
	{
		//ת�����
		if (flight_time > m_time_turn_in_start 
			&& flight_time <= m_time_turn_out_end)
		{
			//����ת�䣬�����в�ƫ����
			if (m_flag_launch_turn)
			{
				delta_sz = 0.0;
				delta_vz = 0.0;
			}
			//����ת�䣬����Բ��ƫ
			else
			{
				//����ƫ����������ת��ʱ��ƫ��Ϊ��
				//delta_sz = - (m_sz - m_turn_radius) * CFlightGlobalFun::FSign(m_turn_angle);
				//��ƫ�����޸�
				//delta_sz = m_sz - m_turn_radius;

				//��ƫ����޷�
				delta_sz = CFlightGlobalFun::Range(m_sz, (10.0 * m_k7g / m_k3g)); //Լ30m
				
				//ת����̣���ƫ���󣬲�ƫ��������
				if(fabs(delta_sz) > (5.0 * m_k7g / m_k3g)) //Լ15m
				{
					m_flag_distance_overlimit = true;
				}

				//��ƫ�仯��
				delta_vz = m_vnz; 
			}
		}
		//ֱ������ȡֱ����ƫ
		else
		{	
			delta_sz = m_sz;
			delta_vz = m_vnz;
		}
		
		//�����ٶ��޷�
		delta_vz = CFlightGlobalFun::Range(delta_vz, 10.0); //��Ӧ10deg��ת�ǣ�10.0/m_k7g->10.0
	}
	

	// 3.��ƫ�������ٶȼ���
	// 3.1 ��ƫPID���ƣ�����ת��ָ���������
	// 3.1.1��ƫ����I
	m_u6g += m_k6g * delta_sz * STEP_5ms;
	m_u6g = CFlightGlobalFun::Range(m_u6g, 4.0);
	if(m_flag_distance_overlimit	//��ƫ����
		|| (fabs(delta_sz) > (45.0 * m_k7g / m_k3g))//��ƫPD���ƣ���ת�ǳ���
		//|| (flight_time < m_time_launch_turn_ok)
		//|| (m_flag_launch_turn) ����ת��������½������ߺ�
		|| (flight_time < m_time_missile_takeoff)	//������ǰ
		|| (m_flag_launch_turn)//����ת�������
		|| ((flight_time > m_time_turn_in_start)&&(flight_time <= m_time_turn_in_end))//ת��������
		|| ((flight_time > m_time_turn_out_start)&&(flight_time <= m_time_turn_out_end)))//��ת�����
	{
		m_u6g = 0.0;
	}
	// 3.1.2��ƫPD����
	m_u3g = m_k3g * delta_sz;
	m_u3g = CFlightGlobalFun::Range(m_u3g,(45.0 * m_k7g));
	m_u7g = m_k7g * delta_vz;

	// 3.1.3���·��->m_uz1
	double m_uz1 = m_u3g + m_u7g + m_u6g;	
	//m_uz1 = - m_uz1;
	
	// 3.2 �޷�->m_uz1
	m_uz1 = CFlightGlobalFun::Range(m_uz1, 10.0);/// 30.0->10.0
	if (flight_time > m_time_turn_in_start
		&& flight_time <= m_time_turn_out_end)
	{
		m_uz1 = CFlightGlobalFun::Range(m_uz1, 10.0);/// 15.0->10.0
	}
	
	// 3.3 �����������->uz_temp
	double uz_temp = 0.0;
	//������ǰ�������в�ƫ����
	if(flight_time < m_time_missile_takeoff)
	{
		uz_temp = 0.0;
	}
	//�����ɺ��������ƫ����
	else if(flight_time < m_time_missile_takeoff + 3.0)
	{
		uz_temp = m_uz1 * (flight_time - m_time_missile_takeoff)/3.0;
	}
	else if(m_flag_launch_turn)
	{
		//ֱ����ƫ���ƣ����ɵ���ƫ�޿���
		if(flight_time < m_time_turn_in_start + 1.0)
		{
			uz_temp = m_uz1_record * (1.0 + m_time_turn_in_start - flight_time)/1.0;
		}
		else
		{
			uz_temp = 0.0;
		}
		//��¼����ת�����ʱ�̣���ƫ���Ƽ�¼
		m_uz2_record = uz_temp;
	}
	//����ת���������
	else if( (flight_time > m_time_launch_turn_ok)&&(flight_time < m_time_launch_turn_ok + 3.0) ) 
	{
		uz_temp = m_uz1 * (flight_time - m_time_launch_turn_ok)/3.0;
	}
	else
	{
		//��ʼֱ���������ɹ��ɺ�ֱ����ƫ����
		if (flight_time <= m_time_turn_in_start) 
		{
			uz_temp = m_uz1;
			m_uz1_record = m_uz1;
		}

		else if (flight_time <= m_time_turn_in_end)
		{
			uz_temp = 0.0;
		}
		//20260716 end��ʼ���뺽����ƫ�ܿ���
		else if (flight_time < (m_time_turn_in_end + 1.0))	
		{
			//��������ƫ����
			uz_temp = m_uz1 * (flight_time - m_time_turn_in_end);
		}
		//����ת��
		else if (flight_time <= m_time_turn_out_end)
		{
			uz_temp = m_uz1;
			
			//���º���ת�䣬��ƫ���Ƽ�¼
			m_uz2_record = m_uz1;
		}
		//����ת�� �� ����ת�� �� ֱ��
		else if (flight_time < (m_time_turn_out_end + 1.0))
		{
			uz_temp = m_uz2_record * (1.0 + m_time_turn_out_end - flight_time)
				+ m_uz1 * (flight_time - m_time_turn_out_end);
		}
		//ֱ��
		else
		{
			uz_temp = m_uz1;
			m_uz1_record = m_uz1;
		}
	}
	
	m_uz = uz_temp;
	
	// 4.�����ڻ�·ָ���ת�Ǽ����ٶ�ָ��
	Calc_Control_Commond();

	// 5.�ڻ�·����
	// 5.1 ��ת�Ǹ���PD����
	m_u2g = m_k2g * (m_gama - m_gama_command);
	m_u5g = m_k5g * (m_wx - m_wx_command);
	
	//����
	m_k4g = 0.01;
	if(flight_time < m_time_missile_takeoff) 
	{
		m_u4g = 0.0;
	}
	else if(flight_time < m_time_combat_status) 
	{
		m_u4g += m_k4g * (m_gama - m_gama_command) * STEP_5ms;
		m_u4g = CFlightGlobalFun::Range(m_u4g, 12.0);
	}
	else
	{
		m_u4g = 0.0;
	}


	// 5.2 ADRC����
	//����ǰ
	if(flight_time < m_time_control)
	{
		m_ug_adrc = 0.0;
		m_u2g = 0.0;
		m_u5g = 0.0;
		m_uz = 0.0;
		m_urg_record = 0.0;
	}
	//��ɹ���
	//else if(flight_time < m_time_separate_booster + 1.0) 
	else if(flight_time < m_time_missile_takeoff) 	 //20260521
	{
		m_ug_adrc = 0.0;

		para_wx_eso_cruise.z1 = m_wx/RTOA;
		para_wx_eso_cruise.z1_pre = m_wx/RTOA;
		//para_wx_eso_cruise.z2 = m_ug_adrc/RTOA*55.0/0.4;
		para_wx_eso_cruise.z2 = (m_u2g + m_u5g)/RTOA*55.0/0.4;
		para_wx_eso_cruise.z2_pre= para_wx_eso_cruise.z2;
		para_wx_eso_cruise.wx = m_wx/RTOA;
		para_wx_eso_cruise.wx_pre = m_wx/RTOA;
	}
	//�����ɺ�Ѳ���������
	else
	{
		//����ţ�����BTTĩ�Ƶ�����ת��ָ�����
		//С���ţ�����STTĩ�Ƶ�����ת��Ϊ�����
		para_wx_eso_cruise.wx = m_wx/RTOA;
		para_wx_eso_cruise.ux = m_urg_record/RTOA;
		wx_eso_calc_takeoff(55.0, 0.4, 4.0);
		m_ug_adrc = RTOA*0.4*para_wx_eso_cruise.z2/ 55.0;
	}
	//ĩ�Ƶ��Σ�����55m/s�ٶȣ��Ż�ģ��
	
	//������������أ����Կ��Ŷ�ȡ�෴��
	m_urg_record = -(m_ug_adrc + m_u2g + m_u5g);
}

void CMathControlRoll::Calc_Control_Gain()
{
	//�޷���������
	double temp_velocity = 50.0;//��ֵ m/s
	double temp_hight = 1500.0;//��ֵ m
	double temp_mass = 133.0;//��ֵ kg
	double temp_q = 1500.0;//��ֵ Pa	
	
	//��ɶο��Ʋ���: �̶��ջ������ĸ�����PD���ƣ��߶ȡ��ٶȶ�ά��ֵ
	double k2g_stage1 = 0.7730;
	double k5g_stage1 = 0.1313;
	double hight_ug_stage1_array[4] = {0.0, 1000.0, 2000.0, 3000.0};//�߶�
	double vel_ug_stage1_array[7] = {20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0};//�ٶ�


	// �ջ������̶�Ϊ1.0Hz�����Ʋ�����ֵ��
	// ������������� Kp[�߶�][�ٶ�]
	double K2g_stage1_matrix[4][7] = {
	    {1.8519, 0.8231, 0.4630, 0.2963, 0.2058, 0.1512, 0.1157},
	    {2.0407, 0.9070, 0.5102, 0.3265, 0.2267, 0.1666, 0.1275},
	    {2.2537, 1.0017, 0.5634, 0.3606, 0.2504, 0.1840, 0.1409},
	    {2.4949, 1.1088, 0.6237, 0.3992, 0.2772, 0.2037, 0.1559}
	};
	// ������΢������ Kd[�߶�][�ٶ�]
	double K5g_stage1_matrix[4][7] = {
	    {0.7600, 0.3277, 0.1787, 0.1107, 0.0744, 0.0528, 0.0390},
	    {0.8420, 0.3642, 0.1992, 0.1239, 0.0835, 0.0595, 0.0441},
	    {0.9347, 0.4053, 0.2223, 0.1387, 0.0938, 0.0671, 0.0499},
	    {1.0395, 0.4519, 0.2486, 0.1555, 0.1054, 0.0756, 0.0565}
	};
	
	//���ƶβ���
	//if(flight_time < m_time_separate_booster + 1.0)
	if(flight_time < m_time_missile_takeoff)
	{

		temp_hight  = CFlightGlobalFun::Range2(m_hz, hight_ug_stage1_array[3], hight_ug_stage1_array[0]);
		temp_velocity = CFlightGlobalFun::Range2(m_v, vel_ug_stage1_array[6], vel_ug_stage1_array[0]);

		m_k2g = CFlightGlobalFun::LAQL2(4, 7, hight_ug_stage1_array, vel_ug_stage1_array, &K2g_stage1_matrix[0][0], temp_hight, temp_velocity);
		m_k5g = CFlightGlobalFun::LAQL2(4, 7, hight_ug_stage1_array, vel_ug_stage1_array, &K5g_stage1_matrix[0][0], temp_hight, temp_velocity);

		//��������һ������
		//m_k5g = 2.0*m_k5g;
		if(m_k5g < 0.15)
			m_k5g = 0.15;
	}
	//Ѳ���β���
	else if(flight_time < m_time_combat_status)
	{

		m_k2g = 0.6612;	//��Ӧ1.5Hz����9rad/s
		m_k5g = 0.1055;
		
		m_k3g = 1.47;
		m_k7g = 4.70;
		m_k6g = 0.07;//m_k3g*0.05Լ0.0735
		
		if(fabs(m_gama) > 30.0) // error less than 2.5%
		{
			double k_temp = fabs(m_gama / RTOA / tan(m_gama / RTOA));
			m_k3g = m_k3g * k_temp;
			m_k7g = m_k7g * k_temp;
		}
	}
	//ĩ�Ƶ���
	else
	{

		m_k2g = 0.6612;	//��Ӧ1.5Hz����9rad/s
		m_k5g = 0.1055;

	}
}
void CMathControlRoll::Calc_Control_Commond()
{
	//����ǰ
	if(flight_time < m_time_control)
	{
		m_gama_command = m_gama;	
		m_gama_command_compensate = 0.0;
	}
	//����������ǰ�������������ƫ����
	//else if(flight_time <= m_time_separate_booster)
	//20260521
	//����ȶ�ǰ�������������ƫ����
	else if(flight_time < m_time_missile_takeoff)
	{
		m_gama_command = 0.0;
		m_gama_command_compensate = 0.0;
		m_gama_record1 = m_gama;
	}
	//Ѳ���Σ������ƫ���Ƽ�ָ�����
	else if(flight_time < m_time_combat_status)
	{
		//�����ɵ�ֱ����ƫ���ƹ���
		//˵��1�������ɲ����½������ߣ���ʼ���ڽϴ��ƫ��m_uz�Ѿ���3s����
		//˵��2������ת����̣������в�ƫ���ƣ�m_uz�Ѿ�����Ϊ��
		if (flight_time <= m_time_turn_in_start)	
		{    
			m_gama_command = m_gama_record1 * exp(-(flight_time - m_time_missile_takeoff) * (flight_time - m_time_missile_takeoff) / 16.0) - m_uz/ m_k2g;//��ƫ����
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
			m_gama_record2 = m_gama_command;
		}
		//�����ת�ǹ��ɣ������в�ƫ����
		//˵��1������ת��ʱ���ӳ�ʼֱ����ƫ���ƹ�ת�� m_gama_record2 ���ɵ� ����ת��
		//˵��2������ת��ʱ���ӷǳ�ʼֱ����ƫ���ƹ�ת�� m_gama_record2 ���ɵ� ����ת��
		else if (flight_time <= m_time_turn_in_end)
		{
			m_gama_command = m_gama_record2 + ROLL_RATE_COMMAND * (flight_time - m_time_turn_in_start) * CFlightGlobalFun::FSign(m_turn_angle);	  
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 10.0);//�Ƕ�5deg->10deg
			m_gama_command_compensate = m_gama_command;
		}
		//���ת��ת������У�Բ����ƫ����
		else if (flight_time <= m_time_turn_out_start)
		{
			m_gama_command_compensate = m_gama_turn_nominal * CFlightGlobalFun::FSign(m_turn_angle);
			m_gama_command = m_gama_turn_nominal * CFlightGlobalFun::FSign(m_turn_angle) - m_uz/ m_k2g;//��ƫ����
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 10.0);//�Ƕ�5deg->10deg
		}
		//�����ת�ǹ��ɣ������в�ƫ����
		else if (flight_time <= m_time_turn_out_end)
		{
			m_gama_command = (m_gama_turn_nominal - ROLL_RATE_COMMAND * (flight_time - m_time_turn_out_start)) * CFlightGlobalFun::FSign(m_turn_angle);	
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, fabs(m_gama_turn_nominal) + 10.0);//�Ƕ�5deg->10deg
			m_gama_command_compensate = m_gama_command;
		}
		//ת�� ���ɵ� ֱ��
		else if(flight_time <= m_time_turn_out_end + 1.0)
		{
			m_gama_command = - m_uz / m_k2g * (flight_time - m_time_turn_out_end);// * (flight_time - m_time_turn_out_end);
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
		}
		//ֱ����ֱ����ƫ
		else
		{
			m_gama_command = - m_uz / m_k2g;
			m_gama_command = CFlightGlobalFun::Range(m_gama_command, ROLL_COMMAND_STATIC_LIMIT);
			m_gama_command_compensate = 0.0;
			m_gama_record2 = m_gama;
		}

		m_gama_record3 = m_gama_command;
		//����ĩ�Ƶ�ǰ����ת���������㣬����STT�Ƶ�
		//m_time_combat_delay = fabs(m_gama) / ROLL_RATE_COMMAND;
		m_time_combat_delay = fabs(m_gama_record3) / ROLL_RATE_COMMAND;
		if(m_time_combat_delay < 0.5) m_time_combat_delay = 0.5;
		if(m_time_combat_delay > 2.0) m_time_combat_delay = 2.0;
		m_time_BTT_guidance_in = flight_time;
	}

	else if(flight_time < m_time_combat_status + m_time_combat_delay)
	{	
		m_gama_command = m_gama_record3*(m_time_combat_delay + m_time_combat_status - flight_time)/m_time_combat_delay
					+ m_gama_command_guidance*(flight_time - m_time_combat_status)/m_time_combat_delay;
	}
	else
	{
		m_gama_command = m_gama_command_guidance;
	}

	//�޷��Ƕȣ�����Խ���ת��ԽС
 	double temp_gama_command_range = 45.0 + (ROLL_COMMAND_DYNMIC_LIMIT - 45.0)/(6*3600.0) * flight_time;	
	if(temp_gama_command_range > ROLL_COMMAND_DYNMIC_LIMIT)
	{
		temp_gama_command_range = ROLL_COMMAND_DYNMIC_LIMIT;
	}
	//��ת��ָ���޷�
	m_gama_command = CFlightGlobalFun::Range(m_gama_command, temp_gama_command_range);
	//������ת���޷�
 	m_gama_command_compensate = CFlightGlobalFun::Range(m_gama_command_compensate, temp_gama_command_range);
	if( fabs(m_gama_command_compensate) < 3.0)
	{
		m_gama_command_compensate_out = 0.0;
	}
	else if( fabs(m_gama_command_compensate) < ROLL_COMMAND_DYNMIC_LIMIT)
	{
		m_gama_command_compensate_out = m_gama_command_compensate - 3.0*CFlightGlobalFun::FSign(m_gama_command_compensate);
	}
	else
	{
		m_gama_command_compensate_out = ROLL_COMMAND_DYNMIC_LIMIT - 3.0*CFlightGlobalFun::FSign(m_gama_command_compensate);
	}

	//������� wx_cmd 
	if(flight_time <= m_time_combat_status)
	{
		if(flight_time < m_time_turn_in_start)
		{
			m_wx_command = 0.0;
		}
		else if(flight_time < m_time_turn_in_end)
		{
			m_wx_command = ROLL_RATE_COMMAND * CFlightGlobalFun::FSign(m_turn_angle); 
		}
		else if(flight_time < m_time_turn_out_start)
		{
			m_wx_command = 0.0;
		}
		else if(flight_time < m_time_turn_out_end)
		{
			m_wx_command = - ROLL_RATE_COMMAND * CFlightGlobalFun::FSign(m_turn_angle);
		}
		else
		{
			m_wx_command = 0.0;
		}
	}
	else
	{
		m_wx_command = 0.0;
	}
}

/*
void CMathControlRoll::Monitor_Data()
{
	extern CSimMonitor sim_monitor;
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_gama_command,"gamacx",ENUM_FILE_CONTROL1);// 1
		sim_monitor.Get_Variable(m_gama_command_compensate_out,"gamacxbc",ENUM_FILE_CONTROL1);// 3
		sim_monitor.Get_Variable(m_wx_command,"wxcx",ENUM_FILE_CONTROL1); // 4
		sim_monitor.Get_Variable(m_uz,"uz",ENUM_FILE_CONTROL1);//��ƫ������ 7
		sim_monitor.Get_Variable(m_z2,"x1",ENUM_FILE_CONTROL1);//ESO״̬��
		sim_monitor.Get_Variable(m_ug_adrc,"ug_adrc",ENUM_FILE_CONTROL1);//��ת��ADRC������ 6

		//����
		sim_monitor.Get_Variable(m_gama,"gama",ENUM_FILE_CONTROL1);//��ת�� 2
		sim_monitor.Get_Variable(m_wx,"wx",ENUM_FILE_CONTROL1);//��ת���ٶ� 5
		sim_monitor.Get_Variable(m_sz,"sz",ENUM_FILE_CONTROL1);//��ƫ 8
		sim_monitor.Get_Variable(m_vnz,"vz",ENUM_FILE_CONTROL1);//�����ٶ� 9
		
	}
	if (sim_monitor.flag_monitor3_valid)
	{
		sim_monitor.Get_Variable(flight_time,"time",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k2g,"k2g",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k5g,"k5g",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k4g,"k4g",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k3g,"k3g",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k7g,"k7g",ENUM_FILE_AERO1);
		sim_monitor.Get_Variable(m_k6g,"k6g",ENUM_FILE_AERO1);
	}
}
*/
