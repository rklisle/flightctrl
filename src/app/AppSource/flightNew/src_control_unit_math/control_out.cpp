#include <cstring>
#include "control_out.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
//extern CSimMonitor sim_monitor;
#include "../port/flightPort.h"

CMathControlOut::CMathControlOut()
{
	p_st_control_out_input = NULL;
	p_st_control_out_output = NULL;
	//p_st_debug_monitor = NULL;
	memset(m_u_control, 0, sizeof(double) * 6);
	m_gama = 0.0;
	m_urg = 0.0;
	m_urg_record = 0.0;
	m_urh = 0.0;
	m_urf = 0.0;
	m_urf_record = 0.0;
	m_u25g = 0.0;
	m_u4g = 0.0;
	m_ug_adrc = 0.0;
	m_uqkf = 0.0;
	m_u2f = 0.0;
	m_u5f = 0.0;
	m_u5h = 0.0;
	m_ugf = 0.0;
	m_urf_zd = 0.0;
	m_urh_zd = 0.0;
	m_time_control_on = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
}
void CMathControlOut::Initial()
{
	
}
void CMathControlOut::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	//Monitor_Data();
}

void CMathControlOut::Get_Data()
{
	m_gama = p_st_control_out_input->gama;
	m_u25g = p_st_control_out_input->u25g;
	m_u4g = p_st_control_out_input->u4g;
	m_ug_adrc = p_st_control_out_input->ug_adrc;
	
	m_uqkf = p_st_control_out_input->uqkf; 
	m_u2f = p_st_control_out_input->u2f;
	m_u5f = p_st_control_out_input->u5f;
	
	m_u5h = p_st_control_out_input->u5h;
	m_ugf = p_st_control_out_input->ugf;
	m_urf_zd = p_st_control_out_input->urf_zd;
	m_urh_zd = p_st_control_out_input->urh_zd;
	m_time_control_on = p_st_control_out_input->time_control_on;
	m_time_separate_booster = p_st_control_out_input->time_separate_booster;
	m_time_combat_status = p_st_control_out_input->time_combat_status;
	m_time_combat_delay = p_st_control_out_input->time_combat_delay;
}

void CMathControlOut::Send_Data()
{
	//��Ƕȵ�����Ƕ�ת��
	p_st_control_out_output->u1 = RudAgl2MotorAglH1(m_u_control[0]);//�����ʾ��ͼ���෴,��I��˳ʱ�루������Ƕ�
	//p_st_control_out_output->u1 *= -1.0;
	p_st_control_out_output->u2 = RudAgl2MotorAglH2(m_u_control[1]);//�����ʾ��ͼ���෴
	//p_st_control_out_output->u2 *= -1.0;
	p_st_control_out_output->u3 = RudAgl2MotorAglH1(m_u_control[2]);//�����ʾ��ͼ����ͬ
	p_st_control_out_output->u4 = RudAgl2MotorAglH2(m_u_control[3]);//�����ʾ��ͼ����ͬ
	p_st_control_out_output->u5 = RudAgl2MotorAglV1(m_u_control[4]);//�����ʾ��ͼ����ͬ	//��V,˳ʱ��(��)��Ƕ�
	p_st_control_out_output->u6 = RudAgl2MotorAglV2(m_u_control[5]);//�����ʾ��ͼ����ͬ

	p_st_control_out_output->uf = m_urf;
	p_st_control_out_output->uh = m_urh;
	p_st_control_out_output->ug = m_urg;
}
void CMathControlOut::Calc_Data()
{
	//����ǰ,�޿�
	if (flight_time < m_time_control_on)
	{
		m_u_control[0] = 0.0;
		m_u_control[1] = 0.0;
		m_u_control[2] = 0.0;
		m_u_control[3] = 0.0;
		m_u_control[5] = 0.0;
		m_u_control[6] = 0.0;
		
		// Simulation monitor flags are unavailable in the embedded build.
	} 
	//����������ǰ
	else if (flight_time < m_time_separate_booster)
	{
		//��ת��̬���ȣ��ڻ�PD + ADRC���ƣ����⻷
		//m_urg = m_u25g + m_ug_adrc;
		m_urg = m_u25g;
		m_urg = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_urg_record = m_urg;
		//�����ת�븩�����ö�ʱ�����ȱ�֤��ת��أ������޷�����
		//m_urg = CFlightGlobalFun::Range(m_urg, 10.0);

		//��������������ʹ��PD���ƣ����й�ת����������ϣ�������
		//������̬���ȣ��ڻ�D���ᣬ���⻷��
		m_urh = m_u5h;// + m_u2h * cos(m_gama / RTOA) + m_u2f * sin(m_gama / RTOA);
		
		//������̬���ȣ��ڻ�PD�����⻷
		m_urf = m_u2f * cos(m_gama / RTOA) + m_u5f + m_uqkf;// - m_u2h * sin(m_gama / RTOA);

		//�����ת�븩�����ö�ʱ�����ȱ�֤��ת��أ������޷�����
		//m_urf = CFlightGlobalFun::Range(m_urf,(15.0 - fabs(m_urg)));
		//m_urg = CFlightGlobalFun::Range(m_urg, (15.0 - fabs(m_urf)));//��Ч

		//�������޷�
		m_u_control[0] = CFlightGlobalFun::Range(-m_urf, RUD_H_ANGLE_MAX);
		m_u_control[1] = CFlightGlobalFun::Range(m_urf, RUD_H_ANGLE_MAX);
		m_u_control[2] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_u_control[3] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		//�����ת�븩�����ö�ʱ���ϳ������沢�޷�����
		//m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[1] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[2] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		//m_u_control[3] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[4] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
		m_u_control[5] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);

		//���Ż��������� ��ת�� �� ���������

		// Simulation monitor flags are unavailable in the embedded build.
	} 
	//����ǰ��Ѳ����
	else if (flight_time < m_time_combat_status)
	{
		//��ת��̬PD + ��̬I
		//m_urg = m_u25g + m_u4g;
		//m_urg = CFlightGlobalFun::Range(m_urg, 10.0);
		//��ת��̬���ȣ��ڻ�PD + ADRC���ƣ����·��ƫ����
		m_urg = m_u25g + m_ug_adrc;
		m_urg = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_urg_record = m_urg;//���½����Ƶ�ǰ�Ŷ�����Ϊ�����Ƶ�ǰΪ��̬

		//����: α�໬����ͨ�����ؿ���
		m_urh = m_urh_zd;
		
		//ǰ�� + ������̬PD + ���·�߶ȿ���
		//m_urf = m_u2f + m_u5f + m_ugf + m_uqkf;
		m_urf = m_u2f + m_u5f + m_uqkf;//�߶ȿ������ϵ�������ָ�ȥ��ֱ�Ӷ�ص�ѹ
		//m_urf = CFlightGlobalFun::Range(m_urf, (15.0 - fabs(m_urg)));
		
		//�������޷�
		m_u_control[0] = CFlightGlobalFun::Range(-m_urf, RUD_H_ANGLE_MAX);
		m_u_control[1] = CFlightGlobalFun::Range(m_urf, RUD_H_ANGLE_MAX);
		m_u_control[2] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_u_control[3] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		//�����ת�븩�����ö�ʱ���ϳ������沢�޷�����
		//m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[1] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[2] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		//m_u_control[3] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[4] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
		m_u_control[5] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);

		// Simulation monitor flags are unavailable in the embedded build.
	} 
	//�����Σ�����ͨ���߶ȿ��� ���ɵ� ���ؿ��ƣ���תͨ��ADRC�����л�������ͨ�����ؿ��Ʋ����л�
	else if (flight_time < (m_time_combat_status + m_time_combat_delay))
	{	
		//��תPD+ADRC
		m_urg = m_u25g + m_ug_adrc;
		//����: Ѳ���Ŷ����ɵ�ADRC
		if(flight_time < (m_time_combat_status + 1.0))
		{
			m_urg = (1.0 - (flight_time - m_time_combat_status)) * m_urg_record 
				+ (flight_time - m_time_combat_status) * m_urg;
		}
		m_urg = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_urg_record = m_urg;
		//m_urg = CFlightGlobalFun::Range(m_urg, 10.0);
		
		//����: α�໬����ͨ�����ؿ���
		m_urh = m_urh_zd;
		
		//ǰ�� + ������̬PD + ���·�߶ȿ���
		//m_urf = m_u2f + m_u5f + m_ugf + m_uqkf;
		m_urf = m_u2f + m_u5f + m_uqkf;//�߶ȿ������ϵ�������ָ�ȥ��ֱ�Ӷ�ص�ѹ
		m_urf_record = m_urf;
		//m_urf = CFlightGlobalFun::Range(m_urf, (15.0 - fabs(m_urg)));

		//�������޷�
		m_u_control[0] = CFlightGlobalFun::Range(-m_urf, RUD_H_ANGLE_MAX);
		m_u_control[1] = CFlightGlobalFun::Range(m_urf, RUD_H_ANGLE_MAX);
		m_u_control[2] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_u_control[3] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		//�����ת�븩�����ö�ʱ���ϳ������沢�޷�����
		//m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[1] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[2] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		//m_u_control[3] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[4] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
		m_u_control[5] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
	} 
	//�����Σ�ĩ�Ƶ�
	else
	{
		//��תͬ��
		m_urg = m_u25g + m_ug_adrc;
		if(flight_time < (m_time_combat_status + 1.0))
		{
			m_urg = (1.0 - (flight_time - m_time_combat_status)) * m_urg_record 
				+ (flight_time - m_time_combat_status) * m_urg;
		}
		m_urg = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_urg_record = m_urg;
		
		//������α��������·���ؿ���
		
		if(flight_time < (m_time_combat_status + m_time_combat_delay + 1.0))
		{
			//�߶ȿ��ƹ��ɵ�ĩ�Ƶ�
			m_urf = (m_urf_zd + m_uqkf)*(flight_time - m_time_combat_status - m_time_combat_delay)
				+ m_urf_record*(m_time_combat_status + m_time_combat_delay + 1.0 - flight_time);
		}
		else
		{
			//������ɣ�����ĩ�Ƶ���ѹ
			m_urf = m_urf_zd + m_uqkf;
		}
		
		//�޷�������ת
		//m_urf = CFlightGlobalFun::Range(m_urf, (15.0 - fabs(m_urg)));
		
		//����α�໬������·���ؿ���
		m_urh = m_urh_zd;
		//�޷�
		//m_urg = CFlightGlobalFun::Range(m_urg, 10.0);
		
		//�������޷�
		m_u_control[0] = CFlightGlobalFun::Range(-m_urf, RUD_H_ANGLE_MAX);
		m_u_control[1] = CFlightGlobalFun::Range(m_urf, RUD_H_ANGLE_MAX);
		m_u_control[2] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		m_u_control[3] = CFlightGlobalFun::Range(m_urg, RUD_G_ANGLE_MAX);
		//�����ת�븩�����ö�ʱ���ϳ������沢�޷�����
		//m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[1] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		//m_u_control[2] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		//m_u_control[3] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[4] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
		m_u_control[5] = CFlightGlobalFun::Range(m_urh, RUD_H_ANGLE_MAX);
		
		// Simulation monitor flags are unavailable in the embedded build.
	}

	//Ϊ�˵���???...
	//m_u_control[0] = 0.0;
	//m_u_control[1] = 0.0;
	//m_u_control[2] = 0.0;
	//m_u_control[3] = 0.0;
	//m_u_control[4] = 0.0;
	//m_u_control[5] = 0.0;
}

//�������˻�����������Ƕ� ת���� ����Ƕȣ�ˮƽ
double CMathControlOut::RudAgl2MotorAglH1(double rud_angle)
{
	//��Ӧ��ϵ
	double rud_angle_array[61] = {-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, 
									-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, 
									-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 
									0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
									10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 
									20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
	double motor_angle_array[61] = {-51.45, -49.11, -46.85, -44.65, -42.52, -40.44, -38.42, -36.45, -34.52, -32.63, 
									 -30.78, -28.97, -27.20, -25.46, -23.75, -22.08, -20.43, -18.81, -17.22, -15.66, 
									 -14.12, -12.61, -11.12, -9.65, -8.21, -6.79, -5.39, -4.01, -2.65, -1.32, 
 									 0.00, 1.30, 2.57, 3.83, 5.06, 6.28, 7.48, 8.66, 9.82, 11.00, 
 									 12.08, 13.18, 14.26, 15.33, 16.37, 17.40, 18.40, 19.39, 20.36, 21.31, 
 									 22.24, 23.15, 24.04, 24.91, 25.76, 26.59, 27.40, 28.19, 28.96, 29.70, 30.43};
	double motor_angle = 0.0;//����ֵ

	//����Ƕ����
	rud_angle = CFlightGlobalFun::Range2(rud_angle, rud_angle_array[60], rud_angle_array[0]);
	motor_angle = CFlightGlobalFun::LAQL1(61,  rud_angle_array, motor_angle_array, rud_angle);
	
	return motor_angle;
}

double CMathControlOut::RudAgl2MotorAglH2(double rud_angle)
{
	//��Ӧ��ϵ
	double rud_angle_array[61] = {-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, 
									-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, 
									-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 
									0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
									10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 
									20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
	double motor_angle_array[61] = {-30.43, -29.70, -28.96, -28.19, -27.40, -26.59, -25.76, -24.91, -24.04, -23.15,
    -22.24, -21.31, -20.36, -19.39, -18.40, -17.40, -16.37, -15.33, -14.26, -13.18,
    -12.08, -11.00,  -9.82,  -8.66,  -7.48,  -6.28,  -5.06,  -3.83,  -2.57,  -1.30,
     0.00,   1.32,   2.65,   4.01,   5.39,   6.79,   8.21,   9.65,  11.12,  12.61,
    14.12,  15.66,  17.22,  18.81,  20.43,  22.08,  23.75,  25.46,  27.20,  28.97,
    30.78,  32.63,  34.52,  36.45,  38.42,  40.44,  42.52,  44.65,  46.85,  49.11, 51.45};
	double motor_angle = 0.0;//����ֵ

	//����Ƕ����
	rud_angle = CFlightGlobalFun::Range2(rud_angle, rud_angle_array[60], rud_angle_array[0]);
	motor_angle = CFlightGlobalFun::LAQL1(61,  rud_angle_array, motor_angle_array, rud_angle);
	
	return motor_angle;
}

//�������˻�����������Ƕ� ת���� ����Ƕȣ���ֱ
double CMathControlOut::RudAgl2MotorAglV1(double rud_angle)
{
	//��Ӧ��ϵ
	double rud_angle_array[61] = {-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, 
									-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, 
									-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 
									0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
									10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 
									20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
	double motor_angle_array[61] = {-36.15, -34.73, -33.34, -31.96, -30.59, -29.25, -27.92, -26.61, -25.31, -24.03, 
										-22.76, -21.51, -20.27, -19.04, -17.83, -16.63, -15.44, -14.26, -13.10, -11.94, 
										-10.80, -9.67, -8.55, -7.44, -6.34, -5.25, -4.17, -3.10, -2.05, -1.00, 
										0.00, 1.07, 2.08, 3.09, 4.09, 5.08, 6.05, 7.02, 7.98, 8.92, 
										9.86, 10.78, 11.70, 12.61, 13.50, 14.38, 15.26, 16.12, 16.97, 17.82, 
										18.65, 19.47, 20.27, 21.07, 21.86, 22.63, 23.39, 24.14, 24.88, 25.61, 26.32};
	double motor_angle = 0.0;//����ֵ

	//����Ƕ����
	rud_angle = CFlightGlobalFun::Range2(rud_angle, rud_angle_array[60], rud_angle_array[0]);
	motor_angle = CFlightGlobalFun::LAQL1(61, rud_angle_array, motor_angle_array, rud_angle);
	
	return motor_angle;
}
double CMathControlOut::RudAgl2MotorAglV2(double rud_angle)
{
	//��Ӧ��ϵ
	double rud_angle_array[61] = {-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, 
									-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, 
									-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 
									0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
									10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 
									20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
	double motor_angle_array[61] = {-26.32, -25.61, -24.88, -24.14, -23.39, -22.63, -21.86, -21.07, -20.27, -19.47,
    -18.65, -17.82, -16.97, -16.12, -15.26, -14.38, -13.50, -12.61, -11.70, -10.78,
    -9.86,  -8.92,  -7.98,  -7.02,  -6.05,  -5.08,  -4.09,  -3.09,  -2.08,  -1.07,
    -0.00,   1.00,   2.05,   3.10,   4.17,   5.25,   6.34,   7.44,   8.55,   9.67,
    10.80,  11.94,  13.10,  14.26,  15.44,  16.63,  17.83,  19.04,  20.27,  21.51,
    22.76,  24.03,  25.31,  26.61,  27.92,  29.25,  30.59,  31.96,  33.34,  34.73, 36.15};
	double motor_angle = 0.0;//����ֵ

	//����Ƕ����
	rud_angle = CFlightGlobalFun::Range2(rud_angle, rud_angle_array[60], rud_angle_array[0]);
	motor_angle = CFlightGlobalFun::LAQL1(61, rud_angle_array, motor_angle_array, rud_angle);
	
	return motor_angle;
}
/*
void CMathControlOut::Monitor_Data()
{
	if (sim_monitor.flag_monitor2_valid)
	{
		sim_monitor.Get_Variable(m_u25g,"u25g",ENUM_FILE_CONTROL1);//��תPD
		sim_monitor.Get_Variable(m_u4g,"u4g",ENUM_FILE_CONTROL1);//��תI
		sim_monitor.Get_Variable(m_ug_adrc,"ugADRC",ENUM_FILE_CONTROL1);//��תADRC
		
		sim_monitor.Get_Variable(m_u2f,"u2f",ENUM_FILE_CONTROL1);//����P
		sim_monitor.Get_Variable(m_u5f,"u5f",ENUM_FILE_CONTROL1);//����D
		sim_monitor.Get_Variable(m_ugf,"ugf",ENUM_FILE_CONTROL1);//�������·���߶ȿ���
		sim_monitor.Get_Variable(m_urf_zd,"unf",ENUM_FILE_CONTROL1);//��������·���ؿ���
		
		sim_monitor.Get_Variable(m_u5h,"u5h",ENUM_FILE_CONTROL1);//����D
		sim_monitor.Get_Variable(m_urh_zd,"unh",ENUM_FILE_CONTROL1);//��������·���ؿ���
		
		sim_monitor.Get_Variable(m_urg,"urg",ENUM_FILE_CONTROL1);//�߼��棬��ת
		sim_monitor.Get_Variable(m_urh,"urh",ENUM_FILE_CONTROL1);//�߼��棬����
		sim_monitor.Get_Variable(m_urf,"urf",ENUM_FILE_CONTROL1);//�߼��棬����
	}
}*/
