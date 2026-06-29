#ifndef _CONTROL_ENGINE_H_
#define _CONTROL_ENGINE_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// ժҪ: ����������ģ��
//        
//
// ��ǰ�汾: 1.0
// ����: wym
// �������: 
//==================================================================/
//#include "../../debug_monitor.h"

//���ſ�����ת�ٿ���ѡ��:  ����(�����˺�)��δ������Ϊת��ģʽ
#define __VEL__CONTROL__MODE__KC__	 0

//˵��1�������ٿ���ʱ�����ٹ�����Сʱ��ǿ�ƿ��ٿ��ƣ�ָ���ٶ��޷�
//˵��2��
const double VEL_COMMAND_MAX_LIMIT = 60.0;	//m per sencond 
const double VEL_COMMAND_CRUISE = 50.0;		//m per sencond
const double VEL_COMMAND_MIN_LIMIT = 40.0;	//m per sencond

const double RPM_COMMAND_MAX_LIMIT = 7000;//r per min
const double RPM_COMMAND_CRUISE = 5380;		// r per min
const double RPM_COMMAND_MIN_LIMIT = 2500;//r per min 

const double KC_COMMAND_MAX_LIMIT = 100;//���Űٷֱ�
const double KC_COMMAND_CRUISE = 70;	   //���Űٷֱ�
const double KC_COMMAND_MIN_LIMIT = 30;	//���Űٷֱ�

//#define __ENGINE_BSFC__	450 //��������Ч������g/(kW.h)
//#define __ENGINE_AFC__		450 //������ʵ�ʺ�����g/(kW.h)


typedef struct  _Stru_Engine_Control_Input
{
	double h_ini;
	double h;
	double mass;//��������
	double mach;//������������ٶ�Ӧ
	double target_velocity;//ָ�����
	double target_time;	//����ʱ�䣬��Է���ʱ��
	double temperature_ground;//�����¶ȣ����ڼ�������
	double missile_average_velocity;//ƽ������
	double radius_zw;//ת��뾶�����ڼ���ת�䲹�����Ż�ת��
	double velocity_command;

	bool flag_launch_turn;	//����ת����̣�ת����������
	bool flag_waypoint_turn;	//����ת�����
	bool flag_alltitude_change;	//�߶Ȼ���
	bool flag_alltitude_climb;	//����
	bool flag_alltitude_decline;	//�½�
	bool flag_flightime_ctrl;		//����ʱ����Ʊ�ʶ���뿪�������ٶ�ָ�� 1���뵽��ʱ�䲹����0�����룻
	bool flag_velocity_control;	//���ٿ��Ʊ�ʶ

	//����ʱ����ر�ʶ
	bool flag_engine_start;//����������������ɵ��ټ��ٵ���������
	bool flag_missile_takeoff;//�����ɣ���ʼ�ٶȿ���
	bool flag_engine_stop;//�������ػ���ָ������Ϊ��
}Stru_Engine_Control_Input;

typedef struct  _Stru_Engine_Control_Output
{
	double control_rpm;
	double control_Kc;
	//double mass_calc;
}Stru_Engine_Control_Output;

class CMathControlEngine
{
public:
	double flight_time;
	int time_tick;
	CMathControlEngine();
	//Stru_Debug_Monitor				* p_st_debug_monitor;
	Stru_Engine_Control_Input		* p_st_engine_control_input;
	Stru_Engine_Control_Output		* p_st_engine_control_output;
	void Run();
	void Initial();
private:
	void Get_Data();
#ifdef __VEL__CONTROL__MODE__KC__
	void Calc_Data_Kc();
#else
	void Calc_Data_Rpm();
#endif
	void Send_Data();
	double m_p;
	double m_p0;
	double m_p1;
	double m_p2;
	double m_p3;

	double m_dltKc_time;//�뿪������,5s���ڲ���
	
#ifdef __VEL__CONTROL__MODE__KC__
	double m_Kc0;//��׼����
	double m_Kc1;//ת��������ţ�����
	double m_Kc2;//�߶Ȼ������ţ�����
	double m_Kc3;//ʱ��������ţ�����
	double m_Kc4;//PI�������Ų���
#else
	double m_n0;//��׼ת��
	double m_n1;//ת�����ת�٣�����
	double m_n2;//�߶Ȼ���ת�٣�����
	double m_n3;//ʱ�����ת�٣�����
#endif
	double m_mach;	 //�����������ٻ���
	double m_Vsonic;//���٣�����ֵ
	double m_air_velocity;//�������������
	double m_target_velocity;//ָ�����
	double m_missile_velocity;//���룬����(����������)
	double m_target_time;//����ʱ�䣬�����ʱ����㣬�ٶȲ���
	
	double m_missile_height;
	double m_missile_height_initial;
	double m_temperature_ground;//�����¶�
	double m_fuel_comsumped;//����ȼ��
	double m_mass_calc;		//��������
	double m_turn_radius;	//ת��뾶

#ifdef __VEL__CONTROL__MODE__KC__
	double m_control_Kc;	//��������
#else
	double m_control_rpm;	//����ת�٣����
#endif

	//double m_state_rpm;		//״̬ת�٣����
	
	double m_velocity_integrator;//�ٶȻ���
	double m_velocity_ground;	//����
	bool m_flag_velocity_control;//�ٶȿ��Ʊ�ʶ��1������ٿ��ƣ�δ�õ�
	bool m_flag_flightime_ctrl;	//����ʱ���ʶ��1����ʱ����ƣ�������ʱ�䲹��

	//��ɹ���ʱ��
	bool m_flag_engine_start;//����������������ɵ��ټ��ٵ���������
	bool m_flag_missile_takeoff;//�����ɣ���ʼ�ٶȿ���
	bool m_flag_engine_stop;//�������ػ���ָ������Ϊ��
	//������ʶ
	bool m_flag_launch_turn;//����(����)ת��
	bool m_flag_alltitude_change;//�߶Ȼ���
	bool m_flag_waypoint_turn;	//����ת�����
	bool m_flag_alltitude_climb;//����
	bool m_flag_alltitude_decline;//�½�
	
};

#endif

//�����䣬���ٿ��ƣ����ڱ�ӿ��ƣ�