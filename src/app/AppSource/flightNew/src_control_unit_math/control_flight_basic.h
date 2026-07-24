#ifndef _CONTROL_FLIGHT_BASIC_H_
#define _CONTROL_FLIGHT_BASIC_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// ժҪ: �ɿ�Ԥ����ģ�飬�����ؼ������������㣬ָ�����̽����
//        
//
// ��ǰ�汾: 1.0
// ����: wym
// �������: 
//==================================================================/
#include "control_roll.h"
#include "control_engine.h"
#include "../data_protocol.h"
/*
typedef struct  _Stru_Way_Point
{
	int num;//������
	
	double longitude;	//���㾭��
	double latitude;	//����γ��
	double height;		//����߶�
	int    route_mode; //��������
	int    formation_mode;	//������Ϣ��δʹ��
	double dltTime;//����ʱ��
	double turn_angle;//�г������� �� ת��Ƕȣ����ã� �� ������
	double turn_radius;//ת��뾶�������뾶��δʹ��
	double velocity;	//�ٶ�ָ��
	double accept_radius;//���ܰ뾶��δʹ��
	
	//���������ݣ���ʶ
	bool if_flightime_ctrl;	//[����ʱ���ʶ��1��Ч��0��Ч��]
	bool if_relativehigh_ctrl;  //[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	bool if_heading_hold;  //[ָ����б�ʶ��1��Ч��0��Ч��]
	bool if_groundspeed_ctrl;  //[���ٿ��Ʊ�ʶ��1���٣�0���٣�]
	bool if_attackangle_ctrl;//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]				
	bool if_turndir_set;//[����ת�䣺��ת����ת��ʶ��]
	bool if_prepare_hover;//[Ԥ������ʶ��1��Ч��0��Ч��]
	//����������
	double outtrack_angle;//[ָ������г��Ƕȣ���ƫ��Ϊ�����ԣ��г��Ƕ�deg]
	int hover_round; //[����Ȧ��]
	double attack_angle;//[������]
}Stru_Way_Point;
*/
typedef struct  _Stru_Command
{
	//�豸�������
	bool flag_separate_booster;	 //���������룬��������Ŀδʹ�ã���������������Զ������(����С����ֵ��0.2s)������Ҫ����ָ��
	bool flag_launch_missile_wing;//����չ������������Ŀδʹ�ã�һ��ִ������Ϊ�������������0.2s��(����С����ֵ��0.4s)
	bool flag_engine_start;		//�����������������Ŀ��Ϊ����ת����ִ������Ϊ�������������1s����Լ10s������ת70%���� �� ������
	bool flag_seeker_on;			//����ͷ��������������Ŀ������װ��絼��ͷ��ִ������Ϊ�������������2s����������װ����ͷ��ʹ��
	bool flag_lock_on_permit;		//����������ʶ����������Ŀ������װ��絼��ͷ��ִ������Ϊ��Ŀ�꺽������Ϊ�������𹥵㣬�Ҿ���Ŀ�꺽�����С��2km��
	bool flag_target_lock;		//Ŀ��ѡ�����������������Ŀ������װ��絼��ͷ��������Ŀ��ѡ�񣬼�������ǰĿ�겻�ٸ���

	//����ʱ�򲿷�
	bool flag_missile_takeoff;	//�����ɣ�70%����ת�ٶȿ���
	bool flag_altitude_control;	//�״θ߶ȿ���
	bool flag_combat_status;		//����Ŀ�꣬����ս��/ĩ�Ƶ�״̬�������ŷ��͡��ӳ�װ�������ս��ָ�
	bool flag_fuze_unlock;		//���Ž���
	bool flag_combat_dive_pullup;	//ĩ�Ƶ�ת�������𣬷���������תת�ٿ���
	bool flag_combat_dive_sidectrl;//ĩ�Ƶ�ת��ƫ����
	bool flag_engine_shutdown;	//�������ػ�
	bool flag_open_umbrella;		//��ɡ����
}Stru_Command;

// λ���붨�壬˳����ṹ���Աһһ��Ӧ
#define BIT_flag_t0                    (1U << 0)
#define BIT_flag_control_on            (1U << 1)
#define BIT_flag_separate_booster      (1U << 2)
#define BIT_flag_engine_start          (1U << 3)
#define BIT_flag_seeker_on             (1U << 4)
#define BIT_flag_initial_hz            (1U << 5)
#define BIT_flag_missile_takeoff       (1U << 6)
#define BIT_flag_launch_turn           (1U << 7)
#define BIT_flag_altitude_control      (1U << 8)
#define BIT_flag_lock_on_permit        (1U << 9)
#define BIT_flag_cooperative_attack    (1U << 10)
#define BIT_flag_combat_status         (1U << 11)
#define BIT_flag_fuze_unlock           (1U << 12)
#define BIT_flag_combat_dive_ok_set    (1U << 13)
#define BIT_flag_combat_dive_sidectrl  (1U << 14)
#define BIT_flag_combat_dive_pullup    (1U << 15)
#define BIT_flag_engine_shutdown       (1U << 16)
#define BIT_flag_open_umbrella         (1U << 17)
#define BIT_flag_alltitude_change      (1U << 18)
#define BIT_flag_alltitude_climb       (1U << 19)
#define BIT_flag_alltitude_decline     (1U << 20)
#define BIT_flag_waypoint_turn         (1U << 21)
#define BIT_flag_turn_out_set          (1U << 22)
#define BIT_flag_flightime_ctrl        (1U << 23)
#define BIT_flag_velocity_control      (1U << 24)
#define BIT_flag_heading_hold          (1U << 25)
#define BIT_flag_turndir_set           (1U << 26)
#define BIT_flag_prepare_hover         (1U << 27)
#define BIT_flag_relativehigh_ctrl     (1U << 28)
#define BIT_flag_attackangle_ctrl      (1U << 29)

typedef struct  _Stru_Control_Time
{
	double time_control;			//����ʱ��
	double time_separate_booster;//����������ʱ��
	double time_launch_missile_wing;//����չ��ʱ�䣬����
	double time_seeker_on;		//����ͷ����ʱ�䣬����
	double time_engine_start;	//����������ʱ�䣬�������������1s����������ת��Ϊ�������
	double time_engine_start_finish;//�������������ʱ�䣬����
	double time_missile_takeoff;	//������ʱ��
	double time_altitude_control;//��ʼ�߶ȿ���
	double time_launch_turn;		//����ת�俪ʼʱ��
	double time_launch_turn_ok;	//����ת�����ʱ��
	double time_cooperative_attack;//Эͬ(����)����
	double time_combat_status;	//ս��ָ��: ������ͷ����Ŀ�ꡱ�򡰹̶�Ŀ������С����ֵ��

	double time_combat_dive_ok;	//��������������ʼ��������
	double time_combat_dive_sidectrl;//��������0.1�����½�������ȥ��ת��ƫ����
	double time_combat_dive_pullup;//��������0.1��ת��������
	
	double time_altitude_change_start;//�߶Ȼ�����ʼʱ��
	double time_altitude_change_end;	//�߶Ȼ�������ʱ��
	double time_turn_in_start;	//��ת�俪ʼʱ��: ���桢����ת�乲��
	double time_turn_in_end;		//��ת�����ʱ��: Ԥ�ƹ�ת�������ʱ��
	double time_turn_out_start;	//��ת�俪ʼʱ��
	double time_turn_out_end;		//��ת�����ʱ��
	
	double time_turn_in_minimum;	//������Сʱ�̣�����ת������ʱ�� + 10s
	double time_arrive_minimum;	//���Σ���Ŀ��㣩����ʱ�䣬����ת������ж�

	double time_engine_shutdown;	//�������ػ�
	double time_open_umbrella;	//��ɡ����
	double time_fuze_unlock;		//���Ž���
	double time_touch_ground;		//����ʱ��
}Stru_Control_Time;

typedef struct  _Stru_Control_flag
{
	bool flag_t0;
	
	bool flag_control_set;
	bool flag_control_on;			//���أ�����ܣ�����set(��������)���ӳٶ�ʱ�俪ʼִ��
	bool flag_separate_booster_set;
	bool flag_separate_booster;	//����������
	bool flag_launch_missile_wing_set;
	bool flag_launch_missile_wing;//����չ��(���)��ɾ��
	bool flag_engine_start_set;
	bool flag_engine_start;		//�������״ε���ת�󳵣���������������/����ָ�������
	
	bool flag_engine_start_finish_set;//��ʼ�����η������󳵣��������������������ת������Ѳ������ָ�����
	//bool flag_engine_start_finish;//ͬʱ���������������ӳ�
	//�������������ƣ���������ɺ���Կ�ʼ����������
	
	bool flag_launch_turn_set;	//ֻ����һ�Σ�����
	bool flag_launch_turn;		//�״θ߶ȿ���5s�󣬿�ʼ����ת�䣬�����������
	bool flag_altitude_control_set;
	bool flag_altitude_control;	//�״ν���߶ȿ���
	
	bool flag_seeker_on_set;		
	bool flag_seeker_on;			//����ͷ��������絼��ͷһ��ɳ�ʱ�乤��������������һ��ʹ�����󼴿ɿ���
	
	bool flag_lock_on_permit;		//����ͷ��������: �л����һ������(�����Ż�Ϊ����Ŀ����С����ֵʱ����������)
	bool flag_cooperative_attack_set;
	bool flag_cooperative_attack;//Эͬ(��ܽ���������)������������һ��Ŀ������С��10km;
	bool flag_combat_status_set;
	bool flag_combat_status;		//ս��ָ�����������(ĩ�Ƶ�)
	bool flag_target_lock;//Ŀ����������ʱ����

	bool flag_combat_dive_ok_set;//��һ������������������ʼ���������о�ΪС��Ŀ��߶�
	bool flag_combat_dive_sidectrl;//�ڶ��������������0.1s�����½������ߣ�������������л�Ϊֱ����ƫ���ƣ���STT�л�ΪBTT
	bool flag_combat_dive_pullup;//�����������������0.1s��������������л�Ϊ�������ƣ���ȥ�����·���ƣ�ָ����Ǵӵ�ǰ�����ǹ��ɵ�����������

	bool flag_alltitude_change;//�߶Ȼ��������б�ʶ
	bool flag_alltitude_climb;//�߶Ȼ���: ����
	bool flag_alltitude_decline;//�߶Ȼ���:�»�
	
	bool flag_waypoint_turn;	//����ת������б�ʶ�����������
	bool flag_turn_out_set;	//��ת������б�ʶ������ת�������ת�乲��
	bool flag_flightime_ctrl;//����ʱ����Ʊ�ʶ���뿪�������ٶ�ָ��
	bool flag_velocity_control;//���ٿ��Ʊ�ʶ

	bool flag_heading_hold;  	//[ָ����б�ʶ��1��Ч��0��Ч��]
	bool flag_turndir_set;	//[����ת�䣺��ת����ת��ʶ��]
	bool flag_prepare_hover;	//[Ԥ������ʶ��1��Ч��0��Ч��]
	
	bool flag_relativehigh_ctrl;//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	bool flag_attackangle_ctrl;	//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]				
	
	//������ɲ�����ֱ��ʱ�����½������ߣ����ж��Ƿ���Ҫ���е���ʱ�����
	
	bool flag_initial_hz;			//��ϸ߶ȳ�ʼ��
	bool flag_missile_takeoff;	//���
	bool flag_engine_shutdown;	//�������ػ�
	bool flag_open_umbrella;		//��ɡ����
	bool flag_fuze_unlock;		//���Ž���
}Stru_Control_flag;

typedef struct  _Stru_Flight_Basic_Input
{
	int missile_ID;
	//int engine_start_result;//δ�õ�
	//double engine_rpm;		//δ�õ�
	double engine_cmd_rpm;	//�����������ơ�����ķ�������ָ��ת�٣���ѡһ
	double engine_cmd_Kc;	//��������������ָ�����ţ���ѡһ
	
	Stru_Data_RadioAlt_To_Controller st_radioalt_data;//���ߵ�߶ȱ������ƣ���������Ŀδʹ��
	Stru_Data_Baro_To_Controller st_baro_data;//���ٹܸ�����
	Stru_Data_Engine_To_Controller st_engine_data;//������������
	Stru_Data_INS_To_Controller	st_ins_data;
	Stru_Data_Seeker_To_Controller	st_seeker_data;
	//Stru_Data_Datalink_To_Controller	st_datalink_data;
	Stru_Data_Datalink_To_ControllerSig	st_datalink_datasig;
}Stru_Flight_Basic_Input;

typedef struct  _Stru_Flight_Basic_Output
{
	double mass_calc;	//��������
	double hz;
	double h_ini;	//��ʼ�߶�
	double nby;
	double nbz;
	double v;	//����
	double vs;	//ˮƽ�ٶ�
	double vnx;
	double vnz;	//�����ٶ�
	double mach;
	double sonic_speed;//���٣�����
	double sz;	//��ƫ
	double zeta;
	double gama;
	double wx;
	double wy;
	double wz;
	double g;	//��������
	double dqf;	//���߽��ٶ�
	double dqh;	
	double qf;
	double qh;
	double time_to_go;
	double phif;//��ܽ�
	double phih;	
	int target_num__choosen;
	double gama_command;
	double ny_command;//ĩ�Ƶ�ָ��
	double nz_command;

	double angle_zw;//��ƫ��Ϊ����-180deg~180deg
	double radius_zw;
	double target_velocity;
	double target_time;
	double target_height;
	//������Ϣstart
	double dynamic_pressure;//��ѹ
	int num_way_point_target;//��ǰ�����
	double target_long;//��·Ŀ���
	double target_lat;
	double target_distance;//��·Ŀ�����룬����20260715
	unsigned int flight_control_state;//����״̬������20260715
	//int token_long;//����ɾ��
	//int token_lat;//����ɾ��
	unsigned int rpmState;//�ɿ��յ��ķ�����ת�٣�����20260715
	
	//double dlt_psic;//������ƫ�ɾ�� 20260715
	double sz_circle;//Բ�켣��ƫ�࣬δʹ��
	double azimuth;//���η�λ�ǣ�������Բ����
	double psicn;//����ƫ��
	double theta;//������ǣ�δʹ��
	double alpha_vg;//���ٹ��ǣ�δʹ��
	double beita_vg;//���ٲ໬�ǣ�δʹ��
	//������Ϣend

	//����ǰװ����Ϣ
	double ground_temperature;//�����¶�
	double ktheta_lauch_enc;
	double ktheta_climb_enc;
	double ktheta_hight_enc;

	//ʵʱ������Ϣ
	double longitude;//ʵʱ����
	double latitude;//ʵʱγ��
	double Rmt_n[3];
	double distance_target;//�������ɾ��룬ʵʱ
	double distance_target_t_combat;//����ĩ�Ƶ�ʱ����Ŀ���룬����һ��
	double gama_turn_nominal;//����ת��뾶��ȷ����ת�Ǳ��ֵ������һ��
	double velocity_average_10s;//���پ�ֵ��ʵʱ
	int ECU_work_cmd;	//����������ָ��������ػ���
	int	count_altitude_change;//�߶ȸ��£���ֵΪ0����ɹ�������״θ߶Ȼ���1�������л�ʱ+1����������½�
	
	//����������ʶ
	bool flag_altitude_change;//�߶Ȼ���
	bool flag_altitude_climb;
	bool flag_altitude_decline;
	bool flag_launch_turn;//�������
	bool flag_waypoint_turn;
	
	//���㴦����Ϣ
	bool flag_flightime_ctrl;//����ʱ����Ʊ�ʶ���뿪�������ٶ�ָ��
	bool flag_velocity_control;//���ٿ��Ʊ�ʶ

	bool flag_heading_hold;  	//[ָ����б�ʶ��1��Ч��0��Ч��]
	bool flag_turndir_set;	//[����ת�䣺��ת����ת��ʶ��]
	bool flag_prepare_hover;	//[Ԥ������ʶ��1��Ч��0��Ч��]
	
	bool flag_relativehigh_ctrl;//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	bool flag_attackangle_ctrl;	//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]	

	bool flag_fire_distribution;//�������䣬δ�õ�
	Stru_Command st_command;
	Stru_Control_Time st_control_time;
}Stru_Flight_Basic_Output;

typedef struct _Stru_Tustin_FirstIO_Filter
{
	double inputdata[2];	//����
	double outputdata[2];	//���
	double T; //�˲�ʱ�䳣��s
	double Ts;//����ʱ��s
} Stru_Tustin_FirstIO_Filter;

class CMathControlFlightBasic
{
public:
	CMathControlFlightBasic(); 
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor					* p_st_debug_monitor;//������Ϣ
	Stru_Route_Data						* p_st_route_data_preflight;//Ԥװ������Ϣ
	Stru_Initial_Data					* p_st_initial_data;//������Ϣ
	Stru_Flight_Basic_Input				* p_st_flight_basic_input;
	Stru_Flight_Basic_Output			* p_st_flight_basic_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	//void Monitor_Data();
	void Calc_Command();	
	void PackControlFlag(const Stru_Control_flag* pSt);
	void Calc_Flight_Data();
	void Calc_Mass_Data();
	void Calc_LOS_Rate();
	void Calc_Hz();
	void Calc_BaroHigh();
	void Calc_BaroSpd();
	void Control_Turn();
	void Control_Altitude_Change();
	void Calc_Dualplane_Guidance();
	void Coord_Rebuild();
	void Update_Task_Info();
	void Change_Task_Info_Online();
	bool Judge_Turn_Error();
	
	int m_missile_ID;
	int m_missile_flight_mode;//0x55 ����ѵ����0xAA �̶���(����)�������ͷ�������
	double m_ground_temperature;
	double m_ktheta_lauch_enc;
	double m_ktheta_climb_enc;//����������������ǣ�������,����ֵ 6deg�����ݷ��亣�θ߶�ȷ������������)
	double m_ktheta_hight_enc;//����Ѳ���������ǵ���ƽ�⹥��
	
	int m_num_way_point;	//�ܺ�����
	int m_num_way_point_target;//Ŀ�꺽��(��ǰ����)
	Stru_Way_Point m_st_way_point[MAX_ROUTE_NUMBER];//���㣬����㲻�ǵ�0���㣻�����Ϊ0��������һ��(Ŀ��)����
	Stru_Way_Point m_st_target;//Ŀ����Ϣ��λ�á�ת��뾶���Ƕ�(�������ת��Ƕȵ�)���ٶȡ��������ͼ���Ϣ����
	
	int valid_count;	//���߽��ٶȼ��㣬��Ŀä�����������С��ĳֵ���ٸ���
	int count_qk;		//��ܻ�����
	int count_v_5;		//���ر���
	int count_fl;		//����������
	int count_qd;		//�������������
	int count_v_50;	//���������뱸��
	int count_takeoff;	//�����ɱ�ʶ
	int count_tg;		//��ʼ�߶ȿ���
	double dlt_time_tg;//��������ʱ�����
	int count_cooperative_attack;//Эͬ���������͵�Ŀ����10km
	int count_virtual;//����������ͷ����Ŀ�꣬���͵�Ŀ����2km
	int count_combat_dive_ok;//��������������ʼ���������оݣ��߶Ȳ�С��0
	int count_distance_recycle;//������հ뾶����
	int count_v50_recycle;//�ٶ�С��50m/s����������
	int count_h300_v54_recycle;//�ٶ�С��54m/s������Ը߶�С��300m����������
	int count_distance_out_recycle;//��Ȧ�ж�
	int count_v54_recycle;//��1)�ٶ�С��54m/s��(2) ��Ը߶�С��250m��(3)��Ȧ�󣬷��о������600m
	int count_h250_recycle;
	int count_distance_out600_recycle;
	int count_h10_recycle;//�߶�С��10m��������
	int count_h5_ny2_recycle;//�߶�С��5m��y����ش���2

	int count_altitude_change;//�߶Ȼ���������¼
	int count_altitude_change_enable;
	int count_altitude_change_lauch_enable;
	int count_altitude_change_energy_enable;//Ѳ��������ʱ������5s��������С
	int count_altitude_change_end;
	int count_away;	//ԭ��Ŀ������
	int count_sd_in;	//С����ǰת�����
	int count_turn_out;//ת��ת��������ֱ��
	int count_turn_error;//ת��Ƕȹ����쳣
	int count_update;	//����������������������

	//��ɡ���ս׶�
	int step_open_umbrella;//��ɡ�׶Σ�0��ʼ��Ч��1���뿪ɡ����(�����л���Ŀ�������Ϊ��ɡ��)��2�������Ȧ�ҷ������ػ���3 ��������ɡ������4������Ȧ��5���㱸�ݿ�ɡ����
							//��������չ��6 �򿪰�ȫ���ң�7���أ�8 �и��ɡ
	//��ɸ߶Ȼ���
	int step_altitude_change_lauch;//��ɸ߶Ȼ����׶Σ�
	//˫ƽ���Ƶ��׶α�ʶ��0��ɽ׶Σ�1 Ѳ���׶� BTT�Ƶ���2 ĩ�Ƶ��׶� BTT�Ƶ���3ĩ�Ƶ��׶� STT�Ƶ���4 ��ĩ�Ƶ��׶ν���������������̣�5 ��ĩ�Ƶ�������תѲ�� BTT�Ƶ���ͬ2��
	int step_dualplane_guidance;
	
	Stru_Control_flag m_st_control_flag;//ʱ��ʱ��
	Stru_Control_Time m_st_control_time;
	unsigned int m_flight_control_state;//����״̬
	
	//����������ϵ���
	double m_hgps;
	double m_longitude;
	double m_latitude;
	double m_ax;
	double m_ay;
	double m_az;
	double m_wx;
	double m_wy;
	double m_wz;
	double m_vtx;
	double m_vty;//��ϵ���������ٶ�
	double m_vtz;
	double m_zeta;
	double m_gama;
	double m_psit;//����ϵ��ƫ����
	
	//���������� 
	int count_ax_tyz;
	int count_ay_tyz;
	int count_az_tyz;
	int count_wx_tyz;
	int count_wy_tyz;
	int count_wz_tyz;
	double m_axtyz;	//��Ұֵ��
	double m_aytyz;
	double m_aztyz;
	double m_wxtyz;	//�˲�����ٶ�
	double m_wytyz;
	double m_wztyz;	
	Stru_Tustin_FirstIO_Filter m_nav_data_filter[6];//����Ϊ���ٶȡ����ٶȵ��˲�
	double m_axflt;	//�˲�����ٶ�
	double m_ayflt;
	double m_azflt;
	double m_wxflt;	//�˲�����ٶ�
	double m_wyflt;
	double m_wzflt;	
	double m_A;		//���η�λ�ǣ���ƫ�� Ϊ���������л�����һ�Σ�����ϵ�ٶ�����;
	double m_psin;	//����ϵƫ���ǣ��������߽��ٶȼ���
	double m_psicn;//����ϵ������ƫ�ǻ򺽼��ǣ���ƫ��Ϊ����-180~180deg
	double m_theta;//����ϵ��������ǣ�-90deg~90deg
	double m_Cbn[9];//����ϵ �� ����ϵת������
	double m_Cnb[9];//����ϵ �� ����ϵת������
	double m_vx;	//�����ٶ�
	double m_sx;	//����λ�ƣ���������ж�
	double m_v;		//���ٶȣ�����
	double m_vs;	//��ϴ���
	double m_hz;	//��ϸ߶�
	double m_vnx;	//�����ٶ�
	double m_vnz;	//�����ٶ�
	double m_vnz_record1;
	double m_vnz_record2;
	double m_ny;	//����ϵy�����
	double m_nz;	//����ϵz�����
	double m_g;		//����γ�ȡ����θ߶����������ڼ������ 	
	//double m_an[3]; //����ϵ x/y/z����ٶ�
	double m_anx;
	double m_au;
	double m_anz;
	

	//���ٹ�: ��ѹ����ѹ�������ÿ��١��߶�
	double m_static_pressure_raw; //����ԭʼ���ݣ�20ms����һ������
	double m_total_pressure_raw;
	//����������
	int count_static_pressure_tyz;
	int count_total_pressure_tyz;
	double m_static_pressure; //��Ұֵ֮������
	double m_total_pressure;
	double m_static_pressure_flt;//�˲�������
	double m_total_pressure_flt;
	Stru_Tustin_FirstIO_Filter m_baro_data_filter[2];//����Ϊ��ѹ����ѹ���˲�
	int m_baroalt_status;	//�߶ȱ�״̬ 0xAA������0xFF�쳣
	double m_hbaro;	//��ѹ�߶�
	double m_Vbaro;//ָʾ����
	double m_v_air;//���٣����е�����Ч���жϺ�
	double m_dynamic_pressure;//��ѹ
	double m_mach;//�����������٣������߶ȡ��¶�����������������
	double m_v_average_1s;//����1s��ֵ������100ms���ٻ���
	double m_v_average_10s;//����10s��ֵ�����ڹ���Ŀ�����С����ʱ�䣬�Լ�����ʱ�����
	double m_v_record_100ms[10];//���پ�ֵ��������������1s���ٶȾ�ֵ
	double m_v_record_1s[10];//���پ�ֵ��������������10s���ٶȾ�ֵ
	//���ݵ����¶ȡ����溣�θ߶ȡ����θ߶ȣ����ƴ��������������¶ȡ����١������ܶȡ�����ѹ����
	double m_air_temperature;
	double m_air_density;
	double m_air_pressure;
	double m_sonic_speed;
	
	//��������1sʱ����
	double m_total_energy;
	double m_total_energy_pre;

	//���ߵ�߶ȱ�����
	double m_radioalt_hight;
	int m_radioalt_status;

	//������״̬��ת��
	int m_engine_state;//����������״̬
	double m_engine_state_rpm;//״̬ת��
	//������ָ��
	int m_ECU_work_cmd;	//0x11������0x22������0x33 ת�ٿ��ƣ�0x44 �ػ���0x55 ����
	double m_cmd_Kc;//���������ţ���ѡһ
	double m_cmd_rpm;//������ת�٣���ѡһ
	//����������
	double m_state_rpm;//������״̬ת�٣��������ɼ������Ź���	
	double m_fuel_comsumped;//����ȼ����������������
	double m_mass_calc;
	double m_left_flight_time_calc;//ʣ�����ʱ�����
	double m_left_flight_dist_calc;//ʣ����к��̹���

	//����ͷ���ݣ����ݷɿر�ʶ������ͷ����(������λ)����������(���������λ)��Эͬ(��ܽ���������)�����ս��ָ��ȱ�ʶ���ⲿ����ָ��
	//����ͷָ������
	//double m_seeker_cmd;//����ͷ����ָ�0x01 �Լ�(�ϵ��Զ�)��0x02 װ��Ŀ��ģ�壬 0x03 ��ǰ��飬0x04/0x05���ã�0x06�ɿ���������(ָ����ܽ�)��0x07����������0x08������0x09��������
	double m_seeker_cmdpara_targettype;//ָʾĿ�����ͣ�0x01 ������0x02 ��������0x03 �̶�����
	double m_seeker_cmdpara_dltheight;//��Ŀ�߶Ȳ�
	double m_seeker_cmdpara_ktheta;//����������
	double m_seeker_cmdpara_psi;//����ƫ����
	double m_seeker_cmdpara_gama;//������ת��
	double m_seeker_cmdpara_phif;//������ܽ�
	double m_seeker_cmdpara_phih;//������ܽ�

	//����ͷ״̬����
	//double m_seeker_state;//����ͷ����״̬��0xAX ������0xFX�쳣��0x5X ������
	double m_seeker_state_track;//����ͷ����״̬��0x00��ʼ��Ч״̬��0x01�Լ���ɽ��������λ,0x02��������ʧ������0x03������0x04����
	double m_seeker_dqf;	//���߽��ٶ�
	double m_seeker_dqh;
	double m_seeker_phif;	//��ܽǣ�״̬��
	double m_seeker_phih;
	double m_seeker_qf;	//���߽ǣ�״̬��
	double m_seeker_qh;
	int m_seeker_pixelf;//��������ƫ�����������ʱ����
	int m_seeker_pixelh;//��������ƫ��¸���������ʱ����
	double m_seeker_distance_target;//����ͷ�����Ŀ���룬�״�򼤹��ֱ����������ͷ��ʱ����
										//�ɽ���������ݸ߶Ȳ���߽ǹ��㣬Ҳ�ɸ���Ŀ�����ش�С�ͽ������Ŀ����룻
	//double m_seeker_targetlong;//Ŀ��λ�ã�δ����
	//double m_seeker_targetlat;
	//double m_seeker_targethight;

	//���������ݣ������⵼����������
	int m_target_num__choosen;//Ŀ��ѡ������δʹ��
	double m_dqf;	//���߽��ٶ�
	double m_dqh;
	double m_dqf_flt;//�˲������߽��ٶ�
	double m_dqh_flt;
	Stru_Tustin_FirstIO_Filter m_guide_data_filter[2];//����Ϊ���ٶȡ����ٶȵ��˲�
	double m_phif;	//��ܽǣ�������ָ�
	double m_phih;
	double m_Qf;	//���߸ߵͽ�
	double m_Qh;	//���߷�λ�ǣ����߱�ƫ��Ϊ��
	double m_Qn;	//�Ƶ�ϵ���߷�λ��
	double m_time_to_go;//��ĩ�Ƶ�������ʱ�� 
	double m_ny_command;
	double m_nz_command;
	double m_gama_command;
	double m_gama_command_record;//ǰһ֡
	
	double m_alpha_target;		//ʵʱ�����Ŀ��㷽λ����ƫ��Ϊ����-180~180deg
	double m_distance_target;	//ʵʱ�����Ŀ���ˮƽ����
	double m_slant_distance_target;//ʵʱ�����Ŀ���б��
	double m_Rmt_n[3];//����ϵ��Ŀ����ʸ�����������򡢲���
	
	double m_distance_target_t_combat;//����ս��ָ�����ĩ�Ƶ���ʱ��Ŀ����룬����һ�Σ�
	double m_gama_target_t_combat;//����ս��ָ�����ĩ�Ƶ���ʱ����ת�ǣ�����һ�Σ�
	
	//Ԥ�����̼����һ�η�λ��
	//distance_AB//���κ��̣�Ϊ��ʱ����
	double m_total_distance;//�ܺ��̣�Ԥ������������λ���º������һ�Σ�,����һ��(δʹ��)
	double m_alpha_AB;		  //���һ���η�λ��/�溽����(��ƫ��Ϊ��)��Ԥ������������λ���º������һ�Σ�,����һ��(δʹ��)��
	//������Ϣ
	double m_longitude_A;//�ѹ����㣬ǰһ���Σ�Ŀ�꺽��
	double m_latitude_A;
	double m_longitude_B;//��ǰĿ�꺽��
	double m_latitude_B;
	double m_longitude_C;//��һ���Σ�Ŀ�꺽��
	double m_latitude_C;
	double m_distance_BP;	//ʵʱ�������ǰ�㵽Ŀ�꺽�����
	double m_distance_BP_projection;//ʵʱ�������ǰ�㵽Ŀ�꺽����룬���߷���ͶӰ
	double m_distance_BP_500;		//��ǰ500ms�����������Ŀ�꺽��亽��ͶӰ����
	double m_distance_BP_500pre;	//ǰһ֡500ms�����������Ŀ�꺽��亽��ͶӰ����
	
	double m_sz;	//����λ�ƣ�������ٶȹ�ͬ�����ڲ�ƫ����
	double m_sz_record1;
	double m_sz_record2;
	double m_sz_radius;//����ת���Բ�ľ��룬���ǲ�ƫ
	
	//�����л�ʱ������һ��������
	int m_route_mode;//��������
	int m_formation_mode;//����������չ��Ϣ��δʹ��
	double m_turn_angle;	//ת��Ƕ�,��ƫ��Ϊ����-180~180deg
	double m_turn_radius;	//ת��뾶
	double m_accept_radius;//���ܰ뾶����������Ϊ���յ㡢�����ʱ��ʹ��
	double m_target_velocity;	//�����ٶ�
	double m_target_height;	//����(����)�߶�
	//ת�����ʵʱ����
	double m_gama_turn_nominal;//ת������й�������Ǳ��ֵ��ת��뾶���Ʋ�����غ���
	double m_x_coordinate_turn;//ת����̣���ǰ�����Բ������
	double m_z_coordinate_turn;
	double m_distance_turn_in_compensate;//ת����ǰ���룬��ת�Ƕȹ��ɲ���
	double m_distance_turn_in;//ת����ǰ���룬������ת�ǹ��ɲ��� �� ������ǰ����������
	double m_psit_t_turn_in;//����򺽼�ת�����ʱ�̼���һ�Σ�ת�俪ʼʱ���溽��ǣ������ж�ת��Ƕȹ����쳣

	double m_outtrack_angle;//[ָ������г��Ƕȣ���ƫ��Ϊ�����ԣ��г��Ƕ�deg]
	int m_hover_round;		//[����Ȧ��]
	double m_attack_angle;	//[������ deg]
	double m_target_height_ground;//���ε���߶�
};


#endif
