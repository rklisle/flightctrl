#ifndef _DATA_PROTOCOL_H_
#define _DATA_PROTOCOL_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// ժҪ: ������������ģ���ͨѶ��������
//        
//
// ��ǰ�汾: 1.0
// ����: wym
// �������: 
//==================================================================/
//#include "..\load_data.h"
//#include "..\datalink_sim_main.h"
//#include "..\target_sim_main.h"
#include <stdbool.h>
// #include "port/flightPort.h"
//==================================================================================
//
//						���䶨��
//
//==================================================================================

#define E_CONST		(1.0/298.257)
#define RE			(6378137.0)

// ��ԭ���� const int �����Ϊ��
#define MAX_ROUTE_NUMBER 64      // ���������ۿػ����·�����
#define MAX_CONNECT_NUMBER 16    // ������������ӵ�������
#define MAX_TARGET_NUMBER 16     // ÿ���������16��Ŀ��
#define MAX_TIME 9999.0
#define STEP_5ms 0.005

// const int MAX_ROUTE_NUMBER = 64;	//���������ۿػ����·�����
// const int MAX_CONNECT_NUMBER = 16;	//������������ӵ�������
// const int MAX_TARGET_NUMBER = 16;	//ÿ���������16��Ŀ��
// const double MAX_TIME = 9999.0;
// const double STEP_5ms = 0.005;

typedef struct _Stru_Initial_Data
{
	int	missile_ID;			//�����
	double longitude_launch;	//�����
	double latitude_launch;	
	double height_launch;
	double initial_parameter1;//Ԥ����ʼ����1�����緢����¶ȵȣ��ɹ��� ���١�������ģ��
	double initial_parameter2;//Ԥ����ʼ����2�����з���ģʽ
	double launch_time;		//����ʱ��	
	double lauch_azimuth;		//���䷽λ��
	double lauch_pitch;		//���丩����
	double lauch_booster_pitch;//������������
	//double type_target;		//Ŀ�������
	//double longitude_target;	//Ŀ��� ����γ���߶�
	//double latitude_target;	
	//double height_target;	
}Stru_Initial_Data;	//������ʼ״̬װ������

typedef struct  _Stru_Way_Point
{
	int num;//������
	
	double longitude;	//���㾭��
	double latitude;	//����γ��
	double height;		//����߶�
	int    route_mode; //��������
	int    formation_mode;	//������Ϣ��δʹ��
	double dltTime;//����ʱ��
	double velocity;	//�ٶ�ָ��
	double turn_angle;//�г������� �� ת��Ƕȣ����ã� �� ������
	double turn_radius;//ת��뾶�������뾶��δʹ��
	double accept_radius;//���ܰ뾶��δʹ��
	
	//���������ݣ���ʶ
	bool if_flightime_ctrl;	//[����ʱ���ʶ��1��Ч��0��Ч��]
	bool if_groundspeed_ctrl;	//[���ٿ��Ʊ�ʶ��1���٣�0���٣�]

	bool if_heading_hold;  	//[ָ����б�ʶ��1��Ч��0��Ч��]
	bool if_turndir_set;		//[����ת�䣺��ת����ת��ʶ��]
	bool if_prepare_hover;	//[Ԥ������ʶ��1��Ч��0��Ч��]
	bool if_relativehigh_ctrl;//[��Ը߶ȣ�����߶ȣ����Ʊ�ʶ��1��Ч��0��Ч]
	bool if_attackangle_ctrl;	//[�����Ǳ�ʶ��1ָ����ǣ�0��Լ����]				
	//����������
	double outtrack_angle;	//[ָ������г��Ƕȣ���ƫ��Ϊ�����ԣ��г��Ƕ�deg]
	int hover_round;		//[����Ȧ��]
	double attack_angle;	//[������]
	double recycle_ground_hight;//���յ����߶�
}Stru_Way_Point;

typedef struct _Stru_Route_Data
{
	int num_rows;	// ������
	int num_columns;	// ����	11
	char ** p_str_title; 	// ����
	// ָ��Stru_Way_Point[num_rows��̬����]
	double * p_route_data;		//��Ϊ��ͬ�����ţ���Ϊ��������: ��š����ȡ�γ�ȡ�ת��뾶���Ƕȡ��ٶȡ��������͡���Ϣ���߶�
	// ���У������ڲ�����
	double longitude_target;
	double latitude_target;
	double height_target;
}Stru_Route_Data;	//�������к�·����

typedef struct _Stru_Mission_Update_Data
{
	int missile_ID;
	int	target_ID;
	int	update_count;//���´���
	int	num_waypoint_updated;	//��ǰ���£���Ч�ĺ�������Ŀ
	double longitude  [MAX_ROUTE_NUMBER];
	double latitude   [MAX_ROUTE_NUMBER];
	double height     [MAX_ROUTE_NUMBER];
	double turn_radius[MAX_ROUTE_NUMBER];
	double turn_angle [MAX_ROUTE_NUMBER];
	double velocity   [MAX_ROUTE_NUMBER];
	int    route_mode [MAX_ROUTE_NUMBER];
	int    formation_mode[MAX_ROUTE_NUMBER];
}Stru_Mission_Update_Data;		//���ߺ���װ������(�ɸ�������滮�㷨���ɣ�������)

typedef struct _Stru_Missile_State_Data
{
	int missile_ID;
	bool flag_missile_launched;
	double gama;
	double psi;
	double zeta;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
	double pitch_LOS;
	double yaw_LOS;
	double time_to_go;
}Stru_Missile_State_Data;		//�������ⲿ���͵ķ���״̬��Ϣ

typedef struct _Stru_Target_State_Data
{
	int target_ID;
	bool   flag_target_distinguished;
	double pitch_LOS_angle_target;
	double yaw_LOS_angle_target;
	double distance_target;
	double longitude_target;
	double latitude_target;
	double rcs_target;
}Stru_Target_State_Data;		//�������ⲿ���͵ķ���״̬��Ϣ
/*
typedef struct _Stru_Data_Send_To_Missile
{
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];	//����
	Stru_Missile_State_Data		st_missile_state_data[MAX_CONNECT_NUMBER];		//����
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];		//Ŀ��
}Stru_Data_Send_To_Missile;		//�������ⲿ���͵ķ���״̬��Ϣ ����Ӧ��Stru_Data_Datalink_To_Controller����һ�£�ͬ������

typedef struct _Stru_Target_Initial_Data_In
{
	double longitude_target;	//Ŀ���ʼ����
	double latitude_target;		//Ŀ���ʼγ��
	double height_target;		//Ŀ���ʼ�߶�
	int target_type;			//Ŀ������
	double time_run;			//Ŀ�꿪ʼ�˶�ʱ��
	double velocity_target;		//Ŀ���˶��ٶ�
	double theta_target;		//Ŀ���˶����
	double psi_target;			//Ŀ���˶���λ��
	double radius_target;		//Ŀ���˶��뾶	
}Stru_Target_Initial_Data_In;	//Ŀ����������

typedef struct _Stru_Target_Status
{
	int target_ID;
	double longitude_target;	
	double latitude_target;		
	double height_target;		
	double vtx_target;
	double vty_target;
	double vtz_target;
}Stru_Target_Status;	//Ŀ��ʵʱ����

typedef enum _TARGET_TYPE
{
	ENUM_FIXED_POSITION = 0,			//�̶�Ŀ��		
	ENUM_LINEAR_MOTION = 1,			//ֱ���˶�
	ENUM_CIRCULAR_MOTION = 2,			//Բ���˶�
	ENUM_LINEAR_ACCELERATE = 3,		//ֱ�߼���
	ENUM_SNAKE_MOTION = 4			//���λ���
}TARGET_TYPE;

*/

//***************** �ⲿ������Ƹ������������� ********************//
typedef struct _Stru_Jamming_Data_In
{
	//����������� 0
	double temperature_environment;
	int    flag_wind;//����ƫ
	double velocity_wind;
	double psi_wind;
	double theta_wind;
	double lp_pitch;//������ƫ ok
	double lp_yaw;
	double lp_roll;
	double lp_lift;
	double lp_drag;
	double lp_side;
	double lp_wx;
	double lp_wy;
	double lp_wz;///14
	
	double lp_rotary_inertia;//������ƫ ok
	int    flag_jggr;//�ṹ���� ok
	double det_mass;//������ƫ ok
	double det_x_centroid;//������ƫ
	double det_y_centroid;
	double det_z_centroid;
	double gama0;	//��ʼ��̬ƫ��
	double zeta0;
	double psi0;
	double wxerr;	//���ٶ�ƫ��
	double wyerr;
	double wzerr;
	double lp_dx;//��Ч��ƫ ok
	double lp_dy;
	double lp_dz;///29

	double Lp_trust_det;//������ƫ
	double Lp_eng_flowvol;//�����ʻ�������ƫ
	double trust_det_pos;
	double trust_det_angle;
	double trust_det_alpha;
	double trust_det_gama;//35
		
}Stru_Jamming_Data_In;	//�������и�����������???...

typedef struct _Stru_Mission_Data_In
{
	Stru_Jamming_Data_In st_jamming_data_in;
	Stru_Initial_Data *	p_st_initial_data;
	Stru_Route_Data	  *	p_st_route_data;
}Stru_Mission_Data_In;	//��������

//****************** �����ڲ����豸��ͨѶ���� *********************//

//���ٹܸ��ۿػ����ݰ�
typedef struct _Stru_Data_Baro_To_Controller  
{
	double static_pressure; //��ѹ���������
	double   total_pressure;	 //��ѹ���������
}Stru_Data_Baro_To_Controller;	//���ٹܸ��ۿػ����ݰ�	

//���ߵ�߶ȱ����ۿػ����ݰ�
typedef struct _Stru_Data_RadioAlt_To_Controller  
{
	int radioalt_status; //����״̬
	double radioalt_hight;//��Ը߶ȣ�Ҳ����߶�
}Stru_Data_RadioAlt_To_Controller;	//���ߵ�߶ȱ����ۿػ����ݰ�	

typedef struct _Stru_Data_Seeker_To_Controller  
{
	bool   flag_combat_status;//�ȶ�����������Ŀ�꣬��ʶ
	bool   flag_seize_stable;	//�����ȶ�����ʶ
	
	double pitch_LOS_rate;		//������
	double yaw_LOS_rate;	
	double pitch_gimbal_angle;//��ܽ�
	double yaw_gimbal_angle;
	
	double longitude_target;	//Ŀ��λ��
	double latitude_target;
	double distance_target;
	
	double pitch_LOS_angle;	//ʧ׼��
	double yaw_LOS_angle;
}Stru_Data_Seeker_To_Controller;	//����ͷ���ۿػ����ݰ�

typedef struct _Stru_Data_INS_To_Controller  
{
	double gama;//��ŷ���ǣ����춫����ϵ������231ת��ǰ���ҵ���ϵ
	double psi;
	double zeta;
	double gamas;//��ŷ���ǣ�321ת��	//014��ʹ��
	double psis;
	double zetas;
	double wx;	//deg
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;//���춫����ϵ
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
	int    GPS_status;
}Stru_Data_INS_To_Controller;	//�ߵ����ۿػ����ݰ�

typedef struct _Stru_Data_Datalink_To_Controller  
{
	//���ߺ���װ������
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];//�������������ӣ����ڵ�
	//��ӿ����ٵ�״̬����
	Stru_Missile_State_Data		st_missile_state_data[MAX_CONNECT_NUMBER];
	//̬�ƹ���Ŀ�����
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];
}Stru_Data_Datalink_To_Controller;	//���������ۿػ����ݰ� ����Ӧ��Stru_Data_Send_To_Missile����һ�£�ͬ������

typedef struct _Stru_Data_Datalink_To_ControllerSig  
{
	//int missile_ID;//δʹ��
	//int	target_ID;	//δʹ��
	int	update_count;//���´���
	int	num_waypoint_updated;	//��ǰ���£���Ч�ĺ�������Ŀ
	double longitude  [MAX_ROUTE_NUMBER];
	double latitude   [MAX_ROUTE_NUMBER];
	double height     [MAX_ROUTE_NUMBER];
	int    route_mode [MAX_ROUTE_NUMBER];//��������
	int    formation_mode[MAX_ROUTE_NUMBER];		//������Ϣ��δʹ��
	double dltTime[MAX_ROUTE_NUMBER];//����ʱ��
	double velocity   [MAX_ROUTE_NUMBER];//�ٶ�ָ��
	double turn_angle [MAX_ROUTE_NUMBER];//�г������� �� ת��Ƕȣ����ã� �� ������
	double turn_radius[MAX_ROUTE_NUMBER];//ת��뾶�������뾶��δʹ��
	double accept_radius[MAX_ROUTE_NUMBER];//���ܰ뾶��δʹ��
}Stru_Data_Datalink_To_ControllerSig;	//���������ۿػ����ݰ����޼�Ⱥ

typedef struct _Stru_Data_Engine_To_Controller
{
	double rpm_engine;		//014 ָ��69��ʵ��ת��
	int ECU_work_status;	//����״̬	//014 ������״̬����״̬ 0ͣ�� 1������ 2ɢ�� 5����
	//int engine_start_result;//����״̬0x55���������У�0xAA������ɣ�0xFF�����쳣	//014��ʹ��
}Stru_Data_Engine_To_Controller;	//���������ۿػ����ݰ�

typedef struct _Stru_Data_Controller_To_Seeker  
{
	bool flag_seeker_on;		//����ͷ����
	bool flag_lock_on_permit;//Ŀ����������
	double pitch_gimbal_angle_calc;	//������ܽ�ָ��
	double yaw_gimbal_angle_calc;	//�����ܽ�ָ��
}Stru_Data_Controller_To_Seeker;	//�ۿػ�������ͷ���ݰ�

typedef struct _Stru_Data_Controller_To_Datalink  
{
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];//��Ⱥ״̬���ɿظ����������Ż���С
	Stru_Missile_State_Data		st_missile_state_data;//����״̬
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];//Ŀ��״̬
}Stru_Data_Controller_To_Datalink;	//�ۿػ������������ݰ�

typedef struct Stru_Data_Controller_To_DatalinkTel  
{
	//������Ϣ
	int curPtNo;			//��ǰ�����
	double curTargetLon;	//��ǰĿ�꺽�㾭��
	double curTargetLat;	//��ǰĿ�꺽��γ��
	double curTargetAlt;	//��ǰĿ�꺽��߶�
	//bool on_takeoff;	//�����ɱ�־	�ڵ�����������ṹ��
	//bool open_umbrella;//��ɡ��־   
	//bool enginge_off;	//����ͣ����־

	//ͨ����
	double rudderRollCmd;	//ͨ���渱��
	double rudderPitchCmd;	//ͨ��������
	double rudderYawCmd;	//ͨ���溽��
	//ң����Ϣ
	double gamaCmd;	//��ת��ָ��
	double nycCmd;	//����ָ��
	double varthetaCmd;//������ָ��
	double heightCmd;//�߶�ָ��
	double ac_dL;//���ɾ�
	double ac_dZ;// ��߾�
	int token_long;// ��������
	int token_late;// ��������
	double thrustCmd;// ����ָ��
	double  ac_dPsi;//�����ƫ��
	double ac_dR;// Բ�켣��߾�
	double cur_thetav;//�켣���
	double Vcmd;//�ٶ�ָ�� 
	double nyCmd_Guidance;//ĩ�Ƶ��������ָ��
	double nzCmd_Guidance;//ĩ�Ƶ��������ָ��
	double pitch_rate_nT_filterOut;//�������߽��ٶ��˲�
	double yaw_rate_nT_filterOut;//ƫ�����߽��ٶ��˲�
	double deltaR;//��Ŀ����
	double dRn; //��Ŀ�������  δ�õ���Ԥ��
	double dRu; //��Ŀ�������  δ�õ���Ԥ��
	double dRe; //��Ŀ�������  δ�õ���Ԥ��
	double Pitch_Preset_Angle;//���۸�����ܽ�
	double Yaw_Preset_Angle;//����ƫ����ܽ�
	int Dubins_stage;//�ű�˹��
	int dubins_type1;//�ű�˹����
	int dubins_type2;//�ű�˹����
	int dubins_type3;//�ű�˹����
	double Dubins_length;//�ű�˹�κ���
	double test1;//����
	double Min_IAS2Vel;//��������ٶ�
	double mx_ESO;
	double fduox_ADRC;//ADRC��ƫ
	double Qv;//��ѹ
	double alpha_ins;	//���ٹ���
	double beta_ins;	//���ٲ໬��
	double MaxRpm; 		//���ת��
	double DFT_freq_max;//��ʶ�˶�Ƶ��
	//���䣬���ڷ�����ʾ
	double rudder_I_cmd;	//�����棬ת��Ϊ����Ƕ�
	double rudder_II_cmd;
	double rudder_III_cmd;
	double rudder_IV_cmd;
	double rudder_V_cmd;
	double rudder_VI_cmd;

	double Kc_cmd;			//����

	double curLon;	//��ǰ���ȡ�γ�ȡ��߶�
	double curLat;	
	double curAlt;	
}Stru_Data_Controller_To_DatalinkTel;	//�ۿػ������������ݰ�

typedef struct _Stru_Data_Controller_To_Actuator  
{
	double control_voltage_I;
	double control_voltage_II;
	double control_voltage_III;
	double control_voltage_IV;
	double control_voltage_V;
	double control_voltage_VI;
}Stru_Data_Controller_To_Actuator;	//�ۿػ��������ݰ�

typedef struct _Stru_Data_Controller_To_Engine  
{
	double control_Kc;	//ָ�����ţ���ѡһ������ 	//014 ȡֵ��Χ0.0~100.0
	double control_rpm;//ָ��ת�٣���ѡһ��δʹ��	//014��ʹ��
	int ECU_work_cmd;	//����ָ��0x00 ��ָ�0x11 �Լ죬0x22 ������0x33 ת�ٿ���,  0x44 �ػ�	//014��ʹ��
}Stru_Data_Controller_To_Engine;	//�ۿػ������������ݰ�

typedef struct _Stru_Data_Controller_To_Switch_Output  
{
	bool flag_separate_booster;		//����������(��ը��˨���и������)
	bool flag_launch_missile_wing;	//����չ��(��ը��˨���)
	bool flag_engine_start;			//�����������(����������)
	
	bool flag_missle_takeoff;	//��� 20260415
	bool flag_engine_shutdown;	//�������ػ� 20260415
	bool flag_open_umbrella;	//��ɡ	20260415
	bool flag_fuze_unlock;		//���Ž��� 20260425
}Stru_Data_Controller_To_Switch_Output;	//�ۿػ�������ָ�δ�õ�


//******************* �����뻷���佻������ ************************//
typedef struct _Stru_Rudder_Reflection
{
	double missile_rudder_delta_I;
	double missile_rudder_delta_II;
	double missile_rudder_delta_III;
	double missile_rudder_delta_IV;
	double missile_rudder_delta_V;//�����
	double missile_rudder_delta_VI;
}Stru_Rudder_Reflection;	//��ƫ���

typedef struct _Stru_Data_Missile_To_Environment 
{
	double rpm_engine;//������ת�٣��ɿظ����������ţ���������������ģ�ͣ����������ת��
	Stru_Rudder_Reflection st_missile_rudder_reflection;//��ƫ���
	Stru_Data_Controller_To_Switch_Output st_missile_status_switch;//���������
}Stru_Data_Missile_To_Environment;	//�����������������

typedef struct _Stru_INS_Data_In
{
	double gama;//��ŷ�� 231ת��
	double psi;
	double zeta;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
}Stru_INS_Data_In;	//�������ߵ���������

typedef struct _Stru_RadioAlt_Data_In
{
	double radioalt_height;
}Stru_RadioAlt_Data_In;	//���������ߵ�߶ȱ���������

typedef struct _Stru_Baro_Data_In
{
	double static_pressure;
	double total_pressure;
}Stru_Baro_Data_In;	//���������ٹ���������

typedef struct _Stru_Engine_Data_In
{
	double air_density;//�����ܶ�
	double air_speed;	 //����
	double angle_of_attack;	//���ǣ�δ�õ�
	double angle_of_side_slip;//�໬�ǣ�δ�õ�					
}Stru_Engine_Data_In;	//��������������������
typedef struct _Stru_Seeker_Data_In
{
	int target_ID;
	double pitch_LOS_angle;
	double yaw_LOS_angle;
	double pitch_LOS_rate;
	double yaw_LOS_rate;
	double distance_target;
	double longitude_target;
	double latitude_target;
}Stru_Seeker_Data_In;	//����ͷ��������

typedef struct _Stru_Data_Environment_To_Missile
{
	//���ߵ�����
	Stru_INS_Data_In st_data_environment_to_ins;
	//���߶ȱ�
	Stru_RadioAlt_Data_In st_data_environment_to_radioalt;
	//�����ٹ�
	Stru_Baro_Data_In st_data_environment_to_baro;
	//��������
	Stru_Engine_Data_In st_data_environment_to_engine;
	//������ͷ����
	Stru_Seeker_Data_In st_data_environment_to_seeker[MAX_TARGET_NUMBER];
}Stru_Data_Environment_To_Missile;	//������������������

//****************** �����ڲ���ģ��佻������ *********************//
typedef struct _Stru_Data_Earth_Model_Out 
{
	double gravitational_acceleration;
	double air_density;
	double air_temperature;
	double air_pressure;
	double sonic_speed;
	double velocity_wind;
	double psi_wind;
	double theta_wind;	
}Stru_Data_Earth_Model_Out;	//����ģ���������

typedef struct _Stru_Data_Earth_Model_In 
{
	double missile_height;
	double missile_longitude;
	double missile_latitude;
}Stru_Data_Earth_Model_In;	//����ģ����������

typedef struct _Stru_Data_Missile_Inertia 
{
	double mass;
	double x_centroid;
	double y_centroid;
	double z_centroid;
	double x_moment_of_inertia;
	double y_moment_of_inertia;
	double z_moment_of_inertia;
}Stru_Data_Missile_Inertia;	//������������

typedef struct _Stru_Data_Aerodynamic_Force 
{
	double lift_force;
	double drag_force;
	double side_force;
	double pitch_moment;
	double yaw_moment;
	double roll_moment;
}Stru_Data_Aerodynamic_Force;	//����������

typedef struct _Stru_Data_Aerodynamic_Force_Coefficient
{
	double mxwx;
	double mywy;
	double mzwz;
	double dcy_dalfa;
	double dmz_dalfa;
	double dcz_dbeta;
	double dmy_dbeta;
	double dcy_ddeltaz;
	double dmz_ddeltaz;
	double dmx_ddeltax;
	double dcz_ddeltay;
	double dmy_ddeltay;
}Stru_Data_Aerodynamic_Force_Coefficient;	//������ϵ��ƫ������

typedef struct _Stru_Data_Missile_Force 
{
	double gravity;
	double thrust;
	double thrust_force[3];//������������������
	double thrust_moment[3];
	Stru_Data_Aerodynamic_Force st_aerodynamic_force;//������������
}Stru_Data_Missile_Force;	//������������

typedef struct _Stru_Data_Function_To_Force_Calc
{
	double mach;
	double air_speed;
	double sonic_speed;
	double air_density;
	double air_temperature;//�����¶�
	double height;
	double wx;
	double wy;
	double wz;
	double angle_of_attack;
	double angle_of_side_slip;
	double gravitational_acceleration;
	double aby;	//����ϵ��������ٶȣ������ھ��������Σ�����������
}Stru_Data_Function_To_Force_Calc;	//��������ģ������

typedef struct _Stru_Data_Function_Solved_Out
{
	double mach;
	double air_speed;
	double angle_of_attack;
	double angle_of_side_slip;
	bool flag_leaving_launcher;	//���
	Stru_INS_Data_In * p_st_data_ins_related;
	Stru_Baro_Data_In * p_st_data_baro_related;
	Stru_Engine_Data_In * p_st_data_engine_related;
	Stru_RadioAlt_Data_In * p_st_data_radioalt_related;
	Stru_Data_Function_To_Force_Calc * p_st_data_function_to_force_calc;
}Stru_Data_Function_Solved_Out;	//���̽������

//��������
typedef struct _Stru_HIL_Data_INPUT
{
	//���ָ��
    double rudderPitchLeft;//I 
    double rudderPitchRight;//II
    double rudderRollLeft;//III
    double rudderRollRight;//IV
	double rudderYawLeft;//V
    double rudderYawRight;//VI

	//������ָ��
    double Kc;	//����

	//����ͷ����
	bool flag_seeker_on;		//����ͷ����
	bool flag_lock_on_permit;//Ŀ����������
	double pitch_gimbal_angle_calc;	//������ܽ�ָ��
	double yaw_gimbal_angle_calc;	//�����ܽ�ָ��
	
	//������ָ��
	bool flag_missile_takeoff;	//���
	bool flag_engine_shutdown;	//�������ػ�
	bool flag_open_umbrella;	//��ɡ����
	bool flag_fuze_unlock;		//���Ž���
}Stru_HIL_Data_INPUT;

//�������
typedef struct _Stru_HIL_Data_OUTPUT
{
    double wx;	//��ϵ���
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    double pitch;
    double yaw;
    double roll;
    double airSpd;//δ�õ�
    double lon;
    double lat;
    double alt;
    double vn;
    double vs;
    double ve;
    double DD1;//���ٹܾ�ѹ
    double DD2;//���ٹ���ѹ
    
    double DD3;//δʹ��
    double DD4;//δʹ��
    double mass;//δʹ��
    double xg;//δʹ��
    double arp;//δʹ��
    
    double rpm_state;//������(״̬ת��)
    double MX_T_Disturb;//δʹ��

	//����ͷ���ݣ�δ�õ�
	double qf;//����ʵ�ֽ�
    double qh;//�������߽�
    double dqf;//�������߽��ٶ�
    double dqh;//�������߽��ٶ�
    int TargetLocked;//����ͷ������ʶ��Ĭ��ֵ0x00Ϊδ������
    
	//����
	int MissileLauched;//��ɱ�ʶ��1Ϊ��ɣ�0Ϊδ���
}Stru_HIL_Data_OUTPUT;

#endif
