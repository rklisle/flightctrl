#ifndef _DATA_PROTOCOL_H_
#define _DATA_PROTOCOL_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 定义仿真所需各模块间通讯数据内容
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
//#include "..\load_data.h"
//#include "..\datalink_sim_main.h"
//#include "..\target_sim_main.h"
#include <stdbool.h>
// #include "port/flightPort.h"
//==================================================================================
//
//						补充定义
//
//==================================================================================

#define E_CONST		(1.0/298.257)
#define RE			(6378137.0)

// 将原来的 const int 定义改为宏
#define MAX_ROUTE_NUMBER 64      // 数据链给综控机最大航路点个数
#define MAX_CONNECT_NUMBER 16    // 数据链最大连接导弹数量
#define MAX_TARGET_NUMBER 16     // 每个弹，最多16个目标
#define MAX_TIME 9999.0
#define STEP_5ms 0.005

// const int MAX_ROUTE_NUMBER = 64;	//数据链给综控机最大航路点个数
// const int MAX_CONNECT_NUMBER = 16;	//数据链最大连接导弹数量
// const int MAX_TARGET_NUMBER = 16;	//每个弹，最多16个目标
// const double MAX_TIME = 9999.0;
// const double STEP_5ms = 0.005;

typedef struct _Stru_Initial_Data
{
	int	missile_ID;			//弹编号
	double longitude_launch;	//发射点
	double latitude_launch;	
	double height_launch;
	double initial_parameter1;//预留初始参数1，例如发射点温度等，可估算 声速、大气等模型
	double initial_parameter2;//预留初始参数2，飞行仿真模式
	double launch_time;		//发射时间	
	double lauch_azimuth;		//发射方位角
	double lauch_pitch;		//发射俯仰角
	double lauch_booster_pitch;//助推器俯仰角
	//double type_target;		//目标点类型
	//double longitude_target;	//目标点 经、纬及高度
	//double latitude_target;	
	//double height_target;	
}Stru_Initial_Data;	//导弹初始状态装订数据

typedef struct  _Stru_Way_Point
{
	int num;//航点编号
	
	double longitude;	//航点经度
	double latitude;	//航点纬度
	double height;		//航点高度
	int    route_mode; //航点类型
	int    formation_mode;	//航点信息，未使用
	double dltTime;//航段时间
	double turn_angle;//切出航迹角 或 转弯角度（复用） 或 打击落角
	double turn_radius;//转弯半径或盘旋半径，未使用
	double velocity;	//速度指令
	double accept_radius;//接受半径，未使用
	
	//处理后数据，标识
	bool if_flightime_ctrl;/*[到达时间标识；1有效，0无效；]*/		
	bool if_relativehigh_ctrl;  /*[相对高度（或真高度）控制标识：1有效，0无效]*/     		
	bool if_heading_hold;  /*[指点飞行标识：1有效，0无效；]*/
	bool if_groundspeed_ctrl;  /*[地速控制标识：1地速，0空速；]*/     		
	bool if_attackangle_ctrl;/*[打击落角标识：1指定落角，0无约束；]*/				
	bool if_turndir_set;/*[盘旋转弯：左转、右转标识；]*/		
	bool if_prepare_hover;/*[预盘旋标识：1有效，0无效；]*/		
	//处理后数据
	double outtrack_angle;/*[指点飞行切出角度，北偏西为正绝对，切出角度deg]*/
	int hover_round; /*[盘旋圈数]*/
	double attack_angle;/*[打击落角]*/
}Stru_Way_Point;

typedef struct _Stru_Route_Data
{
	int num_rows;	// 航点数
	int num_columns;	// 常量	11
	char ** p_str_title; 	// 忽略
	// 指向Stru_Way_Point[num_rows动态生成]
	double * p_route_data;		//行为不同航点编号，列为航点特征: 编号、经度、纬度、转弯半径、角度、速度、航点类型、信息、高度
	// 下列：控制内部处理
	double longitude_target;
	double latitude_target;
	double height_target;
}Stru_Route_Data;	//导弹飞行航路数据

typedef struct _Stru_Mission_Update_Data
{
	int missile_ID;
	int	target_ID;
	int	update_count;//更新次数
	int	num_waypoint_updated;	//当前跟新，有效的航迹点数目
	double longitude  [MAX_ROUTE_NUMBER];
	double latitude   [MAX_ROUTE_NUMBER];
	double height     [MAX_ROUTE_NUMBER];
	double turn_radius[MAX_ROUTE_NUMBER];
	double turn_angle [MAX_ROUTE_NUMBER];
	double velocity   [MAX_ROUTE_NUMBER];
	int    route_mode [MAX_ROUTE_NUMBER];
	int    formation_mode[MAX_ROUTE_NUMBER];
}Stru_Mission_Update_Data;		//在线航迹装订数据(可根据任务规划算法生成，待补充)

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
}Stru_Missile_State_Data;		//导弹向外部发送的飞行状态信息

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
}Stru_Target_State_Data;		//导弹向外部发送的飞行状态信息
/*
typedef struct _Stru_Data_Send_To_Missile
{
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];	//任务
	Stru_Missile_State_Data		st_missile_state_data[MAX_CONNECT_NUMBER];		//导弹
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];		//目标
}Stru_Data_Send_To_Missile;		//导弹向外部发送的飞行状态信息 内容应与Stru_Data_Datalink_To_Controller保持一致，同步更新

typedef struct _Stru_Target_Initial_Data_In
{
	double longitude_target;	//目标初始经度
	double latitude_target;		//目标初始纬度
	double height_target;		//目标初始高度
	int target_type;			//目标类型
	double time_run;			//目标开始运动时间
	double velocity_target;		//目标运动速度
	double theta_target;		//目标运动倾角
	double psi_target;			//目标运动方位角
	double radius_target;		//目标运动半径	
}Stru_Target_Initial_Data_In;	//目标设置数据

typedef struct _Stru_Target_Status
{
	int target_ID;
	double longitude_target;	
	double latitude_target;		
	double height_target;		
	double vtx_target;
	double vty_target;
	double vtz_target;
}Stru_Target_Status;	//目标实时数据

typedef enum _TARGET_TYPE
{
	ENUM_FIXED_POSITION = 0,			//固定目标		
	ENUM_LINEAR_MOTION = 1,			//直线运动
	ENUM_CIRCULAR_MOTION = 2,			//圆形运动
	ENUM_LINEAR_ACCELERATE = 3,		//直线加速
	ENUM_SNAKE_MOTION = 4			//蛇形机动
}TARGET_TYPE;

*/

//***************** 外部仿真控制给导弹输入数据 ********************//
typedef struct _Stru_Jamming_Data_In
{
	//干扰条件编号 0
	double temperature_environment;
	int    flag_wind;//风拉偏
	double velocity_wind;
	double psi_wind;
	double theta_wind;
	double lp_pitch;//气动拉偏 ok
	double lp_yaw;
	double lp_roll;
	double lp_lift;
	double lp_drag;
	double lp_side;
	double lp_wx;
	double lp_wy;
	double lp_wz;///14
	
	double lp_rotary_inertia;//惯量拉偏 ok
	int    flag_jggr;//结构干扰 ok
	double det_mass;//质量拉偏 ok
	double det_x_centroid;//质心拉偏
	double det_y_centroid;
	double det_z_centroid;
	double gama0;	//初始姿态偏差
	double zeta0;
	double psi0;
	double wxerr;	//角速度偏差
	double wyerr;
	double wzerr;
	double lp_dx;//舵效拉偏 ok
	double lp_dy;
	double lp_dz;///29

	double Lp_trust_det;//推力拉偏
	double Lp_eng_flowvol;//耗油率或流量拉偏
	double trust_det_pos;
	double trust_det_angle;
	double trust_det_alpha;
	double trust_det_gama;//35
		
}Stru_Jamming_Data_In;	//导弹飞行干扰条件数据???...

typedef struct _Stru_Mission_Data_In
{
	Stru_Jamming_Data_In st_jamming_data_in;
	Stru_Initial_Data *	p_st_initial_data;
	Stru_Route_Data	  *	p_st_route_data;
}Stru_Mission_Data_In;	//任务数据

//****************** 弹体内部各设备间通讯数据 *********************//

//空速管给综控机数据包
typedef struct _Stru_Data_Baro_To_Controller  
{
	double static_pressure; //静压传感器输出
	double   total_pressure;	 //总压传感器输出
}Stru_Data_Baro_To_Controller;	//空速管给综控机数据包	

//无线电高度表给综控机数据包
typedef struct _Stru_Data_RadioAlt_To_Controller  
{
	int radioalt_status; //工作状态
	double radioalt_hight;//相对高度，也称真高度
}Stru_Data_RadioAlt_To_Controller;	//无线电高度表给综控机数据包	

typedef struct _Stru_Data_Seeker_To_Controller  
{
	bool   flag_combat_status;//稳定跟踪且锁定目标，标识
	bool   flag_seize_stable;	//光轴稳定，标识
	double pitch_LOS_rate;		//角速率
	double yaw_LOS_rate;	
	double pitch_gimbal_angle;//框架角
	double yaw_gimbal_angle;
	double longitude_target;	//目标位置
	double latitude_target;
	double distance_target;
	double pitch_LOS_angle;	//失准角
	double yaw_LOS_angle;
}Stru_Data_Seeker_To_Controller;	//导引头给综控机数据包

typedef struct _Stru_Data_INS_To_Controller  
{
	double gama;//正欧拉角：北天东地理系，经过231转序，前上右弹体系
	double psi;
	double zeta;
	double gamas;//反欧拉角：321转序
	double psis;
	double zetas;
	double wx;	//deg
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;
	double vtx;//北天东地理系
	double vty;
	double vtz;
	double longitude;
	double latitude;
	double height;
	int    GPS_status;
}Stru_Data_INS_To_Controller;	//惯导给综控机数据包

typedef struct _Stru_Data_Datalink_To_Controller  
{
	//在线航迹装订参数
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];//所有网络内链接，弹节点
	//编队控制临弹状态参数
	Stru_Missile_State_Data		st_missile_state_data[MAX_CONNECT_NUMBER];
	//态势构建目标参数
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];
}Stru_Data_Datalink_To_Controller;	//数据链给综控机数据包 内容应与Stru_Data_Send_To_Missile保持一致，同步更新

typedef struct _Stru_Data_Datalink_To_ControllerSig  
{
	//int missile_ID;//未使用
	//int	target_ID;	//未使用
	int	update_count;//更新次数
	int	num_waypoint_updated;	//当前跟新，有效的航迹点数目
	double longitude  [MAX_ROUTE_NUMBER];
	double latitude   [MAX_ROUTE_NUMBER];
	double height     [MAX_ROUTE_NUMBER];
	int    route_mode [MAX_ROUTE_NUMBER];//航点类型
	int    formation_mode[MAX_ROUTE_NUMBER];		//航点信息，未使用
	double dltTime[MAX_ROUTE_NUMBER];//航段时间
	double turn_angle [MAX_ROUTE_NUMBER];//切出航迹角 或 转弯角度（复用） 或 打击落角
	double turn_radius[MAX_ROUTE_NUMBER];//转弯半径或盘旋半径，未使用
	double velocity   [MAX_ROUTE_NUMBER];//速度指令
	double accept_radius[MAX_ROUTE_NUMBER];//接受半径，未使用
}Stru_Data_Datalink_To_ControllerSig;	//数据链给综控机数据包，无集群

typedef struct _Stru_Data_Engine_To_Controller
{
	double rpm_engine;
	int ECU_work_status;	//工作状态
	int engine_start_result;//启动状态0x55启动过程中，0xAA启动完成，0xFF启动异常
}Stru_Data_Engine_To_Controller;	//发动机给综控机数据包

typedef struct _Stru_Data_Controller_To_Seeker  
{
	bool flag_seeker_on;		//导引头开机
	bool flag_lock_on_permit;//目标锁定允许
	double pitch_gimbal_angle_calc;	//俯仰框架角指令
	double yaw_gimbal_angle_calc;	//航向框架角指令
}Stru_Data_Controller_To_Seeker;	//综控机给导引头数据包

typedef struct _Stru_Data_Controller_To_Datalink  
{
	Stru_Mission_Update_Data	st_mission_update_data[MAX_CONNECT_NUMBER];//集群状态，飞控给数据链，优化减小
	Stru_Missile_State_Data		st_missile_state_data;//导弹状态
	Stru_Target_State_Data		st_target_state_data[MAX_TARGET_NUMBER];//目标状态
}Stru_Data_Controller_To_Datalink;	//综控机给数据链数据包

typedef struct Stru_Data_Controller_To_DatalinkTel  
{
	//航点信息
	int curPtNo;			//当前航点号
	double curTargetLon;	//当前目标航点经度
	double curTargetLat;	//当前目标航点纬度
	double curTargetAlt;	//当前目标航点高度
	//bool on_takeoff;	//起飞完成标志	在电气控制输出结构体
	//bool open_umbrella;//开伞标志   
	//bool enginge_off;	//动力停车标志

	//通道舵
	double rudderRollCmd;	//通道舵副翼
	double rudderPitchCmd;	//通道舵升降
	double rudderYawCmd;	//通道舵航向
	//遥测信息
	double gamaCmd;	//滚转角指令
	double nycCmd;	//过载指令
	double varthetaCmd;//俯仰角指令
	double heightCmd;//高度指令
	double ac_dL;//待飞距
	double ac_dZ;// 侧边距
	int token_long;// 纵向令牌
	int token_late;// 侧向令牌
	double thrustCmd;// 推力指令
	double  ac_dPsi;//航向角偏差
	double ac_dR;// 圆轨迹侧边距
	double cur_thetav;//轨迹倾角
	double Vcmd;//速度指令 
	double nyCmd_Guidance;//末制导纵向过载指令
	double nzCmd_Guidance;//末制导侧向过载指令
	double pitch_rate_nT_filterOut;//俯仰视线角速度滤波
	double yaw_rate_nT_filterOut;//偏航视线角速度滤波
	double deltaR;//弹目距离
	double dRn; //弹目北向距离  未用到，预留
	double dRu; //弹目天向距离  未用到，预留
	double dRe; //弹目东向距离  未用到，预留
	double Pitch_Preset_Angle;//理论俯仰框架角
	double Yaw_Preset_Angle;//理论偏航框架角
	int Dubins_stage;//杜宾斯段
	int dubins_type1;//杜宾斯类型
	int dubins_type2;//杜宾斯类型
	int dubins_type3;//杜宾斯类型
	double Dubins_length;//杜宾斯段航程
	double test1;//测试
	double Min_IAS2Vel;//最低折算速度
	double mx_ESO;
	double fduox_ADRC;//ADRC舵偏
	double Qv;//动压
	double alpha_ins;	//地速攻角
	double beta_ins;	//地速侧滑角
	double MaxRpm; 		//最大转速
	double DFT_freq_max;//辨识运动频率
}Stru_Data_Controller_To_DatalinkTel;	//综控机给数据链数据包

typedef struct _Stru_Data_Controller_To_Actuator  
{
	double control_voltage_I;
	double control_voltage_II;
	double control_voltage_III;
	double control_voltage_IV;
	double control_voltage_V;
	double control_voltage_VI;
}Stru_Data_Controller_To_Actuator;	//综控机给舵数据包

typedef struct _Stru_Data_Controller_To_Engine  
{
	double control_Kc;	//指令油门，二选一
	double control_rpm;//指令转速，二选一
	int ECU_work_cmd;	//控制指令0x00 无指令，0x11 自检，0x22 启动，0x33 转速控制,  0x44 关机
}Stru_Data_Controller_To_Engine;	//综控机给发动机数据包

typedef struct _Stru_Data_Controller_To_Switch_Output  
{
	bool flag_separate_booster;		//助推器分离(爆炸螺栓或切割锁点火)
	bool flag_launch_missile_wing;	//弹翼展开(爆炸螺栓点火)
	bool flag_engine_start;			//主发动机点火(空中炮起点火)
	
	bool flag_missle_takeoff;			//起飞 20260415
	bool flag_engine_shutdown;		//发动机关机 20260415
	bool flag_open_umbrella;			//开伞	20260415
}Stru_Data_Controller_To_Switch_Output;	//综控机开关量指令，未用到


//******************* 弹体与环境间交换数据 ************************//
typedef struct _Stru_Rudder_Reflection
{
	double missile_rudder_delta_I;
	double missile_rudder_delta_II;
	double missile_rudder_delta_III;
	double missile_rudder_delta_IV;
	double missile_rudder_delta_V;//航向舵
	double missile_rudder_delta_VI;
}Stru_Rudder_Reflection;	//舵偏输出

typedef struct _Stru_Data_Missile_To_Environment 
{
	double rpm_engine;//发动机转速，飞控给发动机油门，发动机与螺旋桨模型，输出螺旋桨转速
	Stru_Rudder_Reflection st_missile_rudder_reflection;//舵偏输出
	Stru_Data_Controller_To_Switch_Output st_missile_status_switch;//开关量输出
}Stru_Data_Missile_To_Environment;	//导弹给环境输出数据

typedef struct _Stru_INS_Data_In
{
	double gama;//正欧拉 231转序
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
}Stru_INS_Data_In;	//环境给惯导输入数据

typedef struct _Stru_RadioAlt_Data_In
{
	double radioalt_height;
}Stru_RadioAlt_Data_In;	//环境给无线电高度表输入数据

typedef struct _Stru_Baro_Data_In
{
	double static_pressure;
	double total_pressure;
}Stru_Baro_Data_In;	//环境给空速管输入数据

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
}Stru_Seeker_Data_In;	//导引头输入数据

typedef struct _Stru_Data_Environment_To_Missile
{
	//给惯导数据
	Stru_INS_Data_In st_data_environment_to_ins;
	//给高度表
	Stru_RadioAlt_Data_In st_data_environment_to_radioalt;
	//给空速管
	Stru_Baro_Data_In st_data_environment_to_baro;
	//给导引头数据
	Stru_Seeker_Data_In st_data_environment_to_seeker[MAX_TARGET_NUMBER];
}Stru_Data_Environment_To_Missile;	//环境给导弹输入数据

//****************** 环境内部各模块间交换数据 *********************//
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
}Stru_Data_Earth_Model_Out;	//地球模型输出数据

typedef struct _Stru_Data_Earth_Model_In 
{
	double missile_height;
	double missile_longitude;
	double missile_latitude;
}Stru_Data_Earth_Model_In;	//地球模型输入数据

typedef struct _Stru_Data_Missile_Inertia 
{
	double mass;
	double x_centroid;
	double y_centroid;
	double z_centroid;
	double x_moment_of_inertia;
	double y_moment_of_inertia;
	double z_moment_of_inertia;
}Stru_Data_Missile_Inertia;	//导弹惯性数据

typedef struct _Stru_Data_Aerodynamic_Force 
{
	double lift_force;
	double drag_force;
	double side_force;
	double pitch_moment;
	double yaw_moment;
	double roll_moment;
}Stru_Data_Aerodynamic_Force;	//气动力数据

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
}Stru_Data_Aerodynamic_Force_Coefficient;	//气动力系数偏导数据

typedef struct _Stru_Data_Missile_Force 
{
	double gravity;
	double thrust;
	double thrust_force[3];//助推器及发动机推力
	double thrust_moment[3];
	Stru_Data_Aerodynamic_Force st_aerodynamic_force;//气动力及力矩
}Stru_Data_Missile_Force;	//导弹受力数据

typedef struct _Stru_Data_Function_To_Force_Calc
{
	double mach;
	double air_speed;
	double sonic_speed;
	double air_density;
	double air_temperature;//空气温度
	double height;
	double wx;
	double wy;
	double wz;
	double angle_of_attack;
	double angle_of_side_slip;
	double gravitational_acceleration;
	double aby;	//体轴系，法向过载，可用于静气弹变形，气动力计算
}Stru_Data_Function_To_Force_Calc;	//导弹受力模型输入

typedef struct _Stru_Data_Function_Solved_Out
{
	double mach;
	double air_speed;
	double angle_of_attack;
	double angle_of_side_slip;
	bool flag_leaving_launcher;	//离架
	Stru_INS_Data_In * p_st_data_ins_related;
	Stru_Baro_Data_In * p_st_data_baro_related;
	Stru_RadioAlt_Data_In * p_st_data_radioalt_related;
	Stru_Data_Function_To_Force_Calc * p_st_data_function_to_force_calc;
}Stru_Data_Function_Solved_Out;	//方程解算输出

#endif
