#ifndef _DATA_PROTOCOL_H_
#define _DATA_PROTOCOL_H_
//==================================================================/
// 摘要: 定义仿真所需各模块间通讯数据内容
//==================================================================/
//#include "..\load_data.h"
//#include "..\datalink_sim_main.h"
//#include "..\target_sim_main.h"
#include <stdbool.h>
// #include "port/flightPort.h"

#define E_CONST		(1.0/298.257)
#define RE			(6378137.0)
//***************** 外部仿真控制给导弹输入数据 ********************//
#define MAX_ROUTE_NUMBER 64 
#define MAX_CONNECT_NUMBER 16
#define MAX_TARGET_NUMBER 16
#define MAX_TIME 99999.0
#define STEP_5ms 0.005
typedef struct _Stru_Initial_Data
{
	int	missile_ID;			//弹编号
	double longitude_launch;	//发射点经、纬、高度
	double latitude_launch;	
	double height_launch;
	double initial_parameter1;//预留初始参数1，例如发射点温度等，用于估算 声速、大气等模型
	double initial_parameter2;//预留初始参数2，飞行仿真模式
	double launch_time;		//发射时间	
	double lauch_azimuth;		//发射方位角
	double lauch_pitch;		//发射俯仰角
	double lauch_booster_pitch;//助推器俯仰角

	//初始段爬升角、起飞完成后平飞攻角
	double climb_ktheta_enc;	//典型值，8.0deg
	double cruise_ktheta_enc;	//典型值，2.0deg
	//待补充其他装订值???...
	
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
	double dltTime;		//航段时间
	double velocity;	//速度指令
	//一、初始装订时（处理前），不同航点类型，复用为不同功能
	//发射点，初始航向角(北偏西为正) 
	//普通航点或指点飞行，切出航迹角（北偏西为正180.0~180.0）
	//盘旋点，盘旋圈数（北偏西为正，北偏东为负）
	//打击点或虚拟打击点，打击落角（负为有效值，零/正为无效值）
	//二、转弯角度（处理后，北偏东为正180.0~180.0） 或 盘旋圈数 或 无效
	double turn_angle;
	double turn_radius;//转弯半径或盘旋半径，未使用
	double accept_radius;//接受半径，未使用
	
	//处理后数据，标识
	bool if_flightime_ctrl;	//[到达时间标识；1有效，0无效；]
	bool if_groundspeed_ctrl;	//[地速控制标识：1地速，0空速；]

	bool if_heading_hold;  	//[指点飞行标识：1有效，0无效；]
	bool if_turndir_set;		//[盘旋转弯：左转、右转标识；]
	bool if_prepare_hover;	//[预盘旋标识：1有效，0无效；]
	
	bool if_relativehigh_ctrl;//[相对高度（或真高度）控制标识：1有效，0无效]
	bool if_attackangle_ctrl;	//[打击落角标识：1指定落角，0无约束；]				
	//处理后数据
	double outtrack_angle;	//[指点飞行切出角度，北偏西为正绝对，切出角度deg]
	int hover_round;		//[盘旋圈数]
	double attack_angle;	//[打击落角]
	double recycle_ground_hight;//回收点地面高度
}Stru_Way_Point;
typedef struct _Stru_Route_Data
{
	int num_rows;	
	int num_columns;
	char ** p_str_title; 

	double * p_route_data;

	double longitude_target;
	double latitude_target;
	double height_target;
}Stru_Route_Data;
typedef struct _Stru_Mission_Update_Data
{
	int missile_ID;
	int	target_ID;
	int	update_count;
	int	num_waypoint_updated;
	double longitude  [MAX_ROUTE_NUMBER];
	double latitude   [MAX_ROUTE_NUMBER];
	double height     [MAX_ROUTE_NUMBER];
	double turn_radius[MAX_ROUTE_NUMBER];
	double turn_angle [MAX_ROUTE_NUMBER];
	double velocity   [MAX_ROUTE_NUMBER];
	int    route_mode [MAX_ROUTE_NUMBER];
	int    formation_mode[MAX_ROUTE_NUMBER];
}Stru_Mission_Update_Data;
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
}Stru_Missile_State_Data;
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
}Stru_Target_State_Data;
typedef struct _Stru_Jamming_Data_In
{
	//干扰条件编号 0
	double temperature_environment;//发射温度，即助推器平均推力及时长拉偏ok
	int    flag_wind;//突变风拉偏 ok，暂不参与其他耦合
	double velocity_wind;
	double psi_wind;
	double theta_wind;///5
	
	double lp_pitch;//气动拉偏 ok
	double lp_yaw;
	double lp_roll;
	double lp_lift;
	double lp_drag;
	double lp_side;
	double lp_wx;
	double lp_wy;
	double lp_wz;///14
	
	double lp_rotary_inertia;//惯量拉偏 ok ///15
	int    flag_jggr;//结构干扰，1为正干扰力矩，-1为负干扰力矩，0为无干扰 ok
	double det_mass;//质量偏差拉偏 kg ok
	double det_x_centroid;//质心拉偏，m，结构系j ok
	double det_y_centroid;
	double det_z_centroid;
	double gama0;	//初始姿态偏差，认为对准误差很小，一般惯导安装误差 ok ///21
	double zeta0;
	double psi0;
	double wxerr;	//角速度偏差，认为惯导零偏 ok
	double wyerr;
	double wzerr;	///26
	
	double lp_dx;	//舵效拉偏 ok
	double lp_dy;
	double lp_dz;///29

	double Lp_trust_det;//推力拉偏
	double Lp_eng_flowvol;//耗油率或流量拉偏
	//结构系，经Xj轴周向角，得到偏心系
	//推力偏心，取值范围0~0.01m
	//+推力偏心周向角，Yj向为零，结构系绕Xj轴转为正，取值范围-180~180deg，即后向前看逆时针为正
	double trust_det_pos;
	double trust_det_angle;// 33

	//体轴系，经过Xj轴旋转 周向角gama，再经过Zj' 俯仰角，得到推力线坐标系
	//推力偏斜俯仰角，取值范围0~90.0deg，无负值
	//推力偏斜周向角，Yj向为零，结构系绕Xj轴转为正，取值范围-180~180deg，即后向前看逆时针为正
	double trust_det_alpha;
	double trust_det_gama;// 35 

	//助推器推力偏斜俯仰角、周向角
	//说明：质心配置方法决定不存在推力偏心，只有推力偏斜角、周向角
	double booster_det_alpha;//推力偏斜俯仰角
	double booster_det_gama;// 37 推力偏斜周向角

	//大气参数拉偏
	double lp_air_density;//空气密度拉偏
	double lp_air_pressure;//大气压力拉偏
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
	bool flag_combat_status;//收到飞控，锁定允许指令后，导引头进入末制导状态，即开始引导搜
	bool flag_seize_stable; //闭锁跟踪，光轴视场内有目标，并匹配识别成功，并稳定跟踪
	
	double pitch_LOS_rate;		//角速率
	double yaw_LOS_rate;	
	double pitch_gimbal_angle;//框架角
	double yaw_gimbal_angle;
	double pitch_LOS_angle;	//失准角
	double yaw_LOS_angle;

	//雷达或激光导引头有效，
	//光电导引头无效，需要根据目标尺度 或 装订目标高度，才能估计相对距离或目标位置
	double distance_target;	//弹目距离
	double longitude_target;	//目标位置 经度、纬度、距离，可根据导弹当前位置、弹目距离、视场角计算
	double latitude_target;
	double hight_target;
}Stru_Data_Seeker_To_Controller;	//导引头给综控机数据包

typedef struct _Stru_Data_INS_To_Controller  
{
	double gama;//正欧拉角：北天东地理系，经过231转序，前上右弹体系
	double psi;
	double zeta;
	double gamas;//反欧拉角：321转序
	double psis;
	double zetas;
	double wx;	//deg /s
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double au;	//未用到
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
	int    route_mode [MAX_ROUTE_NUMBER];
	int    formation_mode[MAX_ROUTE_NUMBER];	
	double dltTime[MAX_ROUTE_NUMBER];
	double velocity   [MAX_ROUTE_NUMBER];
	double turn_angle [MAX_ROUTE_NUMBER];
	double turn_radius[MAX_ROUTE_NUMBER];
	double accept_radius[MAX_ROUTE_NUMBER];
}Stru_Data_Datalink_To_ControllerSig;	//数据链给综控机数据包，无集群

typedef struct _Stru_Data_Engine_To_Controller
{
	double rpm_engine;		//状态转速
	int ECU_work_status;	//工作状态 0x55启动过程中（自检正常），0xAA启动完成并速度控制，0x88进入关机过程，0x99关机完成，0xFF启动或转速控制异常
	//int engine_start_result;//启动状态，未使用
}Stru_Data_Engine_To_Controller;	//发动机给综控机数据包

typedef struct _Stru_Data_Controller_To_Seeker  
{
	bool flag_seeker_on;		//导引头开机
	bool flag_lock_on_permit;//目标锁定允许或引导搜指令，飞控发送
	bool flag_target_lock;	//闭锁指令
	//情况一，飞控根据导引头搜索到的目标信息，指定目标编号或像素中心点，导引头依此捕获、跟踪目标；
	//情况二，如果装订了目标模板或视场只有一个目标，导引头自动匹配识别目标，并捕获、跟踪目标；过程中，可进行目标切换，飞控发送闭锁指令后，不再切换；
	//当飞控判断，导弹距目标一定距离后未捕获，虚拟导引打击；

	//目标锁定允许或引导搜指令，附加信息
	double pitch_gimbal_angle_calc;	//俯仰框架角指令
	double yaw_gimbal_angle_calc;		//航向框架角指令
	//闭锁指令，附加信息
	//采用情况二，只有指令，无附加信息；
	int target_num__choosen;

	//目标位置解算，相关导航信息
	double wz;
	double wy;
	double gama;
	double zeta;
	double psit;
	double longitude;
	double latitude;
	double hight;
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
	double rudderRollCmd;	//通道舵副翼 pd+adrc
	double rudderPitchCmd;	//通道舵升降 qk + pd 或 qk + ADRC
	double rudderYawCmd;	//通道舵航向 pd 转 三回路过载控制
	//遥测信息
	double gamaCmd;	//滚转角指令
	double nycCmd;	//升力面过载指令
	double varthetaCmd;//俯仰角指令
	double heightCmd;//高度指令
	double vyCmd;//垂速指令 新增，20260714 取值范围-90~90，0.003
	double ac_dL;//待飞距
	double ac_dZ;// 侧边距
	unsigned int flight_control_state;//飞行状态 新增，20260714 无符号32bits整型 当量1
	//int token_long;// 纵向令牌，删除
	//int token_late;// 侧向令牌，删除
	double thrustCmd;// 推力指令，例如油门开度Kc 无符号
	int rpmState;//发动机状态转速 新增，20260714 无符号整型 0~10000rpm，当量2
	//double ac_dPsi;//航向角偏差，删除20260714
	double ac_Vz;//侧向速度，新增，20260714 取值范围-90~90，0.003
	double ac_Vy;//天向速度，新增，20260714 取值范围-90~90，0.003
	double ac_Vx;//射向速度，新增，20260714 取值范围-90~90，0.003
	double ac_dR;// 圆轨迹侧边距，未去掉转弯半径
	double cur_azimuth;//航段方位角 新增，20260714 取值范围-180~180，0.006
	double cur_thetav;//轨迹倾角  取值范围-90~90，0.003
	double cur_psicv;//轨迹偏角  新增，20260714 取值范围-180~180，0.006
	double Vcmd;//速度指令 
	double nyCmd_Guidance;//末制导纵向过载指令
	double nzCmd_Guidance;//末制导侧向过载指令
	double wyCmd;//航向角速度指令 新增，20260714
	double pitch_rate_nT_filterOut;//俯仰视线角速度滤波
	double yaw_rate_nT_filterOut;//偏航视线角速度滤波
	double deltaR;//弹目距离，打击点
	double dRn; //弹目北向距离，仅遥测分析
	double dRu; //弹目天向距离，仅遥测分析
	double dRe; //弹目东向距离，仅遥测分析
	double Pitch_Preset_Angle;//理论俯仰框架角
	double Yaw_Preset_Angle;//理论偏航框架角
	int Dubins_stage;//杜宾斯段，待定
	int dubins_type1;//杜宾斯类型，待定
	int dubins_type2;//杜宾斯类型，待定
	int dubins_type3;//杜宾斯类型，待定
	double Dubins_length;//杜宾斯段航程，待定
	//double test1;//测试，删除
	//double Min_IAS2Vel;//最低折算速度，删除20260714
	double gamac_compensate;//滚转角指令补偿量，新增20260714 取值范围-90~90，0.003
	double uz_gamac;//侧偏控制量，新增20260714 取值范围-90~90，0.003
	double mx_ESO;	//干扰估计状态量z2
	double fduox_ADRC;//ADRC舵偏
	double Qv;//动压
	double alpha_ins;	//地速攻角，未用到
	double beta_ins;	//地速侧滑角，未用到
	//double MaxRpm; 		//最大转速，删除20260714
	//double DFT_freq_max;//辨识运动频率，删除20260714
	double nyflt;	//体轴法向过载，新增20260714  取值范围-5~5, 0.001
	double nzflt;	//体轴侧向过载，新增20260714 取值范围-5~5, 0.001
	int count_altitude_change;//高度机动次数，新增20260714 取值范围0~255
	double mass_calc;//质量估计，新增20260714 取值范围0~250kg 0.01
	double ugf_zetac;//高度控制量，新增20260714 取值范围-15~15，0.001
	double uqkf;//前馈控制量，新增20260714 取值范围-15~15，0.001
	double flight_time;//飞控时间，新增20260714 取值范围0~99999.9s，0.001 占用四字节

	double rudder_I_cmd;	//物理舵，转换为电机角度
	double rudder_II_cmd;
	double rudder_III_cmd;
	double rudder_IV_cmd;
	double rudder_V_cmd;
	double rudder_VI_cmd;

	double Kc_cmd;	//油门，待删除

	double curLon;//当前经度、纬度、高度
	double curLat;
	double curAlt;
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
	double control_Kc;	//指令油门，二选一，有用 
	double control_rpm;//指令转速，二选一，未使用
	int ECU_work_cmd;	//控制指令0x00 无指令，0x11 自检，0x22 启动，0x33 转速控制,  0x44 关机
}Stru_Data_Controller_To_Engine;	//综控机给发动机数据包

typedef struct _Stru_Data_Controller_To_Switch_Output  
{
	bool flag_separate_booster;		//助推器分离(爆炸螺栓或切割锁点火)
	bool flag_launch_missile_wing;	//弹翼展开(爆炸螺栓点火)，未用到
	bool flag_engine_start;			//发动机怠速转70%油门，主发动机点火(空中炮起点火)
	
	bool flag_missle_takeoff;			//起飞 20260415
	bool flag_engine_shutdown;		//发动机关机 20260415
	bool flag_open_umbrella;			//开伞	20260415
	bool flag_fuze_unlock;		//引信解锁 20260425
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
	double radioalt_height;//真高，相对地面高度
}Stru_RadioAlt_Data_In;	//环境给无线电高度表输入数据

typedef struct _Stru_Baro_Data_In
{
	double static_pressure;
	double total_pressure;
}Stru_Baro_Data_In;	//环境给空速管输入数据

typedef struct _Stru_Engine_Data_In
{
	double air_density;//空气密度
	double air_speed;	 //空速
	double angle_of_attack;	//攻角，未用到
	double angle_of_side_slip;//侧滑角，未用到					
}Stru_Engine_Data_In;	//环境给发动机输入数据

typedef struct _Stru_Seeker_Data_In
{
	int target_ID;		//当ID为-1时，表示无效目标
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
	//给发动机
	Stru_Engine_Data_In st_data_environment_to_engine;
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
}Stru_Data_Earth_Model_Out;//地球模型输出数据

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
	double gravity;		//重力
	double thrust;		//合推力
	//double booster;
	double thrust_force[3];//发动机推力
	double thrust_moment[3];
	double booster_force[3];//助推器推力
	double booster_moment[3];	
	Stru_Data_Aerodynamic_Force st_aerodynamic_force;//气动力及力矩
}Stru_Data_Missile_Force;	//导弹力及力矩模型输出

typedef struct _Stru_Data_Function_To_Force_Calc
{
	double mach;
	double air_speed;
	double sonic_speed;//声速
	double air_density;//空气密度
	double air_temperature;//空气温度
	double height;
	double wx;
	double wy;
	double wz;
	double angle_of_attack;
	double angle_of_side_slip;
	double gravitational_acceleration;
	double aby;	//体轴系，法向加速度，可用于静气弹变形，即考虑弹性的气动力计算
}Stru_Data_Function_To_Force_Calc;	//导弹力及力矩模型输入

typedef struct _Stru_Data_Function_Solved_Out
{
	double mach;				//未用到
	double air_speed;			//未用到
	double angle_of_attack;	//未用到
	double angle_of_side_slip;//未用到
	
	bool flag_leaving_launcher;	//离架标识
	Stru_INS_Data_In * p_st_data_ins_related;
	Stru_Baro_Data_In * p_st_data_baro_related;
	Stru_Engine_Data_In * p_st_data_engine_related;
	Stru_RadioAlt_Data_In * p_st_data_radioalt_related;
	Stru_Data_Function_To_Force_Calc * p_st_data_function_to_force_calc;
}Stru_Data_Function_Solved_Out;	//方程解算输出

//仿真输入，即飞控输出
typedef struct _Stru_HIL_Data_INPUT
{
	//舵控指令
    double rudderPitchLeft;//I 
    double rudderPitchRight;//II
    double rudderRollLeft;//III
    double rudderRollRight;//IV
	double rudderYawLeft;//V
    double rudderYawRight;//VI

	//发动机指令
    double Kc;	//油门
	//double engine_cmd;//发动机控制指令

	//导引头数据
	bool flag_seeker_on;		//导引头开机指令
	bool flag_lock_on_permit;//目标锁定允许指令
	bool flag_target_choosen;//指定目标指令（目标选择结果），视情使用；    20270706新增
	//导引头控制指令，目标锁定允许、引导搜(可与锁定允许合并)、匹配转识别(可自动执行，视情发指令)、闭锁跟踪(可自动执行，视情发指令)
	//未捕获到捕获过程：导引头已开机，飞控向导引头发送目标锁定允许（与引导搜合并）指令、俯仰/航向框架角，
	//    导引头依据框架角引导搜索，视场内目标匹配、识别并跟踪目标，跟踪目标后向飞控发送“闭锁跟踪”状态，并忽略飞控引导搜指令；飞控收到后，依据导引头跟踪信息末制导飞行；
	//捕获到丢失目标，再重捕获：飞控依据导引头“闭锁跟踪”框架角、视线角等信息，解算目标位置、距离；丢失目标后，根据最后“闭锁/跟踪”时刻计算目标位置，虚拟导引，直至重新捕获或最近打击点；
	double pitch_gimbal_angle_calc;	//俯仰框架角指令
	double yaw_gimbal_angle_calc;		//航向框架角指令
	double target_num__choosen;//目标选择结果，典型值0，暂不使用  20270706新增

	//开关量补偿，从飞控输出
	bool flag_launch_missile_wing;	//弹翼展开，未用到(一般力及力矩模型处理) 20270706新增
	bool flag_separate_booster;		//助推器分离，力及力矩模型需要（分离后质量减轻），半实物仿真增加 20270706新增
	bool flag_engine_start;			//发动机开机，发动机油门切换，无火工品，未用到 20270706新增
	//开关量指令
	bool flag_missile_takeoff;	//起飞
	bool flag_engine_shutdown;	//发动机关机
	bool flag_open_umbrella;		//开伞回收
	bool flag_fuze_unlock;		//引信解锁
}Stru_HIL_Data_INPUT;

//仿真输出，即飞控输入
typedef struct _Stru_HIL_Data_OUTPUT
{
    double wx;	//组合导航，体轴系
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    double pitch;//地理系姿态角，231转序正欧拉
    double yaw;
    double roll;
    double airSpd;//空速，未用到
    double lon;
    double lat;
    double alt;
    double vn;	//北速
    double vs;	//天速
    double ve;	//东速
    
    double DD1;//空速管静压，用于解算空速、气压高度等
    double DD2;//空速管总压
    double DD3;//未使用
    double DD4;//未使用
    double mass;//未使用
    double xg;//未使用
    double arp;//未使用
    
    double rpm_state;//发动机(状态转速)
    double MX_T_Disturb;//滚转干扰力矩估计，未用到

	//导引头输出数据，后续使用
	double qf;//俯仰实现角
    double qh;//航向视线角
    double dqf;//俯仰视线角速度
    double dqh;//航向视线角速度
    //double phif;//俯仰框架角，新增，暂时不用
    //double phih;//航向框架角，新增，暂时不用
    //double distance_target;//目标距离，新增，暂时不用；光电导引头不输出弹目距离
    int TargetLocked;//导引头锁定标识，默认值0x00为未锁定，0x01为匹配，0x02为识别，0x03为闭锁跟踪，0x04为记忆跟踪
	//待扩展为16个目标特性数据，便于目标选择算法验证；
	//double sigmaf;//俯仰失调角 + 俯仰角 + 俯仰框架角 =  视线角，导引头轴（光轴或雷达轴）与视线之间夹角
    //double sigmah;//航向失调角，新增，暂时不用
    
	//其他
	int MissileLauched;//起飞标识，1为起飞，0为未起飞
	int NavigationMode;//导航参试模式，0为组合导航不参试/仿真全部模拟，1为惯导上状态叠加位置运动，2为角速度/加速度注入叠加噪声 惯导解算
}Stru_HIL_Data_OUTPUT;

#endif
