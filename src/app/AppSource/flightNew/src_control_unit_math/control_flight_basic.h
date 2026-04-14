#ifndef _CONTROL_FLIGHT_BASIC_H_
#define _CONTROL_FLIGHT_BASIC_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 飞控预处理模块，包括关键导航参数解算，指令流程解算等
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
#include "control_roll.h"
#include "control_engine.h"
#include "../data_protocol.h"

typedef struct  _Stru_Way_Point
{
	double longitude;
	double latitude;
	double height;
	double turn_radius;
	double turn_angle;
	double velocity;
	int    route_mode;//航点类型
	int    formation_mode;	//待扩充???...
}Stru_Way_Point;

typedef struct  _Stru_Command
{
	bool flag_separate_booster;	 //助推器分离
	bool flag_launch_missile_wing;//弹翼展开
	bool flag_engine_start;		//发动机点火
	bool flag_seeker_on;			//导引头开机
	bool flag_lock_on_permit;		//锁定允许标识
	bool flag_combat_status;		//锁定目标，进入战斗/末制导状态，给引信发送“延迟装订、保险解除指令”
}Stru_Command;

typedef struct  _Stru_Control_Time
{
	double time_control;			//启控时间
	double time_separate_booster;//助推器分离时间
	double time_launch_missile_wing;//弹翼展开时间
	double time_seeker_on;		//导引头开机时间
	double time_engine_start;	//发动机开机时间
	double time_engine_start_finish;//发动机开机完成时间
	double time_altitude_control;//开始高度控制
	double time_cooperative_attack;//协同(搜索)攻击
	double time_combat_status;	//战斗指令: “导引头锁定目标”或“固定目标点距离小于阈值”
	
	double time_altitude_change_start;//高度机动开始时间
	double time_altitude_change_end;	//高度机动结束时间
	double time_turn_in_start;	//入转弯开始时间: 扇面、航迹转弯共用
	double time_turn_in_end;		//入转弯结束时间: 预计滚转过渡完成时间
	double time_turn_out_start;	//出转弯开始时间
	double time_turn_out_end;		//出转弯结束时间
	double time_turn_in_minimum;
	double time_arrive_minimum;
}Stru_Control_Time;

typedef struct  _Stru_Control_flag
{
	bool flag_initial_hz;			//组合高度初始化
	
	bool flag_control_set;
	bool flag_control_on;			//启控（或离架），在set(满足条件)后，延迟短时间开始执行
	bool flag_separate_booster_set;
	bool flag_separate_booster;	//助推器分离
	bool flag_launch_missile_wing_set;
	bool flag_launch_missile_wing;//弹翼展开(完成)，删除
	bool flag_engine_start_set;
	bool flag_engine_start;		//发动机首次怠速转大车，发出发动机启动/加速指令及满油门
	
	bool flag_engine_start_finish_set;//初始爬升段发动机大车，即最大能力爬升结束，转发动机巡航控制指令及油门
	//bool flag_engine_start_finish;//同时发动机开机，不延迟
	
	bool flag_seeker_on_set;		
	bool flag_seeker_on;			//导引头开机，光电导引头一般可长时间工作，助推器分离一段使劲按后即可开机
	
	bool flag_launch_turn_set;	
	bool flag_launch_turn;		//扇面转弯过程中，结束后无效	
	bool flag_altitude_control_set;
	bool flag_altitude_control;	//首次接入高度控制
	
	bool flag_lock_on_permit;		//导引头锁定允许: 切换最后一个航点(视情优化为，弹目距离小于阈值时，锁定允许)
	bool flag_cooperative_attack_set;
	bool flag_cooperative_attack;//协同(框架角引导搜索)打击，距离最后一个目标点距离小于10km;
	bool flag_combat_status_set;
	bool flag_combat_status;		//战斗指令，进入打击流程(末制导)

	bool flag_alltitude_change;//高度机动过程中标识
	bool flag_alltitude_climb;//高度机动: 爬升
	bool flag_alltitude_decline;//高度机动:下滑
	
	bool flag_waypoint_turn;	//航迹转弯过程中标识
	bool flag_turn_out_set;	//出转弯过程中标识，航迹转弯和扇面转弯共用
}Stru_Control_flag;

typedef struct  _Stru_Flight_Basic_Input
{
	int missile_ID;
	int engine_start_result;
	double engine_rpm;//从发动机接收的状态转速
	double engine_cmd_rpm;//发动给发动机的指令转速，二选一
	double engine_cmd_Kc;//发动给发动机的指令油门，二选一
	
	Stru_Data_RadioAlt_To_Controller st_radioalt_data;//无线电高度表给控制???...
	Stru_Data_Baro_To_Controller st_baro_data;//空速管给控制???...
	Stru_Data_Engine_To_Controller st_engine_data;//发动机给控制???...
	Stru_Data_INS_To_Controller	st_ins_data;
	Stru_Data_Seeker_To_Controller	st_seeker_data;
	Stru_Data_Datalink_To_Controller	st_datalink_data;
}Stru_Flight_Basic_Input;

typedef struct  _Stru_Flight_Basic_Output
{
	double mass_calc;	//估计质量
	double hz;
	double h_ini;
	double nby;
	double nbz;
	double v;
	double vs;	//水平速度
	double vnx;
	double vnz;	//侧向速度
	double mach;
	double sz;	//侧偏
	double zeta;
	double gama;
	double wx;
	double wy;
	double wz;
	double g;	//重力常数
	double dqf;
	double dqh;	
	double qf;
	double qh;
	double time_to_go;
	double phif;
	double phih;
	double angle_zw;
	double radius_zw;
	double target_velocity;
	double target_time;
	double target_height;
	double ground_temperature;
	double distance_target;
	double distance_target_t_combat;
	double gama_turn_nominal;
	double velocity_average_10s;
	int num_way_point_target;
	int	count_altitude_change;
	int ECU_work_cmd;		//发动机控制指令，开机、关机等
	bool flag_launch_turn;
	bool flag_altitude_change;
	bool flag_waypoint_turn;
	bool flag_altitude_climb;
	bool flag_altitude_decline;
	bool flag_velocity_control;	//空速类型
	bool flag_fire_distribution;//火力分配
	Stru_Command st_command;
	Stru_Control_Time st_control_time;
}Stru_Flight_Basic_Output;

class CMathControlFlightBasic
{
public:
	CMathControlFlightBasic(); 
	double flight_time;
	int time_tick;
	//Stru_Debug_Monitor					* p_st_debug_monitor;//调试信息
	Stru_Route_Data						* p_st_route_data_preflight;//预装航线信息
	Stru_Initial_Data					* p_st_initial_data;//发射信息
	Stru_Flight_Basic_Input				* p_st_flight_basic_input;
	Stru_Flight_Basic_Output			* p_st_flight_basic_output;
	void Run();
	void Initial();
private:
	void Get_Data();
	void Calc_Data();
	void Send_Data();
	// void Monitor_Data();
	void Calc_Command();	
	void Calc_Flight_Data();
	void Calc_Mass_Data();
	void Calc_LOS_Rate();
	void Calc_Hz();
	void Calc_BaroHigh();
	void Calc_BaroSpd();
	void Control_Turn();
	void Control_Altitude_Change();
	void Coord_Rebuild();
	void Update_Task_Info();
	//void Change_Task_Info_Online();
	bool Judge_Turn_Error();
	
	int m_missile_ID;
	int m_missile_flight_mode;//0x55 测试训练，0xAA 虚拟打击或导引头捕获后打击

	int m_num_way_point;	//总航点数
	int m_num_way_point_target;//目标航点(当前航段)
	int valid_count;	//视线角速度计算，弹目盲区距离计数，小于某值不再更新
	int count_qk;		//离架或启控
	int count_v_5;		//启控备保
	int count_fl;		//助推器分离
	int count_qd;		//发动机启动完成
	int count_v_50;	//助推器分离备保
	int count_tg;		//开始高度控制
	int count_cooperative_attack;//协同搜索，典型弹目距离10km
	int count_virtual;//虚拟打击或导引头攻击目标，典型弹目距离2km
	
	int count_altitude_change;//高度机动次数记录
	int count_altitude_change_enable;
	int count_altitude_change_end;
	int count_away;	//原理目标点计数
	int count_sd_in;	//小于提前转弯距离
	int count_turn_out;//转弯转出，进入直航
	int count_turn_error;//转弯角度过大，异常
	//int count_update;	//数据链或任务机，航点更新
	double m_ground_temperature;
	Stru_Way_Point m_st_way_point[MAX_ROUTE_NUMBER];//航点，发射点不是第0航点；航点号为0，表征第一个(目标)航点
	Stru_Way_Point m_st_target;//目标信息，位置、转弯半径、角度(正航向或转弯角度等)、速度、航点类型及信息类型
	
	Stru_Control_flag m_st_control_flag;//时序及时间
	Stru_Control_Time m_st_control_time;
	
	//惯性卫星组合导航
	double m_au;
	double m_hgps;
	double m_longitude;
	double m_latitude;
	double m_ax;
	double m_ay;
	double m_az;
	double m_vtx;
	double m_vty;//组合导航输出垂速度
	double m_vtz;
	double m_zeta;
	double m_gama;
	double m_psit;//地理系，偏航角
	//处理后数据 
	double m_vs;	//组合垂速
	double m_hz;	//组合高度

	double m_vx;	//轴向速度
	double m_sx;	//轴向位移，用于离架判断
	
	double m_v;		//合速度，地速
	double m_vnx;	//射向速度
	double m_vnz;	//侧向速度
	double m_g;		//根据纬度、海拔高度修正，用于计算过载 	
	double m_ny;	//天向过载
	double m_nz;	//侧向过载

	//空速管: 总压、动压，计算获得空速、高度
	double m_static_pressure; //输入原始数据
	double m_total_pressure;
	//处理后数据
	int m_baroalt_status;

	//无线电高度表数据
	double m_radioalt_hight;
	int m_radioalt_status;
	
	
	//处理后数据
	double m_hbaro;		
	double m_Vbaro;//气压高度表指示空速
	double m_v_air;//空速
	double m_v_average_1s;
	double m_v_average_10s;
	double m_mach;//马赫数，空速，经过高度、温度修正后马赫数修正
	double m_v_record_100ms[10];
	double m_v_record_1s[10];
	
	//发动机状态及转速
	int m_engine_start_result;//发动机开机状态
	double m_engine_state_rpm;//状态转速
	//发动机指令
	int m_ECU_work_cmd;	//0x11待机，0x22启动，0x33 转速控制，0x44 关机
	double m_cmd_Kc;//发动机油门，二选一
	double m_cmd_rpm;//发动机转速，二选一
	//处理后数据
	double m_state_rpm;//发动机状态转速，传感器采集或油门估计	
	double m_fuel_comsumped;//消耗燃油重量、重量估计
	double m_mass_calc;
	double m_left_flight_time_calc;//剩余飞行时间估计
	double m_left_flight_dist_calc;//剩余飞行航程估计

	//导引头数据，根据飞控标识：导引头开机(电锁零位)、允许锁定(解除电锁零位)、协同(框架角引导搜索)打击、战斗指令等标识，外部生成指令
	//导引头指令数据
	//double m_seeker_cmd;//导引头控制指令，0x01 自检(上电自动)，0x02 装订目标模板， 0x03 射前检查，0x04/0x05备用，0x06飞控引导搜索(指定框架角)，0x07锁定允许，0x08闭锁，0x09修正跟踪
	double m_seeker_cmdpara_targettype;//指示目标类型，0x01 车辆，0x02 飞行器，0x03 固定建筑
	double m_seeker_cmdpara_dltheight;//弹目高度差
	double m_seeker_cmdpara_ktheta;//导弹俯仰角
	double m_seeker_cmdpara_psi;//导弹偏航角
	double m_seeker_cmdpara_gama;//导弹滚转角
	double m_seeker_cmdpara_phif;//引导框架角
	double m_seeker_cmdpara_phih;//引导框架角

	//导引头状态数据
	//double m_seeker_state;//带引头工作状态，0xAX 正常，0xFX异常，0x5X 过程中
	double m_seeker_state_track;//导引头跟踪状态，0x01电锁零位,0x02搜索（或失锁），0x03闭锁，0x04跟踪
	double m_seeker_dqf;	//视线角速度
	double m_seeker_dqh;
	double m_seeker_phif;	//框架角（状态）
	double m_seeker_phih;
	double m_seeker_qf;	//视线角（状态）
	double m_seeker_qh;
	int m_seeker_pixelf;//俯仰像素偏差，左负右正
	int m_seeker_pixelh;//航向像素偏差，下负上正
	double m_seeker_distance_target;//导引头输出弹目距离，雷达或激光可直接输出；
										//可将光或红外根据高度差、视线角估算，也可根据目标像素大小和焦距估计目标距离；

	//处理后数据
	double m_dqf;	//视线角速度
	double m_dqh;
	double m_phif;	//框架角（引导搜指令）
	double m_phih;
	double m_Qf;	//视线角
	double m_Qh;
	double m_time_to_go;	//（末制导）到达时间 
	
	//航线信息
	double m_longitude_A;//已过航点，前一航段，目标航点
	double m_latitude_A;
	double m_longitude_B;//当前目标航点
	double m_latitude_B;
	double m_longitude_C;//下一航段，目标航点
	double m_latitude_C;
	double m_distance_BP;	//当前点到目标点距离
	double m_distance_BP_projection;//实时输出，航线方向投影距离
	double m_distance_BP_500;//当前500ms间隔，导弹到目标点间航线投影距离
	double m_distance_BP_500pre;//前一帧500ms间隔，导弹到目标点间航线投影距离
	double m_alpha_target;		//目标点方位
	double m_distance_target;	//目标点距离
	double m_distance_target_t_combat;//进入战斗指令（进入末制导）时，目标距离；
	double m_A;		//航段方位角，北偏西 为正	
	//distance_AB//航段航程，为临时变量
	double m_sz;	//侧向位移，与侧向速度共同，用于侧偏控制

	double m_psit_t_turn_in;//转弯开始时刻，真航向角
	double m_psin;	//导航系偏航角，用于视线角速度计算
	double m_psicn;//地理系，弹道偏角或航迹角，北偏西为正，180~180deg

	double m_turn_angle;	//转弯角度
	double m_turn_radius;	//转弯半径
	double m_target_velocity;	//航段速度
	double m_target_height;	//航段高度
	double m_gama_turn_nominal;//转弯过程中滚动程序角标称值，转弯半径估计侧向过载后获得
	double m_x_coordinate_turn;//转弯中心
	double m_z_coordinate_turn;
	double m_distance_turn_in_compensate;//转弯提前距离，角度过渡补偿
	double m_distance_turn_in;//转弯提前距离
	
	double m_total_distance;//总航程（预处理）,
	double m_alpha_AB;		//航段方位角（预处理），真航迹角，北偏东为正
};


#endif