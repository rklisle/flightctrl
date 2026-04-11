#ifndef __FLIGHTPORT_H__
#define __FLIGHTPORT_H__

#define E_CONST		(1.0/298.257)
#define RE			(6378137.0)

const int MAX_ROUTE_NUMBER = 64;	//数据链给综控机最大航路点个数
const int MAX_CONNECT_NUMBER = 16;	//数据链最大连接导弹数量
const int MAX_TARGET_NUMBER = 16;	//每个弹，最多16个目标
const double MAX_TIME = 9999.0;
const double STEP_5ms = 0.005;

typedef struct _Stru_Initial_Data
{
	int	missile_ID;			//弹编号
	double longitude_launch;	//发射点
	double latitude_launch;	
	double height_launch;
	double launch_time;		//发射时间
	double initial_parameter1;//预留初始参数1，例如发射点温度等，可估算 声速、大气等模型
	double initial_parameter2;//预留初始参数2
}Stru_Initial_Data;	//导弹初始状态装订数据

typedef struct _Stru_Route_Data
{
	int num_rows;
	int num_columns;
	char ** p_str_title; 
	double * p_route_data;		//行为不同航点编号，列为航点特征: 编号、经度、纬度、转弯半径、角度、速度、航点类型、信息、高度
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

#endif /* FLIGHTPORT_H */
