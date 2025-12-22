#ifndef _OS_FLIGHT_IO_H_
#define _OS_FLIGHT_IO_H_
#include <string.h>
//#define  version '1017

typedef unsigned char		OS_U8;
typedef signed char			OS_S8;
typedef unsigned short		OS_U16;
typedef signed short		OS_S16;
typedef unsigned int		OS_U32;
typedef signed int			OS_S32;
typedef unsigned long long 	OS_U64;
typedef signed long long 	OS_S64;
typedef float				OS_FLOAT;
typedef double				OS_DOUBLE;
typedef void				OS_VOID;
typedef OS_U8				OS_MEM;
typedef unsigned char		OS_BOOL;
#pragma pack(1)
/*********************************************************
 * 控制输入结构体
 * *******************************************************/

typedef struct
{
	OS_DOUBLE initHigh;//起飞点高度
	OS_DOUBLE initDir;//起飞航向角（北偏东0-360）
	OS_DOUBLE initLon;//起飞点经度
	OS_DOUBLE initLat;//起飞点纬度

	OS_DOUBLE wx;	//x轴角速度	deg/s
	OS_DOUBLE wy;	//y轴角速度	deg/s
	OS_DOUBLE wz;	//z轴角速度	deg/s

	OS_DOUBLE ax; 	//x轴加速度	m/s^2
	OS_DOUBLE ay;	//y轴加速度	m/s^2
	OS_DOUBLE az;	//z轴加速度	m/s^2

	OS_DOUBLE airSpd;	//空速  m/s

	OS_DOUBLE navLon;	//组合导航经度  deg
	OS_DOUBLE navLat;	//组合导航纬度  deg
	OS_DOUBLE navHigh;	//组合导航高度 m

	OS_DOUBLE navVn;	    //组合导航北速
	OS_DOUBLE navVs;	    //组合导航天速
	OS_DOUBLE navVe;	    //组合导航东速

	OS_DOUBLE pitch;		//俯仰角 (-90~+90)	低头负抬头正
	OS_DOUBLE yaw;			//偏航角 (-180~+180)左偏正右偏负
	OS_DOUBLE roll;			//滚转角 (-180~+180)左滚负右滚正

	OS_U8 Luanched;		//起飞标志 0未起飞，1已起飞

	OS_DOUBLE DD1;
	OS_DOUBLE DD2;      //副翼舵偏
	OS_DOUBLE DD5;
	OS_DOUBLE DD6;      //偏航舵偏

	OS_DOUBLE mass_fuel;  //耗油量
	
	OS_DOUBLE scoutPitch;	//导引头框架角俯仰
	OS_DOUBLE scoutYaw;		//导引头框架角偏航
	OS_DOUBLE scoutPitchSpd;//导航系视线角速度俯仰
	OS_DOUBLE scoutYawSpd;	//导航系视线角速度偏航
	OS_U8 scoutLocked;		//导引头锁定状态 1锁定 0不锁定

	OS_U8 Msn_updatesig; //航点更新标志
	
}FLIGHT_INPUT;

/*********************************************************
 * 控制输出
 *
 * *******************************************************/

 //  + 10 double
#pragma pack(1)
typedef struct
{	
	//通道舵
	OS_DOUBLE rudderRollCmd;	//通道舵副翼
	OS_DOUBLE rudderPitchCmd;	//通道舵升降
	OS_DOUBLE rudderYawCmd;	//通道舵航向
	
	//物理舵
	OS_DOUBLE rudder1Cmd;   // 左副翼舵	//MML舵机 通过CAN控制
	OS_DOUBLE rudder2Cmd;   // 右副翼舵	//通过CAN控制
	OS_DOUBLE rudder3Cmd;   // 左俯仰舵	//通过CAN控制
	OS_DOUBLE rudder4Cmd;   // 右俯仰舵	//通过CAN控制
	OS_DOUBLE rudder5Cmd;   // 左航向舵	//通过PWM控制
	OS_DOUBLE rudder6Cmd;   // 右航向舵	//通过PWM控制

	//航点信息
	OS_U16 curPtNo;			//当前航点号
	OS_DOUBLE curTargetLon;	//当前目标航点经度
	OS_DOUBLE curTargetLat;	//当前目标航点纬度
	OS_DOUBLE curTargetAlt;	//当前目标航点高度
	
	//发动机推力设定
	OS_DOUBLE engineSet;	//此处可指定为转速或推力
	
	//其它需要代传遥测
	//请在此添加
	OS_DOUBLE gamaCmd;//滚转角指令
	OS_DOUBLE nycCmd;//过载指令
	OS_DOUBLE varthetaCmd;//俯仰角指令
	OS_DOUBLE heightCmd;//高度指令
	OS_DOUBLE ac_dL;//待飞距
	OS_DOUBLE ac_dZ;// 侧边距
	OS_U8 token_long;// 纵向令牌
	OS_U8 token_late;// 侧向令牌
	OS_U8 on_takeoff;//起飞完成标志
	OS_U8 open_umbrella;//开伞标志
	OS_U8 enginge_off;//动力停车标志
	OS_DOUBLE thrustCmd;// 推力指令
	OS_DOUBLE  ac_dPsi;//航向角偏差
	OS_DOUBLE ac_dR;// 圆轨迹侧边距
	OS_DOUBLE cur_thetav;//轨迹倾角
	OS_DOUBLE Vcmd;//速度指令 
	OS_DOUBLE nyCmd_Guidance;//末制导纵向过载指令
	OS_DOUBLE nzCmd_Guidance;//末制导侧向过载指令
	OS_DOUBLE pitch_rate_nT_filterOut;//俯仰视线角速度滤波
	OS_DOUBLE yaw_rate_nT_filterOut;//偏航视线角速度滤波
	OS_DOUBLE deltaR;//弹目距离
	OS_FLOAT dRn; //弹目北向距离
	OS_FLOAT dRu; //弹目天向距离
	OS_FLOAT dRe; //弹目东向距离
	OS_FLOAT Pitch_Preset_Angle;//理论俯仰框架角
	OS_FLOAT Yaw_Preset_Angle;//理论偏航框架角
	OS_U8 Dubins_stage;//杜宾斯段
	OS_S8 dubins_type1;//杜宾斯类型
	OS_S8 dubins_type2;//杜宾斯类型
	OS_S8 dubins_type3;//杜宾斯类型
	//OS_DOUBLE lon_D1; 
	//OS_DOUBLE lat_D1;//切点1坐标
	//OS_DOUBLE lon_D2;
	//OS_DOUBLE lat_D2;//切点2坐标
	//OS_DOUBLE lon_C1;
	//OS_DOUBLE lat_C1;//圆心1坐标
	//OS_DOUBLE lon_C2;
	//OS_DOUBLE lat_C2;//圆心2坐标
	//OS_DOUBLE lon_C3;
	//OS_DOUBLE lat_C3;//圆心3坐标
	OS_DOUBLE Dubins_length;//杜宾斯段航程
	OS_DOUBLE test1;//测试
	OS_DOUBLE Min_IAS2Vel;//最低折算速度
	OS_DOUBLE mx_ESO;
	OS_DOUBLE fduox_ADRC;//ADRC舵偏
	OS_FLOAT Qv;//动压
	OS_FLOAT arp_ins;//惯测攻角
	OS_FLOAT beta_ins;//惯测侧滑角
	OS_FLOAT MaxRpm; //最大转速
	OS_FLOAT DFT_freq_max;//辨识运动频率
	//代传遥测完
}FLIGHT_OUTPUT;
#pragma pack()
//飞控上电后调用
//加载配置文件，当前设计仅支持一个文件加载，可要求增加更多配置文件
//extern void FlightInit();

//每5ms调用一次
//extern void FlightRun(FLIGHT_INPUT* input, FLIGHT_OUTPUT* output);


extern FLIGHT_INPUT *pInput;
extern FLIGHT_OUTPUT *pOutput;
#pragma pack(1)
typedef struct MISSION
{
	OS_U8 MsnCmdType;  //0:不变  1:航点飞行 2:盘旋 3:攻击,   4:退出攻击
	OS_DOUBLE targetLon; //目标经度  deg
	OS_DOUBLE targetLat; //目标纬度   deg
	OS_DOUBLE targetHigh; //目标高度  m

	OS_DOUBLE speed;  //飞行速度    m/s
	OS_U8 speedType;  //速度类型 0:空速 1:地速

	OS_DOUBLE radis;  //飞行半径   m

	OS_DOUBLE inTrack;  //入弯角度  0-360 deg
	OS_DOUBLE outTrack;  //出弯角度   0-360 deg

	OS_U32 arriveTime;  //到达时间

} MISSION;

typedef struct {
	int sn; 			//点号 0-固定为发射点 其他-为规划航路点
	double lon;
	double lat;
	int h; //高度m 
	int w; //航点类型
	int t; //到达时间 秒
	float V_cmd; //飞行马赫数指令
	float outTrack;//航向
//	float radis;		//盘旋半径   
//	float hit_angle;//打击角度
	unsigned char if_airspeed_used; //是否启用空速控制
	unsigned char if_GuideFlight; //是否指点
}RoutePointIn;
#pragma pack()

typedef struct {
	double lon_C1, lon_C2, lon_C3;
	double lat_C1, lat_C2, lat_C3;
	double lon_D1, lon_D2;
	double lat_D1, lat_D2;
	double lon_preset, lat_preset;

	float psi_MT; //弹目连线角度
	float psi_CC; //圆心连线角度
	float psi_CM; //圆心1与进入点连线角度
	float psi_CT; //圆心2与切出点连线角度
	float psi_DR1; //圆心1与切点1连线角度
	float psi_DR2; //圆心2与切点2连线角度
	float psi_D1D2; //切点连线角度
	float outTrack;//切出点角度

	float psi_C1MD, psi_C2MD, psi_C3MD; //圆弧角度差
	float Circ1_psi, Circ2_psi, Circ3_psi;//圆心与航线交线顶点夹角

	float len;    //圆心连线长度
	float dubins_len1; //第一段长
	float dubins_len2; //第二段长
	float dubins_len3; //第三段长
	float dubins_len;   //总轨迹长度
	char dubins_type[3]; //类型
}DubinsStrucIn;

extern void FlightRun();
extern void FlightInit(void);
extern void updateRP(RoutePointIn* inrp, int num);
/*
__declspec(dllexport) extern void UpdateMission(MISSION msn);
__declspec(dllexport) extern void FlightInit(void);
__declspec(dllexport) extern void FlightRun();
__declspec(dllexport) void setFlightInput(FLIGHT_INPUT input1); 
__declspec(dllexport) void getFlightOutput(FLIGHT_OUTPUT* output1); 
__declspec(dllexport) void updateRP(RoutePointIn* inrp, int num);
__declspec(dllexport) void getDubinsParam(DubinsStrucIn* paramss);
__declspec(dllexport) void updateNewRP(RoutePointIn* newinrp, int num, OS_U16 curPtNo, int pt);
*/
#endif



//调用顺序
//1.启动时加载配置文件
//2.配置文件加载完调用初始化函数
//3.初始化完成后，每5ms调用执行函数一次，调用前将pInput赋值，调用后取pOutput值进行控制
