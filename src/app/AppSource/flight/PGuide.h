#ifndef PGUIDE_H
#define PGUIDE_H

#include "Ctrl_Law_Typedef.h"
#include "os_flight_io.h"

extern PGuidPara pGuide_para;

void PGuide_init(FLIGHT_INPUT *Flight_Input);
void SetPGuide(FLIGHT_INPUT *Flight_Input);
void PGuide();

/*[导航方式定义]*/
#define PW_GuideNone       0                      /*[指令遥控]*/
#define PW_GuideDim1       1                      /*[一维导航][定向]*/
#define PW_GuideDim2       2                      /*[二维导航]*/
#define PW_GuideDim3       3                      /*[三维导航]*/
#define PW_GuideDim4       4                      /*[末制导]*/
#define PW_GuideDim5       5                      /*[杜宾斯导航]*/
/*[自主飞行模态定义]*/
#define PW_NavNone         0                      /*[人工引导模态]*/
#define PW_NavEnter        1                      /*[点号切入模态进入]*/
#define PW_NavWayEnter     2					  /*[航线飞行模态进入]*/
#define PW_NavWay          3                      /*[航线飞行模态]*/
#define PW_NavHover        4                      /*[盘旋模态]*/
#define PW_NavHome         5                      /*[回家模态]*/
#define PW_NavLand         6                      /*[着陆模态]*/
#define PW_NavReFly        7                      /*[复飞模态]*/
#define PW_NavTakeOff      8                      /*[起飞模态]*/
#define PW_NavEnterONE     9                      /*[在线点号切入模态]*/
#define PW_NavWayONE       10                      /*[在线航线飞行模态]*/

/*[导航信息源]*/
#define PW_NavGPS          0                      /*[GPS]*/
#define PW_NavDGPS         1                      /*[DGPS]*/
#define PW_NavGuess        2                      /*[推测]*/
/*航线信息*/
#define PW_LineLeft        0                      /*[向左]*/
#define PW_LineRight       1                      /*[向右]*/
/*侧向模态*/
#define PW_LateNone        0                      /*[无定义]*/
#define PW_LateStraight    1                      /*[直飞模态]*/
#define PW_LateLeft        2                      /*[向左模态]*/
#define PW_LateRight       3                      /*[向右模态]*/

#define TOKEN_LateNone     0                      /*[空令牌]*/
#define TOKEN_TurnLeft     1                      /*[指令向左令牌]*/
#define TOKEN_Straight     2                      /*[指令直飞令牌]*/
#define TOKEN_TurnRight    3                      /*[指令向右令牌]*/
#define TOKEN_TrackPsi     4                      /*[航向跟踪令牌]*/
#define TOKEN_TrackWay     5                      /*[航迹跟踪令牌]*/
#define TOKEN_TrackCirc    6                      /*[圆轨迹跟踪令牌]*/
#define TOKEN_LateGuidance     7                  /*[末制导令牌]*/
#define TOKEN_TrackDubins    8                    /*[杜宾斯跟踪令牌]*/
#define TOKEN_TrackHover     9                    /*[盘旋跟踪令牌]*/
/*纵向模态*/
#define PW_LongNone        0                      /*[无定义]*/
#define PW_LongLevel       1                      /*[平飞模态]*/
#define PW_LongClimb       2                      /*[爬升模态]*/
#define PW_LongDive        3                      /*[下滑模态]*/
#define PW_Guidance        4                      /*[制导模态]*/

#define TOKEN_LongNone     0                      /*[空令牌]*/
#define TOKEN_Takeoff      1                      /*[起飞令牌]*/
#define TOKEN_CmdClimb     2                      /*[指令爬升令牌]*/
#define TOKEN_CmdLevel     3                      /*[指令平飞令牌]*/
#define TOKEN_CmdDive      4                      /*[指令下滑令牌]*/
#define TOKEN_WayClimb     5                      /*[自主爬升令牌]*/
#define TOKEN_WayLevel     6                      /*[自主平飞令牌]*/
#define TOKEN_WayDive      7                      /*[自主下滑令牌]*/
#define TOKEN_WayLand      8                      /*[自主着陆令牌]*/
#define TOKEN_TakeoffPre   9                      /*[建立起飞角]*/
#define TOKEN_WayPass     10                      /*[低空通场令牌]*/
#define TOKEN_WayHome     11                      /*[回家令牌]*/
#define TOKEN_LongGuidance     12                 /*[末制导令牌]*/
/*纵向安全阈值*/
#define PW_ViClimb            5                      /*[最低爬升速度]*/  
#define PW_HeightThreshold    10000//7500                   /*[升限高度门限]*/  
#define PW_HeightDecline      500                    /*[高度下降值]*/  
#define PW_StepThreshold      300                    /*[计数门限:300*0.01=3s]*/  
/*发动机停车安全距离*/
#define PW_EngOffLen          600
/*转弯安全距离*/
#define PW_dZThreshold        50
/*转弯安全角度*/
#define PW_dPsiThreshold       5
/*着陆段滑翔比*/
#define PW_Gilde_ratio         17
/*着陆段判断高度*/
#define PW_HeightLand          20
/*佯攻点拉起判断高度*/
#define PW_HeightFeint         0
/*佯攻点拉起高度差值*/
#define PW_dHeightFeint         200
/*理论视线角速度计算弹目距离限幅*/
#define PW_delatRlimit         100
/*开伞判断待飞距*/
#define PW_dL_Land         1100   //11s减速*100m/s速度
/*开伞判断控制半径*/
#define PW_dL_Land_OpenUm  200  //考虑开伞过程需要时间，适当放大
/*滚转角指令限幅*/
#define PW_gama_cmd_limit      45
/*末制导BTT-90滚转角指令限幅*/
#define PW_Guidance_gama_cmd_limit      25
/*末制导小纵向过载时的滚转指令限幅*/
#define PW_Guidance_gama_cmd_min_limit      5
/*飞行最低表速*/
#define PW_Min_IAS 105
/*开伞最低表速*/
#define PW_OpenUm_IAS 90
/*飞行最大速度*/
#define PW_VcmdMax 260
/*俯冲拉起稳定爬升率阈值*/
#define PW_Dive_Pull_VU 10
/*100%大车累计时间*/
#define PW_MaxRpm0Time 60 
/*95%大车累计时间*/
#define PW_MaxRpm1Time 600
/*杜宾斯RSR/LSL劣弧飞行补偿角度*/
#define PW_Dubins_AngleSet 0.5
/*杜宾斯RSR/LSL劣弧飞行角度差阈值*/
#define PW_Dubins_deltAngleThread 5
// 航向提前修正时间
#define PW_GLIDE_TIME          10  // 60   20231120改为50
#define Takeoff_Time   10 //起飞段结束判断时间
#define Takeoff_Protect_Time   30 //起飞段备保时间
/*高度控制切换高度阈值*/
#define Height_Climb_Threshold    100
#define Height_Glide_Threshold    200
/*低空高度阈值*/
#define Low_Attitude_Threshold 200

#define nc_zl_min 0.01  //过载指令小界值 
/*--BTT/STT切换偏航框架角约束--*/
#define Yaw_Angle_Constraint 30
/*--BTT/STT切换侧向过载--*/
#define nz_Constraint 0.3
/*稳定边界连续时间*/
#define PW_DFTStableTime 40 //50ms40拍=0.05*40=2s
/*PID控制*/
//#define PW_KP_height          0.025
//#define PW_KI_height          0.1*PW_KP_height
//#define PW_Kg_height          0.1

void    GUID_LongMan(void);                       /*[航线飞行的纵剖面管理]*/
//void    GUID_MakeDim2(void);
void    GUID_Enter(void);                  /*[点号切入管理]*/
void    GUID_Way(void);                    /*[航线飞行管理]*/
void    GUID_LongSafe(void);               /*[纵向安全模块]*/
void    GUID_VcmdUpdate(void);             /*[速度更新管理]*/
void    GUID_RouteUpdate(void);            /*[航路更新管理]*/
void    GUID_GlideAngleUpdate(void);       /*[下滑角指令更新管理]*/
//void    GUID_MotorSafe(void);              //髮動機熄火安全
//void    GUID_Exit(void);                          /*[推出自主导航响应]*/
void    GUID_Airspeed_Ctrl(unsigned char tag);  /*[空速控制管理]*/
void    GUID_Axis_Change(void);            /*[坐标转换管理]*/
// void    GUID_ReFly (void);
void    GUID_Spec (int vxd, unsigned char vxd_guide);
void    GUID_WxyzCal(void);       /*[理论视线角速度计算]*/
void    GUID_GuidanceLaw(float* value);       /*[末制导律]*/
void    Los_Filter(void); //视线角速度滤波	
void   Guide_L1(double Vm,double L1,double dZ,double dPsi,double radius);  /*[L1制导法]*/ 
void   Guide_Line(double Vm,double dZ,double dPsi);/*[直线侧偏距控制]*/ 
void   Guide_Circ(double Vm, double dZ, double dPsi, double radius);;/*[圆轨迹侧偏距控制]*/
double  Guid_VcmdInit(double len,int flightTime,double dpsi); /*[初始速度指令模型]*/
double  Guid_VcmdReal(int flightTimeToGo,int ac_flightTimeToGo);/*[实时速度指令模型]*/
void    GUID_Inte_Zero(double *dError,double *dError_last,double *dError_I,double *dError_I_last);/*[积分清零]*/
extern unsigned char Mission_Updated_sign; //是否更新任务
void Update_Mission(MISSION new_Mission);//任务更新
void GUID_MISSION(void);//任务处理
#endif
//
