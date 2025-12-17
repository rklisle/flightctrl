#ifndef CTRL_LAW_TYPEDEF_H
#define CTRL_LAW_TYPEDEF_H

#include "WP.h"

#define SCAN_FLY_PROCESS_PERIOD                     5                  // 扫描飞行流程周期
#define RADIAN_2_DEGREE   57.2957795130
/*DFT采样点数量*/
#define PW_DFTNUM 100

typedef unsigned char		OS_U8;
typedef signed char			OS_S8;
typedef unsigned short		OS_U16;
typedef signed short		OS_S16;
typedef unsigned int		OS_U32;
typedef signed int			OS_S32;
typedef float				OS_FLOAT;
typedef double				OS_DOUBLE;
typedef void				OS_VOID;
typedef OS_U8				OS_MEM;
typedef unsigned char		OS_BOOL;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned char BOOL;

#define sind(x)         sin((x)/57.2957795130)
#define cosd(x)         cos((x)/57.2957795130)
#define tand(x)         tan((x)/57.2957795130)

#define atand2(y,x)     57.2957795130*atan2((y),(x))
#define asind(x)        57.2957795130*asin((x))
#define acosd(x)        57.2957795130*acos((x))
#define atand(x)        57.2957795130*atan((x))
// typedef struct {
// 	//================================
// 	//输入参数
// 	double vn_pre; //INM_SndData.vn
// 	double vu_pre; //INM_SndData.vu
// 	double ve_pre; //INM_SndData.ve
// 	double vn_cur; //INM_SndData.vn
// 	double vu_cur; //INM_SndData.vu
// 	double ve_cur; //INM_SndData.ve

// 	double uav_lon; //FC_03_snddata.lon
// 	double uav_lat; //FC_03_snddata.lat
// 	double uav_dgps_high; //FC_03_snddata.dgps_high

// 	double target_lon; //FC_02_snddata.lon
// 	double target_lat; //FC_02_snddata.lat
// 	double target_high; //FC_02_snddata.high

// 	//================================

// 	double zd_track; //航迹坐标系侧向位置

// 	//导航坐标系下位置 _pre前一拍 _cur当前拍
// 	double xd_pre;
// 	double yd_pre;
// 	double zd_pre;
// 	double xd_cur;
// 	double yd_cur;
// 	double zd_cur;
// }NavPara;

typedef struct PGuidPara
{
    //================================
    // 输入参数
     float con_fTimerStep;
    double cur_lon;
    double cur_lat;
    float cur_high;
    float cur_psi;
	float cur_thetav;
    float cur_Vm;        // 当前合速度
    float cur_Vh;        // 当前水平合速度 20231107补加
    float cur_VU;        // 当前天向速度
    float  cur_VE; //当前东向速度
    float  cur_VN; //当前北向速度
    double  cur_VNUEvector[3]; //当前速度导航系矢量
    double  cur_VXYZvector[3]; //当前速度体轴系矢量
    //================================

    // 航段参数
    LineStruc AB;
    LineStruc  D1D2; //杜宾斯直线航线
    LineStruc AB_preset;
    float tan_psi;      //
    float Route_psi;    // 航段夹角
    int ac_dot;         // 当前航段号
    int Reset_dot;      // 备份复位航路点号
    int land_dot;       // 着陆特征航路点号
    unsigned char WayPass_dot;
    float Route_Ele_Angle;  //航段高低角
    float Route_Ele_Angle_GNC;  //航段制导高低角
    float Route_Glide_Angle;  //航段下滑角

    float cur_horizon_distance;         // 当前水平航程  20231107补加
    float cur_total_distance;           // 20231121补加 遥测总航程

    /*转弯段圆心参数**/
    double Circ_lon;
    double Circ_lat;
    unsigned char Circ_make;
    double Circ_psi;
    float Route_doublePsi; // 航段夹角

    // 直线侧边距控制
    float dZ_last;   // 圆轨迹侧偏距
    float dZ_I;      // 圆轨迹侧偏距积分
    float dZ_I_last; // 圆轨迹侧偏距积分

    // 圆轨迹侧边距控制
    float dR;
    float dR_last;   // 圆轨迹侧偏距
    float dR_I;      // 圆轨迹侧偏距积分
    float dR_I_last; // 圆轨迹侧偏距积分
    // 飞行器与圆轨迹相对位置参数
    double ac_dR; // 圆轨迹侧偏距

    double ac_L1; // L1距离
    float Range2go;
    double ac_Circ_dPsi;
    float LateOUTy[2], LateINy[2]; // 滤波数组
    // 飞行器定速模式
    int ac_flightTgo;   // 实际待飞时间
    int flightTgo;      // 指令待飞时间
    int flightTgo_ini;  // 航段待飞时间
    float cur_horizVm; // 当前水平速度
    float ax_cmd_pre;  // 上一拍加速度指令
    float ax_cmd_cur;  // 当前拍加速度指令
    float Vm_cmd_pre;  // 上一拍速度指令
    float Vm_cmd_cur;  // 当前拍速度指令

    // 纵向模态参数
    float height_var; // 高度指令软化
    float height_tmp; // 高度指令初值
    long step_Climb;   // 爬升计步器
    long step_Dive;    // 下滑计步器

    // 高度控制相对位置参数
    float fDHeight; // 高度误差
    float fDHeight_last;
    float fDHeight_I; // 高度误差积分
    float fDHeight_I_last;
    float LongOUTy[2], LongINy[2]; // 滤波数组
    float ny_act;                  // 侧向过载
    float KP_height;               // P增益
    float KI_height;               // I增益
    float Kg_height;               // D增益

    // 导航模式
    unsigned char cmd_guid;    // 期望导航方式
    unsigned char mode_guid;   // 导航方式
    unsigned char nav_guid;    // 自主飞行模态
    unsigned char prompt_guid; //[预导航标志][1=预导航有效][0=无效]

    /*指令标签*/
    unsigned char tag_endwp;
    unsigned char tag_land;
    unsigned char tag_home;
    unsigned char tag_WayPass;
    unsigned char tag_refly;
    unsigned char tag_NextLineEnd; // 最后航段标志位
    unsigned char tag_RealVmCal;   // 实时速度解算标志位
    unsigned char tag_RouteSwitch; //航路切换标志位
    unsigned char tag_RpUpdata; //航路点更新标志

    /*模态标签*/
    unsigned char mode_late;    /*[横侧向飞行模态字]*/
    unsigned char mode_long;    /*[纵向飞行模态字]*/
    unsigned char mode_longpre; /*[前一纵向飞行模态字]*/
    unsigned char mode_line;
    /*模态计数器*/
    unsigned char step_enter;
    unsigned char step_WayPass;

    // 部分状态字
    unsigned char on_takeoff;      // 起飞标志
    unsigned char Long_correct;    // 纵向修正标志
    unsigned char Late_correct;    // 侧向修正标志
    unsigned char Height_cmd_comp; // 高度指令比较标志 0:指令变大 1:指令变小
    // 安全监控
    unsigned char Climbing_failed; // 爬升失败
    int step_Climbing_failed;      // 失败判断计数器
    float height_cmd_max;         // 高度指令限制

    // 输出参数：控制指令
    float gama_cmd;   // 滚转角指令
    float az_cmd;     // 侧向加速度指令
    float ay_cmd;     // 侧向加速度指令
    float az_act;     // 实际侧向加速度
    float ay_act;     // 实际纵向加速度
    float height_cmd; // 高度指令
    float Vm_cmd;     // 速度指令
    float  Vm_cmd_ReturnEnd; // 回收开伞航段终端速度指令
    float  Vm_cmd_ReturnStart; // 回收开伞航段初始速度指令
    unsigned char tag_Vm_cmd_Return; //回收开伞航段速度计算成功标志
    float Min_IAS2Vel; //飞行表速折算最低速度
    float Min_OpenUmIAS2Vel; //开伞表速折算最低速度
    float   ru_Return;
    float ac_Dis2Fly; //航段待飞距
    float psi_cmd;    // 航向角指令
    float fpsi;       // 航向控制时的航向角偏差
    double ac_dPsi;    // 航迹控制时的航向角偏差
    double ac_dZ;      // 侧偏距
    double ac_dL;      // 待飞距
    u8 tag_ac_dL;      // 待飞距记录标签


    // 	int  token_long; /*[纵向飞行模态令牌号]*/
    // 	int  token_late; /*[横侧向飞行模态令牌号]*/
    // 	int  step_WayLong; /*爬升模态计数器*/
    // 	int  step_WayLate; //横侧向模态计数器
    // 	int  tag_Vm_control; /*[速度控制标志]*/
    //  int   tag_Eng_off; /*[停车控制标志]*/
    // 	unsigned char  token_long; /*[纵向飞行模态令牌号]*/
    // 	unsigned char  token_late; /*[横侧向飞行模态令牌号]*/

    unsigned char token_long; /*[纵向飞行模态令牌号]*/
    unsigned char token_late; /*[横侧向飞行模态令牌号]*/

    unsigned char step_WayLong;   /*爬升模态计数器*/
    unsigned char step_WayLate;   // 横侧向模态计数器
    unsigned char tag_Vm_control; /*[速度控制标志]*/
    unsigned char tag_Eng_off;    /*[停车控制标志]*/
    unsigned char  tag_Eng_idle; /*[怠速控制标志]*/
	unsigned char  tag_Open_Umbrella; /*[开伞控制标志]*/
    double fzd;  // 侧边距误差
    double fzdi; // 侧边距误差积分
    double fzd_last;
    double fzdi_last;
    float fdzd;

    u8 token_longpre;
    float ac_dL_LandStart;
    float ac_dL_ReturnStart; //返航段初始待飞距
		
	unsigned char  tag_Airspeed_ctrl; /*[空速控制标志]*/

    double fdpsi;                       // 只遥测

    u8 step_glide;                      // 应急策略状态切换变量

    u8 engine_startup_failed_flag;      // 发动机起动失败标志
    float thetac_record_engine_startup_failed;      // 记录发动机异常前最后的一次俯仰角指令
    float driv_time_engine_startup_failed;          // 记录发动机异常最后一次的时间

    float turn_radius;                  // 转弯半径

	/*发动机改进*/
	float nx_T,ny_T,nz_T;//弹体系加速度
	float aVhori; //水平速度变化率
	float aVtotal;//合速度变化率
	float QB_ENU;
	float QH_ENU;
	float QK_ENU; /*[姿态角][deg]*/ 

	//----矩阵转换参数---//
	double matr_b2NUEQH[3][3];
	double matr_b2NUEQB[3][3];
	double matr_b2NUEQK[3][3];

    double matr_NUE2bQH[3][3];
    double matr_NUE2bQB[3][3];
    double matr_NUE2bQK[3][3];

	double matr_NUE2TRAQH[3][3];
	double matr_NUE2TRAQB[3][3];
	double matr_I[3][3];
    //----惯测攻角和侧滑角----//
    float arf_ins;
    float beta_ins;
    //----任务相关---//
    unsigned char Dubins_stage;
    float hover_radis;//盘旋半径
    float hover_flydis;//盘旋飞行距离
    float hover_round; //盘旋圈数
    //----末制导相关---//
    unsigned char Fient_sign; //佯攻标志
    double tar_lon, tar_lat;
    float tar_alt;
    float deltaR, Bomb_R0;//斜距
    float  Wnue[3], Wxyz[3];
    float  Rnue[3];//导航系弹目距离矢量
    float  DYT_Wxyz[3],DYT_Frame[3]; //导引头输出
    float Pitch_Preset_Angle, Yaw_Preset_Angle;
    float Law_Cmd[3];
    unsigned char BTT_STT_Switch;//BTT-STT切换标志
    unsigned char Terminal_Guidance;//末制导标志
    float gama_cmd_last; //滚转角指令
    double ny_cmd, nz_cmd, nc_zl; //纵向过载指令
    float ny_zl, nz_zl, ny_PPN_zl,nz_zl_judge;//非滚弹体过载指令
    float pitch_rate_nT, yaw_rate_nT;//非滚弹体系视线角速度
    double pitch_rate_nT_filterIn[2], yaw_rate_nT_filterIn[2];//非滚弹体系视线角速度滤波
    double pitch_rate_nT_filterOut[2], yaw_rate_nT_filterOut[2];//非滚弹体系视线角速度滤波
    float GNC_time;//末制导时间
    /********攻击角约束制导*****/
    float FBEC;
    float AttackAngle;//攻击角度约束
    float lambdaD;//弹目视线高低角，目标在弹上方为正
    float lambdaT;//弹目视线方位角,北偏西为正
} PGuidPara;

typedef struct ControlPara
{
	/*******************************滚转指令跟踪*********************/
    float con_fTimerStep;
    float driv_time;
    float fkgama;
    float fkgamai;
    float fkwx;
    double fwxin[2];
    double fwxout[2];
    float fgama_base;
    float fgama;
    float fgamai;
    float fgama_last;
    float fgamai_last;
    float fduox;
    float gama;
    float gama_HeightLoop;
	/**************偏航方向*****************/
    float fkwy;
    float fkbeta;
    float fkbetai;
    float fbetai;
    float fbeta_last;
    float fbetai_last;
    float fduoy;
    float fbeta;
    double fwyin[2];
    double fwyout[2];

	/**************俯仰方向*****************/
    float fkwz;
    float thetac;
    float fktheta;
    float fkthetai;
    float ftheta;
    float fthetai;
    float ftheta_last;
    float fthetai_last;
    float fkny;
    float fknz;
    float fknyi;
    float fknzi;
    float fduoz;
    float fduozi;       // 20231129 加入积分舵面
    float theta;
    float g_fVm;
    float g_fVm_init;
    float ru;
    double fwzin[2];
    double fwzout[2];
    float fduoz_test1;
    float fduoz_test2;
    float fduoz_test3;
	/**************高度方向*****************/
    float cur_alt;              // 当前海拔高度

    double driv_YDin[2];
    double driv_YDout[2];
    float fkvyd;
    float fkyd;
    float fkydi;
    float fkyd2theta;
    float fkyd2thetai;
    float fyd;
    float fydi;
    float fyd_last;
    float fydi_last;
    float fny, fnz;
    float fnyi,fnzi;
    float fny_last, fnz_last;
    float fnyi_last, fnzi_last;
    float driv_vy_Vertical2cmd; //垂直于高度指令的速度
	float ny_cmd;//过载指令
    float ny_cmd_I;//积分过载指令
    float nyc_cmd, nzc_cmd;//末端打击过载指令
    float Height_Rate; //爬升下滑率
	/*************************切换标志**********************/
    float thetac2;
    float thetac3;
    float thetac0;

	float fvd;
	float fvd_last;
	float fvdi_last;
	float fax; 

	double acc_xin[2];
	double acc_xout[2];

	double acc_yin[2];
	double acc_yout[2];

	double acc_zin[2];
	double acc_zout[2];

	u16 FS_DOWN_Count;
	u16 Fdj_Start_Flag;

	/*************************舵控指令*********************/
	u8  launched_sign;
	float g_fUdelta01;
	float g_fUdelta02;
	float g_fUdelta03;
    float g_fUdelta04;
    float g_fUdelta05;
    float g_fUdelta06;
	float val1;
	float val2;
	float val3;

    float ve;
    float vn;
    float vu;

    float qv;
    float qvs;
    float qvl;
    float mxdx,mxdy;
    float jcx;
    float mx_ESO;
    float fduox_ADRC;
    float RollDisturbance_ESO;
    float gama_cmd_tran;
    double gama_cmd_tran_In[2];//滚转角指令一阶Tustin输入
    double gama_cmd_tran_Out[2];//滚转角指令一阶Tustin输出
    float gama0;
    float gamma0_time;
    float RollCompensate_ESO;
    float fkaz2beta;
    float fads_alpha;            // 20231120加入fads攻角
    float beta_cal;
    float g_fVmLimit;
    float climb_thita_cmd;
    float fduoz0;       // 去掉了fduoz_0
    u8 mode_flag;
    float vy_limit;
    float thetac0_time;
    float fy0;
    float gama_error;
    float ESO_step;
    float wx_ESO;
    float gama_ESO;
    float fwy;
    float fwz;
    float height_cmd_tran;
    float fvdi;
    float frpm_Percent;


    float airSpd;      // 真空速

    float feedback_duo_x;           // 实际舵角
    float feedback_duo_y;           // 实际偏航舵角
    // 空速无效标志
    // 
	
	float SREF,LREF;
	float Height_cmd_tran_ke;//高度指令指数过渡系数
	float height_cmd_tran_rate;//高度指令过渡斜率
	
	double Cy_coff_cal; //升力系数计算
	double arp_cmd; //攻角指令计算
	float mass;
    float mass_fuel;
	/******螺旋桨转速控制**********/
	float ThrustF_init; //推力基准
	float ThrustF_delt; //推力增量
    float rpm_delt; //推力增量							//20250401----【mazg】
    float rpm_delt_P, rpm_delt_I;   //比例和积分控制量 
	float Vcmd;
	float fdv,fdvi;//速度误差及积分
	float fdv_last,fdvi_last;
	float ThrustF_cmd;//指令推力
	float EngineSet;

    float Rpm_Idle;
    //----惯测攻角----//
    float arf_ins;
    /******转速限制状态机********/
    unsigned char Eng_Rpm_mode; //转速限制模式
    unsigned short step_MaxRpm0, step_MaxRpm1;
    float MaxRpmValue; //最大限制转速值
    /******DFT辨识*****/
    float flight_state_DFT[PW_DFTNUM];
    float angle_val;
    float state_real[PW_DFTNUM], state_imag[PW_DFTNUM];
    float state_mag[PW_DFTNUM], state_phase[PW_DFTNUM], state_amp[PW_DFTNUM];
    float state_amp_max, state_freq_max;
    unsigned char DFT_Stable_sign;
    unsigned short DFT_Stable_Num;
} ControlPara; // 数值类型待定

#endif


