#ifndef __NAV2_H
#define __NAV2_H

//初始装订数据
typedef struct _NavInitStr{
	
	double antlever_x;//杆壁X  m
	double antlever_y;//杆壁Y  m
	double antlever_z;//杆壁Z  m
	double lat; //纬度 deg
	double lon; //经度 deg
	double alt; //alt高度 m
	double v_n; //北向速度 m/s
	double v_e; //东向速度 m/s
	double v_d; //下向速度 m/s
	double fai0;//初始航向角	
unsigned char cmd;//导航方式
}NavInitStr;

//GNSS输入数据
typedef struct NAV_INPUT_GPS 
{
    unsigned char  Status1;//1表示GNSS定位有效,0表示GNSS定位无效
    unsigned char  Status2;//1表示GNSS定向有效,0表示GNSS定向无效
    unsigned char  PPS;//1表示PPS有效,0表示PPS无效
    double utc;//UTC时间  yyyy/mm/dd/hh/mm/ss.ss
    double Lon;//经度 °
    double Lat;//纬度 °
    double Height;//海拔高 m
    double Heading;//方位角（0-360°）
    double Pitch;//俯仰角（-90°~90°）
    double Track_true;//航向角（0-360°）
    double Roll;//横滚角（-90°-90°）预留
    double S_soln_SVs;//从天线当前参与解算的卫星数量 颗
    double M_soln_SVs;//主天线当前参与解算的卫星数量 颗
    double Pos_east;//东向位置坐标：以基站为原点的地理坐标系下的东向位置  m
    double Pos_north;//北向位置坐标：以基站为原点的地理坐标系下的北向位置  m
    double Pos_up;//天向位置坐标：以基站为原点的地理坐标系下的天顶向位置  m
    double Vel_east;//东向速度：地理坐标系下的东向速度  m/s
    double Vel_north;//北向速度：地理坐标系下的北向速度  m/s
    double Vel_UP;//天向速度：地理坐标系下的天顶向速度  m/s
    double Pdop;// 位置精度因子
    double Hdop;// 水平精度因子
    double Tdop;// 时间精度因子
    //未完继续
}NAV_INPUT_GPS;

//气压计输入数据
typedef struct __qbar 
{
    unsigned char  AIR;//大气机数据状态；0表示大气机数据有效,1表示大气机数据无效
	double AIR_V;// 空速  m/s
	double AIR_H;//高度  m

    //未完继续
}NAV_INPUT_QBAR;


//地磁输入数据
typedef struct __mag
{
    unsigned char  Mag_valid;//地磁数据状态；0表示地磁数据有效,1表示地磁数据无效
	double Mag_fai;// 磁航向  °
    //未完继续
}NAV_INPUT_MAG;

typedef struct
{
    unsigned long DATA_TIME_STMP1ms     ;

    double DATA_ZACCEL 		 ;//Z轴加表输出，数据参见477手册
    double DATA_YACCEL 		 ;//Y轴加表输出
    double DATA_XACCEL 		 ;//X轴加表输出
    double DATA_ZGYRO 		 ;//Z轴陀螺输出
    double DATA_YGYRO 		 ;//Y轴陀螺输出
    double DATA_XGYRO 		 ;//X轴陀螺输出

    double f_DATA_ACC_BIG_X;	//大量程x加速度计
    unsigned  char cmd;         //1：开始对准，2：转导航 3 4
}NAV_INPUT_IMU;



//导航输出数据数据
typedef struct _Navout
{
    double TS;//IMU时间 s
    double fai;//航向deg
    double pich;//俯仰 deg
    double roll;//横滚deg
    double std_fai;
    double std_pitch;
    double std_roll;
    double v_n; //北向速度 m/s
    double v_e; //东向速度 m/s
    double v_d; //天向速度 m/s
    double lat; //纬度 deg
    double lon; //经度 deg
    double alt; //alt高度 m
    unsigned char navstat;//导航状态标志
    unsigned char datastat;//数据质量状态标志
    unsigned char imustat;//IMU数据质量状态标志
    double Wx;//X轴角速度Deg/s
    double Wy;//Y轴角速度Deg/s
    double Wz;//Z轴角速度Deg/s
    double Ax;//X轴加速度m/s2
    double Ay;//Y轴加速度m/s2
    double Az;//Z轴加速度m/s2
}NAV_OUTPUT;

void NavInit(NavInitStr *p);

void NavLoop(NAV_INPUT_IMU* pin1, NAV_INPUT_GPS* pin2, NAV_INPUT_QBAR* pin3, NAV_INPUT_MAG* pin4, NAV_OUTPUT* pout );

#endif




