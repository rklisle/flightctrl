/*
 * modHil.h
 *
 *  Created on: 2024年4月6日
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODHIL_H_
#define SRC_MODULES_MODHIL_H_

#include "../support/os_framework.h"
#pragma pack(1)
typedef struct
{
	OS_U8 runStop;	///1:run,0:stop
	OS_U8 useNav;	//0:用仿真数据，1:用IMU数据
	
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double pitch;
	double yaw;
	double roll;
	double airSpd;
	double lon;
	double lat;
	double alt;
	double vn;
	double vs;
	double ve;

	OS_U16 nav_state;//组合导航状态0x60惯性导航，0x64组合导航，转导航后其他值为导航异常
	
	double DD1;	// 空速管 静压
	double DD2;	// 空速管 总压

	double rpm_engine_state;//发动机转速
	OS_U16 engine_state;//发动机控制
	
	double qf;//导引头数据
	double qh;//导引头数据
	double dqf;//导引头
	double dqh;//导引头数据
	OS_U16	seeker_state;//未锁定0x00，某值为锁定
} STRU_HIL_INPUT;
typedef struct _Stru_Sim_Data_OUTPUT
{
	OS_U8 runStop;	///1:run,0:stop
	OS_U8 useNav;
    double wx;	
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    double pitch;
    double yaw;
    double roll;
    double airSpd;
    double lon;
    double lat;
    double alt;
    double vn;
    double vs;
    double ve;
    double DD1;
    double DD2;
    double DD3;
    double DD4;
    double mass;
    double xg;
    double arp;   
    double rpm_state;
    double MX_T_Disturb;	
	double qf;
    double qh;
    double dqf;
    double dqh;
    int TargetLocked;    	
	int MissileLauched;
}Stru_Sim_Data_OUTPUT;

typedef struct NAV_INPUT_SIMU
{
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
	double lon;
	double lat;
	float alt;
	float vn;
	float vs;
	float ve;
	unsigned char gpsLocated;
}NAV_INPUT_SIMU;
#pragma pack()

extern OS_U8 HilFlightStage();
extern OS_U32 HilRtHandler(STRU_422_MSG_INFO * frame);
extern STRU_HIL_INPUT hilInput;
#endif /* SRC_MODULES_MODHIL_H_ */
