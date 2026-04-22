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
	OS_U8 runStop;	//1:run,0:stop
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
	double DD1;	// 014 空速管 静压
	double DD2;	// 014 空速管 总压
} STRU_HIL_INPUT;
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
