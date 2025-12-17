/*
 * modMEMS202.h
 *
 *  Created on: 2024Äê4ÔÂ13ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODMEMS202_H_
#define SRC_MODULES_MODMEMS202_H_

#include "../support/os_framework.h"

#pragma pack(1)
typedef struct
{
	OS_U8 dataEffective;
//	OS_U8 state;
//	OS_U8 errorLevel;
//	OS_U8 errorCode;
	OS_DOUBLE lon;
	OS_DOUBLE lat;
	OS_FLOAT alt;	
	OS_FLOAT vn;	
	OS_FLOAT vs;
	OS_FLOAT ve;	
	//OS_U16 track;
	OS_FLOAT ax;
	OS_FLOAT az;
	OS_FLOAT ay;
	//OS_FLOAT g;
	OS_FLOAT pitch;
	OS_FLOAT roll;
	OS_FLOAT dir;
	OS_FLOAT wz;
	OS_FLOAT wx;
	OS_FLOAT wy;
	//OS_FLOAT spdAYaw;
	//OS_U16 temp;
	OS_U8 time[7];
	OS_U8 satCount;
	OS_U16 pdop;
	OS_DOUBLE gpsLon;
	OS_DOUBLE gpsLat;
	OS_FLOAT gpsAlt;
	OS_FLOAT gpsVe;
	OS_FLOAT gpsVn;
	OS_FLOAT gpsVs;
	OS_FLOAT gpsDir;
	OS_U8  DirMarker;
}STRU_IMU_INFO;
#pragma pack()

extern OS_U8 ImuInit();
extern OS_U16 ChkImuFrame(OS_MEM* pmData);
extern OS_U32 ImuCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 ImuRtHandler(STRU_422_MSG_INFO * frame);


extern OS_U16 cmdToIMU;
extern OS_U8 ToImuFocus();
extern void ToImuNav();

#define IMU_CHECK_FINISH    (0x10)
#define IMU_FOCUS_FINISH    (0x20)
#define IMU_NAV_GPS         (0x60)
#define IMU_NAV_IMU         (0x64)
#endif /* SRC_MODULES_MODMEMS202_H_ */
