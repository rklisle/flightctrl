
#ifndef SRC_MODNAV_H_
#define SRC_MODNAV_H_

#include "../support/os_framework.h"
#include "../nav/nav2.h"

#define IMU_G0				(9.794265)

#pragma pack(1)
typedef struct
{
	OS_U32 u32Counter;	//32位计数器
	OS_U8 GPSstate;
	OS_U8 Stanumber;
	OS_S32 s32GPSlon;
	OS_S32 s32GPSlat;
	OS_S32 s32GPShigh;
	OS_S16 s32GPSVn;
	OS_S16 s32GPSVs;
	OS_S16 s32GPSVe;
	OS_U16 PDOP;
	OS_U16 GDOP;
	OS_U8  Deltime;
	float f32dWx;
	float f32dWy;
	float f32dWz;
	float f32dVx;
	float f32dVy;
	float f32dVz;
	float f32Wx;
	float f32Wy;
	float f32Wz;
	float f32Nx;
	float f32Ny;
	float f32Nz;
	OS_S16 s16ImuTemp;	//惯测组合内温度T
	OS_U8  u8SelfChk;	//自检状态
	OS_U8  NAVstate;
	OS_S32 s32navLon;
	OS_S32 s32navLat;
	OS_S32 s32navHigh;
	OS_S32 s32navVn;
	OS_S32 s32navVs;
	OS_S32 s32navVe;
	OS_S16 s16Geoyaw;
	OS_S16 s16Geopitch;
	OS_S16 s16Geogama;
	OS_U8  Installstate;//安装模式
	OS_U16 IMUDir;//方位角
	float Q1;//四元数
	float Q2;
	float Q3;
	float Q4;
	OS_U16 Simstate;//仿真模式
	OS_U8 EphemerisEncpState;
	OS_U8 SoftVersion;
	OS_U32 BJTime2000_1_1Second;

}STRU_NAV_INFO;
#pragma pack()

extern OS_DOUBLE StartFocusTime;
extern NavInitStr initData;
extern OS_U32 FlyctrlRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 MsgToFlyCtrl();
extern OS_U8 ToNavModel();
extern OS_U8 StartEncpEphToNav(OS_U8 *data, OS_U16 len);
extern OS_U8 SendDataToFlyCtrl();
extern OS_S8 installMode[3];

#endif /* SRC_MODIMU_H_ */
