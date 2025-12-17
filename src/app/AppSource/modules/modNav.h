
#ifndef SRC_MODNAV_H_
#define SRC_MODNAV_H_

#include "../support/os_framework.h"

#define IMU_G0				(9.794265)

#define InstalMod1 0x6A6B//安装模式
#define InstalMod2 0x6C6D

#define SimalMod1 0x5A5B//仿真模式
#define SimalMod2 0x5C5D



#define StartAlign_H 0x8A8B//启动水平对准
#define StartAlign_V 0x8C8D//启动垂直对准

#define InertialNav   0x7A7B //惯性导航
#define IntegratNav   0x8C8D //组合导航


#pragma pack(1)
typedef struct
{
	OS_U32 u32Counter;	//32位计数器
	OS_U8 GPSstate;
	OS_U8 StanumberMaster;
	OS_U8 StanumberSlave;
	OS_S32 GPSlon;
	OS_S32 GPSlat;
	OS_S32 GPShigh;
	OS_S16 GPSVn;
	OS_S16 GPSVs;
	OS_S16 GPSVe;
	OS_U16 PDOP;
	OS_U16 GDOP;
	OS_U8  Deltime;
	float imuWx16507;
	float imuWy16507;
	float imuWz16507;
	float imuAx16507;
	float imuAy16507;
	float imuAz16507;
    
    float imuWx20689;
	float imuWy20689;
	float imuWz20689;
	float imuAx20689;
	float imuAy20689;
	float imuAz20689;
    
    float imuWx42688;
	float imuWy42688;
	float imuWz42688;
	float imuAx42688;
	float imuAy42688;
	float imuAz42688;
    
	OS_U8 navStatus;
    
	OS_S32 s32navLon;
	OS_S32 s32navLat;
	OS_S32 s32navHigh;
	OS_S32 s32navVn;
	OS_S32 s32navVs;
	OS_S32 s32navVe;
    
	OS_U16 s16dir;
	OS_S16 s16pitch;
	OS_S16 s16roll;
    
   	OS_S8 navInstallMode[3];
	OS_U8 simuMode;
	OS_U8 uploadEphStatus;
	
	OS_U16 magDir;
    OS_U16 magRealDir;
    
	OS_S16 magX;
	OS_S16 magY;
	OS_S16 magZ;
	OS_U8 year;
	OS_U8 month;
	OS_U8 day;
	OS_U8 hour;
	OS_U8 minite;
	OS_U8 second;
	OS_U8 ms;
   
	OS_U16 gpsDir;
	char   gpsDirEnable[2];
	OS_U8 gpsupdate;
	OS_U8 gpsDirEffect;
	OS_U16 gpsTrack;
	
	OS_U16 navUs;
	OS_U16 navUsKa;
    OS_S16 cpuTemp;
}STRU_NAV_INFO;

typedef struct
{
	OS_DOUBLE InitLon;
	OS_DOUBLE InitLat;
	OS_DOUBLE InitHigh;
	OS_DOUBLE InitYaw;
	OS_U8 cmd;	//1:对准 2:转导航 3:发射
} STRU_NAV_INPUT;
#pragma pack()

extern OS_U16 Startalign;//启动对准
extern OS_U16 Startnav;//启动导航
extern OS_U16 Simstate;//仿真模式
extern OS_U16 InstallMode_Lunch;
extern OS_BOOL IsToNAV;//是否已经转导航
extern STRU_NAV_INPUT navInput;

extern OS_U16 IMUEncp;
extern OS_U8  En_SIMIMUIN;
extern OS_U8 imuModuleSetCount;

extern OS_U8 NavStatusUpdata();

extern OS_U32 NavRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 NavCmdHandler(STRU_422_MSG_INFO * frame);
extern double toDeg(double rad);
extern OS_U8 MsgToNAV(OS_U8 msgID, OS_U8 *data, OS_U8 len);
extern OS_U8 NavStatusUpdata();
extern OS_U8 NavInit();
extern OS_U8 ToNavModel();
extern OS_U8 AutoSendToNav();
extern OS_U8 StartEncpEphToNav(OS_U8 *data, OS_U16 len);
extern OS_U8 CalcXYZ();
 OS_U8 SendDataToNav();
 OS_U8 SetNavInstallMode(OS_U16 mode);
 OS_U8 SetNavSimalMode(OS_U16 mode);

#endif /* SRC_MODIMU_H_ */
