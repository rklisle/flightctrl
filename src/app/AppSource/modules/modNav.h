
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
	OS_U32 u32Counter;		// data[0]  2位计数器
	OS_U8 GPSstate;			// data[4]  gnss_recv_422.posType
	OS_U8 StanumberMaster;	// data[5]  gnss_recv_422.TrackStart1
	OS_U8 StanumberSlave;	// data[6]  gnss_recv_422.TrackStart2
	OS_S32 GPSlon;			// data[7]  gnss_recv_422.lon * 1e7
	OS_S32 GPSlat;			// data[11] gnss_recv_422.lat * 1e7
	OS_S32 GPShigh;			// data[15] gnss_recv_422.alt * 1e3
	OS_S16 GPSVn;			// data[19] gnss_recv_422.vn * 100
	OS_S16 GPSVs;			// data[21] gnss_recv_422.vs * 100
	OS_S16 GPSVe;			// data[23] gnss_recv_422.ve * 100
	OS_U16 PDOP;			// data[25] gnss_recv_422.pdop * 1e2
	OS_U16 GDOP;			// data[27] gnss_recv_422.hdop * 1e2
	OS_U8  Deltime;			// data[29] 0
	float imuWx16507;		// data[30] imuSourceData.imuExpensive_wx
	float imuWy16507;		// data[34] imuSourceData.imuExpensive_wy
	float imuWz16507;		// data[38] imuSourceData.imuExpensive_wz
	float imuAx16507;		// data[42] imuSourceData.imuExpensive_ax
	float imuAy16507;		// data[46] imuSourceData.imuExpensive_ay
	float imuAz16507;		// data[50] imuSourceData.imuExpensive_az

	float imuWx20689;		// data[54] imuSourceData.imuCheap20689_wx
	float imuWy20689;		// data[58] imuSourceData.imuCheap20689_wy
	float imuWz20689;		// data[62] imuSourceData.imuCheap20689_wz
	float imuAx20689;		// data[66] imuSourceData.imuCheap20689_ax
	float imuAy20689;		// data[70] imuSourceData.imuCheap20689_ay
	float imuAz20689;		// data[74] imuSourceData.imuCheap20689_az
	
    float imuWx42688;		// data[78] imuSourceData.imuCheap42688_wx
	float imuWy42688;		// data[82] imuSourceData.imuCheap42688_wy
	float imuWz42688;		// data[86] imuSourceData.imuCheap42688_wz
	float imuAx42688;		// data[90] imuSourceData.imuCheap42688_ax
	float imuAy42688;		// data[94] imuSourceData.imuCheap42688_ay
	float imuAz42688;		// data[98] imuSourceData.imuCheap42688_az
	
//0x44 初始化 0x45 上注任务 0x20 对准中 0x3F 对准完成 0x64 纯惯性 0x60 组合导航	
	OS_U8 navStatus;		// data[102]
    
	OS_S32 s32navLon;		// data[103]  nav_output.lon * d2r * 1e7
	OS_S32 s32navLat;		// data[107]  nav_output.lat * d2r * 1e7
	OS_S32 s32navHigh;		// data[111]  nav_output.alt * 1e3
	OS_S32 s32navVn;		// data[115]  nav_output.v_n * 1e3
	OS_S32 s32navVs;		// data[119]  nav_output.v_d * 1e3
	OS_S32 s32navVe;		// data[123]  nav_output.v_e * 1e3
    
	OS_U16 s16dir;			// data[127]  nav_output.fai * 1e2
	OS_S16 s16pitch;		// data[129]  nav_output.pich * 1e2
	OS_S16 s16roll;			// data[131]  nav_output.roll * 1e2
    
   	OS_S8 navInstallMode[3];// data[133]  installMode
	OS_U8 simuMode;			// data[136]  0
	OS_U8 uploadEphStatus;	// data[137]  ephUpdateState	//星历加注反馈
	
	OS_U16 magDir;			// data[138]  magdir * 1e2
    OS_U16 magRealDir;		// data[140]  estimatedir * 1e2
    
	OS_S16 magX;			// data[142]  x * 1e2
	OS_S16 magY;			// data[144]  y * 1e2
	OS_S16 magZ;			// data[146]  z * 1e2
	OS_U8 year;				// data[148]  
	OS_U8 month;			// data[149]  
	OS_U8 day;				// data[150]  
	OS_U8 hour;				// data[151]  
	OS_U8 minite;			// data[152]  
	OS_U8 second;			// data[153]  
	OS_U8 ms;				// data[154]  
   
	OS_U16 gpsDir;			// data[155]  gnss_recv_422.heading * 100  航向
	char   gpsDirEnable[2];	// data[157]  
	OS_U8 gpsupdate;		// data[159]  gnss_recv_422.posType
	OS_U8 gpsDirEffect;		// data[160]  gnss_recv_422.heading_type
	OS_U16 gpsTrack;		// data[161]  gnss_recv_422.track * 100   航迹角
	
	OS_U16 navUs;			// data[163]  calcTimeCpu0 * 1000
	OS_U16 navUsKa;			// data[165]  cpu0usKa
    OS_S16 cpuTemp;			// data[167]  System_GetCoreTemperature() * 100  NAV板子的temp
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
