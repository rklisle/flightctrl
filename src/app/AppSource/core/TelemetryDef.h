# ifndef SRC_CORE_TELEMETRYDEF_H_
# include "Telemetry.h"
#define TEMEMETRY_GROUP_COUNT_40HZ (5)
#define TEMEMETRY_GROUP_COUNT_10HZ (20)

#define TM_PKT_BYTE_PAYLOAD	(0)
#define TM_PKT_BYTE_FLIGHT	(219)
#define TM_PKT_BYTE_200HZ	(219)
#define TM_PKT_BYTE_40HZ	(0)
#define TM_PKT_BYTE_10HZ	(0)
#define TM_PKT_PARAM_COUNT_40hz_10hz (0)

#define TELEMETRY_GROUP_COUNT 	(TEMEMETRY_GROUP_COUNT_40HZ + TEMEMETRY_GROUP_COUNT_10HZ + 2)

telemetry tmGroups[TELEMETRY_GROUP_COUNT] = { 0 };
telemetry tmFlight = { 0 };

telemetryParam tm_zh[] =
{
};
telemetryParam tm_flight[] =
{
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnPaoID",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnGuaID",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnGrpID",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnDevID",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnLead",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"autoStep",	1},
	//导引头：3连接，2断连
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"paylodtp",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"WP_cur",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarLon",	4},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarLat",	4},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarAlt",	2},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnComm1",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnComm2",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWx",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWy",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWz",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAx",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAy",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAz",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsLon",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsLat",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsAlt",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsVn",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsVs",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsVe",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsYear",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsMonth",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsDay",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsHour",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsMinit",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsSec",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsMSec",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"dirEffec",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsDir",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsScCnt",	1},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navLon",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navLat",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navHigh",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVn",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVs",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVe",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navPitch",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navDir",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navRoll",	2},
 //0x00准备       0x20对准中     0x3F对准完成      0x2F对准失败（奇异角、或对准过程中出现较大幅度晃动）
//0x60组合导航模式       0x64纯惯性导航模式  
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navState",	1},
//发射系位置x，单位 米
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navX",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navY",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navZ",	4},
//发射系速度x *10倍，单位 米/秒
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVx",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVy",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVz",	2},
// 静压 *100倍，单位 kpa
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirPress",	2},
// 空速 *10倍， 单位 米/秒
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirSpd",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"GrdSpd",	2},
// 气压高度，单位 米
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirHigh",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqUmb",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqDrop",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqSac1",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqSac2",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"adrc_Mx",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"EngineRp",	2},	//014 油门百分比*10 取值[0~1000]；  280 转速
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"gamaCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"thetaCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nycCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"highCmd",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dL",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dZ",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"tokenlon",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"tokenlat",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dR",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"thetav",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"Vcmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nyCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nzCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DbsLen",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ADRC",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"sctMoLd",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"sctCheck",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"sctLock",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"sctPitch",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"sctYaw",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"vPitchSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"vYawSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"fPitchSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"fYawSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"pitchCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"rollCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"yawCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRn",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRu",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRe",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"PitchPre",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"YawPre",	2},
	// 舵机角度值，一组6个，存的都是控制实际输出角度的100倍
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Cmd",	2}, // roll left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Cmd",	2}, // roll right
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Cmd",	2}, // pitch left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Cmd",	2}, // pitch right
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Cmd",	2}, // yaw left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Cmd",	2}, // yaw right
	// 舵机角度值，一组6个，存的都是角度的100倍
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Read",	2},
};
telemetryParam tm_200hz[] =
{
	/** 预发射状态：0xCC —— 发动机启动完成，转速达标
	 *  起飞完成：	0xEE
	 *  不发射了，发动机停机：0x00
	 */
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"RecvLunc",	1},
	//	1：地面按下起飞，0：没按下
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"startFly",	1},
	/** 起飞模式
	 * 1：加速度 > 30
	 * 2：地面点击起飞
	 * 3：组合导航状态 且 速度 > 10
	 */
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"luanMode",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"luncTime",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commNav",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commEcu",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commBatt",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commPwr",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commImu",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commScot",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commSrv",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commFuse",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"tcCmd",	1},
	// 发射点经度，单位 °，存的时候，除以1e-7
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLon",	4},
	// 发射点纬度，单位 °，存的时候，除以1e-7
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLat",	4},
	// 发射点高度，单位 米，当量1
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataHigh",	2},
	// 发射点射向，单位 北偏东多少度，当量0.01
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataDir",	2},
// 1：SD卡初始化成功； 0xEE或0xFF：SD卡初始化失败
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"sdState",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout1",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout2",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout3",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout4",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseMode",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuse24V",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseActv",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseBIT",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuse5V",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseuf",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseTemp",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseax",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseay",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuseaz",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"cpuTemp",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"cpuTemp2",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"mcuTemp",	2},	//MML 这里的参数对应遥测表
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"Batt28V",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"Batt28A",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"VCombin",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"ACombin",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"VFire",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"Afire",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"PwrCmd",	1},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"FireCmd",	1},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"pwrTemp",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWx",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWy",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWz",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAx",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAy",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAz",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWx2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWy2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWz2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAx2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAy2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAz2",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsMod",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsLon",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsLat",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsHigh",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsVn",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsVs",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsVe",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsPdop",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsUload",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsYear",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsMonth",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsDay",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsHour",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsMinit",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsSec",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsMs",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsTrack",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsDir",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsDirOK",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsLoCnt",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsLoMas",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"gpsLoSla",	1},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navLon",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navLat",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navHigh",	4},
	//导航北速 * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVn",	2},
	//导航天速 * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVs",	2},
	//导航东速 * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVe",	2},
	//俯仰角
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navPitch",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navRoll",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navDir",	2},
 //0x00准备       0x20对准中     0x3F对准完成      0x2F对准失败（奇异角、或对准过程中出现较大幅度晃动）
//0x60组合导航模式       0x64纯惯性导航模式  
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navState",	1},
// HACK: TEST ECU dataPool Define
//280：转速
//014：发动机指令9：实际油压，单位mbar
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"engSetRp",	2},
// 发动机设置转速(not used)
//014：发动机指令86：期望的油门位置
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuSetRp",	2},
//014：指令69，实际转速，[0~9999]	
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuGetRp",	2},
//280：发动机温度 * 10	
//014：用于启动SD卡文件写入。初始化时设置为0x00FF，收到数据链传来数据时设置为0x0001
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp",	2},
//280：发动机电池电压 * 10
//014：指令19，实际喷油1脉宽，单位us
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecu24V",	2},
//280：发动机电池电流 * 10
//014：指令39，实际喷油2脉宽，单位us
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecu24A",	2},
//280：发动机状态：0停机，1启动中，2散热 3故障 4脱机 5运行
//014：发动机状态：0停机，1启动中，2停机中 5运行
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuState",	1},
//014：发动机错误码定义：0无异常 1油压异常 
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuError",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuelRate",	2},//
// 0正常飞行，3距离伞降点<150，4正常开伞，0xCC出安全区
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"flyError",	1},
// 存的是1000倍的实际电流值
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1A",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2A",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3A",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4A",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5A",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6A",	2},
};
telemetryParam tm_40hz_pack[][TM_PKT_PARAM_COUNT_40hz_10hz] =
{
	{//40hz_1
	},
	{//40hz_2
	},
	{//40hz_3
	},
	{//40hz_4
	},
	{//40hz_5
	},
};

telemetryParam tm_10hz_pack[][TM_PKT_PARAM_COUNT_40hz_10hz] =
{
};

#endif

