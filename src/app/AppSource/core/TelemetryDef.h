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
	//???????3?????2????
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"paylodtp",	1},
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"curPtNo",	1},//当前航点号
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarLon",	4},//当前目标航点经度
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarLat",	4},//当前目标航点纬度
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarAlt",	2},//当前目标航点高度
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnComm1",	1},//与飞机ID1通信状态
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"msnComm2",	1},//与飞机ID2通信状态
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWx",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWy",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuWz",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAx",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAy",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"imuAz",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirPress",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirSpd",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"GrdSpd",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirHigh",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"openUmb",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"res2",	4},//预留2
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"res1",	2},//HIL序号，此字段不要动
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"EngineRp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"rollCmd",	2},//通道舵副翼
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"pitchCmd",	2},//通道舵升降	
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"yawCmd",	2},//通道舵航向
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"gamaCmd",	2},//滚转角指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"varthCmd",	2},//俯仰角指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nycCmd",	2},//升力面过载指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"highCmd",	2},//高度指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"vyCmd",	2},//垂速指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dL",	4},//待飞距
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dZ",	2},//侧边距
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"fcstate",	4},//飞行状态 
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"thrusCmd",	2},//推力指令，例如油门开度Kc 无符号
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"rpmState",	2},//发动机状态转速
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_Vx",	2},//射向速度
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_Vy",	2},//天向速度	
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_Vz",	2},//侧向速度
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dR",	2},//圆轨迹侧边距，未去掉转弯半径
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"azimuth",	2},//航段方位角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"thetav",	2},//轨迹倾角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"psicv",	2},//轨迹偏角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"Vcmd",		2},//速度指令 
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nyCmd",	2},//末制导纵向过载指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nzCmd",	2},//末制导侧向过载指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"wyCmd",	2},//航向角速度指令
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"pitch_nT",	2},//俯仰视线角速度滤波
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"yaw_nT",	2},//偏航视线角速度滤波
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"deltaR",	4},//弹目距离，打击点
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRn",		2},//弹目北向距离
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRu",		2},//弹目天向距离
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRe",		2},//弹目东向距离
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"PitchPre",	2},//理论俯仰框架角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"YawPre",	2},//理论偏航框架角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DusState",	4},//杜宾斯段
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DusType1",	4},//杜宾斯类型1
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DusType2",	4},//杜宾斯类型2
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DusType3",	4},//杜宾斯类型3
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DbsLen",	4},//杜宾斯段航程
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"gamacCom",	2},//滚转角指令补偿量
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"uz_gamac",	2},//侧偏控制量
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"mx_ESO",	2},//干扰估计状态量z2
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ADRC",		2},//ADRC舵偏
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"Qv",		4},//动压
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"alphaIns",	2},//地速攻角
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"betaIns",	2},//地速侧滑角	
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nyflt",	2},//体轴法向过载
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nzflt",	2},//体轴侧向过载
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"cnt_alti",	4},//高度机动次数
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"mass_cal",	2},//质量估计
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ugfZetac",	2},//高度控制量
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"uqkf",		2},//前馈控制量
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"fliCount",	4},//飞控帧计数
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"curLon",	4},//当前经度
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"curLat",	4},//当前纬度
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"curAlt",	2},//当前高度
	// 舵机指令+回读（modSrvCtl.c 写入 pDataPoolSrv）
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Cmd",	2},//pitch left  俯仰左
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Cmd",	2},//pitch right 俯仰右
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Cmd",	2},//roll left   副翼左
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Cmd",	2},//roll right  副翼右
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Cmd",	2},//yaw left    航向左
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Cmd",	2},//yaw right   航向右
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Read",	2},
};
telemetryParam tm_200hz[] =//219 B
{
//36B
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"RecvLunc",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"startFly",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"luanMode",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"luncTime",	4},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commNav",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commEcu",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commPwr",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commImu",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commScot",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commSrv",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"commFuse",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"tcCmd",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLon",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLat",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataHigh",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataDir",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"sdState",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout1",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout2",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout3",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"scout4",	2},
// fuze - 17B
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzFeedbk",	1},	// feed back & control signals
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzTask",	1},	// tasks executed
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzFirV",	2},	// fire 12V = 2byte value X 0.0004
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzPrxA",	2},	// Proxy Current = 2byte value X 0.00122
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzIC12V",	2},	// +12V IC Supply Status = 2byte value X 0.003
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzDetV",	2},	// +12V DETO Supply Status = 2byte value X 0.0004
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzDcfA",	2},	// Solenoid Current = 2byte value X 0.000065
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzC1Stat",	2},	// Capacitor 1 Charging Status = 2byte value X 0.0023
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzUnitNo",	2},	// Unit Number
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fzImpSt",	1},	// Impact / Proximity Status
// 5B
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"flyError",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"cpuTemp",	2},	// flyctrl temp
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"cpuTemp2",	2},	// nav temp
// PwrSeqCtl - 27B
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"Batt28V",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"Batt28A",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"GST-V",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"GST-I",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"engineV",	4},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"engineA",	4},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"VCombin",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"ACombin",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"STEER-V",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"STEER-I",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"pwrTemp",	2},
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"PwrCmd",	1},
// NAV - 85B
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWx",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWy",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navWz",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAx",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAy",	4},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navAz",	4},
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
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVn",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVs",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVe",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navPitch",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navRoll",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navDir",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navState",	1},
// ECU - 46B
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuMPwr",  2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuIIgn1", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuIIgn2", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuFuPre", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuFuDut", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuJet1",  2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuJet2",  2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuAirPr", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuATemp", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp1", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp2", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp3", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp4", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuIgn1F", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuIgn2F", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuPumpF", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuModeF", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuChokF", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuRPM",   2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuRPM2",  2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuFbTho", 2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuThoF",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuState", 1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuError", 1},
// SRV Feedback current - 12B
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1A",	2},
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2A",	2},
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3A",	2},
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4A",	2},
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5A",	2},
	// {&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6A",	2},
// reserve
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"res2607",	3},
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
