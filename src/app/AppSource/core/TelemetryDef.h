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
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"WP_cur",	1},	//????????
	{&DataPoolALL[MSN_DATAPOOL_INDEX],	"tarLon",	4},	//?????????
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
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsLon",	4}, //GPS??????
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
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"dirEffec",	1}, //GPS???????????
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsDir",	2}, //GPS?????????????
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"gpsScCnt",	1},	// ????????
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navLon",	4}, //??????????
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navLat",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navHigh",	4},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVn",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVs",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navVe",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navPitch",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navDir",	2},
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navRoll",	2},
 //0x00???       0x20?????     0x3F??????      0x2F?????????????????????????????????????
//0x60????????       0x64???????????  
	{&DataPoolALL[IMU_DATAPOOL_INDEX],	"navState",	1},
//?????????x?????? ??
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navX",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navY",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navZ",	4},
//????????x *10???????? ??/??
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVx",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVy",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"navVz",	2},
// ??? *100???????? kpa
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirPress",	2},
// ???? *10???? ???? ??/??
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirSpd",	2},
// ???? *10???? ???? ??/??
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"GrdSpd",	2},
// ??????????? ??
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"AirHigh",	2},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqUmb",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqDrop",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqSac1",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"seqSac2",	1},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"adrc_Mx",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"EngineRp",	2},	//014 ???????*10 ??[0~1000]??  280 ???
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"gamaCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"thetaCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"nycCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"highCmd",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dL",	4},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"ac_dZ",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"tokenlon",	1}, //????????
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"tokenlat",	1}, //????????
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
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"viewPitc",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"viewYaw",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"vPitchSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"vYawSp",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"pitchCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"rollCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"yawCmd",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRn",	2},	// ��Ŀ�������
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRu",	2},	// ??????????
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"dRe",	2},	// ??????????
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"PitchPre",	2},
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"YawPre",	2},
	// ????????????6???????????????????????100??
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Cmd",	2}, // roll left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Cmd",	2}, // roll right
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Cmd",	2}, // pitch left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Cmd",	2}, // pitch right
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Cmd",	2}, // yaw left
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Cmd",	2}, // yaw right
	// ????????????6?????????????100??
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr1Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr2Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr3Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr4Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr5Read",	2},
	{&DataPoolALL[SRV_DATAPOOL_INDEX],	"Sr6Read",	2},
};
telemetryParam tm_200hz[] =
{
	/** ?????????0xCC ???? ???????????????????
	 *  ??????????????	0xEE
	 *  ????????????????????0x00
	 */
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"RecvLunc",	1},
	//	1????????????0???????
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"startFly",	1},
	/** ?????
	 * 1??????? > 30m/s?
	 * 2???????????
	 * 3?????????? ?? ??? > 10
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
	// ???????????? ???????????1e-7
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLon",	4},
	// ?????????????? ???????????1e-7
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataLat",	4},
	// ???????????? ???????1
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataHigh",	2},
	// ???????????? ???????????????0.01
	{&DataPoolALL[FLY_DATAPOOL_INDEX],	"DataDir",	2},
// 1??SD???????????? 0xEE??0xFF??SD??????????
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
	{&DataPoolALL[PWR_DATAPOOL_INDEX],	"mcuTemp",	2},	// ???????????????
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
	//???????? * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVn",	2},
	//???????? * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVs",	2},
	//???????? * 100
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navVe",	2},
	//??????
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navPitch",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navRoll",	2},
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navDir",	2},
 //0x00???       0x20?????     0x3F??????      0x2F?????????????????????????????????????
//0x60????????       0x64???????????  
	{&DataPoolALL[NAV_DATAPOOL_INDEX],	"navState",	1},
// HACK: TEST ECU dataPool Define
//280?????
//014???????????9??????????????mbar
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"engSetRp",	2},
// ?????????????(not used)
//014???????????86????????????????
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuSetRp",	2},
//014?????69?????????[0~9999]	
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuGetRp",	2},
//280??????????? * 10	
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuTemp",	2},
//280????????????? * 10
//014?????19?????????1??????????us
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecu24V",	2},
//280?????????????? * 10
//014?????39?????????2??????????us
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecu24A",	2},
//280????????????0?????1????????2??? 3???? 4??? 5????
//014????????????0?????1????????2????? 5????
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuState",	1},
//014?????????????????0???? 1????? 
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"ecuError",	1},
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"fuelRate",	2},//
// 0??????????3??????????<150??4??????????0xCC??????????
	{&DataPoolALL[SELF_DATAPOOL_INDEX],	"flyError",	1},
// ?????1000???????????
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

