/*
 * mission.h
 *
 *  Created on: 2024年12月9日
 *      Author: lenovo
 */

#ifndef SRC_MISSION_MISSION_H_
#define SRC_MISSION_MISSION_H_

#include "../support/os_framework.h"

#define SLAVE_COUNT (12)
#define AUTO_MSN_PT_MAX_COUNT	(100)
#define d2r		(57.29577951308402)

extern void MissionInit();
extern void RunMissionTask(int tick);

extern OS_U8 leadID;  //当前指挥机ID
extern OS_U8 selfID;	//本机ID

#pragma pack(1)
typedef struct REPORT_STATUS
{
	OS_U8 selfID;
	OS_U8 commStatus;
	OS_U8 luanchedStatus;
	OS_U8 navStatus;
	OS_U8 gpsSatCount;
	OS_S16 wx;
	OS_S16 wy;
	OS_S16 wz;
	OS_S16 ax;
	OS_S16 ay;
	OS_S16 az;
	OS_S32 lon;
	OS_S32 lat;
	OS_FLOAT alt;
	OS_S16 vn;
	OS_S16 vs;
	OS_S16 ve;
	OS_S16 pitch;
	OS_S16 dir;
	OS_S16 roll;
	OS_S16 TAS;

}REPORT_STATUS;

typedef enum MSN_CMD_TYPE
{
	MSN_CMD_KEEP = 0,
	MSN_CMD_WAYPOINT_FLIGHT,
	MSN_CMD_HOVER,
	MSN_CMD_ATTACK,
	MSN_CMD_CANNEL_ATTACK,
	MSN_CMD_RECYCLE,
}MSN_CMD_TYPE;

typedef enum MSN_DELAY_TYPE
{
	MSN_DELA_IMMEDIATELY = 0,
	MSN_DELA_LATER,
}MSN_DELAY_TYPE;


typedef struct MSN_CMD
{
	MSN_CMD_TYPE MsnCmdType;		//0:不变   1:航点飞行	2:盘旋	3:攻击,   4:退出攻击 5:回收
	MSN_DELAY_TYPE delayType;		//0立即执行，1执行完当前任务后执行
	OS_DOUBLE targetLon;	//目标经度  deg
	OS_DOUBLE targetLat;	//目标纬度   deg
	OS_DOUBLE targetHigh;	//目标高度  m

	OS_DOUBLE speed;		//飞行速度    m/s
	OS_U8 speedType;		//速度类型 0:空速 1:地速

	OS_DOUBLE radis;		//飞行半径   m

	OS_DOUBLE inTrack;		//入弯角度  0-360 deg
	OS_DOUBLE outTrack;		//出弯角度   0-360 deg

	OS_U32 arriveTime;		//到达时间
}MSN_CMD;

typedef struct {
	int sn; 			//点号 0-固定为发射点 其他-为规划航路点
	double lon;
	double lat;
	int h; //高度m 
	int w; //航点类型
	int t; //到达时间 秒
	float V_cmd; //飞行马赫数指令
	float outTrack;//航向
//	float radis;		//盘旋半径   
//	float hit_angle;//打击角度
	unsigned char if_airspeed_used; //是否启用空速控制
	unsigned char if_GuideFlight; //是否指点
}RoutePointIn;

typedef enum MSN_TASK_MODE
{
	AUTO_MSN_MODE = 0,
	MANUAL_MODE,
}MSN_TASK_MODE;


typedef struct FLIGHT_RESTRICTION
{
	double maxAirSpd;
	double minAirSpd;
	double minGapDis;
	double minRadius;
	double judgeDis;
}FLIGHT_RESTRICTION;


typedef enum FORMATION_MODE
{
	//编队模式，
	//1.四周环绕形
	//2.前后队列形
	//3.左右人字形
	MODE_SURROUND = 0,
	MODE_QUEUE,
	MODE_SIDE_TO_SIDE,
}FORMATION_MODE;
#pragma pack(0)

extern REPORT_STATUS slaverStatus[SLAVE_COUNT];
//extern RoutePointIn slaveCmd[SLAVE_COUNT];


extern OS_U32 SlaverHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 MsnCmdHandler(STRU_422_MSG_INFO * frame);
extern void UpdatePredictMsnByGround(STRU_422_MSG_INFO * frame);
extern double haversine_distance(double lat1, double lon1, double lat2, double lon2);
#endif /* SRC_MISSION_MISSION_H_ */
