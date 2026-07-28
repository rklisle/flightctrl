/*
 * mission.c
 *
 *  Created on: 2024??12??9??
 *      Author: lenovo
 */

#include "mission.h"
#include <math.h>
#include "../core/BusInteract.h"
// #include "../flight/os_flight_io.h"
#include "../Modules/modNav.h"
#include "../Modules/modFlash.h"
#include "../FlightSupport.h"
#include "flightPort.h"
#include "tx_api.h"

#ifndef M_PI
#define M_PI (3.1415926)
#endif
static void GetOtherPlaneStatus();
static void ReportStatus();
static void VoteMaster();
static void PathPlanning();
static void BoardcastNewMission();
static void GenerateSlaveCmd();
static void UpdateSlaveCmd();
static void CalcSlaveWayPointFlightTarget();
static void Queue_WayPoint();
static void Surround_WayPoint();
static void SideToSide_WayPoint();
static void Queue_Hover();
static void Surround_Hover();
static void SideToSide_Hover();

static void CalcSlaveHoverTarget();
static void ReceivePlanningCmd();
static void DoSelfMsn();
static void GenerateTeamMsn();

/** 工具类函数 */

double haversine_distance(double lat1, double lon1, double lat2, double lon2);
static OS_U8 JudgeInCricle(double distance, double r);
static OS_U8 JudgeFullCricle(double yaw1, double yaw2);
void calcNextPt(double curLon, double curLat, double dir, double distance, double *outLat, double *outLon);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void LoadPaoIDFromFile();

extern Stru_Initial_Data   g_initial_data;//初始发射数据；
extern Stru_Route_Data     g_route_data;//初始预装订航点信息
extern Stru_Data_Datalink_To_ControllerSig g_DLtoCtrl_sig;	//在线更新多个航点
extern Stru_Data_Controller_To_Switch_Output    g_controller_to_switch;//三个标志位

extern TX_BYTE_POOL byte_pool_0;

OS_U8 leadID = 1;
OS_U8 paoID = 1;
OS_U8 guanID = 1;
OS_U8 groupID = 0xFF;
OS_U8 selfID = 0xFF;
MSN_TASK_MODE curMsnMode = AUTO_MSN_MODE;
REPORT_STATUS slaverStatus[SLAVE_COUNT] = {0};	//每个从机的状态报告
REPORT_STATUS selfStatus = {0};		//定义：本机的状态报告	// 用途：紧急返航时要用

//MSN_CMD slaveCmd[SLAVE_COUNT] = {0};
//MSN_CMD teamMsnCmd = {0};
//MSN_CMD selfCmd = {0};
//MSN_CMD flightCmd = {0};
//MSN_CMD groundCmd = {0};

MSN_CMD autoMsnPt[AUTO_MSN_PT_MAX_COUNT] = {0};
RoutePointIn slaveCmd[SLAVE_COUNT] = {0};// 主机从机编队飞行使用。
RoutePointIn teamMsnCmd = {0};	// 主机从机编队飞行使用。多处使用，谨慎，不能轻易删除
RoutePointIn selfCmd = {0};	// 暂时用的地方不多

// RoutePointIn groundCmd = {0};	// 紧急返航时要用
RoutePointIn Arp[AUTO_MSN_PT_MAX_COUNT] = {0};	// 定义：初始预装订的所有航点信息 //用途： 多处使用，谨慎，不能轻易删除

FLIGHT_RESTRICTION flightRestriction = {0};	//飞行限制参数
FORMATION_MODE formationMode = MODE_SURROUND;	//编队模式

OS_U8 RecoverMark = 0;//回收标志位，当判断可以切航点时看到该标志位，可进入回收流程
void MissionInit()	//初始化任务模块
{
	flightRestriction.maxAirSpd = 45;
	flightRestriction.minAirSpd = 35;
	flightRestriction.minGapDis = 100;
	flightRestriction.minRadius = 500;
	flightRestriction.judgeDis = 250;

	formationMode = MODE_QUEUE;
	SETDATA(pDataPoolMsn,	"msnDevID",	0xFF,	OS_U8);
	SETDATA(pDataPoolMsn,	"msnGrpID",	0xFF,	OS_U8);
	SETDATA(pDataPoolMsn,	"msnLead",	0x1,	OS_U8);
	
	LoadPaoIDFromFile();
}


/***********************************************************
 * 函数名称:RunMissionTask()
 * 输入参数:tick 代表本函数实际调用间隔，每个tick是5ms
 * 函数功能: 按时间调用任务主函数，执行任务指令，本函数按顺序完成了多机的数据采集、广播、投票、
 * 规划、分发、接收、执行
 * 本任务算法为分布式任务算法，在规划路径之前，所有的飞机在逻辑上是等价的，每架飞机都要将自己的
 * 飞机状态报告至其它飞机，也需要收集其它所有飞机的状态，使得每架飞机均有能力进行任务规划。
 * 在任务的执行端，每架飞机也是等价的，都要获取本机的具体任务并执行。
 * 唯一不同的就是，在规划时刻，需要判断自己是不是主机，不是主机则不运算，不分发。是主机则执行
 * 运算和分发。
 * 作者:	成宏璟
 ***********************************************************/
void RunMissionTask(int tick)
{
	if(g_DeviceState.CurrTick % tick != 0)	// tick为10，每50ms执行一次
	{
		return;
	}
	//1.获取各飞机状态
	GetOtherPlaneStatus();	// MML 目前没有从机，只有一架飞机
	//2.将状态发往所有飞机
	ReportStatus();
	//以下动作起飞后才有
	if(DOM_AUTOMATIC & g_DeviceState.workStage)
	{
		//3.选取主机
		VoteMaster();
		//4.进行路径规划(仅主机)
		PathPlanning();		//MML 目前能使用的就是控制输出的标志位（关发动机、开伞）
		//5.将规划指令发往从机(仅主机)
		BoardcastNewMission();
		//6.接收指令(仅从机)
		ReceivePlanningCmd();
		//7.将本机指令按要求推送至飞控算法模块(主从机)
		//DoSelfMsn();
   }
}

/***********************************************************
 * 函数名称:GetOtherPlaneStatus()
 * 函数功能: 获取编队中其它飞机的飞行状态，主要包含飞机位置、姿态及空速
 * 作者:	成宏璟
 ***********************************************************/
static void GetOtherPlaneStatus()
{
	int planeCount = sizeof(slaverStatus)/sizeof(REPORT_STATUS);
	for(int i=0;i<planeCount;i++)
	{
		//判断各个飞机的通信状态
		if(slaverStatus[i].commStatus > 0)
			 slaverStatus[i].commStatus--;
		else
			slaverStatus[i].commStatus = 0;
	}
}

/***********************************************************
 * 函数名称:ReportStatus()
 * 函数功能: 定时将飞机状态报告给其它飞机
 * 作者:	成宏璟
 * 周期性将本机状态发送至数据链，包括本机ID、经纬高、航姿
 ***********************************************************/
static void ReportStatus()
{
	selfStatus.selfID = selfID;
	GetDataFast(pDataPoolImu, "navLon", &selfStatus.lon);
	GetDataFast(pDataPoolImu, "navLat", &selfStatus.lat);
	GetDataFast(pDataPoolImu, "navHigh", &selfStatus.alt);
	GetDataFast(pDataPoolImu, "navPitch", &selfStatus.pitch);
	GetDataFast(pDataPoolImu, "navDir", &selfStatus.dir);
	GetDataFast(pDataPoolImu, "navRoll", &selfStatus.roll);

  //SETDATA(pDataPoolFly,	"EngineRp",	0xCdce,	OS_U16);
	//MsgToDevice(RT_DATA_LINK, BUS_SLAVER_REPORT, sizeof(REPORT_STATUS), (OS_U8*)&selfStatus);
}

/***********************************************************
 * 函数名称:VoteMaster()
 * 函数功能: 判断本机是不是主机，由于采取的是MESH网络，因此只要有任意路径可达主机，则主机
 * 通信就不会中断，当发生中断时，可立刻由ID号最小的飞机接手主机
 * 作者:	成宏璟
 ***********************************************************/
static void VoteMaster()
{
	//当飞机未进入平稳飞行时，认为发射阶段尚未完成，不进行主机选取，认为本机即为主机
	if(g_DeviceState.currTime < 20.0)
	{
		leadID = selfID;
	}
	else
	{
		int planeCount = sizeof(slaverStatus)/sizeof(REPORT_STATUS);
		for(int i=0;i<planeCount;i++)
		{
			//判断各个飞机的通信状态
			if(slaverStatus[i].commStatus != 0)
			{
				if(slaverStatus[i].luanchedStatus == 1)
				{
					leadID = slaverStatus[i].selfID;
					break;
				}
			}
		}
	}
    SETDATA(pDataPoolMsn,	"msnLead",	leadID,	OS_U8);
}

/***********************************************************
 * 函数名称:PathPlanning()
 * 函数功能: 路径规划主算法，仅有主机执行路径规划
 * 执行过程分两部分，一个是有新指令，新指令包括地面新指令，或烧写在flash中的预设指令
 * 第二部分则是没有新指令时，由于其它所有飞机当前飞行高度、位置进行指令更新
 * 作者:	成宏璟
 ***********************************************************/
static void PathPlanning()
{
	if(leadID != selfID)
	{
		//本机不是指挥机，退出
		return;
	}
	//获取当前地面指令/程控指令
	if(curMsnMode == AUTO_MSN_MODE)	//MML 此条件目前永远成立
	{//程控模式，读取预设指令列表
		GenerateTeamMsn();// MML 判控制输出标志位
	}
	if (teamMsnCmd.w == 0)
	{
		//没有新的程控指令及地面指令
		//根据当前飞机位置与前置指令，判断是否更新部分飞机的飞行指令
		UpdateSlaveCmd();//MML 空
		return;
	}
	GenerateSlaveCmd();// MML 无法使用
	//此时slaveCmd已经形成
}

/***********************************************************
 * 函数名称:BoardcastNewMission()
 * 函数功能: 将主机计算的新航点发送至其它飞机，非主机不指令本函数
 * 作者:	成宏璟
 ***********************************************************/
static void BoardcastNewMission()
{
    if (leadID != selfID)
    {
		//本机不是指挥机，退出
        return;
    }
	// 仅主机执行
    for (int i = 0; i < SLAVE_COUNT; i++)
    {
        if (i != selfID && slaveCmd[i].w != 0)
        {
            //MsgToDevice(RT_DATA_LINK, BUS_SLAVER_CMD, sizeof(MSN_CMD), (OS_U8*)&slaveCmd[i]);
        }
    }
}

/***********************************************************
 * 函数名称:ReceivePlanningCmd()
 * 函数功能: 作为从机接收主机发送的任务指令，注意，在本逻辑中，主机自己也是主机的从机
 * 作者:	成宏璟
 ***********************************************************/
static void ReceivePlanningCmd()
{
	for(int i=0;i<SLAVE_COUNT;i++)
	{
		if(slaverStatus[i].selfID == selfID)
		{
			selfCmd = slaveCmd[i];
			break;
		}
	}
}

/***********************************************************
 * 函数名称:DoSelfMsn()
 * 函数功能: 将本机接收的任务指令传递至飞控，此时需要注意的就是判断航点的执行条件，是需要
 * 先执行完上个航点还是说直接执行下个航点，此函数执行完成后，交由控制完成路径规划及飞行,
 * 对已存储的航点是否切下一个航点，也是在本函数判断
 * 作者:	成宏璟
 ***********************************************************/
static void DoSelfMsn()
{
/*
    //此处注意，如果本机是从机才进行从selfcmd到flightcmd的拷贝
    if (leadID != selfID)
    {
        //本机不是指挥机
        if (selfCmd.MsnCmdType == 0)
            return;

		if(selfCmd.delayType == MSN_DELA_IMMEDIATELY)
		{
			memcpy(&flightCmd, &selfCmd, sizeof(MSN_CMD));
			selfCmd.MsnCmdType = 0;
		}
		else if(selfCmd.delayType == MSN_DELA_LATER)
		{
			//判断航点执行情况

			//用当前经纬，及flightCmd经纬进行比较
			if(flightCmd.targetLon < 1)
			{
				//此时说明没有正在执行的航点
				memcpy(&flightCmd, &selfCmd, sizeof(MSN_CMD));
			}

			double dist = haversine_distance(selfStatus.lat * 1e-7, selfStatus.lon * 1e-7,
					flightCmd.targetLat, flightCmd.targetLon);

			if(dist < flightRestriction.judgeDis)
			{
				memcpy(&flightCmd, &selfCmd, sizeof(MSN_CMD));
				selfCmd.MsnCmdType = 0;
			}
		}
	}
	if(flightCmd.MsnCmdType == 0)
	{
		//本机当前指令未更新，不操作
	}
	else
	{
		MISSION fmsn;
		//判断是否为回收点，如果是回收点，则记录回收标志，再将类型置为航点飞行送彭博
		if(flightCmd.MsnCmdType == 5)
		{
			flightCmd.MsnCmdType = 1;
			RecoverMark = 1;
		}
        else
        {
            RecoverMark = 0;   
        }
        fmsn.MsnCmdType = flightCmd.MsnCmdType;
        fmsn.targetLon = flightCmd.targetLon;
        fmsn.targetLat = flightCmd.targetLat;
        fmsn.targetHigh = flightCmd.targetHigh;
        fmsn.speed = flightCmd.speed;
        fmsn.radis = flightCmd.radis;
        fmsn.outTrack = flightCmd.outTrack;
        fmsn.inTrack = flightCmd.inTrack;
        fmsn.arriveTime = flightCmd.arriveTime;
        UpdateMission(fmsn);
        flightCmd.MsnCmdType = 0;
    }
*/
}

/***********************************************************
 * 函数名称:SlaverHandler()
 * 函数功能: 接收指令，包括地面指令、其它飞机发送的状态报告、以及主机发往从机的任务指令
 * 作者:	成宏璟
 * in addition: 数据链过来的指令，但这里啥也没做
 ***********************************************************/
OS_U32 SlaverHandler(STRU_422_MSG_INFO * frame)
{
	switch(frame->u8MsgID)
	{
	case BUS_SLAVER_REPORT:
		break;
	case BUS_SLAVER_CMD:
		break;
	default:
		break;
	}


	return 0;
}

/***********************************************************
 * 函数名称:GenerateSlaveCmd()
 * 函数功能: 任务规划算法，根据主指令计算从机飞行航点指令
 * 作者:	成宏璟
 ***********************************************************/
static void GenerateSlaveCmd()
{
    switch (teamMsnCmd.w)
    {
    case MSN_CMD_WAYPOINT_FLIGHT:
        CalcSlaveWayPointFlightTarget();
        break;
    case MSN_CMD_HOVER:
        CalcSlaveHoverTarget();
        break;
    case MSN_CMD_ATTACK:
        break;
    default:
        break;
    }
}

/***********************************************************
 * 函数名称:UpdateSlaveCmd()
 * 函数功能: 任务更新算法，根据主指令执行情况，计算从机飞行航点指令
 * 作者:	成宏璟
 ***********************************************************/
static void UpdateSlaveCmd()
{

}

/***********************************************************
 * 函数名称:CalcSlaveWayPointFlightTarget()
 * 函数功能: 航点飞行指令的执行函数，计算收到航点飞行指令后，从机的航点位置
 * 作者:	成宏璟
 ***********************************************************/
static void CalcSlaveWayPointFlightTarget()
{
	switch(formationMode)
	{
	case MODE_SURROUND:
		Surround_WayPoint();
		break;
	case MODE_QUEUE:
		Queue_WayPoint();
		break;
	case MODE_SIDE_TO_SIDE:
		SideToSide_WayPoint();
		break;
	}
}

/***********************************************************
 * 函数名称:Queue_WayPoint()
 * 函数功能: 航点飞行指令时，使用前后编队飞行的飞行算法
 * 作者:	成宏璟
 ***********************************************************/
static void Queue_WayPoint()
{
    for (int i = 0; i < SLAVE_COUNT; i++)
    {
        slaveCmd[i].w = MSN_CMD_WAYPOINT_FLIGHT;
        slaveCmd[i].outTrack = teamMsnCmd.outTrack;
        slaveCmd[i].lon = teamMsnCmd.lon;
        slaveCmd[i].lat = teamMsnCmd.lat;
        slaveCmd[i].h = teamMsnCmd.h;
        slaveCmd[i].t = teamMsnCmd.t +
                                 i * (flightRestriction.minGapDis / teamMsnCmd.V_cmd) ;
        //slaveCmd[i].delayType = MSN_DELA_LATER;
    }

}

/***********************************************************
 * 函数名称:Queue_WayPoint()
 * 函数功能: 航点飞行指令时，使用从机围绕主机方式执行的飞行算法
 * 作者:	成宏璟
 ***********************************************************/
static void Surround_WayPoint()
{}

/***********************************************************
 * 函数名称:Queue_WayPoint()
 * 函数功能: 航点飞行指令时，使用人字形飞行算法
 * 作者:	成宏璟
 ***********************************************************/
static void SideToSide_WayPoint()
{}

/***********************************************************
 * 函数名称:CalcSlaveHoverTarget()
 * 函数功能: 盘旋指令时，从机航点指令计算
 * 作者:	成宏璟
 ***********************************************************/
static void CalcSlaveHoverTarget()
{
	switch(formationMode)
		{
		case MODE_SURROUND:
			Surround_Hover();
			break;
		case MODE_QUEUE:
			Queue_Hover();
			break;
		case MODE_SIDE_TO_SIDE:
			SideToSide_Hover();
			break;
		}
}

/***********************************************************
 * 函数名称:Surround_Hover()
 * 函数功能: 盘旋指令时，使用四周围绕飞行算法
 *             +------------------+
             /                      \
        P2  / P5                     .
           /                          \
     P3   P1                           .
         /    P4                        \
        P6                               .
         \                              /
          .                            .
           \                          /
            .                        .
             \                      /
              +--------------------+
 * 作者:	成宏璟
 ***********************************************************/
static void Surround_Hover()
{
    for (int i = 0; i < SLAVE_COUNT; i++)
    {
        slaveCmd[i].w = MSN_CMD_WAYPOINT_FLIGHT;
        slaveCmd[i].outTrack = teamMsnCmd.outTrack;
        slaveCmd[i].lon = teamMsnCmd.lon;
        slaveCmd[i].lat = teamMsnCmd.lat;
        slaveCmd[i].h = teamMsnCmd.h;
        slaveCmd[i].t = teamMsnCmd.t +
                                 i * (flightRestriction.minGapDis / teamMsnCmd.V_cmd) ;
        //slaveCmd[i].delayType = MSN_DELA_LATER;
    }

}

/***********************************************************
 * 函数名称:Queue_Hover()
 * 函数功能: 航点盘旋指令时，使用从机沿主机飞行路径的飞行算法
 *             P4---P3---P2---P1--+
              /                     \
            P5                       .
           /                          \
          P6                           .
         /                              \
        .                               .
         \                              /
          .                            .
           \                          /
            .                        .
             \                      /
              +--------------------+
 *
 * 作者:	成宏璟
 ***********************************************************/
static void Queue_Hover()
{}

/***********************************************************
 * 函数名称:SideToSide_Hover()
 * 函数功能: 盘旋飞行指令时，使用人字形飞行算法
 *             +------------------+
             /                      \
            P1                       .
        P2 /  P4                      \
     P3   .    P5                      .
         /      P6                      \
        .                               .
         \                              /
          .                            .
           \                          /
            .                        .
             \                      /
              +--------------------+
 * 作者:	成宏璟
 ***********************************************************/
static void SideToSide_Hover()
{}

//地面发出了紧急返航指令    
#define RECYCLE_ROUTE_MODE  6

OS_U8 DoReturnHomeward()
{
	int i;
	int found = 0;
	double lon = 0.0;
	double lat = 0.0;
	double alt = 0.0;
	double speed = 0.0;
	int num_col;
	RecoverMark = 1;
	if(g_route_data.p_route_data == NULL || g_route_data.num_rows <= 0)
	{
		return 0;
	}

	num_col = g_route_data.num_columns;
	if(num_col <= 0)
	{
		num_col = 11;
	}

	/* å–æœ€åŽä¸€ä¸?å›žæ”¶èˆ?ç‚¹ï¼ˆä¸Žæœ«ç‚¹å›žæ”¶è?…è??ä¸€è‡´ï¼‰*/
	for(i = 0; i < g_route_data.num_rows; i++)
	{
		int idx = i * num_col;

		if((int)g_route_data.p_route_data[idx + 4] == RECYCLE_ROUTE_MODE)
		{
			lon   = g_route_data.p_route_data[idx + 1];
			lat   = g_route_data.p_route_data[idx + 2];
			alt   = g_route_data.p_route_data[idx + 3];
			speed = g_route_data.p_route_data[idx + 7];
			found = 1;
		}
	}
	if(!found)
	{
		return 0;
	}

	g_DLtoCtrl_sig.update_count++;
	g_DLtoCtrl_sig.num_waypoint_updated = 1;
	g_DLtoCtrl_sig.longitude[0]         = lon;
	g_DLtoCtrl_sig.latitude[0]          = lat;
	g_DLtoCtrl_sig.height[0]            = alt;
	g_DLtoCtrl_sig.route_mode[0]        = RECYCLE_ROUTE_MODE;
	g_DLtoCtrl_sig.formation_mode[0]    = 0;
	g_DLtoCtrl_sig.dltTime[0]           = 0;
	g_DLtoCtrl_sig.turn_radius[0]       = 0;
	g_DLtoCtrl_sig.velocity[0]          = speed;
	g_DLtoCtrl_sig.accept_radius[0]     = 0; /* é£žæŽ§ä¾? route_mode==6 æ—¶é»˜è®? 100 m */
	{
		double curLon = selfStatus.lon * 1e-7;
		double curLat = selfStatus.lat * 1e-7;
		g_DLtoCtrl_sig.turn_angle[0] = calculateBearing(curLat, curLon, lat, lon);
	}
	return 0;
}

/**函数功能：
 * 20秒后，每次(50ms)进入本函数将判断飞机在起飞后，
 * 控制是否让关闭发动机
 * 控制是否让开伞
 */
static void GenerateTeamMsn()
{
	if(g_DeviceState.currTime < 20.0)	// 20s之后，执行
	{
		return;
	}
	OS_U8 takeoff;
	GetDataFast(pDataPoolSelf,	"RecvLunc",	&takeoff );
	if(takeoff != 0xEE)	
	{
		// 起飞未完成
			return;
	}

	// 起飞完成（此时，控制算法中输出起飞标志位为1）
	if(g_controller_to_switch.flag_engine_shutdown == 1)
	{
		SETDATA(pDataPoolMsn,	"msnComm2",	g_controller_to_switch.flag_engine_shutdown, OS_U8);
		StopEngine();	/* åè??ï¼šé?žæŽ§å‘åœè½¦æŒ‡ä»¤ï¼Œè½?ä»¶åœ ECU */
	}
	if(g_controller_to_switch.flag_open_umbrella == 1)
	{
		SETDATA(pDataPoolSelf,  "flyError", 4,  OS_U8);
		DoOpenUm();//正常伞降点开伞(含紧急返航)
		g_controller_to_switch.flag_open_umbrella = 0;
		return;
	}
	/*
	//立刻发出第一个点
	static int curNavPt = 0;
	static int firstPt = 0;
	static int inCricle = 0;
	static double inCricleAngle = 0;
	if(firstPt == 0)
	{
		flightCmd = autoMsnPt[curNavPt];
		firstPt = 1;
		return;
	}   
	//执行地面上注指令
	if(groundCmd.MsnCmdType != 0)
	{
		memcpy(&flightCmd, &groundCmd, sizeof(MSN_CMD));
		if(groundCmd.MsnCmdType == 5)
		{
				RecoverMark = 1;
		}
		groundCmd.MsnCmdType = 0;
	}
    
	double dist = haversine_distance(selfStatus.lat * 1e-7, selfStatus.lon * 1e-7,
	flightCmd.targetLat, flightCmd.targetLon);
    
	if(RecoverMark == 1)
	{
			if(dist < flightRestriction.judgeDis)
	{
					//伞降点
					SETDATA(pDataPoolSelf,  "flyError", 4,	OS_U8);//正常开伞
					DoOpenUm();//伞降点开伞
					return;
			}
	}
    
	if(autoMsnPt[curNavPt].MsnCmdType == 1)//航点飞行
	{
		if(dist < flightRestriction.judgeDis)
		{
			curNavPt++;
			flightCmd = autoMsnPt[curNavPt];
		}
	}
	else if(autoMsnPt[curNavPt].MsnCmdType == 2)
	{
		static int waitCount = 0;
		double dir = selfStatus.dir * 0.01;
		if(inCricle)
		{
			//已经入圈，判断出圈时机
			if(waitCount < 20)//等待10秒飞行后再判断
			{
				waitCount++;
				return;
			}
			if(autoMsnPt[curNavPt].outTrack < 499)
				inCricleAngle = autoMsnPt[curNavPt].outTrack;


			if(JudgeFullCricle(dir, inCricleAngle))
			{
				curNavPt++;
				inCricle = 0;
				flightCmd = autoMsnPt[curNavPt];
			}
		}
		else
		{
			if(JudgeInCricle(dist, autoMsnPt[curNavPt].radis))
			{
				inCricleAngle = dir;
				inCricle = 1;
				waitCount = 0;
			}
		}
	}
	*/
}

void LoadPaoIDFromFile()
{
	char* fileBuf;
	char* p;
	char *endptr;
	p = fileBuf = AllocFileBuffer("5.PaoGuanID.dat", "r");
	if(*p != 0)
	{
		paoID = BinFileToDouble(p, &endptr); p = endptr;// PaoID
		guanID = BinFileToDouble(p, &endptr); p = endptr;// GuanID
	}
	SETDATA(pDataPoolMsn,	"msnPaoID",	paoID,	OS_U8);
	SETDATA(pDataPoolMsn,	"msnGuaID",	guanID,	OS_U8);
}

void LoadSafeArea(unsigned char * buf)
{
    InitSafeArea(buf);
}
OS_U8 InitSafeAreaFromMsn(const OS_U8 *buf)
{
    safePointCount = buf[3];
    for (int i = 0; i < safePointCount; i++) {
        OS_S32 lon = *(OS_S32*)(buf + 8 + i * 8);
        OS_S32 lat = *(OS_S32*)(buf + 12 + i * 8);
        safePoints[i].x = lon * 1e-7;
        safePoints[i].y = lat * 1e-7;
    }
    return 0;
}

OS_U8 nptBuffer[49*12]; //在线更新航点：每个49 B，最多12个
OS_U32 MsnCmdHandler(STRU_422_MSG_INFO * frame)
{
    OS_U8 msgID = frame->u8MsgID;
    switch (msgID)
    {
    case CMD_MSN_UPDATE:	//初始预装订航点
		// 机载端目前最多支持32点
		// 每个航点 17 Byte
        UpdatePredictMsnByGround(frame);
        break;
    case CMD_MSN_NEWPT:	//在线更新航点
		{
			// 在线更新航点， 最多支持12点
			// 每个航点 49 Byte
				/*
			double lon, lat, high, track, speed, arriveTime;
			MISSION msn;
			memcpy(&lon, frame->au8Data + 1, 8);
			memcpy(&lat, frame->au8Data + 9, 8);
			memcpy(&high, frame->au8Data + 17, 8);
			memcpy(&track, frame->au8Data + 25, 8);
			memcpy(&speed, frame->au8Data + 33, 8);
			memcpy(&arriveTime, frame->au8Data + 41, 8);
			msn.MsnCmdType = frame->au8Data[0];
					msn.targetLon = lon;
					msn.targetLat = lat;
					msn.targetHigh = high;
					msn.speed = speed;
					msn.outTrack = track;
					UpdateMission(msn);
				*/
			OS_U8 count = frame->au8Data[0];
			double lon,lat,alt,dir,speed,t;
			unsigned char type;
			// RoutePointIn nrp[AUTO_MSN_PT_MAX_COUNT];
			//新获取指令存入nptBuffer
			memset(nptBuffer, 0, sizeof(nptBuffer));
			memcpy(nptBuffer, frame->au8Data + 1, count * 49);

			g_DLtoCtrl_sig.update_count++;
			g_DLtoCtrl_sig.num_waypoint_updated = count;

			// OS_U16 curpoint;
			// curpoint = pOutput->curPtNo;

			for (int i = 0; i < count; i++)
			{	
				memcpy(&type,  nptBuffer + 49 * i, 1);		
				memcpy(&lon,   nptBuffer + 49 * i + 1, 8);
				memcpy(&lat,   nptBuffer + 49 * i + 9, 8);
				memcpy(&alt,   nptBuffer + 49 * i + 17, 8);
				memcpy(&dir,   nptBuffer + 49 * i + 25, 8);
				memcpy(&speed, nptBuffer + 49 * i + 33, 8);
				memcpy(&t, nptBuffer + 49 * i + 41, 8);
				
				g_DLtoCtrl_sig.longitude		[i] = lon;
				g_DLtoCtrl_sig.latitude			[i] = lat;
				g_DLtoCtrl_sig.height			[i] = alt;
				g_DLtoCtrl_sig.route_mode		[i] = type;
				g_DLtoCtrl_sig.formation_mode	[i] = 0;
				g_DLtoCtrl_sig.dltTime			[i] = t;
				g_DLtoCtrl_sig.turn_angle		[i] = dir;
				g_DLtoCtrl_sig.turn_radius		[i] = 0;
				g_DLtoCtrl_sig.velocity			[i] = speed;
				g_DLtoCtrl_sig.accept_radius	[i] = 0;

				// nrp[i].sn = i + curpoint +1;
				// nrp[i].lon = lon;
				// nrp[i].lat = lat;
				// nrp[i].h = alt;
				// nrp[i].outTrack = dir;
				// nrp[i].t = t;
				// nrp[i].w = type;
				// nrp[i].V_cmd = speed;                                                         
				// //nrp[i].if_airspeed_used = 0;							
			}
			// updateNewRP(nrp, count, curpoint + 1, ptCount); 
		}
		break;
    }
    return 0;
}


unsigned char ptBuffer[17 * 32];	// 每个航点17字节，最多32个航点

/**在该函数中为控制准备航点数据，写入控制航点数据结构体
 */
void UpdatePredictMsnByGround(STRU_422_MSG_INFO * frame)
{
	static int ptCount = 0;	// ×Üº½µãÊýÁ¿

    unsigned char buf[250];
    memcpy(buf, frame->au8Data, frame->u16Len);
    // frame->au8Data中，第[0]字节为标志位，0表示传的是安全区数据；[1]、[2]、[3]表示传的是航点信息
    switch(buf[0])
    {
    case 0:		// 第0包，传安全区
		ptCount = buf[4];	// 总航点数量
        if (ptCount == 0) return;

        groupID = buf[1];
        selfID = buf[2];
		navInput.navAlignMode = buf[5];
    	navInput.navAlignTime = (OS_U16)buf[6] | ((OS_U16)buf[7] << 8);
        //LoadSafeArea(buf + 3);	
		InitSafeAreaFromMsn(buf); 
        
		// if(ptCount){
			g_route_data.num_rows = ptCount;
			g_route_data.num_columns = 11;

			double *pData = NULL;
			tx_byte_allocate(&byte_pool_0, (void**)&pData, 
							ptCount * 11 * sizeof(double), TX_NO_WAIT);
			if(pData == NULL) return;

			g_route_data.p_route_data = pData;
		// }
        break;
    case 1:		// 第1、2、3包，传具体航点信息，每一包航点信息最多只能包括12个航点，总共最多就是30个航点（还是36个航点之类）
    case 2:		// 每个航点是17个字节，包含经纬高、dir、t、type、speed
    case 3:
        if (ptCount == 0) return;
        memcpy(ptBuffer + (buf[0] - 1) * 12 * 17, buf + 1, 12 * 17);
        if (buf[0] * 12 >= ptCount)
        {
			//RoutePointIn rp[AUTO_MSN_PT_MAX_COUNT];
			//MSN_CMD tempPt[AUTO_MSN_PT_MAX_COUNT];
			short dir_of_wp0;
			for (int i = 0; i < ptCount; i++)
			{
				int lon,lat;
				short alt,dir,radis;
				unsigned short speed;
				unsigned char type;
				memcpy(&lon,	ptBuffer + 17 * i + 0, 4);	
				memcpy(&lat,	ptBuffer + 17 * i + 4, 4);
				memcpy(&alt,	ptBuffer + 17 * i + 8, 2);
				memcpy(&dir,	ptBuffer + 17 * i + 10, 2);
				memcpy(&radis,	ptBuffer + 17 * i + 12, 2);
				memcpy(&type,	ptBuffer + 17 * i + 14, 1);
				memcpy(&speed,	ptBuffer + 17 * i + 15, 2);

				g_route_data.p_route_data[i * 11 + 0] = i;          // 
				g_route_data.p_route_data[i * 11 + 1] = lon * 1e-7; // 
				g_route_data.p_route_data[i * 11 + 2] = lat * 1e-7; // 
				g_route_data.p_route_data[i * 11 + 3] = alt;
				g_route_data.p_route_data[i * 11 + 4] = type;		//
				g_route_data.p_route_data[i * 11 + 5] = 0;			//
				g_route_data.p_route_data[i * 11 + 6] = 0;			//
				g_route_data.p_route_data[i * 11 + 7] = speed;	    //
				g_route_data.p_route_data[i * 11 + 8] = dir*0.01;	//
				g_route_data.p_route_data[i * 11 + 9] = radis;		//è½?å¼?åŠå¾„
				g_route_data.p_route_data[i * 11 + 10]= 500.00;	    //æŽ¥æ”¶åŠå¾„
				
				if(i==0)
				{
					dir_of_wp0=dir;
					g_route_data.p_route_data[10]=120;
				}
			}
			// updateRP(Arp, ptCount);	// 014 ?????
			navInput.InitLon  = g_route_data.p_route_data[1];
			navInput.InitLat  = g_route_data.p_route_data[2];
			navInput.InitHigh = g_route_data.p_route_data[3];
			navInput.InitYaw  = dir_of_wp0*0.01;

			g_initial_data.missile_ID		= 0;
			g_initial_data.longitude_launch = g_route_data.p_route_data[1];
			g_initial_data.latitude_launch  = g_route_data.p_route_data[2];
			g_initial_data.height_launch    = g_route_data.p_route_data[3];
			g_initial_data.initial_parameter1 = 15;//é¢„ç•™åˆå?‹å‚æ•?1ï¼Œä¾‹å¦‚å‘å°„ç‚¹æ¸©åº¦ç­‰ï¼Œç”¨äºŽä¼°ç®— å£°é€Ÿã€å¤§æ°”ç­‰æ¨¡åž‹
			g_initial_data.initial_parameter2 = 0;//é¢„ç•™åˆå?‹å‚æ•?2ï¼Œé?žè?Œä»¿çœŸæ¨¡å¼? 
			g_initial_data.launch_time		= 0;                //å‘å°„æ—¶é—´
			g_initial_data.lauch_azimuth	= dir_of_wp0 * 0.01;//å‘å°„æ–¹ä½è§?
			g_initial_data.lauch_pitch		= 12;               //å‘å°„ä¿?ä»°è??
			g_initial_data.lauch_booster_pitch = 32;//åŠ©æŽ¨å™¨ä¿¯ä»°è??
			g_initial_data.climb_ktheta_enc = 9.0;  //åˆå?‹æ?µçˆ¬å‡è??
			g_initial_data.cruise_ktheta_enc = 3.5; //èµ·é?žå®ŒæˆåŽå¹³é?žæ”»è§?

			SETDATA(pDataPoolFly, "DataLon", (g_route_data.p_route_data[1] * 1e7), OS_S32); 
			SETDATA(pDataPoolFly, "DataLat", (g_route_data.p_route_data[2] * 1e7), OS_S32); 
			SETDATA(pDataPoolFly, "DataHigh", g_route_data.p_route_data[3], 	   OS_S16);
			SETDATA(pDataPoolFly, "DataDir", (OS_U16)dir_of_wp0, OS_U16); 
			SETDATA(pDataPoolMsn,   "msnDevID", selfID, OS_U8);
			SETDATA(pDataPoolMsn,   "msnGrpID", groupID, OS_U8);
        }
        break;
    default:
        break;
    }
   
    /*
    MSN_CMD tempPt1[] =
	{
        {1,0,113.602055, 34.957315, 5, 0,0,0, 0, 0,0},
		{1,0,113.621106, 35.129102, 1600.0, 190,0,0, 1, 1,0},
		{1,0,113.766867, 35.301569, 1600.0, 190,0,0, 90, 90,0},
		{1,0,114.011746, 35.008171, 1600.0, 190,0,0, 90, 90,0},
	};
    */
        	
    return;
}

void LoadAutoMsnPt()
{
	MSN_CMD tempPt[] =
	{
		{1,1,109.2335861, 38.5227191, 1550.0, 40,0,0, 140, 0,0},
		{1,1,109.2095525, 38.5128022, 1650.0, 40,0,0, 320, 0,0},
		{1,1,109.1864984, 38.5340795, 1650.0, 35,0,0, 320, 0,0},
		{1,1,109.2118127, 38.5440592, 1750.0, 40,0,0, 140, 0,0},
		{1,1,109.2335861, 38.5227191, 1750.0, 50,0,0, 140, 0,0},
		{1,1,109.2095525, 38.5128022, 1750.0, 40,0,0, 320, 0,0},
		{1,1,109.1962173, 38.5251669, 1750.0, 37,0,0, 320, 0,0},
		{2,0,109.1990802, 38.5402305, 1750.0, 37,0,-600, 0, 100,0},
		{1,0,109.2335861, 38.5227191, 1750.0, 40,0,0, 140, 0,0},
		{1,0,109.2095525, 38.5128022, 1750.0, 40,0,0, 320, 0,0},
		{1,0,109.2040527, 38.5178861, 1750.0, 40,0,0, 320, 0,0},
		{2,0,109.2072170, 38.5321338, 1700.0, 40,0, 700,0, 130,0},
		{2,0,109.2072170, 38.5321338, 1620.0, 40,0, 700,0, -80,0},
		{2,0,109.2072170, 38.5321338, 1550.0, 37,0, 550,0, -80,0},
		{1,0,109.1869504, 38.5336402, 1470.0, 40,0,0, 320, 0,0},
		{1,0,109.2002103, 38.5530974, 1470.0, 35,0,0, 50,  0,0}
	};
	memcpy(autoMsnPt, tempPt, sizeof(tempPt));
}

double inCricleDist[40] = {0};
double fullCricleYaw[5] = {0};

//判断是否进入盘旋圈（连续20次距离差<20米）
static OS_U8 JudgeInCricle(double distance, double r)
{
	memset(fullCricleYaw, 0, sizeof(fullCricleYaw));
	//连续判断20秒, 距离与r相差不超过20米认为进圈
	int judgeCount = sizeof(inCricleDist)/sizeof(double);
	double judgeR = 20;
	//调用时间是0.5秒，因此需要计算40轮
	for(int i=judgeCount - 1;i>0;i--)
	{
		inCricleDist[i] = inCricleDist[i-1];
	}
	inCricleDist[0] = distance - fabs(r);

	int mark = 1;
	for(int i=0;i<judgeCount;i++)
	{
		if(fabs(inCricleDist[i]) > judgeR)
		{
			mark = 0;
			break;
		}
	}
	return mark;
}

//判断是否完成完整盘旋
static OS_U8 JudgeFullCricle(double yaw1, double yaw2)
{
	memset(inCricleDist, 0, sizeof(inCricleDist));
	//避免偏航角在（+-180°位置情况)
	if(yaw2 > 175 )
	{
		yaw2-=45;
		yaw1-=45;
		if(yaw1 < -180)
		{
			yaw1 += 360;
		}
	}
	else if(yaw2 < -175)
	{
		yaw2 += 45;
		yaw1 += 45;
		if(yaw1 > 180)
		{
			yaw1 -= 360;
		}
	}
	int judgeCount = sizeof(fullCricleYaw)/sizeof(double);
	double judgeAngle = 8;
	for(int i=judgeCount - 1;i>0;i--)
	{
		fullCricleYaw[i] = fullCricleYaw[i-1];
	}
	fullCricleYaw[0] = yaw1 - yaw2;
	int mark = 1;
	for(int i=0;i<judgeCount;i++)
	{
		if(fabs(fullCricleYaw[i]) > judgeAngle)
		{
			mark = 0;
			break;
		}
	}
	return mark;

}


//----------------------------计算距离函数--------------------------------
// 地球半径（单位：米）
#define EARTH_RADIUS 6371000.0

// 将角度转换为弧度
double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// 将弧度转换为角度
double radians_to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}
// 计算两点之间的球面距离（米）
double haversine_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = degrees_to_radians(lat2 - lat1);
    double dlon = degrees_to_radians(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(degrees_to_radians(lat1)) * cos(degrees_to_radians(lat2)) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c;
}

//根据当前点、方向、距离计算下一点坐标
void calcNextPt(double curLon, double curLat, double dir, double distance, double *outLat, double *outLon)
{
    double guideLat = asin(sin(curLat / d2r) * cos(distance / EARTH_RADIUS) + cos(curLat / d2r) * sin(distance / EARTH_RADIUS) * cos(dir / d2r));
    guideLat *= d2r;
    double guideLon = curLon / d2r + atan2(sin(dir / d2r) * sin(distance / EARTH_RADIUS) * cos(curLat / d2r),
                cos(distance / EARTH_RADIUS) - sin(curLat / d2r) * sin(guideLat / d2r));
    guideLon *= d2r;
    
    
   /* double targetLat = asin(sin(curLatRad) * cos(dist / earthRadis) +
            cos(curLatRad) * sin(dist / earthRadis) * cos(curTrackRad));
	double targetLon = curLonRad + atan2(sin(curTrackRad) * sin(dist / earthRadis) * cos(curLatRad),
            cos(dist / earthRadis) - sin(curLatRad) * sin(targetLat));
	double targetHigh = pInput->AttackHigh + 250;
    */
    
    *outLat = guideLat;
    *outLon = guideLon;
}

double toRadians(double degree)
{
    return degree / d2r;
}

//计算方位角
double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = toRadians(lon2 - lon1);
    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double bearing = atan2(y, x);

    // 将弧度转换为度
    bearing = bearing * d2r;

    // 确保方位角在0到360度之间
    bearing = fmod((bearing + 360.0), 360.0);

    return bearing;
}