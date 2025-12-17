/********************************************************************************/
/*Write By: Fig                                                                 */
/********************************************************************************/
#ifndef _ROUTE_POINT_H_
#define _ROUTE_POINT_H_

#define RP_MAX_NUMBER 100

#define RP_TW_0              0   //起飞点
#define RP_TW_NORMAL         1   //普通飞行航点
#define RP_TW_HOVER          2   //盘旋点
#define RP_TW_GUIDANCE       3   //末制导攻击点
#define RP_TW_FEINT          4   //佯攻点
#define RP_TW_RETURN         5   //返航开伞点
#define RP_TW_LAND           9   //着陆点
#define RP_TW_DUBINS         10  //杜宾斯指点

typedef union
{
	float         fval;
	unsigned int  uival;
	int           ival;
	unsigned char cval[4];
	unsigned short sval[2];
}RT_Union_Float;

#pragma pack(1)
typedef struct{
	int sn; 			//点号 0-固定为发射点 其他-为规划航路点
	double lon;
	double lat;
	int h; //高度m 
	int w; //特征字
	int t; //到达时间 秒
	float V_cmd; //飞行速度指令
	float outTrack;//切出角度
	float hover_radis;//盘旋半径
	float AttackAngle; //打击角度
	unsigned char if_airspeed_used; //是否启用空速控制
	unsigned char if_GuideFlight; //是否指点
}RoutePoint;
#pragma pack()

extern int RP_NUMBER;
extern int RP_NUMBER_Down;


#endif

//
