/*
 * modGps.h
 *
 *  Created on: 2021楠烇拷10閺堬拷16閺冿拷
 *      Author: QL
 */

#ifndef SRC_MODGPS_H_
#define SRC_MODGPS_H_


#include "../support/os_framework.h"
#include "../core/BusInteract.h"
#include "../nav/nav2.h"

typedef struct {
	int   YEAR;
	int   MONTH;
	int   DAY;
	int   HOUR;
	int   MINITE;
	int   SECOND;
	int   MS;
} TIME_YMD_tt;

typedef struct 
{
	char   head[10];
	int    posType;	//定位类型 1是单点定位
	double alt;
	double lat;
	double lon;
	double vn;
	double vs;
	double ve;
	TIME_YMD_tt TIME_YMD;
	int    TrackStart1;	//跟踪星数主天线
	int    TrackStart2;	//跟踪星数副天线
	double pdop;
	double hdop;
	double gdop;
	int heading_type;	//航向类型，1为航向有效
	double heading;	//航向值
	double track;//航迹角
	double pitch;//gps计算的俯仰角
	double roll;//gps计算的滚转角
}GNSS_RECV_422;

extern OS_U8 ephUpdateState;
/*




extern OS_U8 curGetEph;
extern OS_U8 curSetEph;
extern double flightTrack;

extern NAV_INPUT_GPS  GNSS_INPUT_Data2;
extern GNSS_RECV_422  gnss_recv_422;

extern OS_U8 InitGpsHardware();

extern OS_U32 GpsCmdHandler(STRU_422_MSG_INFO * frame);

extern OS_U16 ChkGpsEphFrame(OS_MEM* pmData);

*/
extern OS_U16 ChkGpsFrame(OS_MEM* pmData);
extern OS_U32 GpsRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 StartGPS();
extern NAV_INPUT_GPS  nav_input_gps;
extern GNSS_RECV_422  gnss_recv_422;
#endif /* SRC_MODGPS_H_ */
