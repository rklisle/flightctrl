/*
 * modGPS.c
 *
 *  Created on: 2021楠烇拷10閺堬拷16閺冿拷
 *      Author: QL
 */


#include <stdint.h>
#include "modGps.h"
#include "../core/Telecontrol.h"
#include "../support/os_bufferLoop.h"
#include "../interface/interface_uart.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>



void calculate_utc_time(int gps_week, int seconds_in_week, int *year, int *month, int *day, int *hour, int *minite, int *second);
 
OS_BOOL gpsStarted = FALSE;
OS_U32 gpsStartCount = 200;
OS_U8 curGetEph = EPH_GPS;
OS_U8 curSetEph;
OS_U8 EphDataGPS[10240] = {0};
OS_U16 ephTailGps = 0;
OS_U8 EphDataBD[10240] = {0};
OS_U16  ephTailBd = 0;
static void SaveGpsIntoPool(GNSS_RECV_422 *gpsRecv);
static OS_S32 AnalysisGnssMsg(uint8_t *GNSScomMsg, int msg_len, GNSS_RECV_422* gnss_infor_temp);
GNSS_RECV_422  gnss_recv_422 = {0};
OS_U8 waittingForCheck = 0;
OS_U8 ephUpdateState = 0;

extern const double DTR;


double flightTrack = 361;
OS_U32 GpsRtHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgId = frame->u8MsgID;
	switch(msgId)
	{
	case BUS_GPS_INFO_REPORT: //0xC3 GPS测试结果 来自GPS模块
	{
		AnalysisGnssMsg(frame->au8Data,frame->u16Len, &gnss_recv_422);
        
		if(flightMode == 1)
		{
			nav_input_gps.Status1 = 1;
		}

		if(waittingForCheck != 0)
		{
			waittingForCheck--;
			if(waittingForCheck == 0)
			{
				//下传星历
				if(curGetEph == EPH_GPS)
				{
					CmdResponseHandler(CMD_GET_EPH_RSP, ephTailGps + 1, EphDataGPS);
					ephTailGps = 0;
					curGetEph = 0xFF;
				}
				else if(curGetEph == EPH_BD)
				{
					CmdResponseHandler(CMD_GET_EPH_RSP, ephTailBd + 1, EphDataBD);
					ephTailBd = 0;
					curGetEph = 0xFF;
				}
			}
		}

	}
		break;
	case BUS_GPS_EPH_REPORT:
	{
		EphDataGPS[0] = EPH_GPS;
		memcpy(EphDataGPS + ephTailGps + 1, frame->au8Data, frame->u16Len);
		ephTailGps += frame->u16Len;
		waittingForCheck = 20;
	}
		break;
	case BUS_BD_EPH_REPORT:
	{
		EphDataBD[0] = EPH_BD;
		memcpy(EphDataBD + ephTailBd + 1, frame->au8Data, frame->u16Len);
		ephTailBd += frame->u16Len;
		waittingForCheck = 20;
	}
		break;
	default:
		break;
	}
	return 0;
}

OS_U16 ChkGpsFrame(OS_MEM* pmData)
{
	OS_U16 len = strlen((char *)pmData);
    if(len < 30)
    {
        return 0;
    }
    //gpsStarted = TRUE;
	OS_U8 cmd = BUS_GPS_INFO_REPORT;
	memmove(pmData + 7, pmData + 1, len);
	pmData[1] = 0xEB;
	pmData[2] = 0x90;
	pmData[3] = len & 0xFF;
	pmData[4] = ((len & 0xFF00) >> 16);
	pmData[5] = 0x06;	//Devid 06
	pmData[6] = cmd;
	//不进行校验和检验
	return len + 9;
}

OS_U8 StartGPS()
{
	if(gpsStarted == FALSE)
	{
        if(gpsStartCount > 0)
        {
            gpsStartCount--;
        }
        else
        {
            UART_PutBuff(rtList[RT_GPS_1].chIndex, (unsigned char*)"KSXT 0.1\r\n", sizeof("KSXT 0.1\r\n"));
            //UART_PutBuff(rtList[RT_GPS_1].chIndex, (unsigned char*)"FRESET\r\n", sizeof("FRESET\r\n"));
         
            UART_PutBuff(rtList[RT_GPS_2].chIndex, (unsigned char*)"GPGSA 0.1\r\n", sizeof("GPGSA 0.1\r\n"));
            gpsStartCount = 200;
        }
	}
    else
	{
		static int sendCount = 101;
		if(sendCount % 100 == 1)
		{
			
			sendCount--;
		}
		return 0;
	}
	return 0;
}

static int splitString(char* str,int len, char stoper, char** result, int result_size)
{
	char* token_start = str;

	//scan current string
			//calc how many pair be found

	int  token_idx= 0;
	for(int k=0;k<len;k++)
		{
			//scan reverse
		if((str[k] == ',') || (str[k] == ';'))
			{
			str[k]= '\0';
			result[token_idx] = token_start;
			token_idx++;
				//find, then insert
			token_start = &str[k+1];
			}
		// prevent overflow
		if((token_idx >= result_size) || (str[k] == stoper))
			{
			break;
		}
	}
	return token_idx;
}

static OS_S32 AnalysisGnssMsg(OS_U8 *GNSScomMsg ,int msg_len, GNSS_RECV_422* gnss_infor_temp )
{
	char* result[100];
	int numTokens;
	
	if( GNSScomMsg[0] == '$' || GNSScomMsg[0] == '#')
	{
       // memcpy(gnssmsg,GNSScomMsg,500);
		numTokens = splitString((char *)GNSScomMsg, msg_len, '*', result, sizeof(result)/sizeof(char*));

		memcpy(gnss_infor_temp->head,result[0],strlen(result[0]));
        
		if(memcmp(GNSScomMsg, "$KSXT",strlen("$KSXT")) == 0)
		{
            //UTC时间，周秒计算
			double utctime = atof(result[1]);
          
			gnss_infor_temp->TIME_YMD.YEAR  = (int)(utctime/ 1e10);
			gnss_infor_temp->TIME_YMD.MONTH = ((int)(utctime / 1e8)) % 100;
			gnss_infor_temp->TIME_YMD.DAY   = (fmod((utctime)/ 1e6 , 100));
			gnss_infor_temp->TIME_YMD.HOUR  = (fmod((utctime)/ 1e4 , 100));
			gnss_infor_temp->TIME_YMD.MINITE= (fmod((utctime)/ 1e2 , 100));
			gnss_infor_temp->TIME_YMD.SECOND= (fmod((utctime)/ 1 , 100));
			gnss_infor_temp->TIME_YMD.MS =  fmod(utctime, 1.0) * 10;//毫秒固定只有两位数
			//经度
			gnss_infor_temp->lon = atof(result[2]);
			//纬度
			gnss_infor_temp->lat = atof(result[3]);
			//海拔高
			gnss_infor_temp->alt = atof(result[4]);
			
			//北速
			gnss_infor_temp->vn = atof(result[18])/3.6;
			//天速
			gnss_infor_temp->vs = atof(result[19])/3.6;
			//东速
			gnss_infor_temp->ve = atof(result[17])/3.6;
			
			//定位状态
			gnss_infor_temp->posType = atoi(result[10]);
            
			//航向状态
			gnss_infor_temp->heading_type = atoi(result[11]);
			
			//航向角
			gnss_infor_temp->heading = atof(result[5]);
			//航迹角
			gnss_infor_temp->track = atof(result[7]);
			//俯仰角
			gnss_infor_temp->pitch = atof(result[6]);
			//主天线定位星数
			gnss_infor_temp->TrackStart1 = atoi(result[13]);
			//从天线定位星数
			gnss_infor_temp->TrackStart2 = atoi(result[12]);
            
            nav_input_gps.Status1 = gnss_infor_temp->posType == 0?0:1;//0无效解 对应1为不定位，0定位
            //nav_input_gps.Status2 = gnss_infor_temp->heading_type == 0?0:1;
            
            gpsStarted = TRUE;
		}
        else if(memcmp(GNSScomMsg, "$GNGSA",strlen("$GPGSA")) == 0)
        {
            gnss_infor_temp->pdop = atof(result[15]);
        }
	}
	return 1;
}

#define GPS_EPOCH_YEAR 1980
#define GPS_EPOCH_MONTH 1
#define GPS_EPOCH_DAY 6
#define SECONDS_IN_A_DAY 86400
#define UTC_OFFSET_SECONDS 18
#define UTC_OFFSET_HOURS 8

// Function to calculate UTC time from GPS week and seconds in the week
void calculate_utc_time(int gps_week, int seconds_in_week, int *year, int *month, int *day, int *hour, int *minite, int *second)
{
    // GPS Epoch is January 6, 1980
    struct tm gps_epoch = {0};
    gps_epoch.tm_year = GPS_EPOCH_YEAR - 1900;  // tm_year is years since 1900
    gps_epoch.tm_mon = GPS_EPOCH_MONTH - 1;     // tm_mon is 0-based (0 = January)
    gps_epoch.tm_mday = GPS_EPOCH_DAY;
    
    // Convert GPS epoch to time_t (seconds since 1970-01-01)
    time_t gps_epoch_time = mktime(&gps_epoch);
    
    // Calculate total seconds from GPS epoch
    time_t total_seconds = gps_epoch_time + (gps_week * 7 * SECONDS_IN_A_DAY) + seconds_in_week;

    // Convert total seconds to UTC time
    struct tm *utc_time = gmtime(&total_seconds);
    
    // Print UTC time
    /*
    printf("UTC Time: %04d-%02d-%02d %02d:%02d:%02d\n",
           utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
           utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec);
           */
    *year =  utc_time->tm_year;
    *month = utc_time->tm_mon + 1;
    *day = utc_time->tm_mday;
    *hour = utc_time->tm_hour;
    *minite = utc_time->tm_min;
    *second = utc_time->tm_sec;
}

