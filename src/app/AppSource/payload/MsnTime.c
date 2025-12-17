/*
 * MsnTime.c
 *
 *  Created on: 2023Äê5ÔÂ23ÈÕ
 *      Author: lenovo
 */
#include "MsnTime.h"
#include <time.h>


OS_U32 GetDateBySecond(OS_U64 bjTimeSecond_2000)
{
	struct tm *local;
	time_t t = 946684800;
	time_t bjTimeSecond_1970 = bjTimeSecond_2000 + t;
	local=localtime(&bjTimeSecond_1970);
	local->tm_mon++;
	local->tm_year += 1900;

	SETDATA(pDataPoolSelf, "DateYear", local->tm_year - 1900 , OS_U16);//u16
	SETDATA(pDataPoolSelf, "DateMoth", local->tm_mon - 1, OS_U8);//u8
	SETDATA(pDataPoolSelf, "DateDay", local->tm_mday, OS_U8);//u8
	SETDATA(pDataPoolSelf, "DateHour", local->tm_hour, OS_U8);//u8
	SETDATA(pDataPoolSelf, "DateMini", local->tm_min, OS_U8);//u8
	SETDATA(pDataPoolSelf, "DateSec", local->tm_sec, OS_U8);//u8
	SETDATA(pDataPoolSelf, "DateMS", g_DeviceState.BJTimeMS, OS_U16);//u16
	return 0;
}

OS_U32 SetSecondByDate(OS_U16 year, OS_U8 month, OS_U8 day, OS_U8 hour, OS_U8 minite, OS_U8 second)
{
    if(month == 0)
    {
        month = 1;
    }
    if(day == 0)
    {
        day = 1;
    }
	OS_U64 bjTimeSecond_2000;
	OS_U64 bjTimeSecond_1970;
	time_t t = 946684800;
	struct tm localStruct;
	struct tm *local = &localStruct;
	local->tm_year = year - 1900;
	local->tm_mon = month - 1;
	local->tm_mday = day;
	local->tm_hour = hour;
	local->tm_min = minite;
	local->tm_sec = second;
	bjTimeSecond_1970 = mktime(local);
	bjTimeSecond_2000 = bjTimeSecond_1970 - t;
	return bjTimeSecond_2000;
}


