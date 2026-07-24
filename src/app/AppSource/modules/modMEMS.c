/*
 * modMEMS202.c
 *
 *  Created on: 2024??4??13??
 *      Author: lenovo
 */


/*
 * modIMU.c
 *
 *  Created on: 2021??10??16??
 *      Author: QL
 */
#include "../StateMachine.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../core/Telecontrol.h"
#include "../interface/interface_uart.h"
#include "../FlightSupport.h"
#include <math.h>
#include "modMEMS.h"
#include "../support/common.h"
#include "../support/os_bufferLoop.h"
#include "modHil.h"
#include "../payload/MsnTime.h"

extern double toDeg(double rad);

#define FOCUS_TIME	(100)//3min+30sec
OS_U16 cmdToIMU = 0x0803;
int toNAV_count = 0;
int toFocus_count = 0;

OS_U8 ImuInit()
{
	SETDATA(pDataPoolImu, "kfWx", 0,	OS_S16);
	SETDATA(pDataPoolImu, "kfWy", 0,	OS_S16);
	SETDATA(pDataPoolImu, "kfWz", 0,	OS_S16);
	SETDATA(pDataPoolImu, "kfAx", 0,	OS_S16);
	SETDATA(pDataPoolImu, "kfAy", 0,	OS_S16);
	SETDATA(pDataPoolImu, "kfAz", 0,	OS_S16);

	// buffLoop[RT_IMU].syncHead_A = 0xEB;
	// buffLoop[RT_IMU].syncHead_B = 0x90;
	// buffLoop[RT_IMU].lenExtern = 0;	//???????????????????????????????????????lenExtern???????????????
	// buffLoop[RT_IMU].head = 0;
	// buffLoop[RT_IMU].tail = 0;
	// buffLoop[RT_IMU].lenPos = 0;	//???????n??????????
	// buffLoop[RT_IMU].fixedLen = 0x7C;
	// buffLoop[RT_IMU].inited = TRUE;
	return 0;
}

OS_U32 errorCount32 = 0;
OS_U16 ChkImuFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[500];
	memcpy(data,pmData, 500);
	//pmData[1]???????????????pmData[0]?rtIndex??
	//pmData[3]???????????????1????????????????
	if(pmData[1] != 0xEB || pmData[2] != 0x90)
	{
		return 0;
	}
    
    OS_U8 temp_crc[2];
    temp_crc[0] = pmData[0x7B];
    temp_crc[1] = pmData[0x7C];
   
    OS_U16 crcValue;
    memcpy(&crcValue, temp_crc, 2);
    
	if(Chk16CRC_U8(pmData + 3, 120, crcValue) == OS_FALSE)
	{//?????????
		errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "imuChkEr", errorCount,OS_U8);//e-3?e-2
		return 0;
	}
	return pmData[3] + 9;
}
OS_S32 luanchLon;
OS_S32 luanchLat;
OS_U32 luanchHigh;
OS_U16 luanchDir;


void ToImuNav()
{
	OS_U8 toImuData[200] = {0};
	toImuData[0] = 0xEB;
	toImuData[1] = 0x90;
	toImuData[2] = 0;
	toImuData[3] = 0;
	toImuData[4] = toNAV_count++;
	toImuData[5] = 0x02;
	Insert16CRC_U8(toImuData + 2, 4, (OS_U16*)((OS_U8 *)(toImuData + 6)));
	// UART_PutBuff(rtList[RT_IMU].chIndex, toImuData, 0x08);
}

OS_U8 ToImuFocus()
{
    OS_S32 luanchLon,luanchLat;
    OS_U16 luanchDir;
    OS_S16 luanchHigh;
   
    GetDataFast(pDataPoolFly, "DataLon", &luanchLon);//
    GetDataFast(pDataPoolFly, "DataLat", &luanchLat);//
    GetDataFast(pDataPoolFly, "DataHigh", &luanchHigh);//
    GetDataFast(pDataPoolFly, "DataDir", &luanchDir);//
    
    double dluanchlon = luanchLon * 1e-7;
    double dluanchlat = luanchLat * 1e-7;
    double dluanchHigh = luanchHigh;
    double dluanchDir = luanchDir * 1e-2;
    
   // dluanchlon = 116.1234567;
   //dluanchlat = 39.7654321;
   // dluanchHigh = 400.1234567;
   // dluanchDir = 42.9876543;
    //dluanchlon = 116;
    //dluanchlat = 39;
    //dluanchHigh = 40;
    //dluanchDir = 0;
    
		OS_U8 toImuData[200] = {0};
    toImuData[0] = 0xEB;
		toImuData[1] = 0x90;
		toImuData[2] = 0x23;
    toImuData[3] = 0;
    toImuData[4] = toFocus_count++;
		toImuData[5] = 0x01;
		toImuData[6] = 1;
		toImuData[7] = 2;
		toImuData[8] = 3;
    memcpy(toImuData + 9, &dluanchlon, 8);
    memcpy(toImuData + 17, &dluanchlat, 8);
    memcpy(toImuData + 25, &dluanchHigh, 8);
    memcpy(toImuData + 33, &dluanchDir, 8);
    
    Insert16CRC_U8(toImuData + 2, 39, (OS_U16*)((OS_U8 *)(toImuData + 41)));
    // UART_PutBuff(rtList[RT_IMU].chIndex, toImuData, 0x2B);
	return 0;
}
extern float fwxhil,fwyhil,fwzhil;
static OS_U8 SaveImuInDataPool(STRU_IMU_INFO* buf)  //014????
{
	 //???????????????????????????????????????????????????????????
	
    int navState = buf->dataEffective;  //0x00???       0x20?????     0x3F??????      0x2F?????????????????????????????????????
	                                      //0x60????????       0x64???????????     
	
    double lon = buf->lon;
    double lat = buf->lat;
    double alt = buf->alt;
    double ve = buf->ve;
    double vn = buf->vn;
    double vs = buf->vs;
    
    double wx = buf->wx;
    double wy = buf->wy;
    double wz = buf->wz;
    double ax = buf->ax;
		double ay = buf->ay;
    double az = buf->az;
    fwxhil = wx;
    fwyhil = wy;
    fwzhil = wz;
    
   
    
    double pitch = buf->pitch ;
    double roll = buf->roll;
    double dir = buf->dir;
    if(dir < 0)
        dir += 360;
    
    int satCount = buf->satCount;
    
    int day = buf->time[2];
    int month =buf->time[1];
    int year = buf->time[0] + 2000;
    
    int minite = buf->time[4];
    int hour = buf->time[3];
    int second = (buf->time[5] + buf->time[6] * 0x100) / 1000;
    int ms = (buf->time[5] + buf->time[6] * 0x100) % 1000;
    
    g_DeviceState.BJTimeSecond = SetSecondByDate(year,month,day,hour,minite,second);
    double gpslon = buf->gpsLon;
    double gpslat = buf->gpsLat;
    double gpsalt = buf->gpsAlt;
    double gpsve = buf->gpsVe;
    double gpsvn = buf->gpsVn;
    double gpsvs = buf->gpsVs;
    double gpsdir = buf->gpsDir;
		int DirMar = buf->DirMarker;
        
    if((g_DeviceState.workStage & DOM_HILSMODE) && hilInput.useNav == 0)
    {
        return 0;
    }

    SETDATA(pDataPoolImu, "gpsLon", gpslon * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "gpsLat", gpslat * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "gpsAlt", gpsalt ,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVn", gpsvn * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVs", gpsvs * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVe", gpsve * 1e2,	OS_S16);
    SETDATA(pDataPoolImu,	"dirEffec",	DirMar,	OS_U8);//GPS???????????
    year = year - 2000;
    SETDATA(pDataPoolImu, "gpsYear", year,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMonth", month, OS_U8);
    SETDATA(pDataPoolImu, "gpsDay", day,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsHour", hour,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMinit", minite,OS_U8);
    SETDATA(pDataPoolImu, "gpsSec", second,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMSec", ms*0.1, OS_U8);
    SETDATA(pDataPoolImu, "gpsDir", gpsdir * 100,  OS_U16);
    SETDATA(pDataPoolImu, "gpsScCnt", satCount,  OS_U8);

    // float imu1,imu2,imu3,imu4,imu5,imu6;
    // imu1 = wy;
    // imu2 = wz;
    // imu3 = wx;
    // imu4 = ay;
    // imu5 = az;
    // imu6 = ax;
    SETDATA(pDataPoolImu, "imuWx", wx,	OS_FLOAT);
    SETDATA(pDataPoolImu, "imuWy", wy,	OS_FLOAT);
    SETDATA(pDataPoolImu, "imuWz", wz,	OS_FLOAT);
    SETDATA(pDataPoolImu, "imuAx", ax,	OS_FLOAT);
    SETDATA(pDataPoolImu, "imuAy", ay,	OS_FLOAT);
    SETDATA(pDataPoolImu, "imuAz", az,	OS_FLOAT);
    
    SETDATA(pDataPoolImu, "navLon", lon * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "navLat", lat * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "navHigh", alt,	    OS_FLOAT);
        
    SETDATA(pDataPoolImu, "navVn", vn * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "navVs", vs * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "navVe", ve * 1e2,	OS_S16);
    
    SETDATA(pDataPoolImu, "navPitch", pitch * 1e2,	OS_S16);			
    SETDATA(pDataPoolImu, "navRoll", roll * 1e2,	OS_S16);       
    SETDATA(pDataPoolImu, "navState", navState,	OS_U8);
    SETDATA(pDataPoolImu, "navDir", dir * 1e2,	OS_U16);

    //??????????????????????????????????
    if((navState == 0) && (DirMar == 1))
    {
        //??0~360deg???-180~180deg????????????????????????
        float navdirmid;
        navdirmid = gpsdir + 180;
        if(navdirmid > 360)
            navdirmid = navdirmid - 360;
        SETDATA(pDataPoolImu, "navDir", navdirmid * 1e2,	OS_U16);
        SETDATA(pDataPoolImu, "navLon", gpslon * 1e7,	OS_S32);
        SETDATA(pDataPoolImu, "navLat", gpslat * 1e7,	OS_S32);
        SETDATA(pDataPoolImu, "navHigh", gpsalt,	OS_FLOAT);
    }
	return 0;
}

/***********************************************************
 * ????????:ImuRtHandler()
 * ????????: ??????????????????????????????????????????????:
 * 			1.????E?????? 0x93		:(1)??????? (2)????falsh?????????????? (3)?????? (4)????????
 * ????????: <TXII-Y1 422???????????>
 * ????:	???Z
 ***********************************************************/
OS_U32 ImuRtHandler(STRU_422_MSG_INFO * frame)  //014????
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case 0x03://???????IMU????????(5ms)
		{
			//???????????????
			SaveImuInDataPool((STRU_IMU_INFO*)frame->au8Data);
            
            //send luanch info to imu
            static OS_U8 startFocus = 2;
            if(startFocus > 0)
            {
                //judge if msn info is updated
                OS_U8 msnID;
                GetDataFast(pDataPoolMsn, "msnDevID", &msnID);
                if(msnID != 0xFF)
                {
                    ToImuFocus();
                    startFocus--;
                }
            }
           
		}
		break;
	case BUS_IMU_SIMU_GPSIMU://
		{
		}
			break;
	case CMD_LAUNCH_REQ:	//??????????????????????????
		break;
	default:
		break;
	}
    g_DeviceState.imuCountDown = 200;

	return 0;
}

/***********************************************************
 * ????????:ImuCmdHandler()
 * ????????: ????????????????????????????????????M???????:
 * 			1.???????????????0x12	:????????????????????????????????
 * 			2.??????0xE0			:?????????3?????????????????????????????????????????????
 * 								:(1)????????????????????0,????????????????????????????
 * 								:(2)?????????????????????????????????????????
 * 			3.?????0xE2			:???????????????????????????????????????
 * ????????: <TXII-Y1 ???????????>
 * ????:	???Z
 ***********************************************************/
OS_U32 ImuCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case CMD_HOR_CALC_REQ://??????? ????????????????????
		{
			ToImuFocus();
		}
		break;
		case CMD_TO_NAV_REQ: //??????????????
		{
			ToImuNav();
			g_DeviceState.workStage |= DOM_NAVON;
		}
		break;
		case CMD_POLAR_TEST_REQ://???????
		{
			//CmdResponseHandler(CMD_POLAR_TEST_RSP, 1, NULL);
		}
		break;
		default:
		break;
	}
	return 0;
}

OS_U8 StartEncpEphToMems202(OS_U8 *data, OS_U16 len)
{
	//ephRecvReadyFlag = TRUE;

	return 0;
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
