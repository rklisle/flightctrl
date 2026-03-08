/*
 * modMEMS202.c
 *
 *  Created on: 2024年4月13日
 *      Author: lenovo
 */


/*
 * modIMU.c
 *
 *  Created on: 2021年10月16日
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
	// buffLoop[RT_IMU].lenExtern = 0;	//若帧中表示长度的字段并非完整帧的长度，则额外长度为lenExtern（如排除帧头校验等）
	// buffLoop[RT_IMU].head = 0;
	// buffLoop[RT_IMU].tail = 0;
	// buffLoop[RT_IMU].lenPos = 0;	//同步头后第n个字节为长度
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
	//pmData[1]开始为标准协议格式，pmData[0]为rtIndex。
	//pmData[3]在标准格式协议中以1字节表示有效数据长度
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
	{//校验和不通过
		errorCount32++;
		OS_U8 errorCount = (errorCount32 & 0xFF);
		SETDATA(pDataPoolImu, "imuChkEr", errorCount,OS_U8);//e-3转e-2
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
static OS_U8 SaveImuInDataPool(STRU_IMU_INFO* buf)
{
	 //上电后为准备状态；对准前以及对准过程中不响应转导航指令，转导航后不再响应对准指令
	
    int navState = buf->dataEffective;  //0x00准备       0x20对准中     0x3F对准完成      0x2F对准失败（奇异角、或对准过程中出现较大幅度晃动）
	                                      //0x60组合导航模式       0x64纯惯性导航模式     
	
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
        
    SETDATA(pDataPoolImu, "gpsLon", gpslon * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "gpsLat", gpslat * 1e7,	OS_S32);
    SETDATA(pDataPoolImu, "gpsAlt", gpsalt ,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVn", gpsvn * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVs", gpsvs * 1e2,	OS_S16);
    SETDATA(pDataPoolImu, "gpsVe", gpsve * 1e2,	OS_S16);
    SETDATA(pDataPoolImu,	"dirEffec",	DirMar,	OS_U8);//GPS航向有效标志
    year = year - 2000;
    SETDATA(pDataPoolImu, "gpsYear", year,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMonth", month, OS_U8);
    SETDATA(pDataPoolImu, "gpsDay", day,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsHour", hour,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMinit", minite,OS_U8);
    SETDATA(pDataPoolImu, "gpsSec", second,	 OS_U8);
    SETDATA(pDataPoolImu, "gpsMSec", ms*0.1, OS_U8);
    SETDATA(pDataPoolImu, "gpsDir", gpsdir * 100,  OS_U16);
// 这个数据NAV没有,然而整个程序也没用到"gpsScCnt"，所以忽略它
    SETDATA(pDataPoolImu, "gpsScCnt", satCount,  OS_U8); 

    if(g_DeviceState.hilCountDown > 0 && hilInput.useNav == 0)
    {
    }
    else
    {
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
				//新增判断导航状态准备中且航向有效标志有效
				if((navState == 0) && (DirMar == 1))
				{
					float navdirmid;
					navdirmid = gpsdir + 180;
					if(navdirmid > 360)
						navdirmid = navdirmid - 360;
					SETDATA(pDataPoolImu, "navDir", navdirmid * 1e2,	OS_U16);
					SETDATA(pDataPoolImu, "navLon", gpslon * 1e7,	OS_S32);
					SETDATA(pDataPoolImu, "navLat", gpslat * 1e7,	OS_S32);
					SETDATA(pDataPoolImu, "navHigh", gpsalt,	OS_FLOAT);
				}
    }
	return 0;
}

/***********************************************************
 * 函数名称:ImuRtHandler()
 * 函数功能: 惯组总线处理函数，本型号智能控制器接收惯组发出的指令:
 * 			1.惯组定时发送帧 0x93		:(1)存数据池 (2)依据falsh数据进行数据处理 (3)水平计算 (4)再存数据池
 * 参考资料: <TXII-Y1 422箭上通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ImuRtHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case 0x03://光纤惯阻IMU定时发送帧(5ms)
		{
			//保存原始输入到数据池
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
	case CMD_LAUNCH_REQ:	//半实物模式，通过遥测口发送的发射指令
		break;
	default:
		break;
	}
    g_DeviceState.imuCountDown = 200;

	return 0;
}

/***********************************************************
 * 函数名称:ImuCmdHandler()
 * 函数功能: 惯组指令处理函数，本型号智能控制器接收地面发出的指令:
 * 			1.惯组数据测量请求0x12	:由于惯组加电即传数，所以直接读数据池返回
 * 			2.水平计算0xE0			:水平计算需要3分钟，因此本指令指明是否需要重新做水平计算、或返回上次计算结果
 * 								:(1)重新做水平计算，立刻返回全0,待水平计算完成后再次反馈地面计算结果
 * 								:(2)不重新做水平计算，返回上次水平计算结果，从数据池中取
 * 			3.转导航0xE2			:设定状态机状态为导航模式，并返回地面数据池中的数据
 * 参考资料: <TXII-Y1 箭地通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ImuCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
		case CMD_HOR_CALC_REQ://对准请求 分水平对准和垂直对准两种模式
		{
			ToImuFocus();
		}
		break;
		case CMD_TO_NAV_REQ: //启动组合导航请求
		{
			ToImuNav();
			g_DeviceState.workStage |= DOM_NAVON;
		}
		break;
		case CMD_POLAR_TEST_REQ://极性测试
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
