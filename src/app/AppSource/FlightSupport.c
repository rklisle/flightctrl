/*
 * FlightSupport.c
 *
 *  Created on: 2022Äê3ÔÂ23ÈÕ
 *      Author: Lenovo
 */
#include "FlightSupport.h"
#include "stmToZynq.h"
#include "./modules/modPwrSeqCtl.h"
#include "./modules/modSrvCtl.h"
#include "./modules/modHil.h"
#include "./modules/modEngine.h"
#include "./modules/modFlash.h"
#include "./modules/modNav.h"
#include "./controller/controller.h"
//#include "./comm/CommHandler.h"
#include "./support/common.h"
#include "./core/DataPool.h"
#include "./flight/os_flight_io.h"
#include "./flight/RoutePoint.h"
#include "./interface/interface_power.h"
#include "./mission/mission.h"
#include "math.h"
#include <stdlib.h>
//#include "flight_data.h"
Point safePoints[20];
int airHighArray[1000] = {0};
int safePointCount;

MISSION homeMsn = {0};

#define d2r		(57.29577951308402)
/***********************************************************
 * º¯ÊýÃû³Æ:FlightSeqOutputHandle()
 * º¯Êý¹¦ÄÜ:¸ù¾Ý·É¿ØÔËËãµÄÊ±Ðò½á¹û£¬´¥·¢Ê±Ðò¿ª¹Ø
  ***********************************************************/
static void FlightSeqOutputHandle()
{
	if(flightSeq.umOpen == 1)
	{
		static int waitOneSec = 0;
		//Á¢¿Ì¹Ø±Õ·¢¶¯»ú£¬³ÖÐø·¢ËÍ·¢¶¯»ú×ªËÙµ½µ¡ËÙÖ¸Áî£¬¼°·¢¶¯»úÍ£»úÖ¸Áî
		if(waitOneSec >= 0 && waitOneSec <10000)
		{	
            if(waitOneSec % 4 == 0)
            {
                OS_U8 engineStatus;
                GetDataFast(pDataPoolSelf, "ecuState", &engineStatus);
                if(engineStatus != 0)//0Í£»ú£¬1Æô¶¯ÖÐ£¬2É¢ÈÈ 3¹ÊÕÏ 4ÍÑ»ú 5ÔËÐÐ
                {
                    SendEngineRpm(12000);
                    StopEngine();//¹Ø±Õ·¢¶¯»ú
                }
            }
		}
		//0.2Ãëºó¿ªÉ¡
		if(waitOneSec == 40)//0.2ÃëÊ±¿ªÉ¡
		{
			TrigerSeqWithWidth(TEST_1 + 1, 100);
		}
		
		if(waitOneSec == 200)//1ÃëÊ±±£³Ö¶æ»ú»Ø0ÇÒ½â³ý¿ØÖÆ
		{			
			ServoCtlOnce_6Rudder(0, 0, 0, 0, 0, 0);
		}
		if(waitOneSec == 3000)//15ÃëÊ±¿ªÆøÄÒÇ°
		{
			TrigerSeqWithWidth(TEST_1 + 2, 100);
		}
		if(waitOneSec == 3080)//15.4ÃëÊ±¿ªÆøÄÒºó
		{
			TrigerSeqWithWidth(TEST_1 + 3, 100);
		}
		if(waitOneSec == 4000)//20Ãë¶æ»úÏÂµç
		{
			//¶ÏËÅ·þµç
			PowerOff(DEVICE_BATT_SRV);
		}
		static OS_U8 yijinggesan = 0;
		if(waitOneSec > 4000 && yijinggesan == 0)
		{
			//¿ªÊ¼ÅÐ¶Ï¸îÉ¡
			OS_S16 navVs;
			GetDataFast(pDataPoolImu, "navVs", 	&navVs);
			float fvs = navVs * 0.01;
            if(g_DeviceState.imuCountDown != 0)
            {
                if(fabs(fvs) < 1.0)
                {
                    TrigerSeqWithWidth(TEST_1 + 0, 100);
                    yijinggesan = 1;
                }
            }
            else
            {
                OS_S16 airHigh;
                GetDataFast(pDataPoolSelf, "AirHigh", &airHigh);
                memmove(&airHighArray[1], &airHighArray[0], sizeof(int) * 999);
                airHighArray[0] = airHigh;
                OS_U8 airStable = 1;
                for(int i=0; i<1000; i++)
                {
                    if(fabs(airHigh - airHighArray[i]) > 5)
                    {
                        airStable = 0;
                        break;
                    }                        
                }
                if(airStable == 1)
                {
                    TrigerSeqWithWidth(TEST_1 + 0, 100);
                    yijinggesan = 1;
                }
            }
		}
		else
		{
			waitOneSec++;
		}
		OS_U16 ecuTemp;
		GetDataFast(pDataPoolSelf,  "ecuTemp", 	&ecuTemp);
		if(ecuTemp * 0.1 < 65)
		{
				//¶Ï·¢¶¯»ú24v¹©µç
				PowerOff(DEVICE_BATT_ENGINE);
		}
	}
}

/***********************************************************
 * º¯ÊýÃû³Æ:FlightTMOutputHandle()
 * º¯Êý¹¦ÄÜ:·ÖÎö·É¿ØÔËËãºóµÄÊä³öÊý¾Ý£¬´æÊý¾Ý³Ø
  ***********************************************************/
void FlightTMOutputHandle()
{
	SETDATA(pDataPoolFly,	"pitchCmd",	pOutput->rudderPitchCmd * 100,	OS_S16);	//Æð¿Ø±êÖ¾
	SETDATA(pDataPoolFly,	"rollCmd",	pOutput->rudderRollCmd* 100,	OS_S16);	//ÖÆµ¼¼¶Êý
	SETDATA(pDataPoolFly,	"yawCmd",	pOutput->rudderYawCmd* 100,		OS_S16);	//¿ØÖÆ¼¶Êý
    
	SETDATA(pDataPoolFly,	"dRn",	pOutput->dRn,		OS_S16);	//¿ØÖÆ¼¶Êý
	SETDATA(pDataPoolFly,	"dRu",	pOutput->dRu,		OS_S16);	//¿ØÖÆ¼¶Êý
	SETDATA(pDataPoolFly,	"dRe",	pOutput->dRe,		OS_S16);	//¿ØÖÆ¼¶Êý
	
	SETDATA(pDataPoolFly,	"PitchPre",	pOutput->Pitch_Preset_Angle * 100,		OS_S16);	//¿ØÖÆ¼¶Êý
	SETDATA(pDataPoolFly,	"YawPre",	pOutput->Yaw_Preset_Angle * 100,		OS_S16);	//¿ØÖÆ¼¶Êý
    
	SETDATA(pDataPoolFly,	"fPitchSp",	pOutput->pitch_rate_nT_filterOut * 500,		OS_S16);	//¿ØÖÆ¼¶Êý
	SETDATA(pDataPoolFly,	"fYawSp",	pOutput->yaw_rate_nT_filterOut * 500,		OS_S16);	//¿ØÖÆ¼¶Êý
 
	SETDATA(pDataPoolMsn,	"WP_cur",	pOutput->curPtNo,	OS_U8);//º½µãºÅ

	SETDATA(pDataPoolMsn,	"tarLon",	pOutput->curTargetLon * 1e7, OS_S32 );//Ä¿±êº½µã¾­¶È
	SETDATA(pDataPoolMsn,	"tarLat",	pOutput->curTargetLat * 1e7, OS_S32 );//Ä¿±êº½µãÎ³¶È
	SETDATA(pDataPoolMsn,	"tarAlt",	pOutput->curTargetAlt * 1, OS_S16 );//Ä¿±êº½µã¸ß¶È

  SETDATA(pDataPoolFly,	"adrc_Mx",	pOutput->mx_ESO * 10, OS_S16 );//adrc
	SETDATA(pDataPoolFly,	"gamaCmd",	pOutput->gamaCmd * 1e2, OS_S16 );//¹ö×ª½ÇÖ¸Áî
	SETDATA(pDataPoolFly,	"thetaCmd",	pOutput->varthetaCmd * 1e2, OS_S16 );//¸©Ñö½ÇÖ¸Áî
	SETDATA(pDataPoolFly,	"nycCmd",	pOutput->nycCmd * 1e3, OS_S16 );//¹ýÔØÖ¸Áî
	SETDATA(pDataPoolFly,	"highCmd",	pOutput->heightCmd , OS_FLOAT );///¸ß¶ÈÖ¸Áî
	SETDATA(pDataPoolFly,	"ac_dL",	pOutput->ac_dL * 10, OS_S32 );//´ý·É¾à
	SETDATA(pDataPoolFly,	"ac_dZ",	pOutput->ac_dZ * 10, OS_S16 );//²à±ß¾à
	SETDATA(pDataPoolFly,	"tokenlon",	pOutput->token_long * 1, OS_U8 );//×ÝÏòÁîÅÆ
	SETDATA(pDataPoolFly,	"tokenlat",	pOutput->token_late * 1, OS_U8 );//²àÏòÁîÅÆ
	SETDATA(pDataPoolFly,	"ac_dPsi",	pOutput->ac_dPsi * 100, OS_S16 );//º½Ïò½ÇÆ«²î
	
	SETDATA(pDataPoolFly,	"ac_dR",	pOutput->ac_dR * 10, OS_S16 );//Ô²¹ì¼£²à±ß¾à
	SETDATA(pDataPoolFly,	"thetav",	pOutput->cur_thetav * 10, OS_S16 );//¹ì¼£Çã½Ç
	SETDATA(pDataPoolFly,	"Vcmd",	pOutput->Vcmd * 10, OS_S16 );//ËÙ¶ÈÖ¸Áî
    
	SETDATA(pDataPoolFly,	"nyCmd",	pOutput->nyCmd_Guidance * 100 , OS_S16 );//ËÙ¶ÈÖ¸Áî
	SETDATA(pDataPoolFly,	"nzCmd",	pOutput->nzCmd_Guidance * 100, OS_S16 );//ËÙ¶ÈÖ¸Áî

	SETDATA(pDataPoolFly,	"DbsLen",	pOutput->Dubins_length , OS_FLOAT );//ËÙ¶ÈÖ¸Áî
	SETDATA(pDataPoolFly,	"ADRC",	    pOutput->fduox_ADRC * 100, OS_S16 );//ËÙ¶ÈÖ¸Áî
	//ÔÝÊ±½èÓÃµ¼º½°æÊý¾Ý		
  SETDATA(pDataPoolNav, "navWx2", pOutput->arp_ins,	OS_FLOAT);//¹ß²â¹¥½Ç
	SETDATA(pDataPoolNav, "navWy2", pOutput->beta_ins,	OS_FLOAT);//¹ß²â²à»¬½Ç
	
	SETDATA(pDataPoolNav, "navWz2", pOutput->MaxRpm,	OS_FLOAT);//×î´ó×ªËÙ
	SETDATA(pDataPoolNav, "navAx2", pOutput->DFT_freq_max,	OS_FLOAT);//±æÊ¶ÔË¶¯ÆµÂÊ
    
	if(pOutput->on_takeoff == 1)
	{
			SETDATA(pDataPoolSelf,	"RecvLunc",	0xEE , OS_U8 );//ËÙ¶ÈÖ¸Áî
	}
}

void FlightSrvOutputHandle()
{
	if((DOM_AUTOMATIC & g_DeviceState.workStage) && flightSeq.luanched == 1)
	{
		//¼ÇÂ¼Ô­Ê¼ÊýÖµ
		SETDATA(pDataPoolSrv, "Sr1Cmd", pOutput->rudder1Cmd * 100, OS_S16);
		SETDATA(pDataPoolSrv, "Sr2Cmd", pOutput->rudder2Cmd * 100, OS_S16);
		SETDATA(pDataPoolSrv, "Sr3Cmd", pOutput->rudder3Cmd * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr4Cmd", pOutput->rudder4Cmd * 100, OS_S16);
		SETDATA(pDataPoolSrv, "Sr5Cmd", pOutput->rudder5Cmd * 100, OS_S16);
		SETDATA(pDataPoolSrv, "Sr6Cmd", pOutput->rudder6Cmd * 100, OS_S16);

		//×ª»»ÎªÎïÀí¶æÊýÖµ
        double rudder1 = pOutput->rudder1Cmd ; // roll left
        double rudder2 = pOutput->rudder2Cmd ; // roll right
        double rudder3 = pOutput->rudder3Cmd ; // pitch left
        double rudder4 = pOutput->rudder4Cmd ; // pitch right
        double rudder5 = pOutput->rudder5Cmd ; // yaw left
        double rudder6 = pOutput->rudder6Cmd ; // yaw right
        
        //test
        //rudder1 = 0;
        //rudder3 = 0;
        //rudder5 = 0;
        //rudder6 = 0;

		//¿ØÖÆÒÑ¿¼ÂÇ¶æ°²×°·½Ê½
		ServoCtlOnce_6Rudder(rudder1, rudder2, rudder3, rudder4, rudder5, rudder6);
	}
}
OS_U8 OutSafeCount = 0;
OS_U8 OutSafeArea = 0;
void FlightEngineOutputHandle()
{
	if(flightSeq.luanched == 1 && ((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC))
	{
		if(OutSafeArea)
		{
			//CurEngineRpm = 18000;
		}
		else
		{
			CurEngineRpm = pOutput->engineSet;
			if(CurEngineRpm == 0)
			{
				int a = 0;
			}
			
		}
		SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
	}
    else if(flightSeq.luanched == 0 && ((g_DeviceState.workStage & DOM_AUTOMATIC) == DOM_AUTOMATIC))
    {
        CurEngineRpm = 50500;
        SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
    }
}

/***********************************************************
 * º¯ÊýÃû³Æ:FlightOutputHandle()
 * º¯Êý¹¦ÄÜ:µ÷ÓÃ·É¿ØÖ÷ÔËËãº¯Êýºó£¬´¦Àí·É¿ØÖ÷º¯ÊýÊä³ö£¬ÕâÐ©Êä³ö°üÀ¨:
 * 1.Ê±ÐòÐÅÏ¢£¬ÓÃÒÔ´¥·¢Ê±Ðò£¬°üÀ¨·ÖÀë¡¢»ð¼ýµã»ð¡¢¿ªÉ¡µÈÐèÇó
 * 2.Ò£²âÐÅÏ¢£¬ÓÃÒÔÏÂ´«µØÃæ
  ***********************************************************/
void FlightOutputHandle()
{
	if(DOM_AUTOMATIC & g_DeviceState.workStage)
	{		
		FlightSrvOutputHandle();
		FlightEngineOutputHandle();
		FlightTMOutputHandle();
		JudgeHomeward();
	}
	FlightSeqOutputHandle();
	CalcXYZ();
}

/***********************************************************
 * º¯ÊýÃû³Æ:FlightInputGenerate()
 * º¯Êý¹¦ÄÜ:µ÷ÓÃ·É¿ØÖ÷ÔËËãº¯ÊýÇ°£¬×¼±¸¸ø·É¿ØµÄÊäÈëÊý¾Ý£¬Ö÷Òª°üÀ¨£º
 * 1.¹ß×éÊý¾Ý
 * 2.GPSÊý¾Ý
 * 3.Ê±¼ä(double,µ¥Î»Ãë,¾«È·µ½0.001)
 * µ±µ±Ç°ÎªÄ£·ÉÄ£Ê½Ê±£¬¹ß×é¡¢GPSÊý¾Ý¾ùÀ´×ÔÓÚflash
  ***********************************************************/
double curAirSpd;
void FlightInputGenerate()
{
		if((g_DeviceState.workStage & DOM_HILSMODE) == DOM_HILSMODE)
		{
			//°ëÊµÎï·ÂÕæÄ£Ê½
			g_DeviceState.currTime = GetCurTime()-g_DeviceState.flightStartTime;
			HilFlightStage();
		}
		else//ÕýÊ½·ÉÐÐÄ£Ê½
		{
			g_DeviceState.currTime = GetCurTime()-g_DeviceState.flightStartTime;        
			float tempf;
			GetDataFast(pDataPoolImu, "imuWx",	&tempf); pInput->wx = tempf;
			GetDataFast(pDataPoolImu, "imuWy",	&tempf); pInput->wy = tempf;
			GetDataFast(pDataPoolImu, "imuWz", 	&tempf); pInput->wz = tempf;
			GetDataFast(pDataPoolImu, "imuAx", 	&tempf); pInput->ax = tempf;
			GetDataFast(pDataPoolImu, "imuAy", 	&tempf); pInput->ay = tempf;
			GetDataFast(pDataPoolImu, "imuAz", 	&tempf); pInput->az = tempf;

			OS_S32 temps32;
			OS_S16 temps16;
			GetDataFast(pDataPoolImu, "navLon", 	&temps32);pInput->navLon = temps32 * 1e-7;
			GetDataFast(pDataPoolImu, "navLat", 	&temps32);pInput->navLat = temps32 * 1e-7;
			GetDataFast(pDataPoolImu, "navHigh", 	&tempf);pInput->navHigh = tempf;
					
					//test
					//SETDATA(pDataPoolImu, "abc", tempf * 10,	OS_U16);
					//OS_S16 temp16Alt;
					//GetDataFast(pDataPoolImu, "abc", 	&temp16Alt);
					//pInput->navHigh = temp16Alt * 0.1;
					//test end
					
			GetDataFast(pDataPoolImu, "navVn", 		&temps16);pInput->navVn = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navVs", 		&temps16);pInput->navVs = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navVe", 		&temps16);pInput->navVe = temps16 * 0.01;
			
			OS_U16 tempu16; //10-19ÐÞ¸ÄOS_S16ÎªOS_U16
			GetDataFast(pDataPoolImu, "navPitch", 	&temps16);pInput->pitch = temps16 * 0.01;
			GetDataFast(pDataPoolImu, "navDir", 	&tempu16);pInput->yaw = tempu16 * 0.01;
			GetDataFast(pDataPoolImu, "navRoll", 	&temps16);pInput->roll = temps16 * 0.01;
			pInput->yaw = -pInput->yaw; //±±Æ«¶«×ª±±Æ«Î÷
			if(pInput->yaw > 180)
				pInput->yaw -= 360;
			if(pInput->yaw < -180)
				pInput->yaw += 360;
			
			pInput->Luanched = flightSeq.luanched;
			
			double V   = sqrt(pow(pInput->navVn,2) + pow(pInput->navVs,2) + pow(pInput->navVe,2));   // µØËÙ¼ÆËã
			//¿ÕËÙ¾ÍÊÇµØËÙ£¬¸ßËÙ·É»úµÄ¿ÕËÙÃ»ÓÃ
			pInput->airSpd = V;
					
			SETDATA(pDataPoolSelf, "GrdSpd", V * 10, OS_S16);//
			
			OS_S16 srv1,srv2;
			GetDataFast(pDataPoolSrv, "Sr1Read", &srv1);
			GetDataFast(pDataPoolSrv, "Sr2Read", &srv2);
			pInput->DD1 = srv1 * 0.01;
			pInput->DD2 = srv2 * 0.01;
			if(g_DeviceState.srvCountDown == 0)
			{
					pInput->DD1 = pOutput->rudder1Cmd;
					pInput->DD2 = pOutput->rudder2Cmd;
			}				
			pInput->DD5 = 0;
			pInput->DD6 = 0;
		}
		//ÉèÖÃÊÓÏß½ÇºÍËø¶¨×´Ì¬
		GetDataFast(pDataPoolFly,	"sctLock",	 &(pInput->scoutLocked));
		OS_S16 tempss16;
		GetDataFast(pDataPoolFly,	"vPitchSp",	 &tempss16);
		pInput->scoutPitchSpd = (double)tempss16 * 0.002 / 57.3;
		GetDataFast(pDataPoolFly,	"vYawSp",	 &tempss16);
		pInput->scoutYawSpd = (double)tempss16 * 0.002 / 57.3;	
		OS_U16 fuel_n;
		GetDataFast(pDataPoolSelf,	"fuelRate",	 &fuel_n);
		pInput->mass_fuel = (double)fuel_n;
}
#define d2r		(57.29577951308402)

/***********************************************************
 * º¯ÊýÃû³Æ:DoFlightRun()
 * º¯Êý¹¦ÄÜ:µ÷ÓÃ·É¿ØÖ÷ÔËËãº¯Êý£¬½öµ±·¢Éäºó½øÈëµ÷ÓÃ
  ***********************************************************/
void DoFlightRun()
{
	if((DOM_AUTOMATIC ) & g_DeviceState.workStage)
	{
		FlightRun(pInput, pOutput);
		pInput->Msn_updatesig = 0;
		
	}
}

OS_U8 InitSafeArea(OS_U8* buf)
{
	OS_S32 lon,lat;
	safePointCount = buf[0];// Î§À¸µãÊý
	for(int i=0;i<safePointCount;i++)
	{
		lon = *((OS_S32*)(buf+ 2 + i*8));
		lat = *((OS_S32*)(buf+ 6 + i*8));
        safePoints[i].x = lon * 1e-7;   // ¾­¶È 
        safePoints[i].y = lat * 1e-7; 	// Î³¶È 
	}
    
	return 0;
    
}

// ¼ÆËãÁ½¸öÏòÁ¿µÄµã»ý
double dotProduct(Point a, Point b) {
    return a.x * b.x + a.y * b.y;
}

// ¼ÆËãÏòÁ¿µÄÄ£
double vectorMagnitude(Point a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

// ¼ÆËãÈý¸öµãÖ®¼äµÄ¼Ð½Ç
double calculateAngle(Point p1, Point p2, Point p3)
{
    // ¼ÆËãÏòÁ¿P1P3ºÍP2P3
    Point vectorP1P3 = {p3.x - p1.x, p3.y - p1.y};
    Point vectorP2P3 = {p3.x - p2.x, p3.y - p2.y};

    // ¼ÆËãÏòÁ¿µÄµã»ý
    double dot = dotProduct(vectorP1P3, vectorP2P3);

    // ¼ÆËãÏòÁ¿µÄÄ£
    double magnitudeP1P3 = vectorMagnitude(vectorP1P3);
    double magnitudeP2P3 = vectorMagnitude(vectorP2P3);

    // ¼ÆËã¼Ð½ÇµÄÓàÏÒÖµ
    double cosTheta = dot / (magnitudeP1P3 * magnitudeP2P3);

    // ¼ÆËã²¢·µ»Ø¼Ð½Ç£¨ÒÔ»¡¶ÈÎªµ¥Î»£©
    return acos(cosTheta);
}

OS_BOOL JudgeInSafe2(double lon, double lat)
{
	Point curPt;
	curPt.x = lon;
	curPt.y = lat;
	double angle[100];
	int i;
	for(i=0; i<safePointCount-1; i++)
	{
		angle[i] = calculateAngle(safePoints[i], safePoints[i+1], curPt);
	}
	angle[i] = calculateAngle(safePoints[i], safePoints[0], curPt);

	double allAngle = 0;
	for(i=0;i<safePointCount;i++)
	{
		allAngle += angle[i];
	}
	OS_BOOL is_inside;
	if(allAngle> 2*3.1 && allAngle < 2*3.2)//2pai
	{
		is_inside = TRUE;
	}
	else
	{
		is_inside = FALSE;
	}
	return is_inside;
}

extern OS_U8 RecoverMark;
OS_U8 JudgeHomeward()
{
	//Ã¿ÃëÅÐ¶ÏÒ»´ÎÊÇ·ñ³ö°²È«Çø·µº½ºÍÈÎÎñ»úÊ§Áª
	if(g_DeviceState.CurrTick % 200 != 0)
		return 1;

	//ÅÐ¶Ï°²È«ÇøÊ»³ö
	if(g_DeviceState.CurrTick < 200 * 2)//2sºóÆðÅÐ°²È«Çø
		return 1;

	int ilon,ilat;
	double lon,lat;
	GetDataFast(pDataPoolImu, "navLon", &ilon);
	GetDataFast(pDataPoolImu, "navLat", &ilat);
	lon = ilon * 1e-7;
	lat = ilat * 1e-7;

	if(OutSafeArea)
	{
		if(OutSafeCount == 30)
		{
			StopEngine();
			CurEngineRpm = 0;
		}
		if(OutSafeCount == 0)
		{
			DoOpenUm();//°²È«Çø¿ªÉ¡
		}
		else
		{
			OutSafeCount--;
		}
	}
	//³ö°²È«Çø¿ªÉ
  static int judgeError = 0;
	if(JudgeInSafe2(lon,lat) == FALSE)
	{
		if(RecoverMark == 1)
		{
				;
		}
		else
		{
			if(judgeError >= 3)
			{
					SETDATA(pDataPoolSelf,  "flyError", 0xCC,	OS_U8);
					if(OutSafeArea == false)
						OutSafeCount = 35;
					OutSafeArea = true;
					CurEngineRpm = 18000;
			}
			else
			{
					judgeError++;
			}
		}
	}
	else
	{
			judgeError = 0;
			OutSafeArea = false;
	}

	//µØÃæÖ¸Áî½ô¼±·µº½£¬µ½·µº½µãºó¿ªÉ¡
	if(homeMsn.MsnCmdType != 0)
	{
		double dist = haversine_distance(lat, lon, homeMsn.targetLat, homeMsn.targetLon);
		if(dist < 150)
		{
			SETDATA(pDataPoolSelf,  "flyError", 0x3,	OS_U8);
			DoOpenUm();//½ô¼±·µº½¿ªÉ¡
		}
	}
	return 0;
}

//extern RoutePoint rp[RP_MAX_NUMBER];
extern int RP_NUMBER;



OS_U8 DoOpenUm()
{
	//½â³ý·É¿ØËã·¨¶Ô¿ØÖÆÏµÍ³µÄ¹ÜÀí
	g_DeviceState.workStage &= ~DOM_AUTOMATIC;

	//¿ªÉ¡
	flightSeq.umOpen = 1;
	//CurEngineRpm = 18000;
	//SETDATA(pDataPoolFly, "EngineRp", CurEngineRpm,	OS_U16);
	//SETDATA(pDataPoolMsn, "OpenUm", 1,	OS_U8);
	return 0;
}


