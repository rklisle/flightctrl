/*
 * modBC.c
 *
 *  Created on: 2022年3月15日
 *      Author: Lenovo
 */
#include "stm32h7xx_hal_adc.h"
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../StateMachine.h"
#include "../navSupport.h"
#include "../modules/modFlash.h"
#include "../modules/modGps.h"
#include "../modules/modImu.h"
#include "../modules/modFlyCtrl.h"
#include "./controller.h"
#include "../support/common.h"
#include <math.h>
#include <stdlib.h>

OS_U8 navState = 0x44;
OS_DOUBLE curPress;
OS_DOUBLE curAirSpdPa;
// OS_DOUBLE curAirSpd;	// MML 未使用，注释
OS_DOUBLE press_height;
OS_DOUBLE AirSpdHistory[150];//0.75秒窗口
OS_U8 SendNavQiuStart = 0;
OS_DOUBLE calibrationValue = 13.2;
OS_U16 calibrationCount = 0;
extern float calcTimeCpu0;
#define d2r		(57.29577951308402)

int UTC_to_Localtime(int *PtrYear, int *PtrMonth, int *PtrDay, int *PtrHour, const int TimeZone);
OS_DOUBLE Average(OS_DOUBLE array[], int len);
float System_GetCoreTemperature();


OS_U8 SelfStatusUpdata()
{
    //启动GPS
	StartGPS();
    /*
	//更新星历注入状态
	static int ephUpdateCount = 0;
	if(ephUpdateState == 1)
	{
		ephUpdateCount++;
		if(ephUpdateCount >= 400)//2s
		{
			ephUpdateState = 4;
		}
	}
	else
	{
		ephUpdateCount = 0;
	}
    */
	return 0;
}

extern void GetMagXYZDIR(double *x, double *y, double *z);

double CalcMagDir(double x, double y, double z)
{
   //installMode[0] = 1;
  // installMode[1] = -2;
   // installMode[2] = -3;
    
    double magxyz[3] = {0};
    magxyz[0] = x;
    magxyz[1] = y;
    magxyz[2] = z;
    x = magxyz[abs(installMode[0])-1] * (installMode[0] > 0?1:-1);
    y = magxyz[abs(installMode[1])-1] * (installMode[1] > 0?1:-1);
    z = magxyz[abs(installMode[2])-1] * (installMode[2] > 0?1:-1);
        
    double dir = atan2(-x, -y) * 57.3;//底部朝上，连接器朝前
	//double dir = atan2(x, -y) * 57.3;//底部朝下，连接器朝前
    if(dir < 0)
        dir += 360;
    return dir;
}

double estimate_magnetic_declination(double lat, double lon) 
{

    const double A = -10.0f;  
    const double B = 0.1f;    
    const double C = -0.2f;  

    float declination = A + B * lon + C * lat;

    if (declination < -180.0f) declination += 360.0f;
    if (declination > 180.0f) declination -= 360.0f;

    return declination;
}

extern double curTime;
OS_U8 SelfFrameOut()
{
	if(FlashProgramming == TRUE)
		return 1;

	static OS_U32 sendCount = 0;
	OS_U8 data[300];
	memcpy(data, &sendCount, 4);
	data[4] = gnss_recv_422.posType == 1?1:0;
	data[5] = gnss_recv_422.TrackStart1;
	data[6] = gnss_recv_422.TrackStart2;
	OS_S32 intTemp;
	//OS_U32 uintTemp;
	intTemp = gnss_recv_422.lon * 1e7;
	memcpy(data + 7, &intTemp , 4);
	intTemp = gnss_recv_422.lat * 1e7;
	memcpy(data + 11, &intTemp , 4);
	intTemp = gnss_recv_422.alt * 1e3;
	memcpy(data + 15, &intTemp , 4);
	OS_S16 shortTemp;
	shortTemp = gnss_recv_422.vn * 100;
	memcpy(data + 19, &shortTemp , 2);
	shortTemp = gnss_recv_422.vs * 100;
	memcpy(data + 21, &shortTemp , 2);
	shortTemp = gnss_recv_422.ve * 100;
	memcpy(data + 23, &shortTemp , 2);
	OS_U16 ushortTemp;
	ushortTemp = gnss_recv_422.pdop * 1e2;
	memcpy(data + 25, &ushortTemp , 2);
	ushortTemp = gnss_recv_422.hdop * 1e2;
	memcpy(data + 27, &ushortTemp , 2);
	data[29] = 0;// pps delay time
    
	OS_FLOAT floatTemp;
    //16507 imu
	floatTemp = imuSourceData.imuExpensive_wx;
	memcpy(data + 30, &floatTemp , 4);
	floatTemp = imuSourceData.imuExpensive_wy;
	memcpy(data + 34, &floatTemp , 4);
	floatTemp = imuSourceData.imuExpensive_wz;
	memcpy(data + 38, &floatTemp , 4);

	floatTemp = imuSourceData.imuExpensive_ax;
	memcpy(data + 42, &floatTemp , 4);
	floatTemp = imuSourceData.imuExpensive_ay;
	memcpy(data + 46, &floatTemp , 4);
	floatTemp = imuSourceData.imuExpensive_az;
	memcpy(data + 50, &floatTemp , 4);

    //20689 imu
    floatTemp = imuSourceData.imuCheap20689_wx;
	memcpy(data + 54, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap20689_wy;
	memcpy(data + 58, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap20689_wz;
	memcpy(data + 62, &floatTemp , 4);

	floatTemp = imuSourceData.imuCheap20689_ax;
	memcpy(data + 66, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap20689_ay;
	memcpy(data + 70, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap20689_az;
    //floatTemp = curTime;
	memcpy(data + 74, &floatTemp , 4);
    
    //42688
    floatTemp = imuSourceData.imuCheap42688_wx;
	memcpy(data + 78, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap42688_wy;
	memcpy(data + 82, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap42688_wz;
	memcpy(data + 86, &floatTemp , 4);

	floatTemp = imuSourceData.imuCheap42688_ax;
	memcpy(data + 90, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap42688_ay;
	memcpy(data + 94, &floatTemp , 4);
	floatTemp = imuSourceData.imuCheap42688_az;
	memcpy(data + 98, &floatTemp , 4);

	data[102] = navState;//组合导航状态

	intTemp = nav_output.lon * d2r * 1e7;
	memcpy(data + 103, &intTemp , 4);
	intTemp = nav_output.lat * d2r * 1e7;
	memcpy(data + 107, &intTemp , 4);
	intTemp = nav_output.alt * 1e3;
	memcpy(data + 111, &intTemp , 4);

	intTemp = nav_output.v_n * 1e3;
	memcpy(data + 115, &intTemp , 4);
	intTemp = nav_output.v_d * 1e3;
	memcpy(data + 119, &intTemp , 4);
	intTemp = nav_output.v_e * 1e3;
	memcpy(data + 123, &intTemp , 4);

    ushortTemp = nav_output.fai * 1e2;
	memcpy(data + 127, &ushortTemp , 2);
	shortTemp = nav_output.pich * 1e2;
	memcpy(data + 129, &shortTemp , 2);
	shortTemp = nav_output.roll * 1e2;
	memcpy(data + 131, &shortTemp , 2);

	data[133] = installMode[0];//安装模式x
	data[134] = installMode[1];//安装模式y
	data[135] = installMode[2];//安装模式z

	data[136] = 0x00;//不仿真
	data[137] = ephUpdateState;//星历加注反馈

	//地磁数据
    double x,y,z,magdir;
    GetMagXYZDIR(&x, &y, &z);
    
    magdir = CalcMagDir(x,y,z);
    
	double declination = estimate_magnetic_declination(nav_output.lat * d2r, nav_output.lon * d2r);
    double estimatedir = magdir - declination;
    
    ushortTemp = magdir * 1e2;
    memcpy(data + 138, &ushortTemp , 2);
    
    ushortTemp = estimatedir * 1e2;
    memcpy(data + 140, &ushortTemp , 2);
    
    shortTemp = x * 1e2;
	memcpy(data + 142, &shortTemp , 2);
	shortTemp = y * 1e2;
	memcpy(data + 144, &shortTemp , 2);
	shortTemp = z * 1e2;
	memcpy(data + 146, &shortTemp , 2);
	
	int year = gnss_recv_422.TIME_YMD.YEAR;
	int month = gnss_recv_422.TIME_YMD.MONTH;
	int day = gnss_recv_422.TIME_YMD.DAY;
	int hour = gnss_recv_422.TIME_YMD.HOUR;
	//处理北京时间到UTC时间
	UTC_to_Localtime(&year, &month, &day, &hour, 8);
	data[148] = ((year - 2000) & 0xFF);
	data[149] = (month & 0xFF);
	data[150] = (day & 0xFF);
	data[151] = (hour & 0xFF);
    data[152] = gnss_recv_422.TIME_YMD.MINITE;
    data[153] = gnss_recv_422.TIME_YMD.SECOND;
    data[154] = (((unsigned char)(gnss_recv_422.TIME_YMD.MS * 0.1)) & 0xFF);
	
	*((OS_U16 *)(data +155)) = gnss_recv_422.heading * 100;
    
	data[157] = gnss_recv_422.TrackStart1>6?1:0;
	data[158] = gnss_recv_422.TrackStart2>6?1:0;

    data[159] = gnss_recv_422.posType == 1?1:0;
    
	nav_input_gps.Status1 = 0;//下传后置零
	data[160] = gnss_recv_422.heading_type == 0?0:1;

	OS_U16 flightTracku16 = gnss_recv_422.track * 100;
	memcpy(data + 161, &flightTracku16, 2);

	calcTimeCpu0 = 1.5;//MS
	OS_U16 cpu0us = calcTimeCpu0 * 1000;
	memcpy(data + 163, &cpu0us, 2);
	static OS_U16 cpu0usKa = 0;
	if(cpu0us > 2000)
	{
		cpu0usKa = cpu0us;
	}
	memcpy(data + 165, &cpu0usKa, 2);

    ushortTemp = System_GetCoreTemperature() * 100;
    memcpy(data + 167, &ushortTemp, 2);
    
	//-------------------------------------------
    //long long tick = GetCurTime() * 200 ;
    //if(tick % 10 == 0)
    {
        MsgToDevice(RT_FLYCTRL, 0x93, 169, data);
        sendCount++;
    }
	return 0;
}



int GetMonthday(int year, int month, int day, int * thisMonthday, int * lastMonthday)
{
	if (day < 1)
		return 0;

	if (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12)
	{
		*thisMonthday = 31;	//本月天数
		*lastMonthday = 30;	//上月天数

		if (day > 31)
			return 0;

		if (month == 3)
		{
			if ((year % 400 == 0) || (year % 4 == 0 && year % 100 != 0))
				*lastMonthday = 29;
			else
				*lastMonthday = 28;
		}
		if (month == 8 || month == 1)
			*lastMonthday = 31;
	}
	else if (month == 4 || month == 6 || month == 9 || month == 11)
	{
		if (day > 30)
			return 0;

		*thisMonthday = 30;
		*lastMonthday = 31;
	}
	else //二月
	{
		*lastMonthday = 31;

		if ((year % 400 == 0) || (year % 4 == 0 && year % 100 != 0))
		{
			*thisMonthday = 29;

			if (day > 29)
				return 0;
		}
		else
		{
			*thisMonthday = 28;

			if (day > 28)
				return 0;
		}

	}

	return 1;
}

int UTC_to_Localtime(int *PtrYear, int *PtrMonth, int *PtrDay, int *PtrHour, const int TimeZone)
{
	int year = *PtrYear;
	int month = *PtrMonth;
	int day = *PtrDay;
	int hour = *PtrHour + TimeZone;

	int thisMonthday = 0;	//本月天数
	int lastMonthday = 0;	//上月天数

	//判断时间是否合法,并获取本月时间及上月时间
	if (!GetMonthday(year, month, day, &thisMonthday, &lastMonthday) || *PtrHour > 24 || *PtrHour < 0)
		return 0;

	if (hour >= 24)
	{
		hour -= 24;
		day += 1;
		if (day > thisMonthday)
		{
			day -= thisMonthday;
			month += 1;
			if (month > 12)
			{
				month -= 12;
				year += 1;
			}
		}
	}

	if (hour < 0)
	{
		hour += 24;
		day -= 1;
		if (day < 1)
		{
			day = lastMonthday;
			month -= 1;
			if (month < 1)
			{
				month = 12;
				year -= 1;
			}

		}

	}

	*PtrYear = year;
	*PtrMonth = month;
	*PtrDay = day;
	*PtrHour = hour;

	return 1;
}
ADC_HandleTypeDef hadc3;


void ADC3_Init(void)
{
    /* USER CODE BEGIN ADC3_Init 0 */

    /* USER CODE END ADC3_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC3_Init 1 */

    /* USER CODE END ADC3_Init 1 */

    /** Common config
    */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode = DISABLE;
    hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC3_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

float System_GetCoreTemperature()
{
     float temperature; 
    uint16_t ts_cal1 =  *(__IO uint16_t *)(0x1FF1E820UL);
    uint16_t ts_cal2 =  *(__IO uint16_t *)(0x1FF1E840UL);	
    
    HAL_ADC_Start(&hadc3); 

    HAL_ADC_PollForConversion(&hadc3, 10);  

    uint32_t adc_value = HAL_ADC_GetValue(&hadc3);
    int a = adc_value - ts_cal1;
    int b = ts_cal2 - ts_cal1;
    
    temperature =(((a)*80.0) / (b)) + 30.0f;
    
    return temperature;
}