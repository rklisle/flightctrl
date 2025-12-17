/*
 * modImu.h
 *
 *  Created on: 2024年3月4日
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODIMU_H_
#define SRC_MODULES_MODIMU_H_

#include    <stdbool.h>
#include    <stdint.h>
#include "../support/os_framework.h"
#include "./modFlyCtrl.h"
//#include "xparameters.h"
//#include "xscugic.h"
//#include "xil_exception.h"
//#include "../nav/nav2.h"
//IMU原始数据


typedef struct ImuSourceData
{
    double imuExpensive_wx;
    double imuExpensive_wy;
    double imuExpensive_wz;
    double imuExpensive_ax;
    double imuExpensive_ay;
    double imuExpensive_az;
    
    double imuCheap42688_wx;
    double imuCheap42688_wy;
    double imuCheap42688_wz;
    double imuCheap42688_ax;
    double imuCheap42688_ay;
    double imuCheap42688_az;
    
    double imuCheap20689_wx;
    double imuCheap20689_wy;
    double imuCheap20689_wz;
    double imuCheap20689_ax;
    double imuCheap20689_ay;
    double imuCheap20689_az;
    
    double mag_x;
    double mag_y;
    double mag_z;
}ImuSourceData;

struct adis16507_output
{
    bool      data_valid;
    
    /* data */
    float       gyro_x;
    float       gyro_y;
    float       gyro_z;
    float       accl_x;
    float       accl_y;
    float       accl_z;
    float       temp;
};

struct imu_data
{
    /* raw value from imu */
    int16_t  raw_x;
    int16_t  raw_y;
    int16_t  raw_z;

    /* output */
    float    x;
    float    y;
    float    z;
};

extern void GetImuOut(struct adis16507_output *out1);

extern void ImuSourceDataUpdate();

extern ImuSourceData  imuSourceData;
extern struct imu_data gyro42688;
extern struct imu_data accel42688;
extern struct imu_data gyro20689;
extern struct imu_data accel20689;
/*
typedef struct _imuInputData {

	    long DATA_CHECKSUM1ms      ;
unsigned long DATA_TIME_STMP1ms     ;
		 long DATA_TEMP_OUT1ms      ;
		 long DATA_ZACCEL1ms 		 ;//1ms Z轴加表输出，数据参见477手册
		 long DATA_YACCEL1ms 		 ;//1ms Y轴加表输出
		 long DATA_XACCEL1ms 		 ;//1ms X轴加表输出
		 long DATA_ZGYRO1ms 		 ;//1ms Z轴陀螺输出
		 long DATA_YGYRO1ms 		 ;//1ms Y轴陀螺输出
		 long DATA_XGYRO1ms 		 ;//1ms X轴陀螺输出
unsigned long DATA_DIAG_STAT1ms     ;

	     long DATA_CHECKSUM2ms      ;
unsigned long DATA_TIME_STMP2ms     ;
		 long DATA_TEMP_OUT2ms      ;
		 long DATA_ZACCEL2ms 		 ;//2ms Z轴加表输出
		 long DATA_YACCEL2ms 		 ;//2ms Y轴加表输出
		 long DATA_XACCEL2ms 		 ;//2ms X轴加表输出
		 long DATA_ZGYRO2ms 		 ;//2ms Z轴陀螺输出
		 long DATA_YGYRO2ms 		 ;//2ms Y轴陀螺输出
		 long DATA_XGYRO2ms 		 ;//2ms X轴陀螺输出
unsigned long DATA_DIAG_STAT2ms     ;

	     long DATA_CHECKSUM3ms      ;
unsigned long DATA_TIME_STMP3ms     ;
		 long DATA_TEMP_OUT3ms      ;
		 long DATA_ZACCEL3ms 		 ;//3ms Z轴加表输出
		 long DATA_YACCEL3ms 		 ;//3ms Y轴加表输出
		 long DATA_XACCEL3ms 		 ;//3ms X轴加表输出
		 long DATA_ZGYRO3ms 		 ;//3ms Z轴陀螺输出
		 long DATA_YGYRO3ms 		 ;//3ms Y轴陀螺输出
		 long DATA_XGYRO3ms 		 ;//3ms X轴陀螺输出
unsigned long DATA_DIAG_STAT3ms     ;

	     long DATA_CHECKSUM4ms      ;
unsigned long DATA_TIME_STMP4ms     ;
		 long DATA_TEMP_OUT4ms      ;
		 long DATA_ZACCEL4ms 		 ;//4ms Z轴加表输出
		 long DATA_YACCEL4ms 		 ;//4ms Y轴加表输出
		 long DATA_XACCEL4ms 		 ;//4ms X轴加表输出
		 long DATA_ZGYRO4ms 		 ;//4ms Z轴陀螺输出
		 long DATA_YGYRO4ms 		 ;//4ms Y轴陀螺输出
		 long DATA_XGYRO4ms 		 ;//4ms X轴陀螺输出
unsigned long DATA_DIAG_STAT4ms     ;

	     long DATA_CHECKSUM5ms      ;
unsigned long DATA_TIME_STMP5ms     ;
		 long DATA_TEMP_OUT5ms      ;
		 long DATA_ZACCEL5ms 		 ;//5ms Z轴加表输出
		 long DATA_YACCEL5ms 		 ;//5ms Y轴加表输出
		 long DATA_XACCEL5ms 		 ;//5ms X轴加表输出
		 long DATA_ZGYRO5ms 		 ;//5ms Z轴陀螺输出
		 long DATA_YGYRO5ms 		 ;//5ms Y轴陀螺输出
		 long DATA_XGYRO5ms 		 ;//5ms X轴陀螺输出
unsigned long DATA_DIAG_STAT5ms     ;

			//大量程加表输出
			double Ax1ms;//1ms x轴加表输出
			double Ax2ms;//2ms x轴加表输出
			double Ax3ms;//3ms x轴加表输出
			double Ax4ms;//4ms x轴加表输出
			double Ax5ms;//5ms x轴加表输出

unsigned  char cmd     ;//1：开始对准，2：转导航
}imuInputData;

extern imuInputData 	IMU_INPUT_Data;

void DeviceDriverHandler(void *CallbackRef);
int InitInteruput(unsigned short DeviceId, unsigned int intId,unsigned char inter_enable, unsigned int Priority, void (*int_handler)(void *));
extern OS_U8 InitSpiImu();
*/
#endif /* SRC_MODULES_MODIMU_H_ */
