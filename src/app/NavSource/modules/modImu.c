/*
 * modImu.c
 *
 *  Created on: 2024年3月4日
 *      Author: lenovo
 */


#include "modImu.h"
#include <math.h>
#include <stdlib.h>
ImuSourceData  imuSourceData = {0};
double AverageIMU(double *inData);

void ImuSourceDataUpdate()
{
    static int _msCount = 0;
    _msCount++;
    if(_msCount == 5)
        _msCount = 0;
    struct adis16507_output out;
    GetImuOut(&out);
    //expensive imu
    imuSourceData.imuExpensive_wx = out.gyro_x;
    imuSourceData.imuExpensive_wy = out.gyro_y;
    imuSourceData.imuExpensive_wz = out.gyro_z;
    
    imuSourceData.imuExpensive_ax = out.accl_x * 0.001; //mm/s^2 to m/s^2
    imuSourceData.imuExpensive_ay = out.accl_y * 0.001;
    imuSourceData.imuExpensive_az = out.accl_z * 0.001;
    
    //cheap imu 42688
    imuSourceData.imuCheap42688_wx = gyro42688.x;
    imuSourceData.imuCheap42688_wy = gyro42688.y;
    imuSourceData.imuCheap42688_wz = gyro42688.z;
    
    imuSourceData.imuCheap42688_ax = accel42688.x;
    imuSourceData.imuCheap42688_ay = accel42688.y;
    imuSourceData.imuCheap42688_az = accel42688.z;
    
     //cheap imu 20689
    imuSourceData.imuCheap20689_wx = gyro20689.x;
    imuSourceData.imuCheap20689_wy = gyro20689.y;
    imuSourceData.imuCheap20689_wz = gyro20689.z;
    
    imuSourceData.imuCheap20689_ax = accel20689.x;
    imuSourceData.imuCheap20689_ay = accel20689.y;
    imuSourceData.imuCheap20689_az = accel20689.z;
    
    //先将惯组内部的各个元器件坐标系统一
	
	//x轴为滚转轴，朝向连接器方向为正
	//y轴为偏航轴，将设备连接螺丝管脚放置桌面时，向上为正
	//z轴为俯仰轴，右手法则朝右为正
	//此处代码为写死状态，无需更改
	double temp0[3], temp1[3], temp2[3];
    //16507
	temp0[0] = imuSourceData.imuExpensive_wx;
	temp0[1] = imuSourceData.imuExpensive_wy;
	temp0[2] = imuSourceData.imuExpensive_wz;

	imuSourceData.imuExpensive_wx = temp0[0];
	imuSourceData.imuExpensive_wy = temp0[1];
	imuSourceData.imuExpensive_wz = temp0[2];
    
    temp0[0] = imuSourceData.imuExpensive_ax;
	temp0[1] = imuSourceData.imuExpensive_ay;
	temp0[2] = imuSourceData.imuExpensive_az;

	imuSourceData.imuExpensive_ax = temp0[0];
	imuSourceData.imuExpensive_ay = temp0[1];
	imuSourceData.imuExpensive_az = temp0[2];
	
    //20689
	temp1[0] = imuSourceData.imuCheap20689_wx;
	temp1[1] = imuSourceData.imuCheap20689_wy;
	temp1[2] = imuSourceData.imuCheap20689_wz;

	imuSourceData.imuCheap20689_wx = temp1[0];
	imuSourceData.imuCheap20689_wy = temp1[1];
	imuSourceData.imuCheap20689_wz = temp1[2];
    
    temp1[0] = imuSourceData.imuCheap20689_ax;
	temp1[1] = imuSourceData.imuCheap20689_ay;
	temp1[2] = imuSourceData.imuCheap20689_az;

	imuSourceData.imuCheap20689_ax = temp1[0];
	imuSourceData.imuCheap20689_ay = temp1[1];
	imuSourceData.imuCheap20689_az = temp1[2];
    
    //42688
    temp2[0] = imuSourceData.imuCheap42688_wx;
	temp2[1] = imuSourceData.imuCheap42688_wy;
	temp2[2] = imuSourceData.imuCheap42688_wz;

	imuSourceData.imuCheap42688_wx = temp2[0];
	imuSourceData.imuCheap42688_wy = temp2[1];
	imuSourceData.imuCheap42688_wz = temp2[2];
    
    temp2[0] = imuSourceData.imuCheap42688_ax;
	temp2[1] = imuSourceData.imuCheap42688_ay;
	temp2[2] = imuSourceData.imuCheap42688_az;

	imuSourceData.imuCheap42688_ax = temp2[0];
	imuSourceData.imuCheap42688_ay = temp2[1];
	imuSourceData.imuCheap42688_az = temp2[2];
    
	//下面是将轴转向至设备安装方向，此处极性由主控板设置参数
    double tempxyz[3];
	tempxyz[0] = imuSourceData.imuExpensive_wx;
	tempxyz[1] = imuSourceData.imuExpensive_wy;
	tempxyz[2] = imuSourceData.imuExpensive_wz;

	imuSourceData.imuExpensive_wx = tempxyz[abs(installMode[0])-1] * (installMode[0] > 0?1:-1);
	imuSourceData.imuExpensive_wy = tempxyz[abs(installMode[1])-1] * (installMode[1] > 0?1:-1);
	imuSourceData.imuExpensive_wz = tempxyz[abs(installMode[2])-1] * (installMode[2] > 0?1:-1);

	tempxyz[0] = imuSourceData.imuExpensive_ax;
	tempxyz[1] = imuSourceData.imuExpensive_ay;
	tempxyz[2] = imuSourceData.imuExpensive_az;

	imuSourceData.imuExpensive_ax = tempxyz[abs(installMode[0])-1] * (installMode[0] > 0?1:-1);
	imuSourceData.imuExpensive_ay = tempxyz[abs(installMode[1])-1] * (installMode[1] > 0?1:-1);
	imuSourceData.imuExpensive_az = tempxyz[abs(installMode[2])-1] * (installMode[2] > 0?1:-1);
    
    tempxyz[0] = imuSourceData.imuCheap20689_wx;
	tempxyz[1] = imuSourceData.imuCheap20689_wy;
	tempxyz[2] = imuSourceData.imuCheap20689_wz;

	imuSourceData.imuCheap20689_wx = tempxyz[abs(installMode[0])-1] * (installMode[0] > 0?1:-1);
	imuSourceData.imuCheap20689_wy = tempxyz[abs(installMode[1])-1] * (installMode[1] > 0?1:-1);
	imuSourceData.imuCheap20689_wz = tempxyz[abs(installMode[2])-1] * (installMode[2] > 0?1:-1);

	tempxyz[0] = imuSourceData.imuCheap20689_ax;
	tempxyz[1] = imuSourceData.imuCheap20689_ay;
	tempxyz[2] = imuSourceData.imuCheap20689_az;

	imuSourceData.imuCheap20689_ax = tempxyz[abs(installMode[0])-1] * (installMode[0] > 0?1:-1);
	imuSourceData.imuCheap20689_ay = tempxyz[abs(installMode[1])-1] * (installMode[1] > 0?1:-1);
	imuSourceData.imuCheap20689_az = tempxyz[abs(installMode[2])-1] * (installMode[2] > 0?1:-1);
    
    //此处将已计算的角速度及加速度存储在宽度为5的数组中，再从数组中计算平均值返回
	//16507
	static double wx[5] = {0};
	static double wy[5] = {0};
	static double wz[5] = {0};
	static double ax[5] = {0};
	static double ay[5] = {0};
	static double az[5] = {0};

	wx[_msCount] = imuSourceData.imuExpensive_wx;
	wy[_msCount] = imuSourceData.imuExpensive_wy;
	wz[_msCount] = imuSourceData.imuExpensive_wz;
	ax[_msCount] = imuSourceData.imuExpensive_ax;
	ay[_msCount] = imuSourceData.imuExpensive_ay;
	az[_msCount] = imuSourceData.imuExpensive_az;

	imuSourceData.imuExpensive_wx = AverageIMU(wx);
	imuSourceData.imuExpensive_wy = AverageIMU(wy);
	imuSourceData.imuExpensive_wz = AverageIMU(wz);
	imuSourceData.imuExpensive_ax = AverageIMU(ax);
	imuSourceData.imuExpensive_ay = AverageIMU(ay);
	imuSourceData.imuExpensive_az = AverageIMU(az);
	
	//20689
	static double wx2[5] = {0};
	static double wy2[5] = {0};
	static double wz2[5] = {0};
	static double ax2[5] = {0};
	static double ay2[5] = {0};
	static double az2[5] = {0};

	wx2[_msCount] = imuSourceData.imuCheap20689_wx;
	wy2[_msCount] = imuSourceData.imuCheap20689_wy;
	wz2[_msCount] = imuSourceData.imuCheap20689_wz;
	ax2[_msCount] = imuSourceData.imuCheap20689_ax;
	ay2[_msCount] = imuSourceData.imuCheap20689_ay;
	az2[_msCount] = imuSourceData.imuCheap20689_az;

	imuSourceData.imuCheap20689_wx = AverageIMU(wx2);
	imuSourceData.imuCheap20689_wy = AverageIMU(wy2);
	imuSourceData.imuCheap20689_wz = AverageIMU(wz2);
	imuSourceData.imuCheap20689_ax = AverageIMU(ax2);
	imuSourceData.imuCheap20689_ay = AverageIMU(ay2);
	imuSourceData.imuCheap20689_az = AverageIMU(az2);
}

double AverageIMU(double *inData)
{
	double sum = 0;
	for(int i=0;i<5;i++)
	{
		double temp = *(inData + i);
		sum += temp;
	}
	return sum * 0.2;
}