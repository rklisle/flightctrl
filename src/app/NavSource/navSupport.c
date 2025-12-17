/*
 * navSupport.c
 *
 *  Created on: 2024年3月4日
 *      Author: lenovo
 */
#include "navSupport.h"
#include "./modules/modGps.h"
#include "./modules/modImu.h"
#include "./modules/modFlyCtrl.h"
#include "./nav/nav2.h"
#include "./core/DataPool.h"
#include <tx_api.h>
#include <math.h>

//#define USE_OUT_IMU

OS_U8 CalcXYZ();

const double ae = (6378137.0);
const double e2 = (6.69437999014e-3);
const double PI = (3.141592653589793);
const double DTR = (PI / 180.0);

void ConSys_EarthWGS84_To_Launch(double * stateWGS84, double * state,double startLon, double startLat, double startHigh, double startPos);
void ConSys_M3x3_transpose(double M_in[3][3], double M_out[3][3]);
void ConSys_EarthLBH_To_EarthFixed(double * stateLBH, double * stateEg);
void ConSys_cx(double th, double M[3][3]);
void ConSys_cy(double th, double M[3][3]);
void ConSys_cz(double th, double M[3][3]);
void ConSys_Matrix3x3(double Matrix1[3][3], double Matrix2[3][3], double Matrixout[3][3]);
void ConSys_Matrix3x1(double Matrix1[3][3], double Matrix2[3], double Matrixout[3]);
void DoCalcXYZ(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double *x, double *y, double *z, double *vx, double *vy, double *vz);
void DoCalc(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double delaySet, double *pitch, double *yaw, double *course);

NAV_OUTPUT          nav_output = {0};
NAV_INPUT_GPS       nav_input_gps = {0};
NAV_INPUT_QBAR    	nav_input_qbar = {0};
NAV_INPUT_MAG       nav_input_mag  = {0};
NAV_INPUT_IMU 		nav_input_imu = {0};

float calcTimeCpu0;
float maxCalcTime = 0;

OS_BOOL useSimuData = OS_FALSE;
OS_U8 NavInputGenerate()
{
	nav_input_gps.Lon = gnss_recv_422.lon;
	nav_input_gps.Lat = gnss_recv_422.lat;
	nav_input_gps.Height = gnss_recv_422.alt;
	nav_input_gps.Vel_north = gnss_recv_422.vn;
	nav_input_gps.Vel_UP = gnss_recv_422.vs;
	nav_input_gps.Vel_east = gnss_recv_422.ve;
	nav_input_gps.Pdop = gnss_recv_422.pdop;
	nav_input_gps.Hdop = gnss_recv_422.hdop;
	nav_input_gps.Heading = gnss_recv_422.track; //gnss_recv.Yaw;
	if(gnss_recv_422.track >360)
		nav_input_gps.Status2 = 0;
	else
		nav_input_gps.Status2 = 1;//1*/
    nav_input_gps.Status2 = 0;
    
    nav_input_gps.Status1 = gnss_recv_422.posType;
	//杨兴使用右前上坐标
	nav_input_imu.DATA_XACCEL = imuSourceData.imuExpensive_az;
	nav_input_imu.DATA_YACCEL = imuSourceData.imuExpensive_ax;
	nav_input_imu.DATA_ZACCEL = imuSourceData.imuExpensive_ay;
	
	nav_input_imu.DATA_XGYRO = imuSourceData.imuExpensive_wz;
	nav_input_imu.DATA_YGYRO = imuSourceData.imuExpensive_wx;
	nav_input_imu.DATA_ZGYRO = imuSourceData.imuExpensive_wy;

	return 0;
   
}

OS_U8 NavOutputHandle()
{
	//对准时间180秒s
	if(StartFocusTime > 1.0 && GetCurTime() * 1000 - StartFocusTime > 180 * 1000)
	{
		navState = 0x3F;
		StartFocusTime = 0;
	}
	if(nav_input_imu.cmd == 2 || nav_input_imu.cmd == 3 || nav_input_imu.cmd == 4)
	{
		navState = (nav_output.navstat & 0B10000) == 0B10000? 0x60 :0x64;
	}
	return 0;
}


OS_U8 DoNavRun()
{
 	if(useSimuData == OS_TRUE)
	{
		//NavLoop(&navInputImu2, &GNSS_INPUT_Data, &qbar_Data, &mag_Data, &nav_output);
	}
	else
	{
		if(nav_input_imu.cmd != 0)
        {
           // unsigned long current_time0 = tx_time_get();
			NavLoop(&nav_input_imu, &nav_input_gps, &nav_input_qbar, &nav_input_mag, &nav_output);
            //unsigned long current_time1 = tx_time_get();
           // calcTimeCpu0 = (current_time1 - current_time0);
           // if(maxCalcTime < calcTimeCpu0)
           //     maxCalcTime = calcTimeCpu0;
        }
	}
	CalcXYZ();
    
	return 0;
}

OS_U8 CalcXYZ()
{
	//double pitch,yaw,roll;
	//DoCalc(initData.lon, initData.lat, initData.alt, initData.fai0, nav_output.lon, nav_output.lat, nav_output.alt, nav_output.v_n, -Navout_Data.v_d, Navout_Data.v_e, 0, &pitch, &yaw, &roll);

	double x = 0, y = 0, z = 0;
	double vx = 0, vy = 0, vz = 0;
	DoCalcXYZ(initData.lon, initData.lat, initData.alt, initData.fai0, nav_output.lon / DTR, nav_output.lat/ DTR, nav_output.alt, nav_output.v_n, -nav_output.v_d, nav_output.v_e, &x, &y, &z, &vx, &vy, &vz);

	//DoCalcXYZ(116.0, 39.0, 50, 45, 116.01, 39.01, 50, 0, 0, 0, &x, &y, &z, &vx, &vy, &vz);
	SETDATA(pDataPoolFly, "x", x * 10, OS_S32);
	SETDATA(pDataPoolFly, "y", y * 10, OS_S32);
	SETDATA(pDataPoolFly, "z", z * 10, OS_S32);
	SETDATA(pDataPoolFly, "vx", vx * 10, OS_S32);
	SETDATA(pDataPoolFly, "vy", vy * 10, OS_S32);
	SETDATA(pDataPoolFly, "vz", vz * 10, OS_S32);
	return 0;
}

void DoCalc(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double delaySet, double *pitch, double *yaw, double *course)
{
	double x = 0, y = 0, z = 0;
	double vx = 0, vy = 0, vz = 0;
	DoCalcXYZ(startLon, startLat, startHigh, startPos, targetLon, targetLat, targetHigh, vn, vs, ve, &x, &y, &z, &vx, &vy, &vz);
	double delaySecond = delaySet / 1000;
	x = x + vx * delaySecond;
	y = y + vy * delaySecond;
	z = z + vz * delaySecond;
	//开始计算角度
	double AngleRoll = atan2(z, x) / DTR;
	double AnglePitch = atan2(y, sqrt(x * x + z * z)) / DTR;

	*yaw = AngleRoll;
	*pitch = AnglePitch;

	*course = (int)(AngleRoll + startPos) % 360;
	if (course < 0)
	{
		course = 360 + course;
	}
}

void DoCalcXYZ(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double *x, double *y, double *z, double *vx, double *vy, double *vz)
{
	startLon *= DTR; startLat *= DTR; startPos *= DTR;
	targetLon *= DTR; targetLat *= DTR;

	double stateWGS84[6];
	stateWGS84[0] = targetLon;
	stateWGS84[1] = targetLat;
	stateWGS84[2] = targetHigh;
	stateWGS84[3] = vn;
	stateWGS84[4] = vs;
	stateWGS84[5] = ve;

	double state[6];
	ConSys_EarthWGS84_To_Launch(stateWGS84, state, startLon, startLat, startHigh, startPos);

	*x = state[0];
	*y = state[1];
	*z = state[2];
	*vx = state[3];
	*vy = state[4];
	*vz = state[5];

}


void ConSys_Matrix3x1(double Matrix1[3][3], double Matrix2[3], double Matrixout[3])
{

	//------------矩阵乘法运算函数------------
	// 矩阵相乘运算 Matrix1*Matrix2 输出矩阵 Matrixout 为3行1列
	//
	// 输入：
	// Matrix1、Matrix2
	//
	// 输出：
	// Matrixout
	//
	// 调用格式:
	// ConSys_Matrix3x1(Matrix1, Matrix2, Matrixout);
	//----------------------------------------

	double sum = 0;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			sum = sum + Matrix1[i][j]*Matrix2[j];
		}
		Matrixout[i] = sum;
		sum = 0;
	}

}

void ConSys_Matrix3x3(double Matrix1[3][3], double Matrix2[3][3], double Matrixout[3][3])
{

	//------------矩阵乘法运算函数------------
	// 矩阵相乘运算 Matrix1*Matrix2 输出矩阵 Matrixout 为3行3列
	//
	// 输入：
	// Matrix1、Matrix2
	//
	// 输出：
	// Matrixout
	//
	// 调用格式:
	// ConSys_Matrix3x1(Matrix1, Matrix2, Matrixout);
	//----------------------------------------

	double sum = 0;
	for(int i=0;i<3;i++)
	{
		for(int k=0;k<3;k++)
		{
			for(int j=0;j<3;j++)
			{
				sum = sum + Matrix1[i][j]*Matrix2[j][k];
			}
			Matrixout[i][k] = sum;
		    sum = 0;
		}
	}

}

void ConSys_cx(double th, double M[3][3])
{

	//--------------X轴旋转函数---------------
	// 三维坐标绕X轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cx(th, M);
	//----------------------------------------

	M[0][0] = 1;
	M[0][1] = 0;
	M[0][2] = 0;
	M[1][0] = 0;
	M[1][1] = cos(th);
	M[1][2] = sin(th);
	M[2][0] = 0;
	M[2][1] = -sin(th);
	M[2][2] = cos(th);

}

void ConSys_cy(double th, double M[3][3])
{

	//--------------Y轴旋转函数---------------
	// 三维坐标绕Y轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cy(th, M);
	//----------------------------------------

	M[0][0] = cos(th);
	M[0][1] = 0;
	M[0][2] = -sin(th);
	M[1][0] = 0;
	M[1][1] = 1;
	M[1][2] = 0;
	M[2][0] = sin(th);
	M[2][1] = 0;
	M[2][2] = cos(th);

}

void ConSys_cz(double th, double M[3][3])
{

	//--------------Z轴旋转函数---------------
	// 三维坐标绕Z轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cz(th, M);
	//----------------------------------------

	M[0][0] = cos(th);
	M[0][1] = sin(th);
	M[0][2] = 0;
	M[1][0] = -sin(th);
	M[1][1] = cos(th);
	M[1][2] = 0;
	M[2][0] = 0;
	M[2][1] = 0;
	M[2][2] = 1;

}

void ConSys_EarthLBH_To_EarthFixed(double * stateLBH, double * stateEg)
{

	//---------------------经纬高计算地固系状态函数--------------------------
	// 输入：
	// stateLBH  经纬高 0-2经纬高
	//
	// 输出：
	// stateEg   地固系(WGS-84)状态 0-2位置
	//
	// 调用格式
	// ConSys_EarthLBH_To_EarthFixed(stateLBH, stateEg);
	//-----------------------------------------------------------------------

	double Lon, Be, H;

	Lon = stateLBH[0];
	Be  = stateLBH[1];
	H   = stateLBH[2];

	double RN = ae / sqrt(1 - e2*sin(Be)*sin(Be));

	double x, y, z;
	x = (RN + H)*cos(Be)*cos(Lon);
	y = (RN + H)*cos(Be)*sin(Lon);
	z = (RN*(1 - e2) + H)*sin(Be);

	*(stateEg + 0) = x;
	*(stateEg + 1) = y;
	*(stateEg + 2) = z;

}

void ConSys_M3x3_transpose(double M_in[3][3], double M_out[3][3])
{

	//------------三维矩阵转置函数------------
	// 3x3矩阵转置函数
	//
	// 输入：
	// M_in
	//
	// 输出：
	// M_out
	//
	// 调用格式:
	// ConSys_M3x3_transpose(M_in, M_out);
	//----------------------------------------

	int i,j;
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			M_out[i][j] = M_in[j][i];
		}
	}

}

void CalcRoxyz(double lon, double lat, double high, double Ge[3][3], double *rox, double *roy, double *roz)
{
	double stateLBH0[3];
	double stateR0[3];
	stateLBH0[0] = lon;
	stateLBH0[1] = lat;
	stateLBH0[2] = high;
	ConSys_EarthLBH_To_EarthFixed(stateLBH0, stateR0);
	double ConSys_R0Fa[3];
	ConSys_Matrix3x1(Ge, stateR0, ConSys_R0Fa);
	*rox = ConSys_R0Fa[0];
	*roy = ConSys_R0Fa[1];
	*roz = ConSys_R0Fa[2];
}

void ConSys_EarthWGS84_To_Launch(double * stateWGS84, double * state,double startLon, double startLat, double startHigh, double startPos)
{

	//---------------------WGS-84系转发射系计算函数--------------------------
	// 输入：
	// stateLBH   地固系(WGS-84)状态 0-2经纬高 3-5北天东速度
	//
	// 输出：
	// state      发射系状态 0-2位置 3-5速度
	//
	// 调用格式
	// ConSys_EarthWGS84_To_Launch(stateWGS84, state);
	//-----------------------------------------------------------------------

	double Lon, Be, H, Vn[3];
	Lon   = stateWGS84[0];
	Be    = stateWGS84[1];
	H     = stateWGS84[2];
	Vn[0] = stateWGS84[3];
	Vn[1] = stateWGS84[4];
	Vn[2] = stateWGS84[5];

	double RN = ae / sqrt(1 - e2*sin(Be)*sin(Be));

	// 地固系位置坐标
	double R_Eg[3];
	R_Eg[0] = (RN + H)*cos(Be)*cos(Lon);
	R_Eg[1] = (RN + H)*cos(Be)*sin(Lon);
	R_Eg[2] = (RN*(1 - e2) + H)*sin(Be);

	double Ge[3][3], Gex[3][3], Gey[3][3], Gez[3][3], Mtemp[3][3];
	ConSys_cy(-(PI / 2 + startPos), Gey);
	ConSys_cx(startLat, Gex);
	ConSys_cz(-(PI / 2 - startLon), Gez);

	ConSys_Matrix3x3(Gey, Gex, Mtemp);
	ConSys_Matrix3x3(Mtemp, Gez, Ge);

	// 发射系状态 但原点在地心
	double statefr[3];
	ConSys_Matrix3x1(Ge, R_Eg, statefr);

	// 北天东坐标系下速度计算
	double MB0[3][3], MA0[3][3], M_EL[3][3];
	ConSys_cz(startLat, MB0);
	ConSys_cy(startPos, MA0);
	ConSys_Matrix3x3(MB0, MA0, M_EL);

	double delta_L = Lon - startLon;

	double MBE[3][3], MdL[3][3];
	ConSys_cz(-Be, MBE);
	ConSys_cx(delta_L, MdL);

	double M_NE[3][3], M_NL[3][3];
	ConSys_Matrix3x3(MBE, MdL, M_NE);
	ConSys_Matrix3x3(M_NE, M_EL, M_NL);

	double M_LN[3][3];
	ConSys_M3x3_transpose(M_NL, M_LN);   // 正交矩阵逆和转置相同

	double V_vector[3];
	ConSys_Matrix3x1(M_LN, Vn, V_vector);

	double ConSys_Rox = 0, ConSys_Roy = 0, ConSys_Roz = 0;
    CalcRoxyz(startLon, startLat, startHigh, Ge, &ConSys_Rox, &ConSys_Roy, &ConSys_Roz);


	*(state + 0) = statefr[0] - ConSys_Rox;
	*(state + 1) = statefr[1] - ConSys_Roy;
	*(state + 2) = statefr[2] - ConSys_Roz;
	*(state + 3) = V_vector[0];
	*(state + 4) = V_vector[1];
	*(state + 5) = V_vector[2];

}
