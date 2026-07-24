#ifndef _GLOBAL_FUNCTION_H_
#define _GLOBAL_FUNCTION_H_
//==================================================================/
// Copyright (c) 2016 
// All rights reserved.
//
// 摘要: 定义仿真所需公用函数
//        
//
// 当前版本: 1.0
// 作者: wym
// 完成日期: 
//==================================================================/
#include <math.h>

#define PI			(3.14159265358979)
#define RTOA		(180.0/PI) 
#define LAT_TO_METER    111319.5 // 1纬度° ≈ 111319.5米（APM开源飞控，简化模型标准值）
#define R_EARTH    6371000.0 // 1纬度° ≈ 111319.5米（APM开源飞控，，简化模型标准值）

#define IN			//输入量
#define OUT			//输出量

class CFlightGlobalFun
{
public:
	static double get_bearing(double lon1, double lat1, double lon2, double lat2);	//近距简化计算方位角
	static double get_distance(double lon1, double lat1, double lon2, double lat2);//近距简化计算弹目距离
	static void Tomas(IN double lon1,			//起始经度 
		IN double lat1, 				//起始纬度
		IN double lon2, 				//目标经度
		IN double lat2, 				//目标纬度
		OUT double *sdm, 				//距离
		OUT double *a12);				//游移方位角
	static double Reject_outlier(double curr_val, double *base_val, int *over_cnt, double delta_max);//剔野值函数
	static void Tustin_FirstIO(double a1,double a2,double b1,double b2,double *r,double *f,double ts);
	static void Tustin_SecondIO(double a1, double a2, double a3, double b1, double b2, double b3, double *r, double *f, double ts);
	static double Nozero_FUN(IN double denominator);
	static double QATN(IN double y, IN double x);
	static double FSign(IN double fData);
	static double Adjust(IN double ori, IN double range);
	static double Range(IN double dData, IN double dMargin);
	static double Range2(IN double dData, IN double dMax, IN double dMin);
	static double Norm(IN int vector_length, IN double *in_vector);
	static void Vincenty_Forward(IN double vin_l1,	//当前经度
		IN double vin_b1,			//当前纬度
		IN double s,					//距离
		IN double alfa12,			//游移方位角
		OUT double* vin_l2_ptr,		//目标经度
		OUT double* vin_b2_ptr);	//目标纬度
	static void Vincenty_Backward(IN double vin_l1,	//当前经度
		IN double vin_b1,			//当前纬度
		IN double vin_l2,			//目标经度		
		IN double vin_b2,			//目标纬度
		OUT double* S_ptr,			//距离
		OUT double* Alfa12_ptr);	//游移方位角
	static double CalcDist(double lon1, 
		double lat1, 
		double lon2, 
		double lat2, 
		double lon, 
		double lat);
	static void CrossProduct(IN double in_vector_A[],	//叉乘向量A
		IN double in_vector_B[],		//叉乘向量B
		OUT double *out_vector_C	//结果向量C
		);
	static double DotProduct(int vector_length,	//点乘向量维数
		double in_vector_A[],		//点乘向量A
		double in_vector_B[]		//点乘向量B
		);
	static double LAQL1(int n,	//插值轴元素个数
		double *x,			//插值轴				
		double *y,			//插值数据（n）
		double u);			//插值输入参数
	static double LAQL2(int n, //插值轴1元素个数
		int m,				//插值轴2元素个数
		double *a,			//插值轴1
		double *b,			//插值轴2
		double *c,			//插值数据（n * m）
		double x,			//插值输入参数1
		double y);			//插值输入参数2
	static double LAQL3(int n, //插值轴1元素个数
		int m,				//插值轴2元素个数
		int l,				//插值轴3元素个数
		double *a,			//插值轴1
		double *b,			//插值轴2
		double *c,			//插值轴3
		double *d,			//插值数据（n * m * l）		
		double x,			//插值输入参数1
		double y,			//插值输入参数2
		double z);			//插值输入参数3
	static double LAQL4(int n,	//插值轴1元素个数
		int m,				//插值轴2元素个数
		int l,				//插值轴3元素个数
		int k,				//插值轴4元素个数
		double *a,			//插值轴1
		double *b,			//插值轴2
		double *c,			//插值轴3
		double *d,			//插值轴4
		double *e,			//插值数据（n * m * l * k）
		double x,			//插值输入参数1
		double y,			//插值输入参数2
		double z,			//插值输入参数3
		double u);			//插值输入参数4
	static double LAQL1fd(int n, 
		double *x,			//对输入参数x分段
		double *y, 
		double u);
	static double LAQL2fd(int n,	
		int m, 
		double *a,			//对输入参数a分段
		double *b, 
		double *c, 
		double x, 
		double y);
	static double LAQL3fd(int n, 
		int m, 
		int l,
		double *a,			//对输入参数a分段
		double *b, 
		double *c, 
		double *d,
		double x, 
		double y, 
		double z);
	static double LAQL1R(int n,	//插值轴元素个数	
		double *x,			//反插值轴				
		double *y,			//插值数据（n）
		double u);			//插值输入参数
	static double LAQL2R(int n, //插值轴1元素个数
		int m,				//插值轴2元素个数
		double *a,			//插值轴1
		double *b,			//反插值轴2
		double *c,			//插值数据（n * m）
		double x,			//插值输入参数1
		double y);			//插值输入参数2
	static double LAQL3R(int n, //插值轴1元素个数
		int m,				//插值轴2元素个数
		int l,				//插值轴3元素个数
		double *a,			//插值轴1
		double *b,			//插值轴2
		double *c,			//反插值轴3
		double *d,			//插值数据（n * m * l）		
		double x,			//插值输入参数1
		double y,			//插值输入参数2
		double z);			//插值输入参数3
	static double LAQL4R(int n,	//插值轴1元素个数
		int m,				//插值轴2元素个数
		int l,				//插值轴3元素个数
		int k,				//插值轴4元素个数
		double *a,			//插值轴1
		double *b,			//插值轴2
		double *c,			//插值轴3
		double *d,			//反插值轴4
		double *e,			//插值数据（n * m * l * k）
		double x,			//插值输入参数1
		double y,			//插值输入参数2
		double z,			//插值输入参数3
		double u);			//插值输入参数4
};

#endif