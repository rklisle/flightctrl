#ifndef _GLOBAL_FUNCTION_H_
#define _GLOBAL_FUNCTION_H_
#include <math.h>

#define PI			(3.14159265358979)
#define RTOA		(180.0/PI) 
#define LAT_TO_METER	111132.8742
#define R_EARTH			6367444.657

#define IN			
#define OUT			

class CFlightGlobalFun
{
public:
	static void Tomas(double lon1,
		double lat1,
		double lon2,
		double lat2,
		double *sdm,
		double *a12);
	static double Reject_outlier(double curr_val, double *base_val, int *over_cnt, double delta_max);
	static void Tustin_FirstIO(double a1,double a2,double b1,double b2,double *r,double *f,double ts);
	static void Tustin_SecondIO(double a1, double a2, double a3, double b1, double b2, double b3, double *r, double *f, double ts);
	static double Nozero_FUN(double denominator);
	static double QATN(double y, double x);
	static double FSign(double fData);
	static double Adjust(double ori, double range);
	static double Range(double dData, double dMargin);
	static double Range2(double dData, double dMax, double dMin);
	static double Norm(int vector_length, double *in_vector);
	static void Vincenty_Forward(double vin_l1,
		double vin_b1,
		double s,
		double alfa12,
		double* vin_l2_ptr,
		double* vin_b2_ptr);
	static void Vincenty_Backward(double vin_l1,
		double vin_b1,
		double vin_l2,
		double vin_b2,
		double* S_ptr,
		double* Alfa12_ptr);
	static double CalcDist(double lon1, 
		double lat1, 
		double lon2, 
		double lat2, 
		double lon, 
		double lat);
	static void CrossProduct(double in_vector_A[],
		double in_vector_B[],
		double *out_vector_C
		);
	static double DotProduct(int vector_length,
		double in_vector_A[],
		double in_vector_B[]
		);
	static double LAQL1(int n,
		double *x,
		double *y,
		double u);
	static double LAQL2(int n,
		int m,
		double *a,
		double *b,
		double *c,
		double x,
		double y);
	static double LAQL3(int n,
		int m,
		int l,
		double *a,
		double *b,
		double *c,
		double *d,
		double x,
		double y,
		double z);
	static double LAQL4(int n,
		int m,
		int l,
		int k,
		double *a,
		double *b,
		double *c,
		double *d,
		double *e,
		double x,
		double y,
		double z,
		double u);
	static double LAQL1fd(int n, 
		double *x,
		double *y, 
		double u);
	static double LAQL2fd(int n,	
		int m, 
		double *a,
		double *b, 
		double *c, 
		double x, 
		double y);
};

#endif