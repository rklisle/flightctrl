#include "FCC_Lib.h"

//-----符号函数----//
double sign(double x)
{
	double z;

	z = 0.0;
	if (x >= 0.0) { z = 1.0; }
	if (x < 0.0) { z = -1.0; }
	return(z);
}
double Limit_In(double lower, double upper, double y)
{
	double z = y;

	if (z < lower)
		z = lower;
	else if (z > upper)
		z = upper;

	return z;
}

double Limit_Out(double lower, double upper, double y)
{
	double z = y;

	if ((lower < z) && (z < upper))
		z = 0;

	return z;
}
double lip1_IN(int n,double  ax,double  *x,double *y)
{
int i,k=0;
double  u,f;
     if((ax>=x[0])&&(ax<=x[n-1]))
	 {
     for(i=0;i<=n-2;i++)
	 {
		 if(ax<=x[i+1])
		 {
			 k=i;
			 break;
		 }
		 else
			 k=n-2;
	 }
	 u=(ax-x[k])/(x[k+1]-x[k]);
	 f=y[k]+u*(y[k+1]-y[k]);
	 }
	 else if(ax<x[0])
	 {
		 f=y[0];
	 }
	 else
	 {
		 f=y[n-1];
	 }
	 return(f);
}
/***********************************************************
函数名称:  Tustin_FirstIO
描    述:  一阶离散变换
滤波参数: a1,a2,b1,b2,ts
输    入:  r[2] 0-最新值 1-上一拍
输    出： f[2] 0-最新值 1-上一拍
************************************************************/
void Tustin_FirstIO(double a1,double a2,double b1,double b2,double *r,double *f,double ts)
{
    double m1=0.0;
    double n0=0.0;
    double n1=0.0;

	m1 = (2.*b1-b2*ts)/(2.*b1+b2*ts);              //前一时刻输出变量的系数
	n0 = (2.*a1+a2*ts)/(2.*b1+b2*ts);              //当前时刻输入变量的系数
	n1 = (-2.*a1+a2*ts)/(2.*b1+b2*ts);             //前一时刻输入变量的系数
	f[0] = m1*f[1]+n0*r[0]+n1*r[1];
}

#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(Tustin_SecondIO,".sdram_code");
#endif
void Tustin_SecondIO(double a1, double a2, double a3, double b1, double b2, double b3,
                     double *r, double *f, double ts)
{
    double m1, m2, n0, n1, n2;
    double T = 0;
    T = ts;
    m1 = (8. * b1 - 2. * b3 * T * T) / (4. * b1 + 2. * b2 * T + b3 * T * T);
    m2 = (2. * b2 * T - b3 * T * T - 4. * b1) / (4. * b1 + 2. * b2 * T + b3 * T * T);
    n0 = (4. * a1 + 2 * a2 * T + a3 * T * T) / (4. * b1 + 2. * b2 * T + b3 * T * T);
    n1 = (2. * a3 * T * T - 8. * a1) / (4. * b1 + 2. * b2 * T + b3 * T * T);
    n2 = (4. * a1 - 2. * a2 * T + a3 * T * T) / (4. * b1 + 2. * b2 * T + b3 * T * T);
    f[0] = m1 * f[1] + m2 * f[2] + n0 * r[0] + n1 * r[1] + n2 * r[2];
}

double deadzone(double lower,double upper,double y)
{
	double z=0.;
	z=y;
	if(z<upper&&z>lower)
		z=0;
	else
		z=y;
	return(z);
}

void matmat(double (*A)[3], double (*B)[3], double (*C)[3],double *D, double *F)
{
	double a[3][3],b[3][3],c[3][3],d[3];
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			a[i][j]=A[i][j];
			b[i][j]=B[i][j];
			c[i][j]=C[i][j];
			
		}
		d[i]=D[i];
		
	}
	F[0]=((A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0])*C[0][0]+(A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1])*C[1][0]+(A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2])*C[2][0])*D[0]+((A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0])*C[0][1]+(A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1])*C[1][1]+(A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2])*C[2][1])*D[1]+((A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0])*C[0][2]+(A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1])*C[1][2]+(A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2])*C[2][2])*D[2];
	F[1]=((A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0])*C[0][0]+(A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1])*C[1][0]+(A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2])*C[2][0])*D[0]+((A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0])*C[0][1]+(A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1])*C[1][1]+(A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2])*C[2][1])*D[1]+((A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0])*C[0][2]+(A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1])*C[1][2]+(A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2])*C[2][2])*D[2];
	F[2]= ((A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0])*C[0][0]+(A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1])*C[1][0]+(A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2])*C[2][0])*D[0]+((A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0])*C[0][1]+(A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1])*C[1][1]+(A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2])*C[2][1])*D[1]+((A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0])*C[0][2]+(A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1])*C[1][2]+(A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2])*C[2][2])*D[2];
}




