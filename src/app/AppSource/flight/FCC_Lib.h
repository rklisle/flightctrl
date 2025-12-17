#ifndef FCC_LIB
#define FCC_LIB


#include "stdio.h"
#include "math.h"

// 基本数学函数
double Limit_In(double lower, double upper, double y);
double Limit_Out(double lower, double upper, double y);
double lip1_IN(int n,double  ax,double  *x,double *y);
void Tustin_FirstIO(double a1,double a2,double b1,double b2,double *r,double *f,double ts);
void Tustin_SecondIO(double a1, double a2, double a3, double b1, double b2, double b3,
                     double *r, double *f, double ts);
double deadzone(double lower,double upper,double y);
void matmat(double (*A)[3], double (*B)[3], double (*C)[3],double *D, double *F);
double sign(double x);
#endif

