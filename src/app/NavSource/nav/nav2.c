/*
 * nav2.c
 *
 *  Created on: 2024年2月25日
 *      Author: lenovo
 */
#include "nav2.h"
#include <math.h>

void NavInit(NavInitStr* p);

//全局变量
#define cuduizhunSecond (20)
#define jingduizhunSecond (160)

double f_delta_hx=0.0;
double f_Delta_HX=0.0;
double f_DLL_Time_global=0.0;

double f_W0_Time=0.0;
double We02=0.0;
double Wn02=0.0;
double Wu02=0.0;
double f_GPS_LOST_TIME_TOTAL=0.0;   /////GPS丢失时间
double f_GPS_LOSTHEAD_TIME_TOTAL=0.0; ////航向丢失时间
double f_Error_Pe = 0.0;
double f_Error_Pn = 0.0;
double f_Error_Pu = 0.0;


double f_DT = 0.005;
double Wibb_iner[3] = {0.0,0.0,0.0};  ////陀螺角
double q1_iner[4] = {0.0};

double f_Inertial_b0N0_Last[9] = {0.0};
double f_Inertial_N0b0_Last[9] = {0.0};

double f_Cn_i0_Array[9] = {0.0};

double f_b0_b_Array[9] = {0.0};

double f_TCB_Inertial_Last[9] = {0.0};

double f_Inertial_N_N0[9] = {0.0};

//double f_C_i0_n0_Array[9]={0.0};

int I_IS_GPS_SET_GLOBAL_INIT_CNT=0;////GPS速度逐级修正变量,修正程序
double DELAYTIME = 10;
/////////////////////////////////////////////////////////////////////////////
double DT2 = 1.0;


#define DEGTORAD (0.017453292)

#define RADTODEG (57.29578049)

/////////////////////////////////////////////////////////////////    data
int gpsflg=0;
////float fwxyz[3],fv[3],fv1[3],v0f[3],v0f1[3];
double q[4]={0.0, 0.0, 0.0, 0.0};
double Lon=116.5053*3.14159265/180.,Lat=39.7928*3.14159265/180.,Lat1=39.7928*3.14159265/180.,Lon1=116.5053*3.14159265/180.;
double Lon_out=106.55,Lat_out=29.527,HH_out=0.0;
int av[7];

double dFe=0.0,dFn=0.0,dFu=0.0;
double Fe=0.0,Fn=0.0,Fu=0.0,Ae0=0.0,An0=0.0,Au0=0.0;
double We0=0.0,Wn0=0.0,Wu0=0.0,Mge0=0.0,Mgn0=0.0,Mgu0=0.0;
double Re=0.0,Rn=0.0,Roe=0.0,Ron=0.0,Rou=0.0;

double We=0.0,Wn=0.0,Wu=0.0;
double Cx,Cy,Cz,De,Dn,Du;
double dx_wie=0.0,dy_wie=0.0,dz_wie=0.0;

double Lat_CMD=0.0;
double Lon_CMD=0.0;
double HH_CMD=0.0;
double f_SHEXIANG_ANGLE=0.0;  ////射向角

int num12=0,Kalmanflg=0;
double Pn=0.0,Rolln=0.0,Yn=0.0;
double T[3][3],Wwe,Wwn,Wwu,Le,Ln,Lu,Rx=0.0,Ry=0.0,Rz=0.0,Vwe,Vwn,Vwu;
double f_Vb[3]={0};
double f_Vn[3]={0};

#define I_STATEDIMEN  (15)
#define I_OBSDIMEN  (9)
double FItr_ARRAY[I_STATEDIMEN*I_STATEDIMEN],Ktr_ARRAY[I_STATEDIMEN*I_STATEDIMEN],Htr_ARRAY[I_STATEDIMEN*I_STATEDIMEN];
double F_ARRAY[I_STATEDIMEN*I_STATEDIMEN],QQ_ARRAY[I_STATEDIMEN*I_STATEDIMEN],P_ARRAY[I_STATEDIMEN*I_STATEDIMEN],RR_ARRAY[I_OBSDIMEN*I_OBSDIMEN],K_ARRAY[I_OBSDIMEN*I_STATEDIMEN];
double F_ARRAY_ORG[I_STATEDIMEN*I_STATEDIMEN]={0.0};
double XX_ARRAY[I_STATEDIMEN],H_ARRAY[I_OBSDIMEN*I_STATEDIMEN],Zg_ARRAY[I_OBSDIMEN];
double FI_ARRAY[I_STATEDIMEN*I_STATEDIMEN];

double HH=26,H1=26.0,Dse=0.0,Dsn=0.0,Dsh=0.0;
double P0=0.0,R0=0.0,Fia0=0.0;
double angle[3]={0.0,0.0,0.0};

double Vn=0.0,Vn1=0.0,DT1=0.0,Ts=0.0;
double Ve=0.0,Ve1=0.0;
double Vu=0.0,Vu1=0.0;
double Tnb[9];
//double F[324],QQ[324],P[324],RR[36],K[108];

///////////////////////////////////////////////////////////////////  data
int ismm[9],jsmm[9];  /*-------?-----------*/
double bmm[I_STATEDIMEN*I_STATEDIMEN],b1mm[I_STATEDIMEN*I_STATEDIMEN];

double d_klm_X=0.0,d_klm_Y=0.0,d_klm_Z=0.0;

double dklm_faiE=0.0,dklm_faiN=0.0,dklm_faiU=0.0;
///////////////////////////////////////////
#define	PI	(3.1415926535)
#define	Wie	(7.292115e-5)
#define	R_earthPara	(6378137.0)
#define	f_earthPara (0.00335233)
double DT=0.005;
//#define     DT2     1.0
/*--------  重力加速度靠GPS的经纬度来计算-------*/
double Grv = 9.795;
double g0_ORG = 9.795;
#define	eps	(1.0e-50)
#define	Rpai (PI/180.0)

#define  DEG2RAD   (PI/180.0)
#define  SEC2RAD   (1.0/3600.0/57.3)
#define  RAD2DEG   (180.0/PI)

void cal_RR(double RR[], double w_inarray[])
{
	double w_total=0.0;
	int i = 0;
	w_total = sqrt(w_inarray[0]*w_inarray[0]+w_inarray[1]*w_inarray[1]+w_inarray[2]*w_inarray[2])*RADTODEG;
	for(i=0;i<81;i++)
	{
		RR[i]=0.0;
	}
//	RR[0]=pow(0.02,2);
//	RR[1*6+1]=pow(0.02,2);
	RR[0]=pow(0.2+  0.2*w_total/10.,2);
	RR[1*9+1]=pow(0.2+  0.2*w_total/10.,2);  ////0519
	RR[2*9+2]=pow(0.1,2);
	RR[3*9+3]=pow((5.5/R_earthPara),2);
	RR[4*9+4]=pow((5.5/R_earthPara*cos(0.7)),2);
	RR[5*9+5]=pow( 3,2);
	RR[6*9+6]=pow((2+3*w_total/10.)*DEG2RAD,2);
	RR[7*9+7]=pow(0.5*DEG2RAD,2);
	RR[8*9+8]=pow(0.5*DEG2RAD,2);////动态链接苦函数结束
}
/////////////////////////////////////////////////////////////////
void matrix(double a[],double b[],double c[],short n,short m,short r)
{
	short i,j,k;
	double y;
	for(i=0;i<n;i++)
	{
		for(j=0;j<r;j++)
		{
			y=0.0;
			for(k=0;k<m;k++)
			{
				y+=a[i*m+k]*b[k*r+j];
			}
			c[i*r+j]=y;
		}
	}
}

void matrixT(double a[],double b[],double c[],short n,short m,short r)
 {
	short i,j,k;
	double y;
	for(i=0;i<n;i++)
	{
		for(j=0;j<r;j++)
		{
			y=0.0;
			for(k=0;k<m;k++)
			{
				y+=a[i*m+k]*b[j*m+k];
			}
			c[i*r+j]=y;
		}
	}
}


/*************************************************************************\
\*Function:拷贝数据到mcu
\*Parameter:
\*		pu8Data:数据首地址
\*		u16DataNum:数据数量
\*Return:
\*		void
\*************************************************************************/

int brinv(double a[],int n)
 {
	int *is,*js,i,j,k,l,u,v;
	double d,p;

    is=ismm;
	js=jsmm;

	for (k=0; k<=n-1; k++)
	{
		d=0.0;
		for (i=k; i<=n-1; i++)
		{
			for (j=k; j<=n-1; j++)
			{
				l = i*n + j;
				p = fabs(a[l]);
				if (p>d)
				{
					d=p; is[k]=i; js[k]=j;
				}
			}
		}
		if (d+1.0==1.0)
		{
			return(0);
		}
		if (is[k]!=k)
		{
			for (j=0; j<=n-1; j++)
			{
				u=k*n+j; v=is[k]*n+j;
				p=a[u]; a[u]=a[v]; a[v]=p;
			}
		}
		if (js[k]!=k)
		{
			for (i=0; i<=n-1; i++)
			{
				u=i*n+k; v=i*n+js[k];
				p=a[u]; a[u]=a[v]; a[v]=p;
			}
		}
		l=k*n+k;
		a[l]=1.0/a[l];
		for (j=0; j<=n-1; j++)
		{
			if (j!=k)
			{
				u=k*n+j; a[u]=a[u]*a[l];
			}
		}
		for (i=0; i<=n-1; i++)
		{
			if (i!=k)
			{
				for (j=0; j<=n-1; j++)
				{
					if (j!=k)
					{
						u=i*n+j;
						a[u]=a[u]-a[i*n+k]*a[k*n+j];
					}
				}
			}
		}
		for (i=0; i<=n-1; i++)
		{
			if (i!=k)
			{
				u=i*n+k; a[u]=-a[u]*a[l];
			}
		}
	}
	for (k=n-1; k>=0; k--)
	{
		if (js[k]!=k)
		{
			for (j=0; j<=n-1; j++)
			{
				u=k*n+j; v=js[k]*n+j;
				p=a[u]; a[u]=a[v]; a[v]=p;
			}
		}
		if (is[k]!=k)
		{
			for (i=0; i<=n-1; i++)
			{
				u=i*n+k; v=i*n+is[k];
				p=a[u]; a[u]=a[v]; a[v]=p;
			}
		}
	}
	return(1);
 }

void Initial(double XX[],double P[],double QQ[],double RR[],double H[],double Lat)
{
	int i;
	//double data=1.0;
	for(i=0;i<I_STATEDIMEN*I_STATEDIMEN;i++)
	{
		P[i]=0.0;
		QQ[i]=0.0;
	}

	for(i=0;i<9*I_STATEDIMEN;i++)
	{
		H[i]=0.0;
	}
    H[0]=1.0;
	H[1*I_STATEDIMEN+1]=1.0;
	H[2*I_STATEDIMEN+2]=1.0;
	H[3*I_STATEDIMEN+3]=1.0;
    H[4*I_STATEDIMEN+4]=1.0;
    H[5*I_STATEDIMEN+5]=1.0;
	H[6*I_STATEDIMEN+8]=1.0;
	H[7*I_STATEDIMEN+6]=1.0;
	H[8*I_STATEDIMEN+7]=1.0;
    for(i=0;i<I_STATEDIMEN;i++)
	{
		XX[i]=0.0;
    }

	P[0]=pow(1,2);
	P[1*I_STATEDIMEN+1]=pow(1,2);
	P[2*I_STATEDIMEN+2]=pow(3,2);

	P[3*I_STATEDIMEN+3]=pow((5./R_earthPara),2);
	P[4*I_STATEDIMEN+4]=pow((5./R_earthPara*cos(Lat)),2);
	P[5*I_STATEDIMEN+5]=pow(20,2);

	P[6*I_STATEDIMEN+6]=pow((3)*PI/180.0,2);
	P[7*I_STATEDIMEN+7]=pow((3)*PI/180.0,2);
	P[8*I_STATEDIMEN+8]=pow((50)*PI/180.0,2);///0517改1

	P[9*I_STATEDIMEN+9]=pow((100)*PI/180.0/3600,2);
	P[10*I_STATEDIMEN+10]=pow((100)*PI/180.0/3600,2);
	P[11*I_STATEDIMEN+11]=pow((100)*PI/180.0/3600,2);   ////2  0513


	P[12*I_STATEDIMEN+12]=pow(1000*9.8E-6,2);
	P[13*I_STATEDIMEN+13]=pow(1000*9.8E-6,2);
	P[14*I_STATEDIMEN+14]=pow(1000*9.8E-6,2);

	//P[15*I_STATEDIMEN+15]=pow(0.00000001,2);
	//P[16*I_STATEDIMEN+16]=pow(0.00000001,2);
	//P[17*I_STATEDIMEN+17]=pow(0.00000001,2);

	QQ[0]=pow(2E-2,2);;  /////加速度噪声
	QQ[1*I_STATEDIMEN+1]=pow(2E-2,2);;
	QQ[2*I_STATEDIMEN+2]=pow(2E-2,2);;

	QQ[3*I_STATEDIMEN+3]=pow((0.008/R_earthPara),2);   ////0519改
	QQ[4*I_STATEDIMEN+4]=pow((0.008/R_earthPara*cos(0.7)),2);
	QQ[5*I_STATEDIMEN+5]=1;

	QQ[6*I_STATEDIMEN+6]=pow(0.02*DEG2RAD,2);  ////0519改
	QQ[7*I_STATEDIMEN+7]=pow(0.02*DEG2RAD,2);
	QQ[8*I_STATEDIMEN+8]=pow(0.04*DEG2RAD,2);

	QQ[9*I_STATEDIMEN+9]=pow(0.1*SEC2RAD,2);
	QQ[10*I_STATEDIMEN+10]=pow(0.1*SEC2RAD,2);
	QQ[11*I_STATEDIMEN+11]=pow(0.1*SEC2RAD,2);

	QQ[12*I_STATEDIMEN+12]=pow(100*9.8E-6,2);  ////加速度漂移
	QQ[13*I_STATEDIMEN+13]=pow(100*9.8E-6,2);  ////加速度漂移
	QQ[14*I_STATEDIMEN+14]=pow(100*9.8E-6,2);  ////加速度漂移

	for(i=0;i<81;i++)
	{
		RR[i]=0.0;
	}

	RR[0]=pow(0.2,2);
	RR[1*9+1]=pow(0.2,2);
	RR[2*9+2]=pow(0.2,2);
	RR[3*9+3]=pow((10.5/R_earthPara),2);
	RR[4*9+4]=pow((10.5/R_earthPara*cos(0.7)),2);
	RR[5*9+5]=pow(3,2);
	RR[6*9+6]=pow(0.5*DEG2RAD,2);
	RR[7*9+7]=pow(0.5*DEG2RAD,2);
	RR[8*9+8]=pow(0.5*DEG2RAD,2);
}

double  f_max(double data1,double data2)
{

   if(data1>data2)
   {
	   return  data1;
   }
   else
   {
	   return  data2;
   }
}

void ag_initP(double  P_1[],double Lat_fun)
{
	int  i=0;
	int  j=0;
	double   f_max_p6=0.0;
	double   f_max_p7=0.0;
	double   f_max_p8=0.0;

	f_max_p6=f_max(sqrt(P_1[6*I_STATEDIMEN+6])*RADTODEG,0.2);
	f_max_p7=f_max(sqrt(P_1[7*I_STATEDIMEN+7])*RADTODEG,0.2);
	f_max_p8=f_max(sqrt(P_1[8*I_STATEDIMEN+8])*RADTODEG,0.2);
	for(i=0;i<9;i++)
	{
		for(j=0;j<I_STATEDIMEN;j++)
		{
			P_1[i*I_STATEDIMEN+j] = 0;
		}
	}
	P_1[0]=pow(1,2);
	P_1[1*I_STATEDIMEN+1]=pow(1,2);
	P_1[2*I_STATEDIMEN+2]=pow(3,2);
	P_1[3*I_STATEDIMEN+3]=pow((5./R_earthPara),2);
	P_1[4*I_STATEDIMEN+4]=pow((5./R_earthPara*cos(Lat_fun)),2);
	P_1[5*I_STATEDIMEN+5]=pow(20,2);
	P_1[6*I_STATEDIMEN+6]=pow(f_max_p6*PI/180.0,2);
	P_1[7*I_STATEDIMEN+7]=pow(f_max_p7*PI/180,2);
	P_1[8*I_STATEDIMEN+8]=pow(f_max_p8*PI/180,2);
}

void quan(double dx,double dy,double dz)
{
   double cc0,cc1,cc2,qq[4];
   int i,j,u,l;
   double Temp[16];
   cc0=sqrt(dx*dx+dy*dy+dz*dz);
   if(cc0==0.0)
   {
	   cc1=0.5;
   }
   else
   {
	   cc1=-sin(cc0/2)/cc0;
   }
   cc2=cos(cc0/2);
   for(i=0;i<=3;i++)
   {
	   for(j=0;j<=3;j++)
	   {
		   u=i*4+j;
		   if(i==j)
		   {
			   Temp[u]=cc2;
		   }
       }
   }
   Temp[1]=Temp[11]=-cc1*dx;
   Temp[4]=Temp[14]=-Temp[1];
   Temp[2]=Temp[13]=-cc1*dy;
   Temp[7]=Temp[8]=-Temp[2];
   Temp[3]=Temp[6]=-cc1*dz;
   Temp[9]=Temp[12]=-Temp[3];
   for(i=0;i<=3;i++)
	{
		qq[i]=0.0;
		for(l=0;l<=3;l++)
		{
			qq[i]=qq[i]+Temp[i*4+l]*q[l];
		}
	}
	for(i=0;i<=3;i++)
	{
	   q[i]=qq[i];
	}
}
void quan1(double dx,double dy,double dz)
{
	double cc0,cc1,qq[4],q1[4];
	int i,j,u,l;
	double Temp[16];
	cc0 = sqrt(dx*dx+dy*dy+dz*dz);
	if(cc0==0.0)
	{
	   cc1=0.5;
	}
	else
	{
	   cc1=sin(cc0/2)/cc0;
	}
	q1[0]=cos(cc0/2);
	q1[1]=cc1*dx;
	q1[2]=cc1*dy;
	q1[3]=cc1*dz;
	for(i=0;i<=3;i++)
	{
		for(j=0;j<=3;j++)
		{
			u=i*4+j;
			if(i==j) Temp[u]=q[0];
		}
	}
	Temp[1]=Temp[11]=-q[1];
	Temp[4]=Temp[14]=-Temp[1];
	Temp[2]=Temp[13]=-q[2];
	Temp[7]=Temp[8]=-Temp[2];
	Temp[3]=Temp[6]=-q[3];
	Temp[9]=Temp[12]=-Temp[3];
	for(i=0;i<=3;i++)
	{
		qq[i]=0.0;
		for(l=0;l<=3;l++)
		{
			qq[i]=qq[i]+Temp[i*4+l]*q1[l];
		}
	}
	for(i=0;i<=3;i++)
	{
		q[i]=qq[i];
	}
}


void quan_cbn_angle(double angle[],double Tnb[])
{
    Tnb[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
    Tnb[1]=2.0*(q[1]*q[2]+q[0]*q[3]);
    Tnb[2]=2.0*(q[1]*q[3]-q[0]*q[2]);
    Tnb[3]=2.0*(q[1]*q[2]-q[0]*q[3]);
    Tnb[4]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
    Tnb[5]=2.0*(q[2]*q[3]+q[0]*q[1]);
    Tnb[6]=2.0*(q[1]*q[3]+q[0]*q[2]);
    Tnb[7]=2.0*(q[2]*q[3]-q[0]*q[1]);
    Tnb[8]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
	angle[0]=atan2(Tnb[3],Tnb[4]);
	angle[1]=asin(Tnb[5]);
	angle[2]=-atan2(Tnb[2],Tnb[8]);

	angle[0]=angle[0]*180.0/3.14159265;
    angle[1]=angle[1]*180.0/3.14159265;
	angle[2]=angle[2]*180.0/3.14159265;

    if(Tnb[4]>0 && Tnb[3]>0) angle[0]=angle[0];
    if(Tnb[4]>0 && Tnb[3]<0) angle[0]=360+angle[0];
    if(Tnb[4]<0 && Tnb[3]>0) angle[0]=angle[0];
    if(Tnb[4]<0 && Tnb[3]<0) angle[0]=360+angle[0];
}


double  f_Cb_I0_It_Array[9]={0.0};    ////初始地心惯性系与实时地心惯性系的变换矩阵
double  f_Cb_E_N_Array[9]={0.0};      ////当地地心坐标系与当地地理坐标系的变换矩阵

double f_GlobalTnb[9]={0.0};
double f_C_B_b0_Array[9]={0.0};         ///// 惯性坐标系的转换矩阵
double f_Cib_Angle_Array[3]={0.0};

double f_Iner_Time=0.0;   ////惯性系对准时间

#define HISTORY_COUNT (cuduizhunSecond * 2)//粗对准时间秒数

double f_V_ib_Array_History[HISTORY_COUNT][3];
double f_V_G0_Array_History[HISTORY_COUNT][3];

double f_P_ib_Array_History[HISTORY_COUNT][3];
double f_P_G0_Array_History[HISTORY_COUNT][3];


double f_V_IB_Array_Single[3]={0.0};   ////单位时间的速度
double f_V_G0_Array_Single[3]={0.0};  ////单位时间的速度

double f_P_IB_Array_Single[3]={0.0};   ////单位时间的位置
double f_P_G0_Array_Single[3]={0.0};  ////单位时间的位置


double f_P_Tm_Ib_Array[3]={0.0};       ////位置
double f_P_Tm_G0_Array[3]={0.0};      ////

double f_V_Tm_Ib_Array[3]={0.0};
double f_V_Tm_G0_Array[3]={0.0};

double f_P_Tl_Ib_Array[3]={0.0};       ////位置
double f_P_Tl_G0_Array[3]={0.0};      ////

double f_V_Tl_Ib_Array[3]={0.0};
double f_V_Tl_G0_Array[3]={0.0};


int    I_PART_CNT=0;
int    I_TWO_CNT=0;
int    I_SINGLE_CNT=0;

int    I_GLOBAL_IMUFREQ=200; ////200HZ的IMU频率

void CAL_New_Q_Iner_ddd(double dx,double dy,double dz);

void  cal_Q_Iner()  ////惯性坐标系对准的Cib矩阵计算
{
	double  Wibbx=0.0,Wibby=0.0,Wibbz=0.0;

	Wibbx=Wibb_iner[0]*f_DT;
	Wibby=Wibb_iner[1]*f_DT;
	Wibbz=Wibb_iner[2]*f_DT;

	CAL_New_Q_Iner_ddd(Wibbx,Wibby,Wibbz);

	return;
}

void CAL_New_Q_Iner_ddd(double dx,double dy,double dz)
{
	double cc0,cc1,qq[4],q1[4];
	short i,j,u,l;
	double Temp[16];
	cc0=sqrt(dx*dx+dy*dy+dz*dz);
	if(cc0==0.0)
	{
		cc1=0.5;
	}
	else
	{
		cc1=sin(cc0/2)/cc0;
	}
	q1[0]=cos(cc0/2);
	q1[1]=cc1*dx;
	q1[2]=cc1*dy;
	q1[3]=cc1*dz;
	for(i=0;i<=3;i++)
	{
		for(j=0;j<=3;j++)
		{
			u=i*4+j;
			if(i==j) Temp[u]=q1_iner[0];
		}
	}
	Temp[1]=Temp[11]=-q1_iner[1];
	Temp[4]=Temp[14]=-Temp[1];
	Temp[2]=Temp[13]=-q1_iner[2];
	Temp[7]=Temp[8]=-Temp[2];
	Temp[3]=Temp[6]=-q1_iner[3];
	Temp[9]=Temp[12]=-Temp[3];
	for(i=0;i<=3;i++)
	{
		qq[i]=0.0;
		for(l=0;l<=3;l++)
		{
			qq[i]=qq[i]+Temp[i*4+l]*q1[l];
		}
	}
   for(i=0;i<=3;i++)
   {
	   q1_iner[i]=qq[i];
   }

}

void Cal_Angle_Iner(double angle[],double Tnb[])
{
    Tnb[0]=q1_iner[0]*q1_iner[0]+q1_iner[1]*q1_iner[1]-q1_iner[2]*q1_iner[2]-q1_iner[3]*q1_iner[3];
    Tnb[1]=2.0*(q1_iner[1]*q1_iner[2]+q1_iner[0]*q1_iner[3]);
    Tnb[2]=2.0*(q1_iner[1]*q1_iner[3]-q1_iner[0]*q1_iner[2]);
    Tnb[3]=2.0*(q1_iner[1]*q1_iner[2]-q1_iner[0]*q1_iner[3]);
    Tnb[4]=q1_iner[0]*q1_iner[0]-q1_iner[1]*q1_iner[1]+q1_iner[2]*q1_iner[2]-q1_iner[3]*q1_iner[3];
    Tnb[5]=2.0*(q1_iner[2]*q1_iner[3]+q1_iner[0]*q1_iner[1]);
    Tnb[6]=2.0*(q1_iner[1]*q1_iner[3]+q1_iner[0]*q1_iner[2]);
    Tnb[7]=2.0*(q1_iner[2]*q1_iner[3]-q1_iner[0]*q1_iner[1]);
    Tnb[8]=q1_iner[0]*q1_iner[0]-q1_iner[1]*q1_iner[1]-q1_iner[2]*q1_iner[2]+q1_iner[3]*q1_iner[3];
	angle[0]=atan2(Tnb[3],Tnb[4]);
	angle[1]=asin(Tnb[5]);
	angle[2]=-atan2(Tnb[2],Tnb[8]);

	angle[0]=angle[0]*180.0/3.14159265;
    angle[1]=angle[1]*180.0/3.14159265;
	angle[2]=angle[2]*180.0/3.14159265;

    if(Tnb[4]>0 && Tnb[3]>0) angle[0]=angle[0];
    if(Tnb[4]>0 && Tnb[3]<0) angle[0]=360+angle[0];
    if(Tnb[4]<0 && Tnb[3]>0) angle[0]=angle[0];
    if(Tnb[4]<0 && Tnb[3]<0) angle[0]=360+angle[0];
}

////计算向量叉乘

void  cal_Vector_Mul(double f_Vector_Array_1[],double f_Vector_Array_2[],double f_Vector_Result_Array[])
{
	f_Vector_Result_Array[0]=f_Vector_Array_1[1]*f_Vector_Array_2[2]-f_Vector_Array_1[2]*f_Vector_Array_2[1];
	f_Vector_Result_Array[1]=f_Vector_Array_1[2]*f_Vector_Array_2[0]-f_Vector_Array_1[0]*f_Vector_Array_2[2];
	f_Vector_Result_Array[2]=f_Vector_Array_1[0]*f_Vector_Array_2[1]-f_Vector_Array_1[1]*f_Vector_Array_2[0];
}

void  Copy_Vector(double f_Vector_In[],double f_Vector_Out[],int Num)
{
	int i=0;
	for(i=0;i<Num;i++)
	{
		f_Vector_Out[i]=f_Vector_In[i];
	}
}

void  cal_StandVector(double f_Vector_Array_in[],double f_Vector_Array_out[])
{
	double f_qq=0.0;
	int j=0;

	f_qq=f_Vector_Array_in[0]*f_Vector_Array_in[0]+f_Vector_Array_in[1]*f_Vector_Array_in[1]+f_Vector_Array_in[2]*f_Vector_Array_in[2];
	f_qq=sqrt(f_qq);

	for(j=0;j<3;j++)
	{
		f_Vector_Array_out[j]=f_Vector_Array_in[j]/f_qq;
	}
}

/////全局计算惯性系对准的函数

int  I_IS_ALIGNMENT=0;  ////是否已对准
double f_cal_Global_DH_Time=0.0;
double f_G0_i0_Array[3]={0.0};  /////用于积分的的地心固连坐标系与
double f_Acc_i0b_Array[3]={0.0};
double f_Acc_Array[3]={0.0};
double f_Gro_Array[3]={0.0};
double f_Gn_Array[3]={0.0};
double f_GnB_Array[3]={0.0};

double f_Ci0_n_Array[9]={0.0};

double f_Ci0_n0_Array[9]={0.0};

double f_Lati_GPS=39.599438375;  ////GPS纬度  116.770997156
double f_Longi_GPS=116.770997156;  ////GPS纬度
double f_Global_Gravity=9.72;
double f_Height_GPS=2.7992;

double f_U_CT_Array[9]={0.0};
double f_V_CT_Array[9]={0.0};
double f_Ib0_i0_New_Array[9]={0.0};
double f_i0_Ib0_New_Array[9]={0.0};

double f_U_Tl_Array[3]={0.0};
double f_U_Tl_Tm_Array[3]={0.0};
double f_U_Tl_Tm_Tl_Array[3]={0.0};

double f_V_Tl_Array[3]={0.0};
double f_V_Tl_Tm_Array[3]={0.0};
double f_V_Tl_Tm_Tl_Array[3]={0.0};

double  angle_Inertial[3]={0.0};

	////FILE *fp_Inertial_Angle=NULL;

double  cal_Gravity(double f_Lati,double f_Height_GPS)
{
	double  f_gravity0=0.0;
	double  f_gravity_out=0.0;
	f_gravity0=9.780318*(1.0+0.0053024*sin(f_Lati)*sin(f_Lati)-0.0000059*sin(2*f_Lati));
	f_gravity_out=f_gravity0/(1+f_Height_GPS/R_earthPara)/(1+f_Height_GPS/R_earthPara);
	return f_gravity_out;
}

void  Initial_Iner()  ////初始化惯性系对准
{
	q1_iner[0]=1.0;
	q1_iner[1]=0.0;
	q1_iner[2]=0.0;
	q1_iner[3]=0.0;
}

int  I_CAL_CNT=0;

void  cal_Global_DH(double f_Gro_Array_Fun[],double f_Acc_Array_Fun[],double f_P_V_IN_Array[],double f_Result_Array[])
{
	int j=0;
	int i=0;

	double  f_Mid_Array[9]={0.0};

	f_cal_Global_DH_Time=f_cal_Global_DH_Time+0.005;  ////全局变量

	I_CAL_CNT = I_CAL_CNT+1;

	if(f_cal_Global_DH_Time<1)
	{
		Initial_Iner();
	}


	if(I_IS_ALIGNMENT==0 && (f_cal_Global_DH_Time>DELAYTIME) )
	{
		I_IS_ALIGNMENT=1;
	}
	f_Global_Gravity=cal_Gravity(f_Lati_GPS*PI/180.0,f_Height_GPS);

	for(j=0;j<3;j++)
	{
		f_Acc_Array[j]=f_Acc_Array_Fun[j];
		f_Gro_Array[j]=f_Gro_Array_Fun[j];
	}
	if(I_IS_ALIGNMENT==1)  /////开始初始对准
	{
		cal_Q_Iner();  ////计算惯性坐标系
		Cal_Angle_Iner(f_Cib_Angle_Array,f_b0_b_Array);
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				f_C_B_b0_Array[i*3+j]=f_b0_b_Array[j*3+i];
			}
		}
		matrix(f_C_B_b0_Array ,f_Acc_Array,f_Acc_i0b_Array,3,3,1);  ////载体坐标系的加速度计在惯性系的投影，
		for(j=0;j<3;j++)
		{
			f_V_IB_Array_Single[j]=f_V_IB_Array_Single[j]+f_Acc_i0b_Array[j]*0.005;
			f_P_IB_Array_Single[j]=f_P_IB_Array_Single[j]+f_V_IB_Array_Single[j]*0.005+0.5*f_Acc_i0b_Array[j]*0.005*0.005;
		}
		f_Iner_Time=f_Iner_Time+0.005;

		f_Ci0_n_Array[0*3+0]=-sin(Wie*f_Iner_Time);
		f_Ci0_n_Array[0*3+1]=cos(Wie*f_Iner_Time);
		f_Ci0_n_Array[0*3+2]=0.0;

		f_Ci0_n_Array[1*3+0]=-sin(f_Lati_GPS*PI/180.0)*cos(Wie*f_Iner_Time);
		f_Ci0_n_Array[1*3+1]=-sin(f_Lati_GPS*PI/180.0)*sin(Wie*f_Iner_Time);
		f_Ci0_n_Array[1*3+2]=cos(f_Lati_GPS*PI/180.0);

		f_Ci0_n_Array[2*3+0]=cos(f_Lati_GPS*PI/180.0)*cos(Wie*f_Iner_Time);
		f_Ci0_n_Array[2*3+1]=cos(f_Lati_GPS*PI/180.0)*sin(Wie*f_Iner_Time);
		f_Ci0_n_Array[2*3+2]=sin(f_Lati_GPS*PI/180.0);


		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				f_Cn_i0_Array[j*3+i]=f_Ci0_n_Array[i*3+j];
			}
		}
		f_Gn_Array[0]=0.0;
		f_Gn_Array[1]=0.0;
		f_Gn_Array[2]=f_Global_Gravity;
		matrix(f_Cn_i0_Array ,f_Gn_Array,f_G0_i0_Array,3,3,1);  ////载体坐标系的加速度计在惯性系的投影，
		for(j=0;j<3;j++)
		{
			f_V_G0_Array_Single[j]=f_V_G0_Array_Single[j]+f_G0_i0_Array[j]*0.005;
			f_P_G0_Array_Single[j]=f_P_G0_Array_Single[j]+f_V_G0_Array_Single[j]*0.005+0.5*f_G0_i0_Array[j]*0.005*0.005;
		}
		I_PART_CNT=I_PART_CNT+1;
		if(I_PART_CNT>=I_GLOBAL_IMUFREQ)
		{
			for(j=0;j<3;j++)
			{
				f_V_ib_Array_History[I_SINGLE_CNT][j]=f_V_IB_Array_Single[j];
				f_V_G0_Array_History[I_SINGLE_CNT][j]=f_V_G0_Array_Single[j];

				f_P_ib_Array_History[I_SINGLE_CNT][j]=f_P_IB_Array_Single[j];
				f_P_G0_Array_History[I_SINGLE_CNT][j]=f_P_G0_Array_Single[j];
			}
			I_SINGLE_CNT=I_SINGLE_CNT+1;
			I_PART_CNT=0;
			for(j=0;j<3;j++)
			{
				f_V_IB_Array_Single[j]=0.0;
				f_V_G0_Array_Single[j]=0.0;

				f_P_IB_Array_Single[j]=0.0;
				f_P_G0_Array_Single[j]=0.0;
			}
		}
	}


	if(I_SINGLE_CNT%2==0&&I_SINGLE_CNT>=40) /////
	{

		for(i=0;i<3;i++)
		{
			f_P_Tm_Ib_Array[i]=0.0;
			f_P_Tm_G0_Array[i]=0.0;

			f_V_Tm_Ib_Array[i]=0.0;
			f_V_Tm_G0_Array[i]=0.0;

			f_P_Tl_Ib_Array[i]=0.0;
			f_P_Tl_G0_Array[i]=0.0;

			f_V_Tl_Ib_Array[i]=0.0;
			f_V_Tl_G0_Array[i]=0.0;

		}

		I_TWO_CNT=I_SINGLE_CNT/2;

		for(i=0;i<I_TWO_CNT;i++)
		{
			for(j=0;j<3;j++)
			{
				f_V_Tl_Ib_Array[j]=f_V_Tl_Ib_Array[j]+f_V_ib_Array_History[i][j];
				f_V_Tl_G0_Array[j]=f_V_Tl_G0_Array[j]+f_V_G0_Array_History[i][j];

				f_P_Tl_Ib_Array[j]=f_P_Tl_Ib_Array[j]+f_V_Tl_Ib_Array[j]*1.0+f_P_ib_Array_History[i][j];
				f_P_Tl_G0_Array[j]=f_P_Tl_G0_Array[j]+f_V_Tl_G0_Array[j]*1.0+f_P_G0_Array_History[i][j];


				f_V_Tm_Ib_Array[j]=f_V_Tm_Ib_Array[j]+f_V_ib_Array_History[i][j];
				f_V_Tm_G0_Array[j]=f_V_Tm_G0_Array[j]+f_V_G0_Array_History[i][j];

				f_P_Tm_Ib_Array[j]=f_P_Tm_Ib_Array[j]+f_V_Tm_Ib_Array[j]*1.0+f_P_ib_Array_History[i][j];
				f_P_Tm_G0_Array[j]=f_P_Tm_G0_Array[j]+f_V_Tm_G0_Array[j]*1.0+f_P_G0_Array_History[i][j];
			}
		}

		for(i=0;i<I_TWO_CNT;i++)
		{
			for(j=0;j<3;j++)
			{
				f_V_Tm_Ib_Array[j]=f_V_Tm_Ib_Array[j]+f_V_ib_Array_History[i+I_TWO_CNT][j];
				f_V_Tm_G0_Array[j]=f_V_Tm_G0_Array[j]+f_V_G0_Array_History[i+I_TWO_CNT][j];

				f_P_Tm_Ib_Array[j]=f_P_Tm_Ib_Array[j]+f_V_Tm_Ib_Array[j]*1.0+f_P_ib_Array_History[i+I_TWO_CNT][j];
				f_P_Tm_G0_Array[j]=f_P_Tm_G0_Array[j]+f_V_Tm_G0_Array[j]*1.0+f_P_G0_Array_History[i+I_TWO_CNT][j];
			}
		}
		////计算
		cal_StandVector(f_P_Tl_G0_Array,f_U_Tl_Array);
		cal_Vector_Mul(f_P_Tl_G0_Array,f_P_Tm_G0_Array,f_Mid_Array);
		cal_StandVector(f_Mid_Array,f_U_Tl_Tm_Array);
		cal_Vector_Mul(f_U_Tl_Tm_Array,f_P_Tl_G0_Array,f_Mid_Array);
		cal_StandVector(f_Mid_Array,f_U_Tl_Tm_Tl_Array);



		cal_StandVector(f_P_Tl_Ib_Array,f_V_Tl_Array);
		cal_Vector_Mul(f_P_Tl_Ib_Array,f_P_Tm_Ib_Array,f_Mid_Array);
		cal_StandVector(f_Mid_Array,f_V_Tl_Tm_Array);

		cal_Vector_Mul(f_V_Tl_Tm_Array,f_P_Tl_Ib_Array,f_Mid_Array);
		cal_StandVector(f_Mid_Array,f_V_Tl_Tm_Tl_Array);


		for(j=0;j<3;j++)
		{
			f_U_CT_Array[0*3+j]=f_U_Tl_Array[j];
			f_U_CT_Array[1*3+j]=f_U_Tl_Tm_Array[j];
			f_U_CT_Array[2*3+j]=f_U_Tl_Tm_Tl_Array[j];


			f_V_CT_Array[0*3+j]=f_V_Tl_Array[j];
			f_V_CT_Array[1*3+j]=f_V_Tl_Tm_Array[j];
			f_V_CT_Array[2*3+j]=f_V_Tl_Tm_Tl_Array[j];

		}

		brinv(f_U_CT_Array,3);
		matrix(f_U_CT_Array, f_V_CT_Array,f_Ib0_i0_New_Array,3,3,3);
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				f_i0_Ib0_New_Array[i*3+j]=f_Ib0_i0_New_Array[j*3+i];
			}
		}
	}

	f_Ci0_n0_Array[0*3+0]=0.0;
	f_Ci0_n0_Array[0*3+1]=cos(Wie*0);
	f_Ci0_n0_Array[0*3+2]=0.0;

	f_Ci0_n0_Array[1*3+0]=-sin(f_Lati_GPS*PI/180.0)*cos(Wie*0);
	f_Ci0_n0_Array[1*3+1]=-sin(f_Lati_GPS*PI/180.0)*sin(Wie*0);
	f_Ci0_n0_Array[1*3+2]=cos(f_Lati_GPS*PI/180.0);

	f_Ci0_n0_Array[2*3+0]=cos(f_Lati_GPS*PI/180.0)*cos(Wie*0);
	f_Ci0_n0_Array[2*3+1]=cos(f_Lati_GPS*PI/180.0)*sin(Wie*0);
	f_Ci0_n0_Array[2*3+2]=sin(f_Lati_GPS*PI/180.0);

	matrix(f_Ci0_n0_Array, f_Ib0_i0_New_Array,f_Inertial_b0N0_Last,3,3,3);

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			f_Inertial_N0b0_Last[i*3+j]=f_Inertial_b0N0_Last[j*3+i];
		}
	}

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			f_Mid_Array[i*3+j]=f_Ci0_n_Array[j*3+i];
		}
	}
	matrix(f_Ci0_n0_Array,f_Mid_Array,f_Inertial_N_N0,3,3,3); ////f_Mid_Array 为Cn_i0


	matrix(f_b0_b_Array,	f_Inertial_N0b0_Last,f_Mid_Array,3,3,3);
	matrix(f_Mid_Array,f_Inertial_N_N0,f_TCB_Inertial_Last,3,3,3);

	////姿态角提取
	angle_Inertial[0]=atan2(f_TCB_Inertial_Last[3],f_TCB_Inertial_Last[4]);
	angle_Inertial[1]=asin(f_TCB_Inertial_Last[5]);
	angle_Inertial[2]=-atan2(f_TCB_Inertial_Last[2],f_TCB_Inertial_Last[8]);

	angle_Inertial[0]=angle_Inertial[0]*180.0/3.14159265;
	angle_Inertial[1]=angle_Inertial[1]*180.0/3.14159265;
	angle_Inertial[2]=angle_Inertial[2]*180.0/3.14159265;

	if(f_TCB_Inertial_Last[4]>0 && f_TCB_Inertial_Last[3]>0) angle_Inertial[0]=angle_Inertial[0];
	if(f_TCB_Inertial_Last[4]>0 && f_TCB_Inertial_Last[3]<0) angle_Inertial[0]=360+angle_Inertial[0];
	if(f_TCB_Inertial_Last[4]<0 && f_TCB_Inertial_Last[3]>0) angle_Inertial[0]=angle_Inertial[0];
	if(f_TCB_Inertial_Last[4]<0 && f_TCB_Inertial_Last[3]<0) angle_Inertial[0]=360+angle_Inertial[0];

	if(fabs(angle_Inertial[0])>10)
	{
		angle_Inertial[0]=angle_Inertial[0];
	}
	f_Iner_Time=f_Iner_Time+0.005;

	f_Result_Array[0]=angle_Inertial[0];
	f_Result_Array[1]=angle_Inertial[1];
	f_Result_Array[2]=angle_Inertial[2];

}



void norm_quan()
{
   int i;
   double qq=0.0;
   for(i=0;i<=3;i++)
   {
	   qq=qq+q[i]*q[i];
   }
   qq=sqrt(qq);
   for(i=0;i<=3;i++)
   {
	   q[i]=q[i]/qq;
   }
}




double b[I_STATEDIMEN*I_STATEDIMEN],b1[I_STATEDIMEN*I_STATEDIMEN];
void DeFia(int n,double FI[],double F[],double DT1)
{
	int i,j;
    matrix(F,F,b,n,n,n);
    matrix(F,b,b1,n,n,n);
	for(i=0;i<n;i++)
    {
		for(j=0;j<n;j++)
        {
			FI[i*n+j]=DT1*F[i*n+j]+(b[i*n+j]+DT1/3.0*b1[i*n+j])*DT1*DT1/2.0;
			if(i==j)
			{
				FI[i*n+j]+=1.0;
			}
        }
	}
}

///////////////////////////////////////////////////////////////
int Kalman(int n,int m,double QQ[],double FI[],double H[],double RR[],double P[],double K[],double DT1)
{
	short i,j,Inv;
	double b[500],c[500];
	double cc[500],s[500],add[500],p1[500];

	matrix(FI,QQ,c,n,n,n);
	matrixT(c,FI,b,n,n,n);

	for(i=0;i<n;i++)
	{
		//for(j=0;j<n;j++) p1[i*n+j]=b[i*n+j]*DT1*DT1;
		for(j=0;j<n;j++) p1[i*n+j]=b[i*n+j]*DT1;
	}
	matrix(FI,P,c,n,n,n);
	matrixT(c,FI,b,n,n,n);
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			P[i*n+j]=p1[i*n+j]+b[i*n+j];
		}
	}

	matrixT(P,H,cc,n,n,m);  ///////  H(6*18)

	matrix(H,cc,s,m,n,m);

	for(i=0;i<m;i++)
	{
		for(j=0;j<m;j++)
		{
			add[i*m+j]=RR[i*m+j]+s[i*m+j];
		}
	}

	Inv=brinv(add,m);
	if(Inv==0)
	{
		return(Inv);
	}
    for(i=0;i<n;i++)
	{
    	for(j=0;j<n;j++)
    	{
    		p1[j*n+i]=P[i*n+j];
    	}
	}
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			b[i*n+j]=(p1[i*n+j]+P[i*n+j])/2;
		}
	}
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			P[i*n+j]=b[i*n+j];
		}
	}

	matrixT(P,H,cc,n,n,m);
	matrix(cc,add,K,n,m,m);
	matrix(K,H,c,n,m,n);
	/////////////////////////////////////////////
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			p1[i*n+j]=-c[i*n+j];
			if(i==j)
			{
				p1[i*n+j]+=1.0;
			}
		}
	}
	matrix(p1,P,c,n,n,n);
	matrixT(c,p1,b,n,n,n);
    matrix(K,RR,cc,n,m,m);
	matrixT(cc,K,p1,n,m,n); /////  K(n*m )
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			P[i*n+j]=p1[i*n+j]+b[i*n+j];
		}
	}
	return(Inv);
}


//////////////////////////////////////////////////////////////////

int Est(int n,int m,double K[],double FI1[],double Zg[],double XX[])
{
	int i;
	double x1[I_STATEDIMEN],x2[I_STATEDIMEN];
    matrix(K,Zg,x1,n,m,1);
	matrix(FI1,XX,x2,n,n,1);
	for(i=0;i<9;i++)
	{
		XX[i]=x1[i];
	}
	for(i=9;i<n;i++)
	{
		XX[i]=x1[i]+x2[i];
	}
   return(1);
}



/********************************************************************************\
\*Function:采样地/信号选择使能
\*Parameter:
\*		GND_SEL_DISENABLE/GND_SEL_ENABLE
\*Return:
\*		void
\********************************************************************************/

/********************************************************************************/
/********************************************************************************/

void  quan_Mine(double dx,double dy,double dz)
{
   double cc0,cc1,cc2,qq[4];
   int i,j,u,l;
   double Temp[16];
   cc0=sqrt(dx*dx+dy*dy+dz*dz);
   if(cc0==0.0)
   {
	   cc1=0.5;
   }
   else
   {
	   cc1=-sin(cc0/2)/cc0;
   }
   cc2=cos(cc0/2);
   for(i=0;i<=3;i++)
   {
	   for(j=0;j<=3;j++)
	   {
		   u=i*4+j;
		   if(i==j)
		   {
			   Temp[u]=cc2;
		   }
       }
   }
   Temp[1]=Temp[14]=-cc1*dx;
   Temp[4]=Temp[11]=-Temp[1];
   Temp[2]=Temp[7]=-cc1*dy;
   Temp[8]=Temp[13]=-Temp[2];
   Temp[3]=Temp[9]=-cc1*dz;
   Temp[6]=Temp[12]=-Temp[3];
	for(i=0;i<=3;i++)
	{
		qq[i]=0.0;
		for(l=0;l<=3;l++)
		{
			qq[i]=qq[i]+Temp[i*4+l]*q[l];
		}
	}
	for(i=0;i<=3;i++)
	{
		q[i]=qq[i];
	}
}
/////全局变量
int Initial_attitude_flg=0;////////1:姿态对准；0：姿态未对准
int Initial_position_flg=0;////////1:位置对准；0：位置未对准
int Initial_head_flg=0;////////1:方位对准；0：方位对准
////double  fai_CMD=0.0;  ////外部传来的航向信息

double antlever_x1=0.0;//杆壁X  m
double antlever_y1=0.0;//杆壁Y  m
double antlever_z1=0.0;//杆壁Z  m

void NavInit(NavInitStr* p)  /////接口函数1,导航参数初始化
{
    antlever_x1=p->antlever_x;
    antlever_y1=p->antlever_y;//杆壁Y  m
	antlever_z1=p->antlever_y;//杆壁Z  m
	Lat=p->lat*DEG2RAD; //纬度 deg
	Lon=p->lon*DEG2RAD; //经度 deg
	HH=p->alt; //alt高度 m
	H1=p->alt; //alt高度 m

	Lat_CMD=p->lat;
	Lon_CMD=p->lon;
	HH_CMD=p->alt;
	f_SHEXIANG_ANGLE=p->fai0;  ////射向角

	//////////初始化卡尔曼滤波,2024.0219,改  原为Initial(XX_ARRAY,P_ARRAY,QQ_ARRAY,RR_ARRAY,H_ARRAY,lat);
	Initial(XX_ARRAY,P_ARRAY,QQ_ARRAY,RR_ARRAY,H_ARRAY,p->lat);
	return;

}


int I_READ_CNT=0;

unsigned char I_STATE_DH=0;////命令状态
unsigned char I_STATE_DH_PRE=0;  ////命令状态字上一时刻

double start_heading=0.0;
double start_pitch=0.0;
double start_roll=0.0;

int I_AIR_CNT=0;
int I_IS_INITPOS=0;  /////GPS是否首次收到,位置初始化.
int I_IS_INIT_HEAD=0;  /////GPS是否首次收到,航向初始化.

void  cal_nav(double  Wibb_in[],double f_acc_array_in[])
{
	int     i=0;
//	int     j=0;
	double  Wibb[3]={0.0};
	double  Ax=0.0;
    double  Ay=0.0;
	double  Az=0.0;
	for(i=0;i<3;i++)
	{
		Wibb[i]=Wibb_in[i];
	}
    Ax=f_acc_array_in[0];
	Ay=f_acc_array_in[1];
	Az=f_acc_array_in[2];
	g0_ORG = 9.780318*(1+0.0053024*sin(Lat)*sin(Lat)-0.0000059*sin(2*Lat)*sin(2*Lat));
	Grv = g0_ORG/(1+HH/6378137)/(1+HH/6378137);
	Re=R_earthPara*(1+f_earthPara*sin(Lat)*sin(Lat));
	Rn=R_earthPara*(1-2*f_earthPara+3*f_earthPara*sin(Lat)*sin(Lat));
	Roe=-Vn/Rn;
	Ron=Ve/Re;
	Rou=Ron*tan(Lat);
	We=Roe;
	Wn=Ron+Wie*cos(Lat);
	Wu=Rou+Wie*sin(Lat);
	De=(We)*DT;
	Dn=(Wn)*DT;
	Du=(Wu)*DT;
	quan_cbn_angle(angle,Tnb);
	dx_wie=Tnb[0*3+0]*(-De)+Tnb[0*3+1]*(-Dn)+Tnb[0*3+2]*(-Du);
	dy_wie=Tnb[1*3+0]*(-De)+Tnb[1*3+1]*(-Dn)+Tnb[1*3+2]*(-Du);
	dz_wie=Tnb[2*3+0]*(-De)+Tnb[2*3+1]*(-Dn)+Tnb[2*3+2]*(-Du);
	quan1(dx_wie,dy_wie,dz_wie);
	d_klm_X=Tnb[0*3+0]*(dklm_faiE)+Tnb[0*3+1]*(dklm_faiN)+Tnb[0*3+2]*(dklm_faiU);
	d_klm_Y=Tnb[1*3+0]*(dklm_faiE)+Tnb[1*3+1]*(dklm_faiN)+Tnb[1*3+2]*(dklm_faiU);
	d_klm_Z=Tnb[2*3+0]*(dklm_faiE)+Tnb[2*3+1]*(dklm_faiN)+Tnb[2*3+2]*(dklm_faiU);
	quan1(d_klm_X,d_klm_Y,d_klm_Z);
	dklm_faiE=0;
	dklm_faiN=0;
	dklm_faiU=0;
	Cx=Wibb[0]*DT;
	Cy=Wibb[1]*DT;
	Cz=Wibb[2]*DT;
	quan1(Cx,Cy,Cz);
	norm_quan();
	quan_cbn_angle(angle,Tnb);
	dFe=(Tnb[0]*Ax+Tnb[3]*Ay+Tnb[6]*Az)*DT;
	dFn=(Tnb[1]*Ax+Tnb[4]*Ay+Tnb[7]*Az)*DT;
	dFu=(Tnb[2]*Ax+Tnb[5]*Ay+Tnb[8]*Az)*DT;
	Fe=Tnb[0]*Ax+Tnb[3]*Ay+Tnb[6]*Az;
	Fn=Tnb[1]*Ax+Tnb[4]*Ay+Tnb[7]*Az;
	Fu=Tnb[2]*Ax+Tnb[5]*Ay+Tnb[8]*Az;
	Ve1=Ve+dFe+((Wie*sin(Lat)+Wu)*Vn-(Wie*cos(Lat)+Wn)*Vu)*DT;
	Vn1=Vn+dFn+(-(Wie*sin(Lat)+Wu)*Ve+We*Vu)*DT;
	Vu1=Vu+dFu+((Wie*cos(Lat)+Wn)*Ve-We*Vn-Grv)*DT;
	Dse=((Tnb[0]*Ax+Tnb[3]*Ay+Tnb[6]*Az)+((Wie*sin(Lat)+Wu)*Vn-(Wie*cos(Lat)+Wn)*Vu)*DT)*DT/2.0;
	Dsn=((Tnb[1]*Ax+Tnb[4]*Ay+Tnb[7]*Az)+(-(Wie*sin(Lat)+Wu)*Ve+We*Vu)*DT)*DT/2.0;
	Dsh=((Tnb[2]*Ax+Tnb[5]*Ay+Tnb[8]*Az)+((Wie*cos(Lat)+Wn)*Ve-We*Vn-Grv)*DT)*DT/2.0;

	Dsh = 0;
	Dsn = 0;
	Dse = 0;
	Lat1=Lat+(DT*Vn1+Dsn)/Rn;
	Lon1=Lon+(DT*Ve1+Dse)/(Re*cos(Lat));
	H1=HH+Vu1*DT+Dsh;
	DT1=DT1+0.005;
}

/////计算F 整开始
int I_AVERAGE_FF_CNt=0;
double F_ARRAY_AVE[I_STATEDIMEN*I_STATEDIMEN]={0.0};
void cal_FF(double  Wibb[],int  I_KLM_VALID_FUN)
{
	int   i=0;
/////F阵计算开始
	for(i=0;i<I_STATEDIMEN*I_STATEDIMEN;i++)
	{
		F_ARRAY_ORG[i]=0.0;
	}
//-----------------第1行------Ve--------------------------------
	F_ARRAY_ORG[0]=(Vn*tan(Lat)-Vu)/Re;
	F_ARRAY_ORG[1]=Wu+Wie*sin(Lat);
	F_ARRAY_ORG[2]=-(Wn+Wie*cos(Lat));
	F_ARRAY_ORG[3]=2*Wie*(Vu*sin(Lat)+Vn*cos(Lat))+Ron*Vn/cos(Lat)/cos(Lat);
	F_ARRAY_ORG[7]=-Fu;
	F_ARRAY_ORG[8]=Fn;
	F_ARRAY_ORG[12]=Tnb[0];
	F_ARRAY_ORG[13]=Tnb[3];
	F_ARRAY_ORG[14]=Tnb[6];
//-----------------第2行------Vn--------------------------------
	F_ARRAY_ORG[1*I_STATEDIMEN]=-2*Wu;
	F_ARRAY_ORG[1*I_STATEDIMEN+1]=-Vu/Rn;
	F_ARRAY_ORG[1*I_STATEDIMEN+2]=Roe;
	F_ARRAY_ORG[1*I_STATEDIMEN+3]=-(2*Ve*Wie*cos(Lat)+Ve*Ron/cos(Lat)/cos(Lat));
	F_ARRAY_ORG[1*I_STATEDIMEN+6]=Fu;
	F_ARRAY_ORG[1*I_STATEDIMEN+8]=-Fe;
	F_ARRAY_ORG[1*I_STATEDIMEN+12]=Tnb[1];
	F_ARRAY_ORG[1*I_STATEDIMEN+13]=Tnb[4];
	F_ARRAY_ORG[1*I_STATEDIMEN+14]=Tnb[7];
//------------------第3行-------Vu-----------------------------------
	F_ARRAY_ORG[2*I_STATEDIMEN]=2*Wn;
	F_ARRAY_ORG[2*I_STATEDIMEN+1]=-2*Roe;
	F_ARRAY_ORG[2*I_STATEDIMEN+3]=-2*Ve*Wie*sin(Lat);
	F_ARRAY_ORG[2*I_STATEDIMEN+6]=-Fn;
	F_ARRAY_ORG[2*I_STATEDIMEN+7]=Fe;
	F_ARRAY_ORG[2*I_STATEDIMEN+12]=Tnb[2];
	F_ARRAY_ORG[2*I_STATEDIMEN+13]=Tnb[5];
	F_ARRAY_ORG[2*I_STATEDIMEN+14]=Tnb[8];
///------------------第4行------Dfi-----------------------------------
	F_ARRAY_ORG[3*I_STATEDIMEN+1]=1.0/Rn;
///------------------第5行-------Dlamda-------------------------------
	F_ARRAY_ORG[4*I_STATEDIMEN+0]=1.0/(Re*cos(Lat));
	F_ARRAY_ORG[4*I_STATEDIMEN+3]=Ron*tan(Lat)/cos(Lat);
//-------------------第6行-------Dh----------------------------------
	F_ARRAY_ORG[5*I_STATEDIMEN+2]=1.0;
///-------------------第7行-------俯仰角----------------------------------
	F_ARRAY_ORG[6*I_STATEDIMEN+1]=-1.0/Rn;
	F_ARRAY_ORG[6*I_STATEDIMEN+7]=Wu;
	F_ARRAY_ORG[6*I_STATEDIMEN+8]=-Wn;
	F_ARRAY_ORG[6*I_STATEDIMEN+9]=Tnb[0];
	F_ARRAY_ORG[6*I_STATEDIMEN+10]=Tnb[3];
	F_ARRAY_ORG[6*I_STATEDIMEN+11]=Tnb[6];
///-------------------第8行-----------------------------------------
	F_ARRAY_ORG[7*I_STATEDIMEN+0]=1.0/Re;
	F_ARRAY_ORG[7*I_STATEDIMEN+3]=-Wie*sin(Lat);
	F_ARRAY_ORG[7*I_STATEDIMEN+6]=-Wu;
	F_ARRAY_ORG[7*I_STATEDIMEN+8]=We;
	F_ARRAY_ORG[7*I_STATEDIMEN+9]=Tnb[1];
	F_ARRAY_ORG[7*I_STATEDIMEN+10]=Tnb[4];
	F_ARRAY_ORG[7*I_STATEDIMEN+11]=Tnb[7];
//---------------------第9行---------------------------------------
	F_ARRAY_ORG[8*I_STATEDIMEN+0]=tan(Lat)/Re;
	F_ARRAY_ORG[8*I_STATEDIMEN+3]=Ron/cos(Lat)/cos(Lat)+Wie*cos(Lat);
	F_ARRAY_ORG[8*I_STATEDIMEN+6]=  Wn;
	F_ARRAY_ORG[8*I_STATEDIMEN+7]=-We;
	F_ARRAY_ORG[8*I_STATEDIMEN+9]=Tnb[2];
	F_ARRAY_ORG[8*I_STATEDIMEN+10]=Tnb[5];
	F_ARRAY_ORG[8*I_STATEDIMEN+11]=Tnb[8];
	I_AVERAGE_FF_CNt=I_AVERAGE_FF_CNt+1;
	for(i=0;i<I_STATEDIMEN*I_STATEDIMEN;i++)
	{
		F_ARRAY_AVE[i]=F_ARRAY_AVE[i]*(I_AVERAGE_FF_CNt-1)/I_AVERAGE_FF_CNt+F_ARRAY_ORG[i]/I_AVERAGE_FF_CNt;
	}
	if(I_KLM_VALID_FUN==1)
	{
		for(i=0;i<I_STATEDIMEN*I_STATEDIMEN;i++)
		{
			F_ARRAY[i]=F_ARRAY_AVE[i];
			F_ARRAY_AVE[i]=0;
		}
		I_AVERAGE_FF_CNt=0;
	}
}

void  cal_Z_AND_R_ZITAI(double  f_ZITAI_ARRAY_Fun[],int  I_VALID_ARRAY[],double  f_acc_array[3],double  f_Z_array[],double  f_RR_Array[])
{
	double   fai_ange_STD=0.0;  ///航向角标准
	double   f_theta_ange_STD=0.0;  ///航向角标准
	double   f_gama_ange_STD=0.0;  ///航向角标准
	double   f_average_ax=0.0;
	double   f_average_ay=0.0;
	double   f_average_az=0.0;
	double  f_acc_root=0.0;
	f_average_ax = f_acc_array[0];
	f_average_ay = f_acc_array[1];
	f_average_az = f_acc_array[2];
	f_acc_root = sqrt(f_average_ax*f_average_ax+f_average_ay*f_average_ay+f_average_az*f_average_az);
	fai_ange_STD = f_ZITAI_ARRAY_Fun[0];
	f_theta_ange_STD = f_ZITAI_ARRAY_Fun[1];
	f_gama_ange_STD = f_ZITAI_ARRAY_Fun[2];
	f_Z_array[0]=Ve1-0;
	f_Z_array[1]=Vn1-0;
	f_Z_array[2]=Vu1-0;
	f_Z_array[3]=0;
	f_Z_array[4]=0;
	f_Z_array[5]=0;
	f_Z_array[6]=(angle[0]-fai_ange_STD)*PI/180;
	f_Z_array[7]=(angle[1]-f_theta_ange_STD)*PI/180;
	f_Z_array[8]=(angle[2]-f_gama_ange_STD)*PI/180;
	f_RR_Array[0]=pow(0.1,2);
	f_RR_Array[1*9+1]=pow(0.1,2);
	f_RR_Array[2*9+2]=pow(0.1,2);
	f_RR_Array[3*9+3]=pow((500000/R_earthPara),2);
	f_RR_Array[4*9+4]=pow((500000/R_earthPara*cos(0.7)),2);
	f_RR_Array[5*9+5]=pow(500000,2);
	f_RR_Array[6*9+6]=pow(0.1*DEG2RAD,2);
	f_RR_Array[7*9+7]=pow((0.1+(f_acc_root/9.8-1)*0.3)*DEG2RAD,2);
	f_RR_Array[8*9+8]=pow((0.1+(f_acc_root/9.8-1)*0.3)*DEG2RAD,2);
}

double  f_average_Ax=0.0;
double  f_average_Ay=0.0;
double  f_average_Az=0.0;
int   I_Average_Cnt=0;

void  cal_acc_angle(double  f_fai_std,double f_Acc_Array_ForDh[],double  f_angle_array_std[],int  *ptr_I_VALID)
{
	double  f_Pitch_ave=0.0;
	double  f_roll_ave=0.0;
	I_Average_Cnt=I_Average_Cnt+1;
	f_average_Ax=f_average_Ax*(I_Average_Cnt-1)/I_Average_Cnt+f_Acc_Array_ForDh[0]/I_Average_Cnt;
	f_average_Ay=f_average_Ay*(I_Average_Cnt-1)/I_Average_Cnt+f_Acc_Array_ForDh[1]/I_Average_Cnt;
	f_average_Az=f_average_Az*(I_Average_Cnt-1)/I_Average_Cnt+f_Acc_Array_ForDh[2]/I_Average_Cnt;
	f_angle_array_std[0]=f_fai_std;
	f_Pitch_ave=atan(f_average_Ay/sqrt(f_average_Ax*f_average_Ax+f_average_Az*f_average_Az))*180/PI;
	f_roll_ave=atan2(-f_average_Ax,f_average_Az)*180/PI;
	*ptr_I_VALID=0;
	if(I_Average_Cnt>=800)
	{
        f_angle_array_std[0]=f_fai_std;        /////航向标准
		f_angle_array_std[1]=f_Pitch_ave;        /////标准
		f_angle_array_std[2]=f_roll_ave;        /////航向标准
		f_average_Ax=0;
		f_average_Ay=0;
		f_average_Az=0;
		I_Average_Cnt=0;
		*ptr_I_VALID=1;
	}
}

void  cal_Z_GPS(double  Wibb_FUN[],double  f_alarm_array[],double  f_GPS_ARRAY_Fun[],int  I_VALID_ARRAY[],double  f_gps_delay_array_fun[])
{
	double f_GPS_Ve_IN_Fun=0.0;
	double f_GPS_Vn_IN_Fun=0.0;
	double f_GPS_Vu_IN_Fun=0.0;
	double f_GPS_lati_IN_Fun=0.0;
	double f_GPS_height_IN_Fun=0.0;
    double f_GPS_longi_IN_Fun=0.0;
    double f_GPS_HEAD_IN_fun=0.0;
//	int    I_GPS_VALID_Fun=0;
	int    I_GPS_HEAD_VALID_Fun=0;
	double  f_Delta_Head=0.0;
//	I_GPS_VALID_Fun=I_VALID_ARRAY[0];
	I_GPS_HEAD_VALID_Fun=I_VALID_ARRAY[1];
	f_GPS_Ve_IN_Fun=f_GPS_ARRAY_Fun[0];
	f_GPS_Vn_IN_Fun=f_GPS_ARRAY_Fun[1];
	f_GPS_Vu_IN_Fun=f_GPS_ARRAY_Fun[2];
	f_GPS_longi_IN_Fun=f_GPS_ARRAY_Fun[3];
	f_GPS_lati_IN_Fun=f_GPS_ARRAY_Fun[4];
	f_GPS_height_IN_Fun =f_GPS_ARRAY_Fun[5];
	f_GPS_HEAD_IN_fun=f_GPS_ARRAY_Fun[6];
	Pn=angle[1]*PI/180.0;
	Rolln=angle[2]*PI/180.0;
	Yn=angle[0]*PI/180.0;
	Pn=angle[1]*PI/180.0;
	Rolln=angle[2]*PI/180.0;
	Yn=angle[0]*PI/180.0;
	T[0][0]=cos(Rolln)*cos(Yn)+sin(Yn)*sin(Rolln)*sin(Pn);
	T[0][1]=sin(Yn)*cos(Pn);
	T[0][2]=sin(Rolln)*cos(Yn)-cos(Rolln)*sin(Yn)*sin(Pn);
	T[1][0]=-cos(Rolln)*sin(Yn)+sin(Rolln)*cos(Yn)*sin(Pn);
	T[1][1]=cos(Yn)*cos(Pn);
	T[1][2]=-sin(Yn)*sin(Rolln)-cos(Rolln)*cos(Yn)*sin(Pn);
	T[2][0]=-sin(Rolln)*cos(Pn);
	T[2][1]=sin(Pn);
	T[2][2]=cos(Rolln)*cos(Pn);
	Wwe=T[0][0]*Wibb_FUN[0]+T[0][1]*Wibb_FUN[1]+T[0][2]*Wibb_FUN[2]-We;
	Wwn=T[1][0]*Wibb_FUN[0]+T[1][1]*Wibb_FUN[1]+T[1][2]*Wibb_FUN[2]-Wn;
	Wwu=T[2][0]*Wibb_FUN[0]+T[2][1]*Wibb_FUN[1]+T[2][2]*Wibb_FUN[2]-Wu;
	Le=T[0][0]*Rx+T[0][1]*Ry+T[0][2]*Rz;  /////外杆臂,
	Ln=T[1][0]*Rx+T[1][1]*Ry+T[1][2]*Rz;  /////外杆臂
	Lu=T[2][0]*Rx+T[2][1]*Ry+T[2][2]*Rz;  /////外杆臂
	Vwe=Wwn*Lu-Wwu*Ln;
	Vwn=Wwu*Le-Wwe*Lu;
	Vwu=Wwe*Ln-Wwn*Le;
	if(I_STATE_DH==3 || I_STATE_DH==4)//转导航
	{
		Zg_ARRAY[0]=Ve1-f_GPS_Ve_IN_Fun-f_gps_delay_array_fun[0];  ////GPS延迟-f_gps_delay_array_fun[0]东向速度
		Zg_ARRAY[1]=Vn1-f_GPS_Vn_IN_Fun-f_gps_delay_array_fun[1];  ////GPS延迟-f_gps_delay_array_fun[1]东向速度
		Zg_ARRAY[2]=Vu1-f_GPS_Vu_IN_Fun-f_gps_delay_array_fun[2];  ////GPS延迟-f_gps_delay_array_fun[2]东向速度
		Zg_ARRAY[3]=Lat1-f_GPS_lati_IN_Fun*DEG2RAD-f_gps_delay_array_fun[3]/R0;
		Zg_ARRAY[4]=Lon1-f_GPS_longi_IN_Fun*DEG2RAD-f_gps_delay_array_fun[4]/R0/0.766;
		Zg_ARRAY[5]=H1-f_GPS_height_IN_Fun-f_gps_delay_array_fun[5];
		if(I_GPS_HEAD_VALID_Fun==1)
		{
			f_Delta_Head=angle[0]-f_GPS_HEAD_IN_fun;
			if(f_Delta_Head>180)
			{
				f_Delta_Head=f_Delta_Head-360;
			}
			else if(f_Delta_Head<-180)
			{
				f_Delta_Head=f_Delta_Head+360;
			}
			Zg_ARRAY[6]=f_Delta_Head*PI/180;
		}
		else
		{
			Zg_ARRAY[6]=0.0;
		}
		Zg_ARRAY[7]=0;
		Zg_ARRAY[8]=0;
	}
}

int I_IS_INIT_HX = 0;
void Klm_Total(double  f_Delta_Time)
{
	DeFia(I_STATEDIMEN,FI_ARRAY,F_ARRAY,f_Delta_Time);
	Kalman(I_STATEDIMEN,I_OBSDIMEN,QQ_ARRAY,FI_ARRAY,H_ARRAY,RR_ARRAY,P_ARRAY,K_ARRAY,f_Delta_Time);
	Est(I_STATEDIMEN,I_OBSDIMEN,K_ARRAY,FI_ARRAY,Zg_ARRAY,XX_ARRAY);
}

int  I_IS_W0_Zero_Finished=0;
double	Wx_Zero_realtime_jiaoyan=0.0;
double	Wy_Zero_realtime_jiaoyan=0.0;
double	Wz_Zero_realtime_jiaoyan=0.0;
double	Wx_Zero_realtime_jiaoyan_ORG=0.0;
double	Wy_Zero_realtime_jiaoyan_ORG=0.0;
double	Wz_Zero_realtime_jiaoyan_ORG=0.0;
	

double  f_Static_Time_Array[10] = {0};
double  f_HJ_HEAD = 0.0;  		////航迹角,GPS  速度计算
int I_HJ_VALID_GPS = 0;      	////0616改
double f_angle_std_save[3]={0};
double f_acc_add_save[10][3]={{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};   /////加速度延迟补偿量
double f_acc_one_array[3]={0.0};
int    I_GPS_DELAY_CNT=0;   /////GPS延迟计数
double f_gps_delay_array[6]={0.0};
int I_GPS_CNT_SERIAL_INDEX=0;    /////I_GPS_CNT_SERIAL_INDEX    GPS  I_GPS_CNT_SERIAL_INDEX串口收数计数
int I_GPS_CNT_PPS_INDEX=0;   /////I_GPS_CNT_PPS_INDEX    GPS  I_GPS_CNT_PPS_INDEX串口收数计数
int I_AVERAGE_CNT_W0 = 0;

void NavLoop(NAV_INPUT_IMU* pin1,NAV_INPUT_GPS* pin2, NAV_INPUT_QBAR* pin3, NAV_INPUT_MAG* pin4, NAV_OUTPUT* pout)
{
	I_STATE_DH = pin1->cmd;
	int  i=0;
  	int     I_IMU_QUALITY_out=0;
	double  omb[3]={0.0};//,gpsv=0.0;


	double  f_lati_GPS=0.0;
	double  f_Longi_GPS=0.0;
	double  f_Height_GPS=0.0;

	double f_VN_GPS=0.0;
	double f_VU_GPS=0.0;
	double f_VE_GPS=0.0;

	double f_Headangle_GPS=0.0;

	int   I_VALID_GPS=0;
	int   I_HEAD_VALID_GPS=0;

	int   I_NAV_STATE=0;


	double r_Wx_Org;
	double r_Wy_Org;
	double r_Wz_Org;
	double r_Ax_Org;
	double r_Ay_Org;
	double r_Az_Org;

	double  f_Wx=0.0;
	double  f_Wy=0.0;
	double  f_Wz=0.0;

	double  f_Ax=0.0;
	double  f_Ay=0.0;
	double  f_Az=0.0;

	double  f_Acc0_Array[3]={0.013720832 ,-0.031660706,	0.001189469}; ////加表表度已表定0422标定

	double  f_acc_alignment_array[3][3]={{1.0000000E+0,0.0000000E+0,0.0000000E+0},
	{0.0,1.0,0.000},
	{0.0,0.0,1.0}};


	double  f_Gro0_Array[3] = {0.0, 0.0, 0.0};   /////{ 664.2757908,	-348.3568404,	236.7094752};
	double  f_Gro_Scale_Array[3] = {1, 1, 1};
	double  f_gro_alignment_array[3][3] = { {1.0, 0.0, 0.0},
											{0.0, 1.0, 0.0},
											{0.0, 0.0, 1.0} };


	double ascal[3]={1.000308272,  1.000937658 , 1.000207102};////加表表度已表定0422标定
	double wscal_zheng[3]={0.991925694,0.955045139,1.004225694};  ////陀螺标0422标定  	double wscal_Fu[3]={1.123325139,	0.842437056,0.906712431}; /////2024.05.13 标定结果
	double  wscal_Fu[3]={1.0097761,0.955045139,1.003390972};  ////加陀螺标0422标定    	double wscal_zheng[3]={0.996401458,	1.171089375	,1.06955305};  /////2024.05.13 标定结果

	double Ax=0.0,Ay=0.0,Az=0.0,Ab[3];  ////B系的加速度,

	double f_Gro_Array_ForDh[3]={0.0};
	double f_Acc_Array_ForDh[3]={0.0};

	double f_P_V_IN_Array_ForDh[3]={0.0};
	double f_Result_Array_ForDh[3]={0.0};


	double  f_GPS_lati_IN=30.0;////7	GPS纬度（10HZ）
	double  f_GPS_longi_IN=106.0;////8	GPS经度
	double  f_GPS_height_IN=300.0;////9	GPS高度
	int     I_GPS_VALID_IN=0;////15	GPS有效标志

	int   j=0;
	double  Wibb[3]={0.0};

	int   I_ZT_VALID=0;

	double  f_angle_std[3]={0.0};
	double  Wibb_in[3]={0.0};
	double  f_acc_array_in[3]={0.0};
	int   I_VALID_ARRAY[2]={0};
	double f_alarm_array[3]={0.0};  ////外杆臂
	double  f_GPS_ARRAY_Fun1[7]={0.0};
	double  f_delta_imu_gpsHX=0.0;

 	f_DLL_Time_global=f_DLL_Time_global+0.005;
	
    ////辅助信息

    ////IMU数据传递完毕

    /////GPS数据传递开始
	I_GPS_VALID_IN=pin2->Status1;////gps有效标志
	f_GPS_lati_IN=pin2->Lat;
	f_GPS_longi_IN=pin2->Lon;
	f_GPS_height_IN=pin2->Height;


    f_lati_GPS=pin2->Lat;
    f_Longi_GPS=pin2->Lon;
    f_Height_GPS=pin2->Height;
//    f_PDOP=0.0;

	f_VN_GPS=pin2->Vel_north;
	f_VU_GPS=pin2->Vel_UP;
	f_VE_GPS=pin2->Vel_east;

	f_Headangle_GPS=pin2->Heading;

    f_lati_GPS=pin2->Lat;
    f_Longi_GPS=pin2->Lon;
    f_Height_GPS=pin2->Height;

//	f_UTCTIME_GPS=pin2->utc;
//	f_PPS_SEC=pin2->PPS;

	I_VALID_GPS=pin2->Status1;
	I_HEAD_VALID_GPS=pin2->Status2;

	r_Wx_Org = pin1->DATA_XGYRO;
	r_Wy_Org = pin1->DATA_YGYRO;
	r_Wz_Org = pin1->DATA_ZGYRO;
               
	r_Ax_Org = pin1->DATA_XACCEL;
	r_Ay_Org = pin1->DATA_YACCEL;
	r_Az_Org = pin1->DATA_ZACCEL;

	f_Wx=r_Wx_Org/1.000+0;
	f_Wy=r_Wy_Org/1.000+0;
	f_Wz=r_Wz_Org/1.000+0;

	f_Ax=r_Ax_Org/1.000*1;
	f_Ay=r_Ay_Org/1.000*1;
	f_Az=r_Az_Org/1.000*1;
	if(r_Wx_Org>0)
	{
		r_Wx_Org=r_Wx_Org/wscal_zheng[0];
	}
	if(r_Wx_Org<0)
	{
		r_Wx_Org=r_Wx_Org/wscal_Fu[0];
	}
	if(r_Wy_Org>0)
	{
		r_Wy_Org=r_Wy_Org/wscal_zheng[1];
	}
	if(r_Wy_Org<0)
	{
		r_Wy_Org=r_Wy_Org/wscal_Fu[1];
	}
	if(r_Wz_Org>0)
	{
		r_Wz_Org=r_Wz_Org/wscal_zheng[2];
	}
	if(r_Wz_Org<0)
	{
		r_Wz_Org=r_Wz_Org/wscal_Fu[2];
	}
	  ////导航解算开始

	omb[0]=f_Wx;
	omb[1]=f_Wy;
	omb[2]=f_Wz;

	Ab[0]=f_Ax;
	Ab[1]=f_Ay;
	Ab[2]=f_Az;

///------------陀螺校零-------------------
	for(i=0;i<3;i++)
	{
		Ab[i]=(Ab[i]-f_Acc0_Array[i])/ascal[i];
	}
	Ax=(Ab[0]*f_acc_alignment_array[0][0]+Ab[1]*f_acc_alignment_array[0][1]+Ab[2]*f_acc_alignment_array[0][2]);
	Ay=(Ab[0]*f_acc_alignment_array[1][0]+Ab[1]*f_acc_alignment_array[1][1]+Ab[2]*f_acc_alignment_array[1][2]);
	Az=(Ab[0]*f_acc_alignment_array[2][0]+Ab[1]*f_acc_alignment_array[2][1]+Ab[2]*f_acc_alignment_array[2][2]);
//---------陀螺仪安装误差修正-----------------------------------------------------------
	for(i=0;i<3;i++)
	{
		omb[i]=(omb[i]-f_Gro0_Array[i])/f_Gro_Scale_Array[i];
	}
	Wibb_iner[0]=(omb[0]*f_gro_alignment_array[0][0]+omb[1]*f_gro_alignment_array[0][1]+omb[2]*f_gro_alignment_array[0][2])*3.14159265/180.;
	Wibb_iner[1]=(omb[0]*f_gro_alignment_array[1][0]+omb[1]*f_gro_alignment_array[1][1]+omb[2]*f_gro_alignment_array[1][2])*3.14159265/180.;
	Wibb_iner[2]=(omb[0]*f_gro_alignment_array[2][0]+omb[1]*f_gro_alignment_array[2][1]+omb[2]*f_gro_alignment_array[2][2])*3.14159265/180.;

    f_Acc_Array_ForDh[0]=(Ax);
	f_Acc_Array_ForDh[1]=(Ay);
	f_Acc_Array_ForDh[2]=(Az);
    if((fabs(Ay)>350))////超过35g,那么用大量程加表
	{
		//f_Acc_Array_ForDh[1]=(f_DATA_ACC_BIG_X_IN);
	}

	f_Gro_Array_ForDh[0]=Wibb_iner[0];
	f_Gro_Array_ForDh[1]=Wibb_iner[1];
	f_Gro_Array_ForDh[2]=Wibb_iner[2];



	for(j=0;j<3;j++)
	{
		Wibb[j]=f_Gro_Array_ForDh[j];
	}
	Ax=f_Acc_Array_ForDh[0];
	Ay=f_Acc_Array_ForDh[1];
	Az=f_Acc_Array_ForDh[2];

	if(I_STATE_DH == 0)
	{
		;/////空循环
	}
	else if(I_STATE_DH == 1) /////惯性系对准状态
	{
		if(f_DLL_Time_global<cuduizhunSecond)//20秒粗对准
		{
			cal_Global_DH(f_Gro_Array_ForDh,f_Acc_Array_ForDh,f_P_V_IN_Array_ForDh,f_Result_Array_ForDh);////初始对准
			cal_acc_angle(f_SHEXIANG_ANGLE,f_Acc_Array_ForDh,f_angle_std,&I_ZT_VALID);  //////根据加速度计,测量
			start_heading=f_Result_Array_ForDh[0];
			start_pitch=f_Result_Array_ForDh[1];
			start_roll=f_Result_Array_ForDh[2];

			Fia0=(f_SHEXIANG_ANGLE+0.0)*PI/180.;////摄向角
			P0=(start_pitch+0.0)*PI/180.;
			R0=(start_roll+0.0)*PI/180.;
			q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
			q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
			q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
			q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
			norm_quan();
			quan_cbn_angle(angle,Tnb);

			Ve=0;
			Vn=0;
			Vu=0;
			Ve1=0;
			Vn1=0;
			Vu1=0;

			Lat1= Lat_CMD*DEG2RAD;
			Lon1= Lon_CMD*DEG2RAD;
			H1= HH_CMD;

			Lat= Lat_CMD*DEG2RAD;
			Lon= Lon_CMD*DEG2RAD;
			HH= HH_CMD;
			I_NAV_STATE=1;//当前状态对准中
		}
		if(f_DLL_Time_global>=cuduizhunSecond && f_DLL_Time_global<=cuduizhunSecond + jingduizhunSecond) /////陀螺仪校零状态
		{
			//20秒到180秒精对准
			I_NAV_STATE = 1;//当前状态对准中
			/////通用模块结束,导航模块
			for(i=0;i<3;i++)
			{
				Wibb_in[i]=f_Gro_Array_ForDh[i];
				f_acc_array_in[i]=f_Acc_Array_ForDh[i];
			}

			cal_acc_angle(f_SHEXIANG_ANGLE,f_Acc_Array_ForDh,f_angle_std,&I_ZT_VALID);  //////根据加速度计,测量

			cal_nav(Wibb_in, f_acc_array_in);
			Ve=0;
			Ve1=0;
			Vn=0;
			///----------------------------------------------------------------------------------
			Vn1=0;
			Vu=0;
			Vu1=0;
			f_W0_Time = f_W0_Time + 0.005;
			if(I_ZT_VALID==1)
			{
				if(I_IS_INIT_HX==0)
				{
					I_IS_INIT_HX=1;
					Fia0=f_SHEXIANG_ANGLE*PI/180.;////摄向角
					P0=(f_angle_std[1]+0.0)*PI/180.;
					R0=(f_angle_std[2]+0.0)*PI/180.;
					q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
					q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
					q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
					q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
					norm_quan();
					quan_cbn_angle(angle,Tnb);
					f_W0_Time=0.0;
				}
				f_delta_hx=(f_angle_std[0]-angle[0] );
				if(f_delta_hx>180.0)
				{
					f_delta_hx=f_delta_hx-360.0;
				}
				if(f_delta_hx<-180.0)
				{
					f_delta_hx=f_delta_hx+360.0;
				}
				if(fabs(f_W0_Time)>=9.99)  ////容错处理
				{
					I_AVERAGE_CNT_W0=I_AVERAGE_CNT_W0+1;
					Wz_Zero_realtime_jiaoyan_ORG=Wz_Zero_realtime_jiaoyan_ORG*(I_AVERAGE_CNT_W0-1)/I_AVERAGE_CNT_W0+ f_delta_hx/f_W0_Time/I_AVERAGE_CNT_W0;
					Wx_Zero_realtime_jiaoyan_ORG=Wx_Zero_realtime_jiaoyan_ORG*((I_AVERAGE_CNT_W0-1))/I_AVERAGE_CNT_W0+(angle[1]-f_angle_std[1])/f_W0_Time/I_AVERAGE_CNT_W0;
					Wy_Zero_realtime_jiaoyan_ORG=Wy_Zero_realtime_jiaoyan_ORG*(I_AVERAGE_CNT_W0-1)/I_AVERAGE_CNT_W0+(angle[2]-f_angle_std[2])/f_W0_Time/I_AVERAGE_CNT_W0;
					P0=(f_angle_std[1]+0.0)*PI/180.;
					R0=(f_angle_std[2]+0.0)*PI/180.;
					q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
					q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
					q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
					q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
					norm_quan();
					quan_cbn_angle(angle,Tnb);
					f_W0_Time=0.0;
				}

				for(i=0;i<3;i++)
				{
					f_angle_std_save[i]=f_angle_std[i];
				}
			}///if(I_ZT_VALID==1)
		}/////if(f_DLL_Time_global>=20.0&&f_DLL_Time_global<=180.0)

		if(f_DLL_Time_global>=120.0)////姿态导航，对准完成前就进入导航了
		{
			Wibb_in[0]=f_Gro_Array_ForDh[0]-Wx_Zero_realtime_jiaoyan_ORG*DEGTORAD;
			Wibb_in[1]=f_Gro_Array_ForDh[1]-Wy_Zero_realtime_jiaoyan_ORG*DEGTORAD;
			Wibb_in[2]=f_Gro_Array_ForDh[2]-Wz_Zero_realtime_jiaoyan_ORG*DEGTORAD;
			for(i=0;i<3;i++)
			{
				f_acc_array_in[i]=f_Acc_Array_ForDh[i];
			}
			I_NAV_STATE = 2;
			cal_nav(Wibb_in, f_acc_array_in);
			Ve=0;
			Vn=0;
			Vu=0.0;
			Ve1=0;
			Vn1=0;
			Vu1=0.0;
			Lat1= Lat_CMD*DEG2RAD;
			Lon1= Lon_CMD*DEG2RAD;
			H1= HH_CMD;

			Lat= Lat_CMD*DEG2RAD;
			Lon= Lon_CMD*DEG2RAD;
			HH= HH_CMD;
		}
	}//对准完成
	else if(I_STATE_DH==3||I_STATE_DH==2)//转导航指令
	{
		for(i=0;i<3;i++)
		{
			f_acc_array_in[i]=f_Acc_Array_ForDh[i];
		}
        Wibb_in[0]=f_Gro_Array_ForDh[0]-Wx_Zero_realtime_jiaoyan_ORG*DEGTORAD;
		Wibb_in[1]=f_Gro_Array_ForDh[1]-Wy_Zero_realtime_jiaoyan_ORG*DEGTORAD;
		Wibb_in[2]=f_Gro_Array_ForDh[2]-Wz_Zero_realtime_jiaoyan_ORG*DEGTORAD;
		cal_nav(  Wibb_in, f_acc_array_in);
		Ve=0;
		Vn=0;
		Vu=0.0;
		Ve1=0;
		Vn1=0;
		Vu1=0.0;
		Lat1= Lat_CMD*DEG2RAD;
		Lon1= Lon_CMD*DEG2RAD;
		H1= HH_CMD;

		Lat= Lat_CMD*DEG2RAD;
		Lon= Lon_CMD*DEG2RAD;
		HH= HH_CMD;

		cal_acc_angle(f_SHEXIANG_ANGLE,f_Acc_Array_ForDh,f_angle_std,&I_ZT_VALID);

		if(I_ZT_VALID==1)
		{
			for(i=0;i<3;i++)
			{
				f_angle_std_save[i]=f_angle_std[i];
			}
            Fia0=angle[0]*PI/180.;////摄向角
			P0=(f_angle_std[1]+0.0)*PI/180.;
			R0=(f_angle_std[2]+0.0)*PI/180.;
			q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
			q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
			q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
			q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
			norm_quan();
			quan_cbn_angle(angle,Tnb);
		}

  		if(I_STATE_DH_PRE != I_STATE_DH)  ////收到再次对准指令，航向回零
  		{
			Fia0=f_SHEXIANG_ANGLE*PI/180.;////摄向角
			P0=(f_angle_std_save[1]+0.0)*PI/180.;///0616改原为  P0=(f_angle_std[1]+0.0)*PI/180.;
			R0=(f_angle_std_save[2]+0.0)*PI/180.;///0616改原为  P0=(f_angle_std[2]+0.0)*PI/180.;
			q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
			q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
			q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
			q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
			norm_quan();
			quan_cbn_angle(angle,Tnb);
  		}
		I_STATE_DH_PRE=I_STATE_DH;
		I_NAV_STATE=3;
	}
	else if(I_STATE_DH > 3 && I_STATE_DH < 10)////发射后
	{
		/////校零确定,第一拍
		if(I_IS_W0_Zero_Finished==0)
		{
			I_IS_W0_Zero_Finished=1;
			Wx_Zero_realtime_jiaoyan=Wx_Zero_realtime_jiaoyan_ORG;
			Wy_Zero_realtime_jiaoyan=Wy_Zero_realtime_jiaoyan_ORG;
			Wz_Zero_realtime_jiaoyan=Wz_Zero_realtime_jiaoyan_ORG;

			Fia0=f_SHEXIANG_ANGLE*PI/180.;////摄向角
			
			P0=(f_angle_std_save[1]+0.0)*PI/180.;
			R0=(f_angle_std_save[2]+0.0)*PI/180.;
			q[0]=cos(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0);
			q[1]=cos(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0)+sin(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0);
			q[2]=cos(Fia0/2.0)*cos(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*sin(P0/2.0)*cos(R0/2.0);
			q[3]=cos(Fia0/2.0)*sin(P0/2.0)*sin(R0/2.0)-sin(Fia0/2.0)*cos(P0/2.0)*cos(R0/2.0);
			norm_quan();
			quan_cbn_angle(angle,Tnb);
		//========================================================
		}

		/////校零确定,第一拍,结束,并将初始的对准姿态角赋予导航的状态

		/////通用模块开始,导航模块
		/////通用模块结束,导航模块\
		////-------------纯捷联导航开始------------------------//
		////-------------1,纯捷联导航结束------------------------
		/////2辅助信息开始
		/////2.0为初次收到GPS 的位置和速度,开始
		f_GPS_LOST_TIME_TOTAL = f_GPS_LOST_TIME_TOTAL + 0.005;  ////位置速度丢失时间累积
		if(I_GPS_VALID_IN == 1)
		{
			f_GPS_LOST_TIME_TOTAL = 0;////GPS丢失的时间
		}
		f_GPS_LOSTHEAD_TIME_TOTAL=f_GPS_LOSTHEAD_TIME_TOTAL+0.005;   ////航向丢失时间累积
		if(I_HEAD_VALID_GPS == 1)
		{
            if(sqrt(f_VE_GPS*f_VE_GPS + f_VN_GPS*f_VN_GPS)<5.0||fabs(angle[1])>30.0||fabs(angle[2])>30.0)  /////当速度太低，或者府仰角，横滚角大于30度，那么航迹无效
			{
            	I_HEAD_VALID_GPS=0;
			}
		}
		if(I_HEAD_VALID_GPS==1)
		{
			f_GPS_LOSTHEAD_TIME_TOTAL=0.0;////GPS丢失的时间
		}
		if(f_GPS_LOST_TIME_TOTAL>30.0)  /////状态转换
		{
			I_IS_INITPOS = 0;
			I_IS_INIT_HEAD = 0;
		}
		if(I_GPS_VALID_IN==1)
		{
			f_GPS_LOST_TIME_TOTAL=0.0;////GPS丢失的时间
		}
		if(f_GPS_LOST_TIME_TOTAL<30.0 && (I_IS_INITPOS==1))  /////GPS位置，速度组合
		{
			I_NAV_STATE|=0x10; ////GPS位置速度有效
		}
		else
		{
			I_NAV_STATE&=0xEF;////GPS位置速度无效
		}
		if(f_GPS_LOST_TIME_TOTAL<30.0&&I_IS_INIT_HEAD==1)  /////GPS航向，位置，速度组合
		{
			I_NAV_STATE|=0x20;  ////GPS航向有效
		}
		else
		{
			I_NAV_STATE&=0xDF;  ////GPS航向无效
		}
		if(I_GPS_VALID_IN==1&&I_IS_INITPOS==0)
		{
			if(I_IS_INITPOS==0)
			{
				I_IS_GPS_SET_GLOBAL_INIT_CNT = I_IS_GPS_SET_GLOBAL_INIT_CNT + 1;
				if(I_IS_GPS_SET_GLOBAL_INIT_CNT<=10)  ////分4拍修正速度
				{
					Ve=Ve-(Ve-f_VE_GPS)/3.0;
					Vu=Vu-(Vu-f_VU_GPS)/3.0;
					Vn=Vn-(Vn-f_VN_GPS)/3.0;
					//	Vu=0.0;

					Ve1=Ve1-(Ve1-f_VE_GPS)/3.0;
					Vu1=Vu1-(Vu1-f_VU_GPS)/3.0;
					Vn1=Vn1-(Vn1-f_VN_GPS)/3.0;

					Lon1=Lon1-(Lon1-f_GPS_longi_IN*PI/180.0)/4;
					Lat1=Lat1-(Lat1-f_GPS_lati_IN*PI/180.0)/4;
					H1=H1-(H1-f_GPS_height_IN)/4;

					Lon=Lon1;
					Lat=Lat1;
					HH=H1;
				}
                else
                {
                	Ve=f_VE_GPS;
					Vn=f_VN_GPS;
					Vu=f_VU_GPS;
					Ve1=f_VE_GPS;
					Vu1=f_VU_GPS;
					Vn1=f_VN_GPS;
					Lon=f_GPS_longi_IN*PI/180.0;
					Lat=f_GPS_lati_IN*PI/180.0;
					HH=f_GPS_height_IN;
					Lon1=f_GPS_longi_IN*PI/180.0;
					Lat1=f_GPS_lati_IN*PI/180.0;
					H1=f_GPS_height_IN;
				}
				if(I_IS_GPS_SET_GLOBAL_INIT_CNT==10)
				{
					I_IS_INITPOS=1;
					I_IS_GPS_SET_GLOBAL_INIT_CNT=0;
				}
			}
			f_Global_Gravity=cal_Gravity(f_GPS_lati_IN*PI/180.0,f_Height_GPS);   /////2024,0219改,原为   f_Global_Gravity=cal_Gravity(f_lati_GPS*PI/180.0,f_Height_GPS);
			DT1=0;
			I_GPS_VALID_IN=0;
			f_GPS_LOST_TIME_TOTAL=0.0;
			ag_initP(P_ARRAY, 0.766);
		}
		Wibb_in[0]=f_Gro_Array_ForDh[0]-Wx_Zero_realtime_jiaoyan*DEGTORAD;
		Wibb_in[1]=f_Gro_Array_ForDh[1]-Wy_Zero_realtime_jiaoyan*DEGTORAD;
		Wibb_in[2]=f_Gro_Array_ForDh[2]-Wz_Zero_realtime_jiaoyan*DEGTORAD;
		for(i=0;i<3;i++)
		{
			f_acc_array_in[i]=f_Acc_Array_ForDh[i];
		}
		cal_nav(  Wibb_in, f_acc_array_in);
	  	for(i=9;i>=1;i--)
	  	{
			for(j=0;j<3;j++)
			{
				f_acc_add_save[i][j]=f_acc_add_save[i-1][j];
			}
	  	}
	  	f_acc_one_array[0]=Fe;
		f_acc_one_array[1]=Fn;
		f_acc_one_array[2]=Fu-9.80;
		for(j=0;j<3;j++)
		{
			f_acc_add_save[0][j]=f_acc_one_array[j];
		}
		///		 f_AIR_V=sqrt(f_GPS_Ve_IN*f_GPS_Ve_IN+f_GPS_Vn_IN*f_GPS_Vn_IN);
        ////2.1  大气信息结束
		/////2辅助信息结束
		for(j=0;j<6;j++)
		{
		    f_gps_delay_array[j]=0.0;	
		}
		I_GPS_CNT_SERIAL_INDEX=(int)(f_DLL_Time_global*200);    /////I_GPS_CNT_SERIAL_INDEX    GPS  I_GPS_CNT_SERIAL_INDEX串口收数计数
		I_GPS_CNT_PPS_INDEX=(int)(f_DLL_Time_global*200);   /////I_GPS_CNT_PPS_INDEX    GPS  I_GPS_CNT_PPS_INDEX串口收数计数
		I_GPS_DELAY_CNT=0;

		//chj 请杨兴注意此处是否写错了
		if((I_GPS_CNT_PPS_INDEX>I_GPS_CNT_SERIAL_INDEX)&&(I_GPS_CNT_SERIAL_INDEX-I_GPS_CNT_PPS_INDEX)<10)
		{
			I_GPS_DELAY_CNT = I_GPS_CNT_SERIAL_INDEX - I_GPS_CNT_PPS_INDEX;
		}

		for(i=0;i<I_GPS_DELAY_CNT;i++)
		{
		   for(j=0;j<3;j++)
		   {
			   f_gps_delay_array[j]= f_gps_delay_array[j]+f_acc_add_save[i][j]*0.005;
		   }

		}
		f_gps_delay_array[3]=f_VN_GPS*I_GPS_DELAY_CNT*0.005;////北向位置延迟误差(米)
		f_gps_delay_array[4]=f_VE_GPS*I_GPS_DELAY_CNT*0.005;////北向位置延迟误差(米)
		f_gps_delay_array[5]=f_VU_GPS*I_GPS_DELAY_CNT*0.005;////北向位置延迟误差(米)
		////GPS组合,GPS 航向组合
		if(I_STATE_DH==3||I_STATE_DH==4)
		{
			Kalmanflg=I_GPS_VALID_IN;
		}
	    cal_FF(Wibb,Kalmanflg);
		if(Kalmanflg != 0)
		{
			/////F阵计算开始
			cal_RR(RR_ARRAY,Wibb_in);
			f_alarm_array[0]=antlever_z1;
			f_alarm_array[1]= antlever_x1;
			f_alarm_array[2]=antlever_y1;
			f_GPS_ARRAY_Fun1[0]=f_VE_GPS;
			f_GPS_ARRAY_Fun1[1]=f_VN_GPS;
			f_GPS_ARRAY_Fun1[2]=f_VU_GPS;

			//	  if(f_GPS_lati_IN>1&&f_GPS_longi_IN>1)
			f_GPS_ARRAY_Fun1[3]=f_Longi_GPS;
			f_GPS_ARRAY_Fun1[4]=f_lati_GPS;
			f_GPS_ARRAY_Fun1[5]=f_Height_GPS;

			///	  f_GPS_ARRAY_Fun1[6]=f_Headangle_GPS; ////test  0414
			
			///  I_HEAD_VALID_GPS=I_VALID_GPS;////test  0414
			f_GPS_ARRAY_Fun1[6]=f_Headangle_GPS; /////0519, f_GPS_ARRAY_Fun1[6]=f_HJ_HEAD;
			I_VALID_ARRAY[0]=I_VALID_GPS;
			I_VALID_ARRAY[1]=I_HEAD_VALID_GPS;
			////卡尔曼率波解算开始
			cal_Z_GPS(  Wibb,f_alarm_array,f_GPS_ARRAY_Fun1,I_VALID_ARRAY,f_gps_delay_array);
			cal_RR(RR_ARRAY,Wibb_in);
			Klm_Total(0.1);
			dklm_faiE=XX_ARRAY[6];
			dklm_faiN=XX_ARRAY[7];
			dklm_faiU=XX_ARRAY[8];
			Ve=Ve1-XX_ARRAY[0];
			Vn=Vn1-XX_ARRAY[1];
			Vu=Vu1-XX_ARRAY[2];
			Lat=Lat1-XX_ARRAY[3];
			Lon=Lon1-XX_ARRAY[4];
			HH=H1-XX_ARRAY[5];
			for(i=0;i<9;i++)
			{
				XX_ARRAY[i]=0.0;
			}
		}
		else
		{
			Ve=Ve1;
			Vn=Vn1;
			Vu=Vu1;
			Lat=Lat1;
			Lon=Lon1;
			HH=H1;
		}
		////卡尔曼率波解算结束
		cal_acc_angle(f_Headangle_GPS,f_acc_array_in,f_angle_std,&I_ZT_VALID);	////调试,水平姿态求解开始
		if(I_ZT_VALID==1)
		{
			for(i=0;i<3;i++)
			{
					  f_angle_std_save[i]=f_angle_std[i];
			}
		}///if(I_ZT_VALID==1)
		f_delta_imu_gpsHX=angle[0]-f_Headangle_GPS;
		if(f_delta_imu_gpsHX > 180)
		{
			f_delta_imu_gpsHX = f_delta_imu_gpsHX - 360.0;
		}
		else if(f_delta_imu_gpsHX < -180.0)
		{
			f_delta_imu_gpsHX = f_delta_imu_gpsHX + 360.0;
		}
		/////卡尔曼率波结束
	}
	////导航解算
	pout->Ax= f_acc_array_in[0];
	pout->Ay= f_acc_array_in[1];
	pout->Az= f_acc_array_in[2];

	pout->Wx=Wibb_in[0]*RAD2DEG;
	pout->Wy=Wibb_in[1]*RAD2DEG;
	pout->Wz=Wibb_in[2]*RAD2DEG;

	pout->datastat = I_IMU_QUALITY_out;

	pout->lat = Lat;
	pout->lon = Lon;
	pout->alt = HH;
	pout->v_n = Vn;
	pout->v_d = Vu;
	pout->v_e = Ve;
	pout->pich = angle[1];
	pout->roll = angle[2];
	pout->fai = angle[0];

	pout->navstat = I_NAV_STATE;
	pout->TS = pin1->DATA_TIME_STMP1ms*0.001;
}

