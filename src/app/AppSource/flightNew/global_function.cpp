#include "global_function.h"
//#include "MissileModel_src/environment_earth_model.h"
#include "port/flightPort.h"

void CFlightGlobalFun::Tomas(double lon1, 
							 double lat1, 
							 double lon2, 
							 double lat2, 
							 double *sdm, 
							 double *a12)
{
	double dlon = 0.0;
	double tmp1,tmp2,tmp3,tmp4,tmp5;
	double u1 = 0.0;
	double u2 = 0.0;
	double L1 = 0.0;
	double sigma = 0.0;
	double U1 = 0.0;
	double V1 = 0.0;
	double X1 = 0.0;
	double Y1 = 0.0;
	double A1 = 0.0;
	double C1 = 0.0;
	double n1 = 0.0;
	double n2 = 0.0;
	double n3 = 0.0;
	double dSigma1 = 0.0;
	double dSigma2 = 0.0;
	double M1 = 0.0;
	double F1 = 0.0;
	double G1 = 0.0;
	double Q1 = 0.0;
	double dL = 0.0;
	double t1 = 0.0;
	double t2 = 0.0;

	dlon = (lon2 - lon1)/RTOA;     
	u1 = atan((1.0 - E_CONST) * tan(lat1/RTOA));
	u2 = atan((1.0 - E_CONST) * tan(lat2/RTOA));

	tmp1 = sin((u2 - u1) / 2.0);
	tmp2 = cos((u2 - u1) / 2.0);
	tmp3 = sin((u1 + u2) / 2.0);
	tmp4 = sin(dlon / 2.0);
	L1 = tmp1 * tmp1 + (tmp2 * tmp2 - tmp3 * tmp3) * tmp4 * tmp4;
	if (L1 >= 1.0)
	{
		L1 = 1.0 - (1.0e-15);
	}
	if (L1 < 1.0e-15)
	{
		L1 = 1.0e-15;
	}

	sigma = 2.0 * asin(sqrt(L1));
	tmp4 = cos((u1 - u2) / 2.0);

	U1 = 2.0 * tmp3 * tmp3 * tmp4 * tmp4 / Nozero_FUN(1.0 - L1);

	tmp3 = sin((u1 - u2) / 2.0);
	tmp4 = cos((u1 + u2) / 2.0);
	V1 = 2.0 * tmp3 * tmp3 * tmp4 * tmp4 / L1;
	X1 = U1 + V1;
	Y1 = U1 - V1;

	tmp3 = sin(sigma);
	tmp4 = cos(sigma);
	tmp5 = sigma / tmp3;
	A1 = 8.0 * tmp5 * tmp5 * tmp4;
	C1 = tmp5 - (A1 - 2.0 * tmp4) / 2.0;
	n1 = X1 * (A1 + C1 * X1);
	n2 = Y1 * (8.0 * tmp5 * tmp5 + 2.0 * Y1 * tmp4);
	n3 = 4.0 * tmp5 * X1 * Y1;
	dSigma1 = E_CONST * (tmp5 * X1 - Y1) / 4.0;
	dSigma2 = E_CONST * E_CONST * (n1 - n2 + n3) / 64.0;

	if(sdm != 0)
		*sdm = RE * sigma - RE * tmp3 * (dSigma1 - dSigma2);/*弹目距离*/

	M1 = 32.0 * tmp5 - (20.0 * tmp5 - A1) * X1 - (8.0 * tmp5 * tmp5 + 4.0) * Y1;
	F1 = 2.0 * Y1 - 2.0 * tmp4 * (4.0 - X1);
	G1 = E_CONST * tmp5 / 2.0 + E_CONST * E_CONST * M1 / 64.0;
	Q1 = -(F1 * G1 * tan(dlon)) / 4.0;
	dL = (dlon + Q1) / 2.0;

	tmp3 = sin((u1 + u2) / 2.0);
	tmp4 = cos((u1 + u2) / 2.0);
	t1 = -QATN(tmp1 * cos(dL), tmp4 * sin(dL));
	t2 = -QATN(-tmp2 * cos(dL), tmp3 * sin(dL));

	if(a12 != 0)
		*a12 = (t1 + t2 - 2.0 * PI * (int)((t1 + t2) / (2.0 * PI)))*RTOA;/*大地方位角*/
}


void CFlightGlobalFun::Vincenty_Forward(double vin_l1,
										double vin_b1,
										double s,
										double alfa12,
										double* vin_l2_ptr,
										double* vin_b2_ptr)
{
	double re=6378137.0;
	double e=1.0/298.257;
	double rp=(1-e)*re;
	double e12=(re*re-rp*rp)/(rp*rp);
	double a,b,c,d,f,m,sigmam_2,tan_u1;
	double k1,sin_m;
	double u1,sigma1,sigma,sigma_b,Delta_sigma,dlamda;
	double x1,y1;
	int i = 0;

	tan_u1=(1-e)*tan(vin_b1/RTOA);
	u1=atan(tan_u1)*RTOA;
	sigma1=atan(tan_u1/cos(alfa12/RTOA))*RTOA;
	if(cos(alfa12/RTOA)<0.0 && tan_u1>0.0)
		sigma1=sigma1+180.0;
	else if(cos(alfa12/RTOA)<0.0 && tan_u1<0.0)
		sigma1=sigma1-180.0;
	else
	{

	}
	sin_m=cos(u1/RTOA)*sin(alfa12/RTOA);
	m=asin(sin_m)*RTOA;
	k1=(sqrt(1.0+e12*(1.0-sin_m*sin_m))-1.0)/(sqrt(1.0+e12*(1.0-sin_m*sin_m))+1.0);
	a=(1.0+k1*k1/4.0)/(1-k1);
	b=k1*(1.0-3.0*k1*k1/8.0);
	sigma=RTOA*s/(rp*a);

	do
	{
		i++;
		sigma_b=sigma;
		sigmam_2=2*sigma1+sigma;
		f=2.0*cos(sigmam_2/RTOA)*cos(sigmam_2/RTOA)-1.0;
		d=b*cos(sigmam_2/RTOA)*(4.0*sin(sigma/RTOA)*sin(sigma/RTOA)-3.0)*(2.0*f-1.0)/6.0;
		Delta_sigma=RTOA*b*sin(sigma/RTOA)*(cos(sigmam_2/RTOA)+b*(f*cos(sigma/RTOA)-d)/4.0);
		sigma=RTOA*s/(rp*a)+Delta_sigma;
	}while((fabs(sigma-sigma_b)>0.17e-9)&&(i<=60));

	(*vin_b2_ptr)=atan((sin(u1/RTOA)*cos(sigma/RTOA)+cos(u1/RTOA)*sin(sigma/RTOA)*cos(alfa12/RTOA))/((1-e)*sqrt(sin_m*sin_m+pow(sin(u1/RTOA)*sin(sigma/RTOA)-cos(u1/RTOA)*cos(sigma/RTOA)*cos(alfa12/RTOA),2.0))))*RTOA;
	x1=sin(sigma/RTOA)*sin(alfa12/RTOA);
	y1=cos(u1/RTOA)*cos(sigma/RTOA)-sin(u1/RTOA)*sin(sigma/RTOA)*cos(alfa12/RTOA);
	dlamda=atan(x1/y1)*RTOA;
	if(y1<0.0 && x1>0.0) 
		dlamda=dlamda+180.0;
	else if(y1<0 && x1<0.0) 
		dlamda=dlamda-180.0;
	else
	{

	}
	c=e*cos(m/RTOA)*cos(m/RTOA)*(4.0+e*(4.0-3.0*cos(m/RTOA)*cos(m/RTOA)))/16.0;
	(*vin_l2_ptr)=vin_l1+dlamda-RTOA*(1.0-c)*e*sin_m*(sigma/RTOA+c*sin(sigma/RTOA)*(cos(sigmam_2/RTOA)+f*c*cos(sigma/RTOA)));
	if(fabs(*vin_l2_ptr)>180.0)
	{
		if((*vin_l2_ptr)>0.0)
			(*vin_l2_ptr)=(*vin_l2_ptr)-360.0;
		else
			(*vin_l2_ptr)=(*vin_l2_ptr)+360.0;
	}

}

void CFlightGlobalFun::Vincenty_Backward(IN double vin_l1,
								  IN double vin_b1,			
								  IN double vin_l2,				
								  IN double vin_b2,			
								  OUT double* S_ptr,		
								  OUT double* Alfa12_ptr)
{
	double r_d=57.29578;
	double re=6378137.0;
	double e=1.0/298.257;
	double rp=(1-e)*re;
	double e12=(re*re-rp*rp)/(rp*rp);
	double dlamda,lamda_d,sigma,Delta_sigma,u1,u2;
	double a,b,c,d,f,k1,m,x,y,v_alfa21,v_alfa12;
	double sin_sigma,cos_sigma,c_2sigma;
	double temp;

	/*	
	if(fabs(vin_b1-vin_b2)<0.000001 && fabs(vin_l1-vin_l2)<0.000001)
	{
	(*S_ptr)=0.0; (*Alfa12_ptr)=0.0; 
	return;
	}
	*/	
	u1=RTOA*atan((1-e)*tan(vin_b1/RTOA));
	u2=RTOA*atan((1-e)*tan(vin_b2/RTOA));
	dlamda=vin_l2-vin_l1;

	do 
	{
		lamda_d=dlamda;
		sin_sigma=sqrt(cos(u2/RTOA)*sin(dlamda/RTOA)*cos(u2/RTOA)*sin(dlamda/RTOA)
			+(cos(u1/RTOA)*sin(u2/RTOA)
			-sin(u1/RTOA)*cos(u2/RTOA)*cos(dlamda/RTOA))*(cos(u1/RTOA)*sin(u2/RTOA)
			-sin(u1/RTOA)*cos(u2/RTOA)*cos(dlamda/RTOA)));
		cos_sigma=sin(u1/RTOA)*sin(u2/RTOA)+cos(u1/RTOA)*cos(u2/RTOA)*cos(dlamda/RTOA);
		sigma=RTOA*atan(sin_sigma/cos_sigma);
		temp = cos(u1/RTOA)*cos(u2/RTOA)*sin(dlamda/RTOA)/sin(sigma/RTOA);
		if(temp>1.0) temp=1.0;
		if(temp<-1.0) temp=-1.0;
		m=RTOA*asin(temp);
		c=e*cos(m/RTOA)*cos(m/RTOA)*(4+e*(4-3*cos(m/RTOA)*cos(m/RTOA)))/16.0;
		c_2sigma=cos(sigma/RTOA)-2*sin(u1/RTOA)*sin(u2/RTOA)/(cos(m/RTOA)*cos(m/RTOA));
		f=2*c_2sigma*c_2sigma-1.0;
		y=(1-c)*e*sin(m/RTOA)*(sigma/r_d+c*sin(sigma/RTOA)*(c_2sigma+f*c*cos(sigma/RTOA)));
		dlamda=vin_l2-vin_l1+y*r_d;
	}while(fabs(dlamda-lamda_d)>=0.3e-11); 

	y=1-sin(m/RTOA)*sin(m/RTOA);
	k1=(sqrt(1+e12*y)-1)/(sqrt(1+e12*y)+1);
	b=k1*(1-0.375*k1*k1);
	a=(1+0.25*k1*k1)/(1-k1);
	d=b*c_2sigma*(4*sin(sigma/RTOA)*sin(sigma/RTOA)-3)*(2*f-1)/6.0;
	Delta_sigma=r_d*b*sin(sigma/RTOA)*(c_2sigma+b*(f*cos(sigma/RTOA)-d)/4.0);
	(*S_ptr)=(sigma/r_d-Delta_sigma/r_d)*rp*a;
	x=cos(u2/RTOA)*sin(dlamda/RTOA);
	y=cos(u1/RTOA)*sin(u2/RTOA)-sin(u1/RTOA)*cos(u2/RTOA)*cos(dlamda/RTOA);
	v_alfa12=RTOA*atan(x/y);
	if(y<0.0)
		v_alfa12=v_alfa12+180.0;
	else if(x<0)
		v_alfa12=v_alfa12+360.0;
	x=-cos(u1/RTOA)*sin(dlamda/RTOA);
	y=cos(u2/RTOA)*sin(u1/RTOA)-cos(u1/RTOA)*sin(u2/RTOA)*cos(dlamda/RTOA);
	v_alfa21=RTOA*atan(x/y);
	if(y<0.0)
		v_alfa21=v_alfa21+180.0;
	else if(x<0)
		v_alfa21=v_alfa21+360.0;
	(*Alfa12_ptr)=v_alfa12;
}


double CFlightGlobalFun::Nozero_FUN(double denominator)//防止分母为零
{
	double m_ftemp=0.0;

	if(fabs(denominator)<1e-10)
	{
		if (denominator>0.0)
		{
			m_ftemp=1e-10;
		}
		else if (denominator<0.0)
		{
			m_ftemp=-(1e-10);
		}
		else
		{
			m_ftemp=1e-10;
		}
	}
	else
	{
		m_ftemp=denominator;
	} 
	return m_ftemp; 
}


double CFlightGlobalFun::QATN(double y, double x)
{
	double m_fret = 0.0;

	if (fabs(x) < 1.0e-16)
	{
		m_fret = PI / 2.0 * FSign(y);
	}
	else if (x > 0.0)
	{
		m_fret = atan(y/x);
	}
	else
	{
		if (fabs(y) > 1.0e-16)
		{
			m_fret = atan(y/x) + PI * FSign(y);
		}
		else
		{
			m_fret = PI;
		}
	}

	return m_fret;

}


double CFlightGlobalFun::FSign(double fData)
{
	double m_fret = 1.0;

	if (fData < 0.0)
	{
		m_fret = -1.0;
	}
	return m_fret;
}


double CFlightGlobalFun::Adjust(double ori, double range)
{
	double m_fdata = 0.0 ;

	if (ori <= (-range))
	{
		m_fdata = ori + range * 2.0;
	}
	else if (ori > range)
	{
		m_fdata = ori - range * 2.0;
	}
	else
	{
		m_fdata = ori;
	}

	return m_fdata;
}

double CFlightGlobalFun::Range2(double dData, double dMax, double dMin)
{
	double m_fret = dData;

	if (dData > dMax)
	{
		m_fret = dMax;
	}
	else if (dData < dMin)
	{
		m_fret = dMax;
	}
	else
	{
		m_fret = dData;
	}
	
	return m_fret;
}

double CFlightGlobalFun::Range(double dData, double dMargin)
{
	double m_fret = dData;

	if (fabs(dData) >= dMargin)
	{
		m_fret = dMargin * FSign(dData);
	}

	return m_fret;
}
double CFlightGlobalFun::Norm(int vector_length, double *in_vector)
{
	double norm = 0.0;
	for (int i=0; i<vector_length; i++)
	{
		norm += in_vector[i] * in_vector[i];
	}
	norm = sqrt(norm);
	return norm;
}


double CFlightGlobalFun::CalcDist(double lon1, 
								  double lat1, 
								  double lon2, 
								  double lat2, 
								  double lon, 
								  double lat)
{
	double m_afI1[3],m_afI2[3],m_afIp[3],m_afI12[3],m_afI0[3];
	double m_f = 0.0;
	double m_fDist = 0.0;

	m_afI1[0] = cos(lat1/RTOA) * cos(lon1/RTOA);
	m_afI1[1] = cos(lat1/RTOA) * sin(lon1/RTOA);
	m_afI1[2] = sin(lat1/RTOA);

	m_afI2[0] = cos(lat2/RTOA) * cos(lon2/RTOA);
	m_afI2[1] = cos(lat2/RTOA) * sin(lon2/RTOA);
	m_afI2[2] = sin(lat2/RTOA);

	m_afI12[0] = m_afI2[1]*m_afI1[2] - m_afI2[2]*m_afI1[1];
	m_afI12[1] = m_afI2[2]*m_afI1[0] - m_afI2[0]*m_afI1[2];
	m_afI12[2] = m_afI2[0]*m_afI1[1] - m_afI2[1]*m_afI1[0];

	m_f = sqrt(m_afI12[0]*m_afI12[0] + m_afI12[1]*m_afI12[1] + m_afI12[2]*m_afI12[2]);
	if(fabs(m_f) <= 1.0e-8)
	{	
		m_f = 1.0e-8;
	}

	m_afI0[0] = m_afI12[0] / m_f;
	m_afI0[1] = m_afI12[1] / m_f;
	m_afI0[2] = m_afI12[2] / m_f;

	m_afIp[0] = cos(lat/RTOA) * cos(lon/RTOA);
	m_afIp[1] = cos(lat/RTOA) * sin(lon/RTOA);
	m_afIp[2] = sin(lat/RTOA);

	m_fDist = RE * (m_afIp[0]*m_afI0[0] + m_afIp[1]*m_afI0[1] + m_afIp[2]*m_afI0[2]);
	return(m_fDist);  
}

void CFlightGlobalFun::CrossProduct(double in_vector_A[],
									double in_vector_B[],
									double *out_vector_C)
{
	*(out_vector_C)		= in_vector_A[1] * in_vector_B[2]
						- in_vector_A[2] * in_vector_B[1];
	*(out_vector_C + 1) = in_vector_A[2] * in_vector_B[0]
						- in_vector_A[0] * in_vector_B[2];
	*(out_vector_C + 2) = in_vector_A[0] * in_vector_B[1]
						- in_vector_A[1] * in_vector_B[0];
}

double CFlightGlobalFun::DotProduct(int vector_length,
									double in_vector_A[],
									double in_vector_B[])
{
	double dot_product = 0.0;
	for (int i=0; i<vector_length; i++)
	{
		dot_product += in_vector_A[i] * in_vector_B[i];
	}
	return dot_product;
}

double CFlightGlobalFun::LAQL1(int n, 
							   double *x, 
							   double *y, 
							   double u)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double vary = 0.0;

	if(u <= x[0]  )	return(y[0]  );
	if(u >= x[n-1])	return(y[n-1]);
	for(i=0; i<n; i++)
	{
		if(fabs(u - x[i]) < 0.000001)	
			return(y[i]);
	}
	for(i=0; i<(n-1); i++)
	{
		j = i;
		if((u - x[i]) >0.0 && (u - x[i+1]) < 0.0) break;
	}
	vary = (y[j+1] - y[j]) / (x[j+1] - x[j]);
	v = vary * (u - x[j]) + y[j];
	return(v);
}


double CFlightGlobalFun::LAQL2(int n, 
							   int m, 
							   double *a, 
							   double *b, 
							   double *c, 
							   double x, 
							   double y)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	if(x <= a[0])  
	{
		v = LAQL1(m, b, c, y);
		return(v);
	}
	if(x >= a[n - 1]) 
	{
		v = LAQL1(m, b, &c[(n-1)*m], y);
		return(v);
	}
	for(i=0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001) 
		{
			v = LAQL1(m, b, &c[i*m], y);
			return(v);
		}
	}
	for(i=0; i<(n - 1); i++)
	{
		j = i;
		if((x - a[i]) > 0.0 && (x - a[i+1]) < 0.0) 
			break;
	}
	v2 = LAQL1(m, b, &c[(j + 1) * m], y);
	v1 = LAQL1(m, b, &c[j * m],       y);
	v  =(x - a[j]) * (v2 - v1) / (a[j + 1] - a[j]) + v1;
	return(v);
}

double CFlightGlobalFun::LAQL3(int n, 
							   int m, 
							   int l,
							   double *a,
							   double *b, 
							   double *c, 
							   double *d,
							   double x, 
							   double y, 
							   double z)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	if(x <= a[0])  
	{
		v = LAQL2(m, l, b, c, d, y, z);
		return(v);
	}
	if(x >= a[n - 1]) 
	{
		v = LAQL2(m, l, b, c, &d[(n - 1) * m * l], y, z);
		return(v);
	}
	for(i=0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001) 
		{
			v = LAQL2(m, l, b, c, &d[i * m * l], y, z);
			return(v);
		}
	}
	for(i=0; i<(n - 1); i++)
	{
		j = i;
		if((x - a[i]) > 0.0 && (x - a[i + 1]) < 0.0) 
			break;
	}
	v2 = LAQL2(m, l, b, c, &d[(j + 1) * m * l], y, z);
	v1 = LAQL2(m, l, b, c, &d[j * m * l],       y, z);
	v  = (x - a[j]) * (v2 - v1) / (a[j+1] - a[j]) + v1;
	return(v);
}


double CFlightGlobalFun::LAQL4(int n,
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
							   double u)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	for(i =0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001)
		{
			v  =LAQL3(m, l, k, b, c, d, &e[i * m * l * k], y, z, u);
			return(v);
		}
	}

	if(x <= a[0])
	{
		v = LAQL3(m, l, k, b, c, d, e, y, z, u);
		return(v);
	}
	else if(x >= a[n - 1])
	{
		v = LAQL3(m, l, k, b, c, d, &e[(n - 1) * m * l * k], y, z, u);
		return(v);
	}
	else
	{
		for(i = 0; i<(n - 1); i++)
		{
			j = i;
			if((x - a[i]) > 0.0 && (x - a[i + 1]) <0.0) 
				break;
		}
	}		

	v2 = LAQL3(m, l, k, b, c, d, &e[(j + 1) * m * l * k], y, z, u);
	v1 = LAQL3(m, l, k, b, c, d, &e[j * m * l * k]      , y, z, u);
	v  = (x - a[j]) * (v2 - v1) / (a[j + 1] - a[j]) + v1;
	return(v);
}

double CFlightGlobalFun::LAQL1fd(int n, 
								 double *x, 
								 double *y, 
								 double u)
{
	int i;
	double v=0;
	if(u<=x[0])
	{
		v = y[0];
		return(v);
	}
	if(u>=x[n-1]) 
	{
		v = y[n-1];
		return(v);
	}
	for(i=0;i<(n-1);i++)
	{
		if( u>x[i] && u<=x[i+1])
		{
			v=y[i];
			return(v);
		}
	}
	return(v);
}


double CFlightGlobalFun::LAQL2fd(int n, 		//温度数
								 int m, 		//时间数
								 double *a, 	//温度向量
								 double *b, 	//时间向量
								 double *c, 	//推力二维表
								 double x,	//温度
								 double y)	//时间
{
	int i;
	double v=0;//,v1,v2;

	if( x <= a[0])
	{
		v=LAQL1(m,b,&c[0*m],y);
		return(v);
	}

	if( x >= a[n-1])
	{
		v=LAQL1(m,b,&c[(n-1)*m],y);
		return(v);
	}
	
	for(i=0;i<(n-1);i++)
	{
		if( x >= a[i] && x <a[i+1])
		{
			v=LAQL1(m,b,&c[i*m],y);
			return(v);
		}
	}	
	return(v);
}


double CFlightGlobalFun::LAQL3fd(int n, 
								 int m, 
								 int l,
								 double *a,
								 double *b, 
								 double *c, 
								 double *d,
								 double x, 
								 double y, 
								 double z)
{
	int i;
	double v=0;//,v1,v2;

	//before 0点单边舵效
	if( x <= a[0]) //if(fabs(x)<=a[0]) 
	{
		v=LAQL2(m,l,b,c,d,y,z);
		return(v);
	}
	if( x >= a[n-1])//if(fabs(x)>=a[n-1])
	{
		v=LAQL2(m,l,b,c,&d[(n-1)*m*l],y,z);
		return(v);
	}
	for(i=0;i<(n-1);i++)
	{
		if(  x >= a[i] && x < a[i+1])//if( fabs(x)>=a[i] && fabs(x)<=a[i+1])
		{
			v=LAQL2(m,l,b,c,&d[i*m*l],y,z);
			return(v);
		}
	}
	return(v);
}

//反插值函数：通过输入参数反求插值轴数据
double CFlightGlobalFun::LAQL1R(int n, 
							   double *x, 
							   double *y, 
							   double u)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double vary = 0.0;

	if(u <= y[0]  )	return(x[0]  );
	if(u >= y[n-1])	return(x[n-1]);

	for(i=0; i<n; i++)
	{
		if(fabs(u - y[i]) < 0.000001)	return(x[i]);
	}
	for(i=0; i<(n - 1); i++)
	{
		j = i;
		if((u - y[i]) > 0.0 && (u - y[i+1]) < 0.0) break;
	}
	vary = (x[j+1] - x[j]) / (y[j+1] - y[j]);
	v = vary * (u - y[j]) + x[j];
	return(v);
}


double CFlightGlobalFun::LAQL2R(int n, 
							   int m, 
							   double *a, 
							   double *b, 
							   double *c, 
							   double x, 
							   double y)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	if(x <= a[0])  
	{
		v = LAQL1R(m, b, c, y);
		return(v);
	}
	if(x >= a[n - 1]) 
	{
		v = LAQL1R(m, b, &c[(n-1)*m], y);
		return(v);
	}
	for(i=0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001) 
		{
			v = LAQL1R(m, b, &c[i*m], y);
			return(v);
		}
	}
	for(i=0; i<(n - 1); i++)
	{
		j = i;
		if((x - a[i]) > 0.0 && (x - a[i+1]) < 0.0) 
			break;
	}
	v2 = LAQL1R(m, b, &c[(j + 1) * m], y);
	v1 = LAQL1R(m, b, &c[j * m],       y);
	v  =(x - a[j]) * (v2 - v1) / (a[j + 1] - a[j]) + v1;
	return(v);
}

double CFlightGlobalFun::LAQL3R(int n, 
							   int m, 
							   int l,
							   double *a,
							   double *b, 
							   double *c, 
							   double *d,
							   double x, 
							   double y, 
							   double z)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	if(x <= a[0])  
	{
		v = LAQL2R(m, l, b, c, d, y, z);
		return(v);
	}
	if(x >= a[n - 1]) 
	{
		v = LAQL2R(m, l, b, c, &d[(n - 1) * m * l], y, z);
		return(v);
	}
	for(i=0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001) 
		{
			v = LAQL2R(m, l, b, c, &d[i * m * l], y, z);
			return(v);
		}
	}
	for(i=0; i<(n - 1); i++)
	{
		j = i;
		if((x - a[i]) > 0.0 && (x - a[i + 1]) < 0.0) 
			break;
	}
	v2 = LAQL2R(m, l, b, c, &d[(j + 1) * m * l], y, z);
	v1 = LAQL2R(m, l, b, c, &d[j * m * l],       y, z);
	v  = (x - a[j]) * (v2 - v1) / (a[j+1] - a[j]) + v1;
	return(v);
}


double CFlightGlobalFun::LAQL4R(int n,
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
							   double u)
{
	int i = 0;
	int j = 0;
	double v = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;

	for(i =0; i<n; i++)
	{
		if(fabs(x - a[i]) < 0.000001)
		{
			v  =LAQL3R(m, l, k, b, c, d, &e[i * m * l * k], y, z, u);
			return(v);
		}
	}

	if(x <= a[0])
	{
		v = LAQL3R(m, l, k, b, c, d, e, y, z, u);
		return(v);
	}
	else if(x >= a[n - 1])
	{
		v = LAQL3R(m, l, k, b, c, d, &e[(n - 1) * m * l * k], y, z, u);
		return(v);
	}
	else
	{
		for(i = 0; i<(n - 1); i++)
		{
			j = i;
			if((x - a[i]) > 0.0 && (x - a[i + 1]) <0.0) 
				break;
		}
	}		

	v2 = LAQL3R(m, l, k, b, c, d, &e[(j + 1) * m * l * k], y, z, u);
	v1 = LAQL3R(m, l, k, b, c, d, &e[j * m * l * k]      , y, z, u);
	v  = (x - a[j]) * (v2 - v1) / (a[j + 1] - a[j]) + v1;
	return(v);
}