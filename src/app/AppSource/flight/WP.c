//#include <math.h>
#include "WP.h"
#include "PGuide.h"
#include "FCC_Lib.h"

//RoutePoint rp[RP_MAX_NUMBER];
int RP_NUMBER;
int RP_NUMBER_Down;
float flight_len;
DubinsStruc Point = { 0 };
double Hover_lon, Hover_lat; //盘旋点坐标
double Tangency_lon, Tangency_lat;//切点坐标
void WP_InitRoutePoint(void)
{
    RP_NUMBER     =    6;

		//写航路点
	rp[0].sn      =    0;
	rp[0].lon     =    113.602055;//     //0.000001f;     
    rp[0].lat     =    34.957315;//42.855372;//     //0.000001f;
    rp[0].h       =    5;//2000;//
    rp[0].w       =    RP_TW_0;
	rp[0].V_cmd   =    0.8 * 340;
	rp[0].t       =    0;// 实际投放点
	rp[0].if_airspeed_used   = 1;		// 实际投放点

	rp[1].sn = 1;       //40km正北爬升+平飞
	rp[1].lon = 113.602055;//113.602055;     //98.298926;//98.28706;// 0.000001f;
	rp[1].lat = 35.317044;//34.9701673;//43.416814;//43.3000339;//41.317775;//41.313311;//    //46.4620898;//40.085316;//40.069607;// 0.000001f;
	rp[1].h = 3000;//30;//
	rp[1].w = RP_TW_NORMAL;
	rp[1].V_cmd = 0.6 * 340;  //飞行时为引导点，需保持与指点飞行第一个点速度一致
	rp[1].t = 1520;//1466;//644;//641;//          //0.6ma： 1725;      //0.7ma： 1676;      //0.8ma： 1466                    //2932;
	rp[1].if_airspeed_used = 1;

	rp[2].sn = 2;       //40km正东平飞
	rp[2].lon = 114.042917;//113.5689424;//98.179938;//99.744731;//102.52478;//    //98.5496;// 0.000001f;
	rp[2].lat = 35.317044;//34.9701673;//46.695651;//46.462080;//42.497572;//42.488643;//41.313311;//43.3000339;//    //39.952772;// 0.000001f;
	rp[2].h = 3000;//30;//
	rp[2].w = RP_TW_NORMAL;
	rp[2].V_cmd = 0.6 * 340;
	rp[2].t = 3040;//2933;//1288;//1282;//          //0.6ma： 3450;      //0.7ma： 3352;      //0.8ma： 2933                            //143;  //20+123
	rp[2].if_airspeed_used = 1;

	rp[3].sn = 3;       //40km正南平飞
	rp[3].lon = 114.042917;//113.5689424;//98.179938;//99.744731;//102.52478;//    //98.821681;//0.000001f;
	rp[3].lat = 34.957312;//34.9545049;//49.974487;//49.624146;//43.677370;//43.663976;//40.137977;//    //40.219063;//0.000001f;
	rp[3].h = 3000;//30;//
	rp[3].w = RP_TW_NORMAL;
	rp[3].V_cmd = 0.6 * 340;
	rp[3].t = 4560;//4398;//1931;//1924;//          //0.6ma： 5176;      //0.7ma： 5028;      //0.8ma： 4398                  //336;                        //20+123+193
	rp[3].if_airspeed_used = 1;

	rp[4].sn = 4;       //30km正西平飞
	rp[4].lon = 113.713728;//113.601055;//100.460265;//99.428072;//98.207332;//98.153631;//    //98.442194;//98.52750;//99.069332;// 0.000001f;
	rp[4].lat = 34.957315;//34.9545049;//52.786202;//44.857167;//    //40.302051;//40.23755;//40.460777;// 0.000001f;
	rp[4].h = 3000;//30;//
	rp[4].w = RP_TW_NORMAL;//RP_TW_NORMAL;
	rp[4].V_cmd = 0.6 * 340;
	rp[4].t = 6080;//5948;//2575;//2565;//          //0.6ma： 6901;      //0.7ma： 6704;      //0.8ma： 5948                            //457;//20+123+193+121
	rp[4].if_airspeed_used = 1;


	rp[5].sn = 5; //末制导点
	rp[5].lon = 113.658925;//100.460265;//99.428072;//98.207332;//98.153631;//    //98.442194;//98.52750;//99.069332;// 0.000001f;
	rp[5].lat = 34.957315;//5km:34.655455;//10km：34.610540;//52.786202;//44.857167;//    //40.302051;//40.23755;//40.460777;// 0.000001f;
	rp[5].h = 10;//30;//
	rp[5].w = RP_TW_GUIDANCE;//RP_TW_NORMAL;
	rp[5].V_cmd = 0.6 * 340;
	rp[5].t = 6080;//5948;//2575;//2565;//          //0.6ma： 6901;      //0.7ma： 6704;      //0.8ma： 5948                            //457;//20+123+193+121
	rp[5].if_airspeed_used = 1;

	memset(&flight_len, 0, sizeof(flight_len));
}


/*(-180~180)轉換為(0~360)*/
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_Psi2Psi360,".sdram_code");
#endif
double WP_Psi2Psi360(double angle)
{
	double ret=angle;

	if((angle>=-180) && (angle<0)){
		ret=angle+360;
	}
	if (angle >= 360)
		ret = angle - 360;
	return ret;
}

/*
 輸入值：angle
 輸齣值：角度轉換為-180~180
 */
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_Psi2Psi,".sdram_code");
#endif
double  WP_Psi2Psi(double angle)
{
    double  tmp_angle;

    if      (angle > 180.0f)  tmp_angle = angle - 360.0f;
    else if (angle <-180.0f)  tmp_angle = angle + 360.0f;
    else                      tmp_angle = angle;
    return (tmp_angle);
}

/*
 輸入值：
 輸齣值：
 */
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_Psi,".sdram_code");
#endif
double  WP_Psi(double x, double y)
{
    double  tmp_psi;

    tmp_psi = atan2(y,x)*RADIAN_2_DEGREE;
    if ((tmp_psi<=180.0f) && (tmp_psi>90.0f))
        tmp_psi = 450.0f - tmp_psi;
    else
        tmp_psi = 90.0f - tmp_psi;

    return (tmp_psi);
}

#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_Pos2XY,".sdram_code");
#endif
void  WP_Pos2XY(double lon_ref,double lat_ref, double lon,double lat, double *x,double *y)
{
    double   iPsi, Clat;
    double  dLon, dLat;

    dLon=lon-lon_ref;
    dLat=lat-lat_ref;

    iPsi = (lat_ref+lat)/RADIAN_2_DEGREE*0.5f;
    Clat = cos(iPsi);
    *x = (6383487.606f*Clat - 5357.31f*cos(3*iPsi))* dLon/ RADIAN_2_DEGREE;
    *y = (6367449.134f - 32077.0f*cos(2*iPsi)) * dLat / RADIAN_2_DEGREE; 
}

#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_XY2Pos,".sdram_code");
#endif
//-------lon_ref,lat_ref:参考点经纬度; x:东向距离,y:北向距离; lon,lat：所求点经纬度----//
void  WP_XY2Pos(double lon_ref, double lat_ref, double x, double y,  double *lon, double *lat)
{
    double  R=6378165.0f;
    double  Clat;

    Clat = cos(lat_ref/RADIAN_2_DEGREE);
    *lon = lon_ref + x/Clat*RADIAN_2_DEGREE/R;
    *lat = lat_ref + y*RADIAN_2_DEGREE/R;
}

#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_RotateXY,".sdram_code");
#endif
void  WP_RotateXY(double Xen, double Yen, double Psi, double *Xr, double *Yr)
{ 
    double  SPsi;
    double  CPsi;

    CPsi=cos(Psi/RADIAN_2_DEGREE); 
    SPsi=sin(Psi/RADIAN_2_DEGREE); 

    *Xr = (SPsi*Xen+CPsi*Yen);
    *Yr = (SPsi*Yen-CPsi*Xen);
}

/*
 导弹尚未到达rp[dot]
 构建rp[dot]和rp[dot+1]航线
 航线高度=rp[dot]的高度
 航线特征=rp[dot]的特征
 若rp[dot]或rp[dot+1]不存在,航线构建失败
 输出:航线
*/
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_GetLine,".sdram_code");
#endif
int  WP_GetLine(int dot, LineStruc *AB)
{
    RoutePoint  A, B;                                                     
    int         a, b, flag=0;
  	float       VA;  //音速
	float  psi_tmp = 0.0f;
	double  x = 0.0f, y = 0.0f;

    a = WP_GetPoint(dot,   &A);
    b = WP_GetPoint(dot+1, &B);
	
    if (a && b) {
		//---判断是否为盘旋点,如果是，则计算盘旋圆切点坐标及航向，并将盘旋点坐标改为切点坐标进行指点飞行---//
		if (B.w == RP_TW_HOVER) {
			if (WP_MakeDR(A.lon, A.lat, B.lon, B.lat, B.hover_radis, -1, &Tangency_lon, &Tangency_lat, &psi_tmp))
			{
				Hover_lon = B.lon;
				Hover_lat = B.lat;  //盘旋圆心点赋值

				AB->Hover_Round = B.outTrack;//盘旋圈数赋值

				B.lon = Tangency_lon;
				B.lat = Tangency_lat;
				B.outTrack = psi_tmp;
				B.w = RP_TW_NORMAL;
				B.if_GuideFlight = RP_TW_DUBINS;//类型更改为指点，待指点飞行完成后再进入盘旋

				AB->vxd_PreHover = 1; //进入预盘旋状态
			}
		}
		//----若完成盘旋,根据切点与下一航点计算切入航向并赋值---//
		if (A.w == RP_TW_HOVER) {
			WP_Pos2XY(Tangency_lon, Tangency_lat, B.lon, B.lat, &x, &y);
			B.outTrack = WP_Psi(x, y);
			B.if_GuideFlight = RP_TW_DUBINS;//类型更改为指点
		}

        WP_MakeLine(A.lon,A.lat, B.lon,B.lat, AB);

        AB->alt = B.h; //取航段高度为终点高度[可更改]
		AB->delt_alt = B.h - A.h; //高度指令差值
        AB->vxd1 = A.w; //获取A点任务特征
		AB->vxd2 = B.w; //获取B点任务特征
		AB->outTrack = B.outTrack;
		AB->hover_radis = B.hover_radis;//获取B点盘旋半径
		AB->AttackAngle = B.AttackAngle;
		AB->FlightTime = B.t - A.t;
		AB->ac_FlightTime = 0;

		AB->vxd_guide = B.if_GuideFlight; //指点特征

		VA=20.047*sqrt(288.15-0.0063*(AB->alt));  //0803
		AB->Velocity_cmd  = B.V_cmd;//取马赫指令为B点指令[可更改]

		AB->if_airspeed_used = B.if_airspeed_used;//根据B点判断是否启用空速控制

        flag = 1;
    }

    return (flag);
}
void WP_GetLine2Point(RoutePoint A, RoutePoint B, LineStruc* AB)
{
	float  VA;  //音速

	WP_MakeLine(A.lon, A.lat, B.lon, B.lat, AB);
	AB->alt = B.h; //取航段高度为终点高度[可更改]
	AB->vxd1 = A.w; //获取A点任务特征
	AB->vxd2 = B.w; //获取B点任务特征
	AB->outTrack = B.outTrack;
	AB->FlightTime = B.t - A.t;
	AB->ac_FlightTime = 0;

	VA = 20.047 * sqrt(288.15 - 0.0063 * (AB->alt));  //0803
	AB->Velocity_cmd = B.V_cmd;//取马赫指令为B点指令[可更改]

	AB->if_airspeed_used = B.if_airspeed_used;//根据B点判断是否启用空速控制

}
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_MakeLine,".sdram_code");
#endif
void  WP_MakeLine(double lon_a,double lat_a, double lon_b,double lat_b, LineStruc *AB)
{
    double  x=0.0f,y=0.0f;
    double  xw=0.0f,yw=0.0f;

    WP_Pos2XY(lon_a,lat_a, lon_b,lat_b, &x,&y);
    AB->psi = WP_Psi(x,y);
    AB->lon = lon_b;    
    AB->lat = lat_b;
    WP_RotateXY(x,y, AB->psi, &xw,&yw);    AB->len=xw;
}

/*
 根据航线和导弹当前经纬、航向计算侧偏距、待飞距和航向偏角
*/
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_LateWay,".sdram_code");
#endif
void  WP_LateWay(LineStruc *AB, double lon,double lat,double psi, double *dZ,double *dL,double *dPsi)
{
	
	
    double  AxEN,AyEN;                   /*[(lon,lat)在东北系的坐标]*/
    double  AxWP,AyWP;                   /*[(lon,lat)在航段AB上的坐标]*/

    WP_Pos2XY(AB->lon,AB->lat, lon,lat, &AxEN,&AyEN);
    WP_RotateXY(AxEN,AyEN, AB->psi, &AxWP,&AyWP);

    *dPsi = WP_Psi2Psi(psi - AB->psi);  /*[相对航线偏航角][左偏为-,右偏为+]*/
    *dZ   = -AyWP;                      /*[侧偏距][左偏为-,右偏为+]*/
    *dL   = -AxWP;

	
}

/*
 输入参数: dot 基于0
 导弹尚未到达rp[dot]
 计算导弹当前位置到着陆点距离
*/
/*
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_GetR2g,".sdram_code");
#endif
double  WP_GetR2g(int dot)
{
   int  idx=0,result=0,EndDot=0; 
   double  x, y, xw, yw, psi, Sum=0.0f;
   RoutePoint  P,A,B;
   
   for (idx=dot+1; idx<RP_NUMBER; idx++) {
        result = WP_GetPoint(idx, &P);
        if (result) {
			if((P.w & 0x00FF)==0x80){ //[找到着陆特征点后跳出循环]
				EndDot=idx;
				break;
			}
        }
    }

    for (idx=dot+1; idx<EndDot; idx++) {
      WP_GetPoint(idx, &A); 
      WP_GetPoint(idx+1, &B); 
      WP_Pos2XY(A.lon,A.lat,B.lon,B.lat, &x,&y);
      psi=WP_Psi(x,y);      
      WP_RotateXY(x,y, psi, &xw,&yw);     
      Sum+=xw; 
    }
  
      WP_GetPoint(dot+1, &P);    
      WP_Pos2XY(pGuide_para.cur_lon,pGuide_para.cur_lat,P.lon,P.lat, &x,&y);
      psi=WP_Psi(x,y);      
      WP_RotateXY(x,y, psi, &xw,&yw);     
      Sum+=xw;
      
      return (Sum);    
}*/
/*-----------------------------------WayPoint-----------------------------------*/
/*
 输入参数: dot基于0
*/
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_GetPoint,".sdram_code");
#endif
int  WP_GetPoint(int dot, RoutePoint *A)
{	
	if(dot<0 || dot>= RP_NUMBER){  //20250819 RP_MAX_NUMBER->RP_NUMBER
		return 0;
	}

    A->sn            = rp[dot].sn;
    A->lon           = rp[dot].lon;// 0.000001f;
    A->lat           = rp[dot].lat;// 0.000001f;
    A->h             = rp[dot].h;
    A->w             = rp[dot].w;
	A->t             = rp[dot].t;
	A->V_cmd         = rp[dot].V_cmd;
	A->outTrack      = rp[dot].outTrack;
	A->hover_radis   = rp[dot].hover_radis;
	A->AttackAngle =   rp[dot].AttackAngle;
	A->if_airspeed_used = rp[dot].if_airspeed_used;

	//----对航点进行指点/非指点判断---//
	if (A->outTrack >= 0.0)
		A->if_GuideFlight = RP_TW_DUBINS;
	else
		A->if_GuideFlight = 0;
	return 1;
}

/*
 根据a点生成圆轨迹圆心点坐标
 */
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_MakeCirc,".sdram_code");
#endif
int  WP_MakeCirc(double lon_a,double lat_a, double dpsi,double psi,double len2turn,double *lon_b,double *lat_b,double *circ_psi)
{
    double  x=0.0f,y=0.0f;
	double  Xr=0.0f,Yr=0.0f;
    double lon=0.0f,lat=0.0f;

	if (fabs(len2turn)<1.0f)
		return 0;

	x=-len2turn;
	y=x/tan(dpsi/RADIAN_2_DEGREE);
	
	Xr=x*sin(psi/RADIAN_2_DEGREE)-y*cos(psi/RADIAN_2_DEGREE);
	Yr=x*cos(psi/RADIAN_2_DEGREE)+y*sin(psi/RADIAN_2_DEGREE);
	
    WP_XY2Pos(lon_a,lat_a,Xr,Yr, &lon,&lat);
	*lon_b=lon;
	*lat_b=lat;

	WP_Pos2XY(*lon_b,*lat_b, lon_a,lat_a, &x,&y);
    *circ_psi = WP_Psi(x,y);

	return 1;
}
/*
 输入参数：1.根据切入点和已知圆心点及盘旋半径生成切点坐标 2.顺逆时针
 输出参数：盘旋圆的切点坐标及切出航向
 */
int WP_MakeDR(double lon_a, double lat_a, double lon_circ, double lat_circ, float radis, char dir, double* lon_b, double* lat_b, float* psi)
{
	double  x = 0.0f, y = 0.0f;
	float  psi_a_circ = 0;//切入点和圆心点连线角度
	float  len = 0;
	float  delt_angle = 0;//切入点和圆心联系角度 - 切入点与切点连线角度
	float  DR_angle = 0; //切点与水平线夹角
	float  psi_circ_DR = 0;//圆心与切点连线角度
	double lon = 0.0f, lat = 0.0f;
	double Xe = 0.0f, Yn = 0.0f;//
	WP_Pos2XY(lon_a, lat_a, lon_circ, lat_circ, &x, &y);

	psi_a_circ = WP_Psi(x, y);//切入点和圆心点连线角度 北偏东
	len = sqrt(x * x + y * y);

	delt_angle = asind(radis / len);//切入点-圆心连线与切入点-切点连线夹角

	//---求解圆心与切点连线航向----//
	if (dir == -1) //逆时针
	{
		if (psi_a_circ >= 0 && psi_a_circ <= 90) //圆心点在第一象限
		{
			DR_angle = 90 - (psi_a_circ + delt_angle); //切点与水平线夹角 
			psi_circ_DR = 90 + (90 - DR_angle);//圆心与切点连线航向

			*psi = 90 - DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 90 && psi_a_circ <= 180)////圆心点在第二象限
		{
			DR_angle = (psi_a_circ + delt_angle) - 90; //切点与水平线夹角
			psi_circ_DR = 270 - (90 - DR_angle);

			*psi = 90 + DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 180 && psi_a_circ <= 270)////圆心点在第三象限
		{
			DR_angle = 270 - (psi_a_circ + delt_angle); //切点与水平线夹角
			psi_circ_DR = 270 + (90 - DR_angle);

			*psi = 270 - DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 270 && psi_a_circ < 360)////圆心点在第四象限
		{
			DR_angle = (psi_a_circ + delt_angle) - 270; //切点与水平线夹角
			psi_circ_DR = 90 + (90 - DR_angle);

			*psi = 270 + DR_angle;//切入点与切点连线角度 北偏东
		}
	}
	else //顺时针
	{
		if (psi_a_circ >= 0 && psi_a_circ <= 90) //圆心点在第一象限
		{
			DR_angle = 90 - (psi_a_circ - delt_angle); //切入点与切点连线角度
			psi_circ_DR = 270 + (90 - DR_angle);

			*psi = 90 - DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 90 && psi_a_circ <= 180)////圆心点在第二象限
		{
			DR_angle = (psi_a_circ - delt_angle) - 90; //切入点与切点连线角度
			psi_circ_DR = 90 - (90 - DR_angle);

			*psi = 90 + DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 180 && psi_a_circ <= 270)////圆心点在第三象限
		{
			DR_angle = 270 - (psi_a_circ - delt_angle); //切入点与切点连线角度
			psi_circ_DR = 90 + (90 - DR_angle);

			*psi = 270 - DR_angle;//切入点与切点连线角度 北偏东
		}
		else if (psi_a_circ > 270 && psi_a_circ < 360)////圆心点在第四象限
		{
			DR_angle = (psi_a_circ - delt_angle) - 270; //切入点与切点连线角度
			psi_circ_DR = 90 - (90 - DR_angle);

			*psi = 270 + DR_angle;//切入点与切点连线角度 北偏东
		}
	}
	Xe = radis * sin(psi_circ_DR / RADIAN_2_DEGREE); //psi_circ_DR为北偏西
	Yn = radis * cos(psi_circ_DR / RADIAN_2_DEGREE);

	WP_XY2Pos(lon_circ, lat_circ, Xe, Yn, &lon, &lat);
	*lon_b = lon;
	*lat_b = lat;

	return 1;
}
/*
 计算导弹实时位置与圆轨迹的侧偏距，到切出点的直线距离，角度
 */
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_LateCirc,".sdram_code");
#endif
void  WP_LateCirc(double circ_lon,double circ_lat,double radius,double circ_psi,double psi_tmp,double ac_lon,double ac_lat,double ac_psi,double *dZ,double *L1,double *dPsi)
{
     double  AxEN,AyEN;                   /*[(lon,lat)在东北系的坐标]*/
     double  AxWP,AyWP;                   /*[(lon,lat)在圆心与导弹连线上的坐标]*/
	 double  Apsi;                        /*[(lon,lat)与圆心连线的航向角]*/  
	 double  Dpsi;                        /*[(lon,lat)-圆心连线与切出点-圆心连线的航向夹角*/   

     WP_Pos2XY(circ_lon,circ_lat,ac_lon,ac_lat,&AxEN,&AyEN);
	 Apsi   =  WP_Psi(AxEN,AyEN);
     WP_RotateXY(AxEN,AyEN,Apsi, &AxWP,&AyWP);
//	 AxWP=sqrt(AxEN*AxEN+AyEN*AyEN);
//	 printf("len==%f\n",AxWP);

	 if(psi_tmp>0)   /*[航路右转]*/
	 {
		 Dpsi   =  circ_psi - Apsi + psi_tmp;
		 *dZ    =  radius - AxWP;               /*[侧偏距][左偏为-,右偏为+]*/ 
		 pGuide_para.mode_line = PW_LineRight;


		 if((ac_psi-Apsi)>=0){
			*dPsi=ac_psi-Apsi-90;
		 }
		 else{
			*dPsi=ac_psi-Apsi+270;
		 }
	 }

	 else            /*[航路左转]*/ 
	 {
		 Dpsi   =  Apsi - circ_psi - psi_tmp;
	     *dZ    =  AxWP - radius ;               /*[侧偏距][左偏为-,右偏为+]*/ 
		 pGuide_para.mode_line = PW_LineLeft;

		 if((ac_psi-Apsi)<=0){
			*dPsi=ac_psi-Apsi+90;
		 }
		 else{
			*dPsi=ac_psi-Apsi-270;
		 }
	 }
	 *L1    =  fabs(2*radius*sin(0.5*Dpsi/RADIAN_2_DEGREE));

}
/*
 计算导弹到下一航点的待飞时间
 */
#ifdef DSP_6713_SDRAM
#pragma CODE_SECTION(WP_LateTgoCal,".sdram_code");
#endif
int WP_LateTgoCal(double len,double VHorizontal,double dpsi)
{
	int tgo      = 0;
	double V_len = 0;

	V_len = VHorizontal*cos(dpsi/RADIAN_2_DEGREE);

	tgo   = (int)(len/V_len);

	return tgo;
}
/*
 计算Dubins飞行时的待飞距离
 */
float WP_GetDubinsR2g(DubinsStruc dubins, double horizVm)
{
	float R2g;

	flight_len += horizVm * wp_fTimerStep;

	R2g = dubins.dubins_len - flight_len;

	return R2g;
}
/*
 计算Dubins飞行时的侧偏距，到切出点的直线距离，角度
 */
void WP_LateDubins(LineStruc* AB, LineStruc* AB_preset, double cur_lon, double cur_lat, double cur_psi, double radius, DubinsStruc dubins, double horizVm, double* dZ, double* dL, double* dPsi, unsigned char* st)
{
	double ac_dZ = 0., ac_L1 = 0., ac_dL = 0., ac_dL_dubins = 0., ac_dPsi = 0.;
	const float len_Threshold = 30;
	unsigned char stage_st = 0;
	/*----计算待飞距----*/
	ac_dL_dubins = WP_GetDubinsR2g(dubins, horizVm);
	if (ac_dL_dubins > (dubins.dubins_len2 + dubins.dubins_len3 + len_Threshold)) {/*----杜宾斯第一段----*/
		stage_st = 1;
		WP_LateCirc(dubins.lon_C1, dubins.lat_C1, radius, dubins.Circ1_psi, dubins.psi_C1MD, cur_lon, cur_lat, cur_psi, &ac_dZ, &ac_L1, &ac_dPsi);
	}
	else if (ac_dL_dubins > (dubins.dubins_len3 + len_Threshold)) {/*----杜宾斯第二段----*/
		stage_st = 2;
		if (strcmp(dubins.dubins_type, "RLR") == 0 || strcmp(dubins.dubins_type, "LRL") == 0) //CCC
		{
			WP_LateCirc(dubins.lon_C3, dubins.lat_C3, radius, dubins.Circ3_psi, dubins.psi_C3MD, cur_lon, cur_lat, cur_psi, &ac_dZ, &ac_L1, &ac_dPsi);
		}
		else //CSC
		{
			WP_LateWay(AB, cur_lon, cur_lat, cur_psi, &ac_dZ, &ac_dL, &ac_dPsi); //解算当前侧边距和航向角差
		}
	}
	else if (ac_dL_dubins > len_Threshold) { /*----杜宾斯第三段----*/
		stage_st = 3;
		WP_LateCirc(dubins.lon_C2, dubins.lat_C2, radius, dubins.Circ2_psi, dubins.psi_C2MD, cur_lon, cur_lat, cur_psi, &ac_dZ, &ac_L1, &ac_dPsi);
	}
	else {/*----杜宾斯结束，沿虚拟点航线飞行----*/
		stage_st = 4;
		WP_LateWay(AB_preset, cur_lon, cur_lat, cur_psi, &ac_dZ, &ac_dL, &ac_dPsi); //解算当前侧边距和航向角差
	}
	*dZ = ac_dZ;
	*dL = ac_dL_dubins;
	*dPsi = ac_dPsi;
	*st = stage_st;
}
/*
 根据Dubins切出点生成切出后的直线航段
 */
void WP_DubinsMakeCD(double lon_a, double lat_a, double psi, double length, double* lon_b, double* lat_b)
{
	double  Xr = 0.0f, Yr = 0.0f;
	double lon = 0.0f, lat = 0.0f;

	Xr = length * sin(-psi / RADIAN_2_DEGREE); //psi为北偏西
	Yr = length * cos(-psi / RADIAN_2_DEGREE);

	WP_XY2Pos(lon_a, lat_a, Xr, Yr, &lon, &lat);
	*lon_b = lon;
	*lat_b = lat;
}
/*根据当前位姿与目标位姿计算杜宾斯圆的两个圆心及切点*/
DubinsStruc WP_DubinsCal(double lonM, double latM, float psiM, double lonT, double latT, float psiT, float radius)
{
	DubinsStruc Temp = { 0 };
	double   x = 0.0f, y = 0.0f;
	double  xw = 0.0f, yw = 0.0f;
	float xen = 0.0f, yen = 0.0f;
	char dubins_type[] = "RSR";
	float  CC_DR_angle;
	float  C1_theta, C2_theta, C3_theta; //圆弧角度差
	float  angle_set = 0;
	float  deltAngle = 0; 


	WP_Pos2XY(lonM, latM, lonT, latT, &x, &y);
	Point.psi_MT = WP_Psi(x, y); //弹目连线北偏东

	//-----分别计算RSR、RSL、LSR、LSL四种情况，根据路径最短原则确定dubins曲线----//

	if (strcmp(dubins_type, "RSR") == 0)	//RSR杜宾斯
	{
		//----圆心坐标----//    
		xen = radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = -radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = radius * cos(-psiT / RADIAN_2_DEGREE);
		yen = -radius * sin(-psiT / RADIAN_2_DEGREE);//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//---圆心与切点连线角度---//
		Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC - 90);  //北偏东为正
		Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC - 90);  //北偏东为正

		//----切点坐标----//	
		xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
		yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
		WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

		xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
		yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
		WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标

		//----切出点航向与圆心连线（切线）航向比较:若切出点航向较小，则为优弧，为避免优弧转弯，需将切出点航向补偿至超过圆心连线（切线）航向值---PB20251121--//
		//----基准北偏东----//
		if ((Point.psi_CC > (360 - psiT))&& (Point.psi_CC < (360 - psiT+ PW_Dubins_deltAngleThread))) {
			angle_set = Point.psi_DR2 - Point.psi_CT + PW_Dubins_AngleSet;//补偿角度：使得此时切出点角度为目标航向角+PW_Dubins_AngleSet
		}
		//----RSR轨迹长度---//
		C1_theta = Point.psi_DR1 - Point.psi_CM;
		if (C1_theta < 0) {
			C1_theta = C1_theta + 360;
		}
		Point.psi_C1MD = C1_theta * 0.5;
		Point.Circ1_psi = Point.psi_CM + C1_theta * 0.5;

		C2_theta = Point.psi_CT - Point.psi_DR2 + angle_set; //叠加补偿角度保证劣弧航行
		if (C2_theta < 0) {
			C2_theta = C2_theta + 360;
		}
		Point.psi_C2MD = C2_theta * 0.5;
		Point.Circ2_psi = Point.psi_DR2 + C2_theta * 0.5;

		Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
		Point.dubins_len2 = Point.len;
		Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
		Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

		//---更新type---//
		strcpy(Point.dubins_type, dubins_type);
		//----参量保存用于最短路径比较---//
		WP_DubinsUpdate(&Temp, &Point);
		strcpy(dubins_type, "RSL");
	}

	if (strcmp(dubins_type, "RSL") == 0)	//RSL杜宾斯
	{
		//----圆心坐标----//    
		xen = radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = -radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = -(radius * cos(-psiT / RADIAN_2_DEGREE));
		yen = -(-radius * sin(-psiT / RADIAN_2_DEGREE));//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//----圆心连线长度>2R时进入RSL----//
		if (Point.len > 2 * radius)
		{
			//---圆心与切点连线角度---//
			Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC - acos(radius / (Point.len * 0.5)) * RADIAN_2_DEGREE);        //北偏东为正
			Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC - acos(radius / (Point.len * 0.5)) * RADIAN_2_DEGREE + 180);  //北偏东为正
			CC_DR_angle = acos(radius / (Point.len * 0.5));

			//----切点坐标----//	
			xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

			xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
			WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标

			//----RSL轨迹长度---//
			C1_theta = Point.psi_DR1 - Point.psi_CM;
			if (C1_theta < 0) {
				C1_theta = C1_theta + 360;
			}
			Point.psi_C1MD = C1_theta * 0.5;
			Point.Circ1_psi = Point.psi_CM + C1_theta * 0.5;

			C2_theta = Point.psi_DR2 - Point.psi_CT;
			if (C2_theta < 0) {
				C2_theta = C2_theta + 360;
			}
			Point.psi_C2MD = -C2_theta * 0.5;
			Point.Circ2_psi = Point.psi_DR2 - C2_theta * 0.5;

			Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
			Point.dubins_len2 = Point.len * sin(CC_DR_angle);
			Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
			Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

			if (Temp.dubins_len < Point.dubins_len)//取路径最短者
			{
				WP_DubinsUpdate(&Point, &Temp);
			}
			else
			{
				strcpy(Point.dubins_type, dubins_type);
				//----参量保存用于最短路径比较---//
				WP_DubinsUpdate(&Temp, &Point);
			}
		}
		else {
			WP_DubinsUpdate(&Point, &Temp);
		}
		//---更新type---//	
		strcpy(dubins_type, "LSR");
	}

	if (strcmp(dubins_type, "LSR") == 0)	//RSL杜宾斯
	{
		//----圆心坐标----//    
		xen = -radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = -(-radius * cos(-psiT / RADIAN_2_DEGREE));
		yen = -(radius * sin(-psiT / RADIAN_2_DEGREE));//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//----圆心连线长度>2R时进入RSL----//
		if (Point.len > 2 * radius)
		{
			//---圆心与切点连线角度---//
			Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC + acos(radius / (Point.len * 0.5)) * RADIAN_2_DEGREE);        //北偏东为正
			Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC + acos(radius / (Point.len * 0.5)) * RADIAN_2_DEGREE - 180);  //北偏东为正
			CC_DR_angle = acos(radius / (Point.len * 0.5));

			//----切点坐标----//	
			xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

			xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
			WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标

			//----LSR轨迹长度---//
			C1_theta = Point.psi_CM - Point.psi_DR1;
			if (C1_theta < 0) {
				C1_theta = C1_theta + 360;
			}
			Point.psi_C1MD = -C1_theta * 0.5;
			Point.Circ1_psi = Point.psi_CM - C1_theta * 0.5;

			C2_theta = Point.psi_CT - Point.psi_DR2;
			if (C2_theta < 0) {
				C2_theta = C2_theta + 360;
			}
			Point.psi_C2MD = C2_theta * 0.5;
			Point.Circ2_psi = Point.psi_DR2 + C2_theta * 0.5;

			Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
			Point.dubins_len2 = Point.len * sin(CC_DR_angle);
			Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
			Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

			if (Temp.dubins_len < Point.dubins_len)//取路径最短者
			{
				WP_DubinsUpdate(&Point, &Temp);
			}
			else
			{
				strcpy(Point.dubins_type, dubins_type);
				//----参量保存用于最短路径比较---//
				WP_DubinsUpdate(&Temp, &Point);
			}
		}
		else {
			WP_DubinsUpdate(&Point, &Temp);
		}
		//---更新type---//	
		strcpy(dubins_type, "LSL");
	}

	if (strcmp(dubins_type, "LSL") == 0)	//LSL杜宾斯
	{
		//----圆心坐标----//    
		xen = -radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = -radius * cos(-psiT / RADIAN_2_DEGREE);
		yen = radius * sin(-psiT / RADIAN_2_DEGREE);//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//---圆心与切点连线角度---//
		Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC + 90);  //北偏东为正
		Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC + 90);  //北偏东为正

		//----切点坐标----//	
		xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
		yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
		WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

		xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
		yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
		WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标
		
		//----切出点航向与圆心连线（切线）航向比较:若切出点航向较小，则为优弧，为避免优弧转弯，需将切出点航向补偿至超过圆心连线（切线）航向值---PB20251121--//
		//----基准北偏东----//
		if (fabs(360 - psiT) < 1e-5) //将psiT北偏西360转为0
			psiT = 360 - psiT;
		if ((Point.psi_CC < (360 - psiT)) && (Point.psi_CC > (360 - psiT - PW_Dubins_deltAngleThread))) {
			angle_set = -Point.psi_DR2 + Point.psi_CT + PW_Dubins_AngleSet;//补偿角度：使得此时切出点角度为目标航向角+PW_Dubins_AngleSet
		}
		//----LSL轨迹长度---//
		C1_theta = Point.psi_CM - Point.psi_DR1;
		if (C1_theta < 0) {
			C1_theta = C1_theta + 360;
		}
		Point.psi_C1MD = -C1_theta * 0.5;
		Point.Circ1_psi = Point.psi_CM - C1_theta * 0.5;

		C2_theta = Point.psi_DR2 - Point.psi_CT + angle_set; //叠加补偿角度保证劣弧航行;
		if (C2_theta < 0) {
			C2_theta = C2_theta + 360;
		}
		Point.psi_C2MD = -C2_theta * 0.5;
		Point.Circ2_psi = Point.psi_DR2 - C2_theta * 0.5;

		Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
		Point.dubins_len2 = Point.len;
		Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
		Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

		if (Temp.dubins_len < Point.dubins_len)//取路径最短者
		{
			WP_DubinsUpdate(&Point, &Temp);
		}
		else
		{
			strcpy(Point.dubins_type, dubins_type);
			//----参量保存用于最短路径比较---//
			WP_DubinsUpdate(&Temp, &Point);
		}
		//---更新type---//	
		strcpy(dubins_type, "RLR");
	}
	if (strcmp(dubins_type, "RLR") == 0)	//CCC-RLR杜宾斯
	{
		//----圆心坐标----//    
		xen = radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = -radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = radius * cos(-psiT / RADIAN_2_DEGREE);
		yen = -radius * sin(-psiT / RADIAN_2_DEGREE);//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//----圆心连线长度<4R时进入CCC----//
		if (Point.len < 4 * radius)
		{
			//---余弦定理计算外切圆等腰三角形底边夹角---//
			float psi_C3C1C2 = acos(Point.len / (2 * 2 * radius)) * RADIAN_2_DEGREE;

			//---圆心与切点连线角度---//
			Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC + psi_C3C1C2);  //北偏东为正
			Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC - psi_C3C1C2 + 180);  //北偏东为正	

			//----切点坐标----//	
			xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

			xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
			WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标

			//----圆心3坐标----//
			xen = 2 * radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = 2 * radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//圆心3距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_C3, &Point.lat_C3);

			//----RLR轨迹长度---//
			C1_theta = Point.psi_DR1 - Point.psi_CM;
			if (C1_theta < 0) {
				C1_theta = C1_theta + 360;
			}
			Point.psi_C1MD = C1_theta * 0.5;
			Point.Circ1_psi = Point.psi_CM + C1_theta * 0.5;

			C2_theta = Point.psi_CT - Point.psi_DR2;
			if (C2_theta < 0) {
				C2_theta = C2_theta + 360;
			}
			Point.psi_C2MD = C2_theta * 0.5;
			Point.Circ2_psi = Point.psi_DR2 + C2_theta * 0.5;

			//---圆3为优弧---//
			C3_theta = 180 + 2 * psi_C3C1C2;
			Point.psi_C3MD = -C3_theta * 0.5;
			Point.Circ3_psi = Point.psi_DR1 + (90 - psi_C3C1C2);

			Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
			Point.dubins_len2 = radius * C3_theta / RADIAN_2_DEGREE;
			Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
			Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

			if (Temp.dubins_len < Point.dubins_len)//取路径最短者
			{
				WP_DubinsUpdate(&Point, &Temp);
			}
			else
			{
				strcpy(Point.dubins_type, dubins_type);
				//----参量保存用于最短路径比较---//
				WP_DubinsUpdate(&Temp, &Point);
			}
		}
		else {
			WP_DubinsUpdate(&Point, &Temp); //直接赋值
		}
		//---更新type---//	
		strcpy(dubins_type, "LRL");
	}
	if (strcmp(dubins_type, "LRL") == 0)	//CCC-LRL杜宾斯
	{
		//----圆心坐标----//    
		xen = -radius * cos(-psiM / RADIAN_2_DEGREE);
		yen = radius * sin(-psiM / RADIAN_2_DEGREE);//圆心1位置转到北天东系	   
		WP_XY2Pos(lonM, latM, xen, yen, &Point.lon_C1, &Point.lat_C1); //圆心1坐标

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, lonM, latM, &x, &y);
		Point.psi_CM = WP_Psi(x, y);//圆心1与进入点连线北偏东

		xen = -radius * cos(-psiT / RADIAN_2_DEGREE);
		yen = radius * sin(-psiT / RADIAN_2_DEGREE);//圆心2位置转到北天东系		   
		WP_XY2Pos(lonT, latT, xen, yen, &Point.lon_C2, &Point.lat_C2); //圆心2坐标

		WP_Pos2XY(Point.lon_C2, Point.lat_C2, lonT, latT, &x, &y);
		Point.psi_CT = WP_Psi(x, y);//圆心2与切出点连线北偏东

		WP_Pos2XY(Point.lon_C1, Point.lat_C1, Point.lon_C2, Point.lat_C2, &x, &y);
		Point.psi_CC = WP_Psi(x, y); //圆心连线北偏东
		WP_RotateXY(x, y, Point.psi_CC, &xw, &yw);
		Point.len = xw;        //圆心连线长度

		//----圆心连线长度<4R时进入CCC----//
		if (Point.len < 4 * radius)
		{
			//---余弦定理计算外切圆等腰三角形底边夹角---//
			float psi_C3C1C2 = acos(Point.len / (2 * 2 * radius)) * RADIAN_2_DEGREE;

			//---圆心与切点连线角度---//
			Point.psi_DR1 = WP_Psi2Psi360(Point.psi_CC - psi_C3C1C2);  //北偏东为正
			Point.psi_DR2 = WP_Psi2Psi360(Point.psi_CC + psi_C3C1C2 - 180);  //北偏东为正	

			//----切点坐标----//	
			xen = radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//切点1距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_D1, &Point.lat_D1); //切点1坐标

			xen = radius * sin(Point.psi_DR2 / RADIAN_2_DEGREE);
			yen = radius * cos(Point.psi_DR2 / RADIAN_2_DEGREE);//切点2距离圆心2北天东位置		   
			WP_XY2Pos(Point.lon_C2, Point.lat_C2, xen, yen, &Point.lon_D2, &Point.lat_D2); //切点2坐标	

			//----圆心3坐标----//
			xen = 2 * radius * sin(Point.psi_DR1 / RADIAN_2_DEGREE);
			yen = 2 * radius * cos(Point.psi_DR1 / RADIAN_2_DEGREE);//圆心3距离圆心1北天东位置			
			WP_XY2Pos(Point.lon_C1, Point.lat_C1, xen, yen, &Point.lon_C3, &Point.lat_C3);

			//----LRL轨迹长度---//
			C1_theta = Point.psi_CM - Point.psi_DR1;
			if (C1_theta < 0) {
				C1_theta = C1_theta + 360;
			}
			Point.psi_C1MD = -C1_theta * 0.5;
			Point.Circ1_psi = Point.psi_CM - C1_theta * 0.5;

			C2_theta = Point.psi_DR2 - Point.psi_CT;
			if (C2_theta < 0) {
				C2_theta = C2_theta + 360;
			}
			Point.psi_C2MD = -C2_theta * 0.5;
			Point.Circ2_psi = Point.psi_DR2 - C2_theta * 0.5;

			//---圆3为优弧---//
			C3_theta = 180 + 2 * psi_C3C1C2;
			Point.psi_C3MD = C3_theta * 0.5;
			Point.Circ3_psi = Point.psi_DR1 - (90 - psi_C3C1C2);

			Point.dubins_len1 = radius * C1_theta / RADIAN_2_DEGREE;
			Point.dubins_len2 = radius * C3_theta / RADIAN_2_DEGREE;
			Point.dubins_len3 = radius * C2_theta / RADIAN_2_DEGREE;
			Point.dubins_len = Point.dubins_len1 + Point.dubins_len2 + Point.dubins_len3;

			if (Temp.dubins_len < Point.dubins_len)//取路径最短者
			{
				WP_DubinsUpdate(&Point, &Temp);
			}
			else
			{
				strcpy(Point.dubins_type, dubins_type);
				//----参量保存用于最短路径比较---//
				WP_DubinsUpdate(&Temp, &Point);
			}
		}
		else {
			WP_DubinsUpdate(&Point, &Temp); //直接赋值
		}
	}

	WP_Pos2XY(Point.lon_D1, Point.lat_D1, Point.lon_D2, Point.lat_D2, &x, &y);
	Point.psi_D1D2 = WP_Psi(x, y);//切点连线北偏东
	Point.outTrack = psiT;//
	return Point;
}
/*赋值Dubins航点*/
void WP_DubinsUpdate(DubinsStruc* to, DubinsStruc* from)
{
	memcpy(to, from, sizeof(DubinsStruc));
}