#ifndef _WP_H_
#define _WP_H_

#include "RoutePoint.h"
#define wp_fTimerStep 0.005

typedef struct
{
    double psi;          /*[航段方位角][deg]*/
    double alt;          /*[航段高度][m]*/
	float  delt_alt;     /*[高度差值][m]*/
    int vxd1;            /*[A点任务特征]*/
    int vxd2;            /*[B点任务特征]*/
    float outTrack;      /*[B点切出角度]*/
	float hover_radis;   /*[B点盘旋半径]*/
	float AttackAngle;   /*[攻击点攻击角度]*/
    double lon, lat;     /*[航段终点经纬度][deg]*/
    double len;          /*[航段长度][m]*/
    int FlightTime;      /*[指令航时][s]*/
    float ac_FlightTime; /*[实际航时][s]*/
    float Velocity_cmd;  /*[航段速度指令]*/
	unsigned char if_airspeed_used;              /*[是否启用空速控制]*/     
	unsigned char vxd_guide;                     /*[B点是否为指点]*/
	unsigned char vxd_PreHover;                  /*[是否进入预盘旋]*/
	unsigned char Hover_Round;                   /*[盘旋圈数]*/
} LineStruc;

typedef struct {
	double lon_C1, lon_C2, lon_C3;
	double lat_C1, lat_C2, lat_C3;
	double lon_D1, lon_D2;
	double lat_D1, lat_D2;
	double lon_preset, lat_preset;

	float psi_MT; //弹目连线角度
	float psi_CC; //圆心连线角度
	float psi_CM; //圆心1与进入点连线角度
	float psi_CT; //圆心2与切出点连线角度
	float psi_DR1; //圆心1与切点1连线角度
	float psi_DR2; //圆心2与切点2连线角度
	float psi_D1D2; //切点连线角度
	float outTrack;//切出点角度

	float psi_C1MD, psi_C2MD, psi_C3MD; //圆弧角度差
	float Circ1_psi, Circ2_psi, Circ3_psi;//圆心与航线交线顶点夹角

	float len;    //圆心连线长度
	float dubins_len1; //第一段长
	float dubins_len2; //第二段长
	float dubins_len3; //第三段长
	float dubins_len;   //总轨迹长度
	char dubins_type[3]; //类型
}DubinsStruc;
void WP_InitRoutePoint(void);
/*(-180~180)轉換為(0~360)*/
double WP_Psi2Psi360(double angle);
/*[将航向方位角限製在(-180~+180)]*/
double WP_Psi2Psi(double angle);
/*[计算直角坐标(x,y)形成的航向方位角]*/
double WP_Psi(double x, double y);
/*[将经纬度转化为东北系直角坐标]*/
void WP_Pos2XY(double lon_ref, double lat_ref, double lon, double lat, double *x, double *y);
/*[将东北系直角坐标转化为经纬度]*/
void WP_XY2Pos(double lon_ref, double lat_ref, double x, double y, double *lon, double *lat);
/*将XY旋转Psi角得到新的XY*/
void WP_RotateXY(double x, double y, double Psi, double *xR, double *yR);
/*[取第dot个航段数据]*/
int WP_GetLine(int dot, LineStruc *AB);
/*[根据A,B两点求取航段数据]*/
void  WP_GetLine2Point(RoutePoint A, RoutePoint B, LineStruc* AB);
/*[根据A,B两点求取航段数据]*/
void WP_MakeLine(double lon_a, double lat_a, double lon_b, double lat_b, LineStruc *AB);
/*[求位置(lon,lat)相对于航段AB的待飞距离和侧偏距]*/
void WP_LateWay(LineStruc *AB, double lon, double lat, double psi, double *dZ, double *dL, double *dPsi);
/*[取第dot个航路点数据]*/
int WP_GetPoint(int dot, RoutePoint *A);
double WP_GetR2g(int dot);
float   WP_GetDubinsR2g(DubinsStruc dubins, double horizVm);
/*生成圆轨迹*/
int WP_MakeCirc(double lon_a, double lat_a, double dpsi, double psi, double len2turn, double *lon_b, double *lat_b, double *circ_psi);
/*生成盘旋圆切点坐标*/
int WP_MakeDR(double lon_a, double lat_a, double lon_circ, double lat_circ, float radis, char dir, double* lon_b, double* lat_b, float* psi);
/*计算导弹实时位置与圆轨迹的侧偏距，到切出点的直线距离，角度*/
void WP_LateCirc(double circ_lon, double circ_lat, double radius, double circ_psi, double psi_tmp, double ac_lon, double ac_lat, double ac_psi, double *dZ, double *L1, double *dPsi);
/*计算导弹到下一航点的待飞时间*/
int WP_LateTgoCal(double len, double VHorizontal, double dpsi);
void WP_LateDubins(LineStruc* AB, LineStruc* AB_preset, double circ_lon, double circ_lat, double cur_psi, double radius, DubinsStruc dubins_temp, double horizVm, double* dZ, double* L1, double* dPsi, unsigned char* st);
void WP_DubinsMakeCD(double lon_a, double lat_a, double psi, double length, double* lon_b, double* lat_b);
DubinsStruc WP_DubinsCal(double lonM, double latM, float psiM, double lonT, double latT, float psiT, float radius); /*[杜宾斯圆计算]*/
void WP_DubinsUpdate(DubinsStruc* to, DubinsStruc* from);
extern RoutePoint rp[RP_MAX_NUMBER];
#endif
//
