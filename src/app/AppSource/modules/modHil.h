/*
 * modHil.h
 *
 *  Created on: 2024��4��6��
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODHIL_H_
#define SRC_MODULES_MODHIL_H_

#include "../support/os_framework.h"
#pragma pack(1)
typedef struct
{
	OS_U8 runStop;	///1:run,0:stop
	OS_U8 useNav;	//0:�÷������ݣ�1:��IMU����
	
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
	double pitch;
	double yaw;
	double roll;
	double airSpd;
	double lon;
	double lat;
	double alt;
	double vn;
	double vs;
	double ve;

	OS_U16 nav_state;//��ϵ���״̬0x60���Ե�����0x64��ϵ�����ת����������ֵΪ�����쳣
	
	double DD1;	// ���ٹ� ��ѹ
	double DD2;	// ���ٹ� ��ѹ

	double rpm_engine_state;//发动机状态转速
	OS_U16 engine_state;//����������
	
	double qf;//俯仰实现角
	double qh;//航向视线角
	double dqf;//俯仰视线角速度
	double dqh;///航向视线角速度
	OS_U16	seeker_state;//导引头锁定标识，默认值0x00为未锁定；
} STRU_HIL_INPUT;
typedef struct _Stru_Sim_Data_OUTPUT
{
	OS_U8 runStop;	///1:run,0:stop
	OS_U8 useNav;
    double wx;	
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    double pitch;
    double yaw;
    double roll;
    double airSpd;
    double lon;
    double lat;
    double alt;
    double vn;
    double vs;
    double ve;
    double DD1;
    double DD2;
    double DD3;
    double DD4;
    double mass;
    double xg;
    double arp;   
    double rpm_state;
    double MX_T_Disturb;	
	double qf;
    double qh;
    double dqf;
    double dqh;
    int TargetLocked;    	
	int MissileLauched;
}Stru_Sim_Data_OUTPUT;

typedef struct NAV_INPUT_SIMU
{
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
	double lon;
	double lat;
	float alt;
	float vn;
	float vs;
	float ve;
	unsigned char gpsLocated;
}NAV_INPUT_SIMU;
#pragma pack()

extern OS_U8 HilFlightStage();
extern OS_U32 HilRtHandler(STRU_422_MSG_INFO * frame);
extern STRU_HIL_INPUT hilInput;
/* ��λ��д�� HIL DD4 ����ţ�0x82 res1 ԭ���ش������ڲ��� HIL ��Ȧ�ӳ١� */
extern OS_U16 g_hilEchoSequence;
#endif /* SRC_MODULES_MODHIL_H_ */
