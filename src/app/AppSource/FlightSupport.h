/*
 * FlightSupport.h
 *
 *  Created on: 2022年3月23日
 *      Author: Lenovo
 */
#include "./support/os_framework.h"


#ifndef SRC_FLIGHTSUPPORT_H_
#define SRC_FLIGHTSUPPORT_H_

extern void FlightOutputHandle();
extern void FlightInputGenerate();
extern void DoFlightRun();
extern void NavZeroInit();

extern OS_U8 InitSafeArea();
extern OS_U8 JudgeHomeward();
extern OS_U8 DoReturnHomeward();
extern OS_U8 DoOpenUm();
extern void UpdateViewAngle();
extern OS_BOOL JudgeInFuseArea(double lon, double lat);

extern OS_BOOL lockEngine;
extern OS_U8 SrvProtect;
extern OS_BOOL receiveMark;

// 定义二维点结构体
typedef struct {
    double x;
    double y;
} Point;

extern int safePointCount;
extern Point safePoints[20];
#endif /* SRC_FLIGHTSUPPORT_H_ */
