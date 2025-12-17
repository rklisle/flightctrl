/*
 * navSupport.h
 *
 *  Created on: 2024Äê3ÔÂ4ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_NAVSUPPORT_H_
#define SRC_NAVSUPPORT_H_

#include "./support/os_framework.h"
#include "./nav/nav2.h"
#include "./stmToZynq.h"
extern NAV_OUTPUT       nav_output;
extern NAV_INPUT_MAG    mag_Data;
extern NAV_INPUT_IMU 	nav_input_imu;
extern NAV_INPUT_GPS    nav_input_gps;

extern OS_BOOL useSimuData;
OS_U8 NavInputGenerate();
OS_U8 NavOutputHandle();
OS_U8 DoNavRun();

#endif /* SRC_NAVSUPPORT_H_ */
