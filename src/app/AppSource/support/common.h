/*
 * common.h
 *
 *  Created on: 2022Äê4ÔÂ29ÈÕ
 *      Author: Lenovo
 */

#ifndef SRC_SUPPORT_COMMON_H_
#define SRC_SUPPORT_COMMON_H_

#include "os_bufferQueue.h"
#include "os_basic.h"
#include "os_framework.h"

#define MAX_SIMUL_FRAME	(3)
#define USR_UART_PL_CK_COUNT	(1)
#define USR_UART_PL_CK_422MAX	(10)

const STRU_STANDARD_FRAME* PeekStandardMessage();
const STRU_CAN_MSG* PeekCanMessage();
OS_U16 ChkStandardFrame(OS_MEM* pmData);
OS_U16 ChkDataLinkFrame(OS_MEM* pmData);
OS_S32 SendCRCed422Message(OS_U8 ck, OS_U8 ch, OS_MEM* pmData, OS_S16 crcStartOffset, OS_U16 u16Length);

extern void DoCalcXYZ(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double *x, double *y, double *z, double *vx, double *vy, double *vz);
OS_U8 uav_density(float alt, float *ru);
OS_BOOL doubleEqual(double a, double b);


#endif /* SRC_SUPPORT_COMMON_H_ */
