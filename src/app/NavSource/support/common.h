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
#define USR_UART_PL_CK_422MAX	(6)

const STRU_STANDARD_FRAME* PeekStandardMessage();
const STRU_CAN_MSG* PeekCanMessage();
OS_U16 ChkStandardFrame(OS_MEM* pmData);
OS_S32 SendCRCed422Message(OS_U8 ck, OS_U8 ch, OS_MEM* pmData, OS_S16 crcStartOffset, OS_U16 u16Length);

OS_U8 GeoQtoLunchDeg7801(OS_FLOAT Q[4],OS_FLOAT dircet, OS_DOUBLE lat, OS_FLOAT *pitch, OS_FLOAT *yaw, OS_FLOAT *roll);
OS_U8 GeoQtoLunchDegMems(OS_FLOAT Q[4], OS_DOUBLE lunchLon, OS_DOUBLE lunchLat, OS_DOUBLE dir, OS_DOUBLE curlon, OS_DOUBLE curlat, OS_FLOAT *pitch, OS_FLOAT *yaw, OS_FLOAT *roll);

OS_U8 uav_density(float alt, float *ru);
OS_BOOL doubleEqual(double a, double b);

double EcllipesToAltitude(double lon, double lat, double ecllipseHigh);
#endif /* SRC_SUPPORT_COMMON_H_ */
