/*
 * Telemetry.h
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */

#ifndef SRC_CORE_TELEMETRY_H_
#define SRC_CORE_TELEMETRY_H_

#include "DataPool.h"

typedef struct telemetryParam
{
	p_DataPool pPool;
	char paramCode[PARAM_CODE_MAXLEN];
	OS_U8 byteCount;
}telemetryParam;

typedef struct telemetry
{
	OS_U16 telemetryTickInterval;
	OS_U16 telemetryParaCount;
	OS_U8 telemetryGroupID;
	OS_U16	telemetryPktBytes;
	telemetryParam *telemetryParamList;
}telemetry;


extern void InitTelemetry();//遥测、存储初始化
extern void TelemetryFrameOut();//遥测、存储输出



#endif /* SRC_CORE_TELEMETRY_H_ */
