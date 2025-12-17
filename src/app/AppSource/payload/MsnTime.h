/*
 * MsnTime.h
 *
 *  Created on: 2023Äê5ÔÂ23ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_PAYLOAD_MSNTIME_H_
#define SRC_PAYLOAD_MSNTIME_H_

#include "../support/os_framework.h"
#include "../core/DataPool.h"
extern OS_U32 GetDateBySecond(OS_U64 bjTimeSecond_2000);
extern OS_U32 SetSecondByDate(OS_U16 year, OS_U8 month, OS_U8 day, OS_U8 hour, OS_U8 minite, OS_U8 second);
#endif /* SRC_PAYLOAD_MSNTIME_H_ */
