/*
 * Telecontrol.h
 *
 *  Created on: 2022Äê1ÔÂ21ÈÕ
 *      Author: ChengHongjing
 */

#ifndef SRC_CORE_TELECONTROL_H_
#define SRC_CORE_TELECONTROL_H_

#include "DataPool.h"

extern OS_U32 CmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 CmdResponseHandler(OS_U8 msgID, OS_U16 msgLen, OS_U8* buf);


#endif /* SRC_CORE_TELECONTROL_H_ */
