/*
 * Telecontrol.h
 *
 *  Created on: 2022年1月21日
 *      Author: ChengHongjing
 */

#ifndef SRC_CORE_TELECONTROL_H_
#define SRC_CORE_TELECONTROL_H_

#include "DataPool.h"

extern OS_U32 CmdHandler(STRU_422_MSG_INFO * frame);//数据链接收消息梳理
extern OS_U8 CmdResponseHandler(OS_U8 msgID, OS_U16 msgLen, OS_U8* buf);//数据链，地面站应答消息反馈


#endif /* SRC_CORE_TELECONTROL_H_ */
