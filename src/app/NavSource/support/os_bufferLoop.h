/*
 * os_bufferLoop.h
 *
 *  Created on: 2022Äê4ÔÂ29ÈÕ
 *      Author: Lenovo
 */

#ifndef SRC_SUPPORT_OS_BUFFERLOOP_H_
#define SRC_SUPPORT_OS_BUFFERLOOP_H_
#include "../core/BusInteract.h"
#include "os_framework.h"
#include "os_types.h"

#define BUFFER_LOOP_POOL_MAX	(4096)
#define FRAME_MIN_LEN			(8)

#pragma pack(1)
typedef struct buff_loop
{
	OS_BOOL inited;
	int head;
	int tail;
	OS_U8 syncHead_A;
	OS_U8 syncHead_B;
	int lenPos;
	int lenExtern;
	int fixedLen;
	OS_U8 BUFFER[BUFFER_LOOP_POOL_MAX];
}BUFF_LOOP;
#pragma pack()

OS_U8 PushLoopMem(OS_U8 rtIndex, OS_U8 *mem, OS_U16 len);
OS_S32 PopFrameLoopMem(OS_U8 rtIndex, OS_U8 *memFrame, OS_U16* u16len);
extern BUFF_LOOP buffLoop[MODULE_COUNT];
#endif /* SRC_SUPPORT_OS_BUFFERLOOP_H_ */
