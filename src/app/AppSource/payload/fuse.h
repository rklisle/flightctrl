/*
 * fuse.h
 *
 *  Created on: 2025Äê4ÔÂ19ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_PAYLOAD_FUSE_H_
#define SRC_PAYLOAD_FUSE_H_
#include "../support/os_framework.h"

extern OS_U8 InitFuse();
extern OS_U32 FuseRtHandler(STRU_422_MSG_INFO *data);
extern OS_U16 ChkFuseStandardFrame(OS_MEM* pmData);

#pragma pack(1)
typedef struct
{
	OS_U8 fuseMode;
	OS_U8 power24VMode;
    OS_U8 activeStatus;
    OS_U8 bitStatus;
    OS_U16 chechsum1;
    OS_U16 ms20tick;
    OS_S16 az;
    OS_S16 ay;
    OS_S16 ax;
    OS_U16 temp;
    OS_U16 g;
    OS_U16 power5V;
    OS_U16 fuseuf;
    OS_U8 keep;
    OS_U16 chechsum;
}FUSE_LONG_STATUS;

#pragma pack(0)

#endif /* SRC_PAYLOAD_FUSE_H_ */
