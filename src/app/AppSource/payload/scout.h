/*
 * scout.h
 *
 *  Created on: 2023Äê5ÔÂ19ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_PAYLOAD_SCOUT_H_
#define SRC_PAYLOAD_SCOUT_H_
#include "../support/os_framework.h"

extern OS_U8 IsLoadImage;
extern OS_U8 ScoutAutoSend();
extern OS_U8 InitScout();
extern OS_U32 ScoutRtHandler(STRU_422_MSG_INFO *data);
extern OS_U16 ChkScoutStandardFrame(OS_MEM* pmData);
extern void ScoutGroundHandler(STRU_422_MSG_INFO *data);
#pragma pack(1)
typedef struct
{
	  OS_U8 ctrlback;
    OS_U8 modback;
    OS_U8 lowlightback;
    OS_U8 imageback;
    OS_U8 selfcheck;
    OS_S16 viewPitchSpd;
    OS_S16 viewYawSpd;
    OS_U8 targetback;
    OS_U8 lightState;       //1 zero, 2 finding, 3 follow, 4 remember
    OS_S16 scoutPitch;
    OS_S16 scoutYaw;
    OS_U32 usedImageFrameCount;
    OS_U8 scoutCount;
    OS_S16 wz;
    OS_S16 wy;
    OS_U8 version[3];
}SCOUT_STATUS;

typedef struct 
{
    OS_U8 ctrlCmd;
    OS_U8 modelIndex;
    OS_U8 workMode;
    OS_U8 lowlighlCtrl;
    OS_U8 modelbookCmd;
    OS_U8 targetType;
    OS_U16 targetDis;
    OS_U16 spd;
    OS_S16 pitch;
    OS_S16 dir;
    OS_S16 roll;
    OS_S16 pitchSpd;
    OS_S16 dirSpd;
    OS_S16 rollSpd;
    OS_S32 lon;
    OS_S32 lat;
    OS_S32 targetlon;
    OS_S32 targetlat;
    OS_S16 high;
    OS_S16 targetHigh;
    OS_S16 scoutPitchSet;
    OS_S16 scoutyawSet;
    OS_U16 offsety;
    OS_U16 offsetx;
    OS_U32 useOffsetFrameCount;
    OS_U8 back[8];
}SCOUT_CMD;
#pragma pack(0)

#endif /* SRC_PAYLOAD_SCOUT_H_ */
