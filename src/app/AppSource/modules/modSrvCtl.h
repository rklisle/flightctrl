/*
 * modSrvCtl.h
 *
 *  Created on: 2021?~{(:~}10??16??
 *      Author: QL
 */

#ifndef SRC_MODSRVCTL_H_
#define SRC_MODSRVCTL_H_

#include "../support/os_framework.h"
#include <stdbool.h>


#define SERVO_TEST_RSP_FRM_LEN		(0x33)
#define SERVO_ZEROENCAP_RSP_FAM_LEN	(0x33)
#define SERVO_MINLOOP_RSP_FRM_LEN	(0xC)

#define SERVO_ZEROENCAP_TRANS_FRM_LEN	(0x1F)

#define SERVO_TEST_RSP_PAYLOAD_LEN			(SERVO_TEST_RSP_FRM_LEN 	- _422_FRAME_HEADER_LEN - _422_FRAME_FOOTER_LEN)
#define SERVO_ZEROENCAP_RSP_PAYLOAD_LEN		(SERVO_ZEROENCAP_RSP_FAM_LEN - _422_FRAME_HEADER_LEN - _422_FRAME_FOOTER_LEN)
#define SERVO_MINLOOP_RSP_PAYLOAD_LEN		(SERVO_MINLOOP_RSP_FRM_LEN 	- _422_FRAME_HEADER_LEN - _422_FRAME_FOOTER_LEN)
#define SERVO_ZERO_ENCAPTRANS_PAYLOAD_LEN 	(SERVO_ZEROENCAP_TRANS_FRM_LEN - _422_FRAME_HEADER_LEN - _422_FRAME_FOOTER_LEN)
//--------------------------------------
#define SERVO_CONTROL_PAYLOAD_LEN	(23)
#define SERVO_ZEROENCAP_PAYLOAD_LEN	(23)
#define SERVO_CHECK_REQ_LEN			(23)

// CAN~{6f;zO`9X6(Re~}
#define CAN_CMD_ID_BASE     0x00000600UL  // ~{V8An~}ID~{;yV7~}
#define CAN_RESP_ID_BASE    0x00000580UL  // ~{OlS&~}ID~{;yV7~}
#define SERVO_MIN_ANGLE     (-100.0)      // ~{WnP!=G6H~}
#define SERVO_MAX_ANGLE     (100.0)       // ~{Wn4s=G6H~}

// ~{6f;z=Z5c~}ID~{6(Re~}
typedef enum {
    SERVO_NODE_1 = 0x25,  // CAN¶æ»ú 0x25	// ×ó¸±Òí¶æ
    SERVO_NODE_2 = 0x26,  // CAN¶æ»ú 0x26	// ÓÒ¸±Òí¶æ
    SERVO_NODE_3 = 0x27,  // CAN¶æ»ú 0x27	// ×ó¸©Ñö¶æ
    SERVO_NODE_4 = 0x28,  // CAN¶æ»ú 0x28	// ÓÒ¸©Ñö¶æ
} ServoNodeID;

// CAN~{C|An=a99~}
typedef struct
{
	OS_U32 ID;            // CAN~{@)U9~}ID
	OS_U8 length;         // ~{J}>]3$6H~}
	OS_U8 data[8];        // ~{J}>]~}
}CAN_CMD;

extern CAN_CMD canCmds[];

typedef enum
{
	CMD_START_REPORT = 0,
	CMD_STOP_REPORT,
	CMD_SET_INTERVAL,
	CMD_SET_500K,
	CMD_SET_1000K,
	CMD_SET_NODEID,
	CMD_DO_CTRL,
	CMD_GET_CTRL,
	CMD_SET_ZERO,
	CMD_SAVE,
	CMD_GET_ID,
}CAN_CMD_ID;

typedef enum
{
	CMD_NULL = 0xFF,
    CMD_READ = 0x11,
}RECV_CMD_ID;

typedef struct
{
	float tickStep;
	float freq;
	unsigned char enable[8];
	float zeroAngle;
	float ampAngle;
}FREQ_SCAN;


#pragma pack(1)
typedef struct
{
	OS_U8 selfCheck;	//32???????~{!B~}
	OS_S16 srv1Cmd;
	OS_S16 srv1Read;	//0.001
    OS_S16 srv2Cmd;		//0.001
	OS_S16 srv2Read;
    OS_S16 srv3Cmd;
	OS_S16 srv3Read;
    OS_S16 srv4Cmd;
	OS_S16 srv4Read;
    OS_S16 srv5Cmd;
	OS_S16 srv5Read;
    OS_S16 srv6Cmd;
	OS_S16 srv6Read;
	OS_S16 srv1A;	//0.001
    OS_S16 srv2A;
    OS_S16 srv3A;
    OS_S16 srv4A;
    OS_S16 srv5A;
    OS_S16 srv6A;
    OS_U16 keep;
    OS_U8 mark;
}STRU_SRV_INFO;


typedef struct
{
	OS_U8 u8Acuator_Enb;	//?~{!'~}??????
	OS_S16 Acuator_Offset;	//????????
	OS_U8 Acuator_Amp;		//?????~{(4~}??
}STRU_SERVO_ONE;

typedef struct
{
	OS_U16 Freq;
	OS_U16 Amp;
	OS_S16 count;
	OS_U8 LevelOneAcuator_Enb;
	OS_U8 LevelTwoAcuator_Enb;
	OS_U8 LevelThreeAcuator_Enb;
}STRU_SERVO_MinLoopTest_REQUEST;
#pragma pack()

OS_U8 CanRtServoHandler(OS_U32 id, OS_BOOL ext_id, const OS_U8* data, OS_U8 len);

// ~{:/J}IyCw~}
OS_U8 SrvStatusUpdata();

// CAN~{O`9X:/J}~}
OS_U8 Servo_SetAngle_CAN(ServoNodeID node, double angle);
OS_U8 Servo_ReadAngle_CAN(ServoNodeID node, double *angle);
OS_U8 Servo_SetTorqueZero_CAN(ServoNodeID node);
OS_U8 Servo_SetMidpoint_CAN(ServoNodeID node);
OS_U8 Servo_SendCANFrame(OS_U32 id, OS_U8 *data, OS_U8 len);
OS_U8 AngleToPosition_CAN(double angle, OS_U8 *high, OS_U8 *low);
double PositionToAngle_CAN(OS_U8 high, OS_U8 low);

OS_U8 StartMiniLoop(float freq, float amp, float zero, OS_U8 enable[6]);
OS_U8 MiniLoopSimulation();
extern OS_U8 ServoCtlOnce_6Rudder(double actDeg_1, double actDeg_2, double actDeg_3, 
    double actDeg_4, double actDeg_5, double actDeg_6);
extern OS_U32 ServoCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 SrvRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 MsgToSrv(OS_DOUBLE ctrlDeg[6], OS_U8 ctrlMode/*control = 0x02, 0x44=setZero*/);

// extern OS_U8 InitSrv();
extern OS_U16 ChkSrvFrame(OS_MEM* pmData);

#endif /* SRC_MODSRVCTL_H_ */
