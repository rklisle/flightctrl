/*
 * modSrvCtl.h
 *
 *  Created on: 2021年10月16日
 *      Author: QL
 */

#ifndef SRC_MODSRVCTL_H_
#define SRC_MODSRVCTL_H_

#include "../support/os_framework.h"



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

typedef struct
{
	OS_U16 ID;
	OS_U8 length;
	OS_U8 data[8];
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
	OS_U8 selfCheck;	//32位计数器
	OS_S16 srv1Cmd;
	OS_S16 srv1Read;
    OS_S16 srv2Cmd;
	OS_S16 srv2Read;
    OS_S16 srv3Cmd;
	OS_S16 srv3Read;
    OS_S16 srv4Cmd;
	OS_S16 srv4Read;
    OS_S16 srv5Cmd;
	OS_S16 srv5Read;
    OS_S16 srv6Cmd;
	OS_S16 srv6Read;
	OS_S16 srv1A;
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
	OS_U8 u8Acuator_Enb;	//扫频使能
	OS_S16 Acuator_Offset;	//控制零偏
	OS_U8 Acuator_Amp;		//控制幅值
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

extern OS_U8 SrvStatusUpdata();

OS_U8 StartMiniLoop(float freq, float amp, float zero, OS_U8 enable[6]);
OS_U8 MiniLoopSimulation();
extern OS_U8 ServoCtlOnce_6Rudder(double actDeg_1, double actDeg_2, double actDeg_3, 
    double actDeg_4, double actDeg_5, double actDeg_6);
extern OS_U32 ServoCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 SrvRtHandler(STRU_422_MSG_INFO * frame);
extern OS_U8 MsgToSrv(OS_DOUBLE ctrlDeg[6], OS_U8 ctrlMode/*control = 0x02, 0x44=setZero*/);

extern OS_U8 InitSrv();
extern OS_U16 ChkSrvFrame(OS_MEM* pmData);

#endif /* SRC_MODSRVCTL_H_ */
