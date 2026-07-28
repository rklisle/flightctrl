/*
 * modSrvCtl.c
 *
 *  Created on: 2021??10??16??
 *      Author: QL
 */

#include "modSrvCtl.h"
#include "modPwrSeqCtl.h"
#include "../FlightSupport.h"
#include "./modEngine.h"
#include "../interface/interface_power.h"
#include "../interface/interface_gpio.h"
#include "../interface/interface_can.h"
#include "../interface/interface_uart.h"
#include "../Interface/interface_timer.h"  // 新增timer接口
#include "../core/BusInteract.h"
#include "../core/DataPool.h"
// #include "../flight/os_flight_io.h"
#include "../support/os_bufferLoop.h"
#include "tx_api.h"

#include <math.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#ifndef PI
#define PI		(3.141592653589793)           //3.1415926535
#endif
#define d2r		(57.29577951308402)

int cycleCount;
double toRad(double deg)
{
	return deg / d2r;
}

double toDeg(double rad)
{
	return rad * d2r;
}
STRU_SERVO_MinLoopTest_REQUEST g_stServoMinLoopTestReq={0};

static FREQ_SCAN fs = {
		0.005,	// tickStep
		0.25,	// freqs
		{1,			1,		1,		1,1,1,1,1},	//eable
		5, // zeroAngle
		10 // ampAngle
	};

OS_U8 AutoZero();
    
//OS_U8 InitSrv()	//舵机: 没人调了，这是原先串口的Init
//{
//    // 初始化CAN通信（假设已经有CAN初始化函数）
//    // CAN_Init(500000); // 500Kbps波特率
//    
//    // 初始化PWM（5、6号舵机）
//    // PWM_Init(); // 初始化PWM输出

//	buffLoop[RT_SRV].syncHead_A = 0xEE;
//	buffLoop[RT_SRV].syncHead_B = 0x00;
//	buffLoop[RT_SRV].lenExtern = 6;	
//	buffLoop[RT_SRV].head = 0;
//	buffLoop[RT_SRV].tail = 0;
//	buffLoop[RT_SRV].lenPos = 1;	
//	buffLoop[RT_SRV].fixedLen = 48;
//	buffLoop[RT_SRV].inited = TRUE;
//    return 0;
//}

OS_U16 ChkSrvFrame(OS_MEM* pmData)	//舵机 需要更新CAN帧检查逻辑 这个没人调了。是原先串口的。
{
	if(pmData == PTR_NULL)
		return 0;
	OS_U8 data[100];
	memcpy(data,pmData, 100);

	if(pmData[1] != 0xEE || pmData[2] != 0x00|| pmData[3] != 0xAA )
	{
		return 0;
	}
    OS_U8 temp_crc[2];
    temp_crc[0] = pmData[48];
    temp_crc[1] = pmData[47];
   
    OS_U16 crcValue;
    memcpy(&crcValue, temp_crc, 2);
   
    OS_U16 t = crc16_ccitt(pmData + 1, 46);
    if(t != crcValue)
    {
        return 0;
    }
	pmData[3] = pmData[5];
	pmData[4] = 0;
	pmData[5] = 0;
	pmData[6] = 0x11;
	return pmData[3] + 8;
}

/**
 * @brief 角度转换为CAN舵机位置值
 * @param angle 角度值（-100.0° 到 +100.0°）
 * @param high 输出高字节
 * @param low 输出低字节
 * @return 0:成功, 1:角度超出范围
 */
uint8_t AngleToPosition_CAN(double angle, uint8_t *high, uint8_t *low)
{
    // 角度范围检查
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
        return 1;
    }
    
    // 角度×10得到整数（精度0.1°）
    int16_t pos_value = (int16_t)(angle * 10.0);
    
    // 直接使用int16_t（本身就是补码存储）
    *low = (uint8_t)(pos_value & 0xFF);
    *high = (uint8_t)((pos_value >> 8) & 0xFF);
    
    return 0;
}

/**
 * @brief CAN舵机位置值转换为角度
 * @param high 高字节
 * @param low 低字节
 * @return 角度值
 */
double PositionToAngle_CAN(uint8_t high, uint8_t low)
{
    // 将两个字节组合成16位有符号整数
    int16_t pos_value = (high << 8) | low;
    
    // 转换为角度
    return (double)pos_value / 10.0;
}

/**
 * @brief 设置CAN舵机角度
 * @param node 舵机节点ID
 * @param angle 目标角度（-100.0° 到 +100.0°）
 * @return 0:成功, 1:角度超出范围, 2:CAN发送失败
 */
OS_U8 Servo_SetAngle_CAN(ServoNodeID node, double angle)
{
    uint8_t high, low;
    
    // 角度转换为位置值
    if (AngleToPosition_CAN(angle, &high, &low) != 0) {
        return 1;  // 角度超出范围
    }
    
    // 构建CAN数据帧（位置设置命令）
    uint8_t data[8] = {
        0x22,       // 字节1：固定
        0x03,       // 字节2：位置设置索引
        0x60,       // 字节3：子索引
        0x00,       // 字节4：固定
        low,        // 字节5：位置低字节
        high,       // 字节6：位置高字节
        0x00,       // 字节7：保留
        0x00        // 字节8：保留
    };
    
    // 计算CAN ID（指令ID基址 + 节点号）
    uint32_t can_id = CAN_CMD_ID_BASE | node;
    
    // 发送CAN帧
    return Servo_SendCANFrame(can_id, data, 8);
}

/**
 * @brief 发送CAN帧到舵机
 * @param id CAN扩展ID（29位）
 * @param data 数据
 * @param len 数据长度
 * @return 0:成功, 1:失败
 */
OS_U8 Servo_SendCANFrame(OS_U32 id, OS_U8 *data, OS_U8 len)
{
    // 使用FDCAN1（fdCan[0]）发送舵机控制指令
    // 舵机使用CAN扩展帧（29位ID）,500Kbps
    if (SendCanFrameExt(CAN_RT_SRV, id, len, data) == 0) {
        return 0;  // 成功
    } else {
        return 1;  // 失败
    }
    return 1;
}

static OS_U8 Servo_IsKstSdoResponse(const OS_U8 *data)
{
    switch (data[0]) {
    case 0x4B:
    case 0x43:
    case 0x4F:
    case 0x60:
    case 0x80:
        return 1;
    default:
        return 0;
    }
}

static void Servo_ApplyAngleReadback(OS_U8 node, double angle)
{
    switch (node) {
    case SERVO_NODE_1:
        SETDATA(pDataPoolSrv, "Sr1Read", (OS_S16)(angle * 100), OS_S16);
        break;
    case SERVO_NODE_2:
        SETDATA(pDataPoolSrv, "Sr2Read", (OS_S16)(angle * 100), OS_S16);
        break;
    case SERVO_NODE_3:
        SETDATA(pDataPoolSrv, "Sr3Read", (OS_S16)(angle * 100), OS_S16);
        break;
    case SERVO_NODE_4:
        SETDATA(pDataPoolSrv, "Sr4Read", (OS_S16)(angle * 100), OS_S16);
        break;
    case SERVO_NODE_5:
        SETDATA(pDataPoolSrv, "Sr5Read", (OS_S16)(angle * 100), OS_S16);
        break;
    case SERVO_NODE_6:
        SETDATA(pDataPoolSrv, "Sr6Read", (OS_S16)(angle * 100), OS_S16);
        break;
    default:
        break;
    }
}

// ????????
// #define PSC_VALUE       23      // ???24
// #define ARR_VALUE       30029   // ????30030????
#define CNT_TIME_US     0.1f    // ???????0.1??s

#define PULSE_MIN_US    900     // ????????
#define PULSE_MAX_US    2100    // ???????
#define PULSE_CENTER_US 1500    // ??????????0??
#define US_PER_DEG      10.0f   // ????????s??

#define CCR_MIN         ((uint32_t)(PULSE_MIN_US / CNT_TIME_US))      // 9000
#define CCR_MAX         ((uint32_t)(PULSE_MAX_US / CNT_TIME_US))      // 21000

uint32_t angle_to_ccr(double angle_deg) {
    // ??????????1500??s + angle??10??s
    double pulse_us = PULSE_CENTER_US + (angle_deg * US_PER_DEG);

    // ?????CCR??pulse_us / 0.1
    uint32_t ccr = (uint32_t)(pulse_us / CNT_TIME_US);
    
    // ???????????????
    if (ccr < CCR_MIN) ccr = CCR_MIN;
    if (ccr > CCR_MAX) ccr = CCR_MAX;
    
    return ccr;
}
/***********************************************************
 * ????????: MsgToSrv()
 * ????????: ???????????????????????????????????????????????????
 * ????:	???
 ***********************************************************/
OS_U8 MsgToSrv(OS_DOUBLE ctrlDeg[SRV_CHANNEL_COUNT], OS_U8 ctrlMode, OS_U8 srvCount)/*control=0x02, setZero=0x44*/
{
    if (srvCount > SRV_CHANNEL_COUNT)
        srvCount = SRV_CHANNEL_COUNT;
    if (srvCount < 6)
        srvCount = 6;
    SETDATA(pDataPoolSrv, "Sr1Cmd", ctrlDeg[0] * 100, OS_S16); 
	SETDATA(pDataPoolSrv, "Sr2Cmd", ctrlDeg[1] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr3Cmd", ctrlDeg[2] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr4Cmd", ctrlDeg[3] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr5Cmd", ctrlDeg[4] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr6Cmd", ctrlDeg[5] * 100, OS_S16);
	if (ctrlMode == 0x02) 
	{
#ifdef SERVO_CAN
		Servo_SetAngle_CAN(SERVO_NODE_1, ctrlDeg[1]);       
		Servo_SetAngle_CAN(SERVO_NODE_2, ctrlDeg[2]);       
		Servo_SetAngle_CAN(SERVO_NODE_3, ctrlDeg[3]);       
		Servo_SetAngle_CAN(SERVO_NODE_4, ctrlDeg[4]);      
        Servo_SetAngle_CAN(SERVO_NODE_5, ctrlDeg[0]);       
		Servo_SetAngle_CAN(SERVO_NODE_6, ctrlDeg[5]);      
#else
        AngleServo_SetAngle(SERVO_PWM1, (float)ctrlDeg[4]);	//实控舵机航向左
        AngleServo_SetAngle(SERVO_PWM2, (float)ctrlDeg[2]);	//实控舵机副翼左
        AngleServo_SetAngle(SERVO_PWM3, (float)ctrlDeg[0]);	//实控舵机俯仰左
        AngleServo_SetAngle(SERVO_PWM4, (float)ctrlDeg[1]);	//实控舵机俯仰右
        AngleServo_SetAngle(SERVO_PWM5, (float)ctrlDeg[3]);	//实控舵机副翼右
        AngleServo_SetAngle(SERVO_PWM6, (float)ctrlDeg[5]);	//实控舵机航向右
        // AngleServo_SetAngle(SERVO_PWM1, (float)ctrlDeg[0]); 	
        // AngleServo_SetAngle(SERVO_PWM2, (float)ctrlDeg[1]);	
        // AngleServo_SetAngle(SERVO_PWM3, (float)ctrlDeg[2]);	
        // AngleServo_SetAngle(SERVO_PWM4, (float)ctrlDeg[3]);	
        // AngleServo_SetAngle(SERVO_PWM5, (float)ctrlDeg[4]);	
        // AngleServo_SetAngle(SERVO_PWM6, (float)ctrlDeg[5]);	
        if (srvCount >= 7)
            AngleServo_SetAngle(SERVO_PWM7, (float)ctrlDeg[6]);
#endif
    }
	else if (ctrlMode == 0x44) 
	{
#ifdef SERVO_CAN
        Servo_SetMidpoint_CAN(SERVO_NODE_1);
        Servo_SetMidpoint_CAN(SERVO_NODE_2);
        Servo_SetMidpoint_CAN(SERVO_NODE_3);
        Servo_SetMidpoint_CAN(SERVO_NODE_4);
        Servo_SetMidpoint_CAN(SERVO_NODE_5);
        Servo_SetMidpoint_CAN(SERVO_NODE_6);
#else
        AngleServo_SetAngle(SERVO_PWM2, 0.0f);
        AngleServo_SetAngle(SERVO_PWM3, 0.0f);
        AngleServo_SetAngle(SERVO_PWM4, 0.0f);
        AngleServo_SetAngle(SERVO_PWM5, 0.0f);
        AngleServo_SetAngle(SERVO_PWM1, 0.0f);
        AngleServo_SetAngle(SERVO_PWM6, 0.0f);
        if (srvCount >= 7)
            AngleServo_SetAngle(SERVO_PWM7, 0.0f);
#endif
    }
    return 0;
}

/**
 * @brief ????CAN??????????
 */
OS_U8 Servo_SetMidpoint_CAN(ServoNodeID node)
{
    uint8_t data[8] = {
        0x22,  // ???1?????
        0x09,  // ???2??????????????
        0x30,  // ???3????????
        0x00,  // ???4?????
        0x00, 0x00, 0x00, 0x00  // ????
    };
    
    uint32_t can_id = KST_CAN_SDO_TX_ID(node);
    
    return Servo_SendCANFrame(can_id, data, 8);
}

/**
 * @brief ????CAN???????0???????????
 */
OS_U8 Servo_SetTorqueZero_CAN(ServoNodeID node)
{
    uint8_t data[8] = {
        0x22,  // ???1?????
        0x10,  // ???2??????0????????
        0x30,  // ???3????????
        0x00,  // ???4?????
        0x00, 0x00, 0x00, 0x00  // ????
    };
    
    uint32_t can_id = KST_CAN_SDO_TX_ID(node);
    
    return Servo_SendCANFrame(can_id, data, 8);
}

/***********************************************************
 * ????????: ServoCtlOnce_6Rudder()
 * ????????: ????????????????????????????????6??????????
 * ????:	???
 ***********************************************************/
OS_U8 ServoCtlOnce_6Rudder( double actDeg_1,
                            double actDeg_2,
                            double actDeg_3,
                            double actDeg_4,
                            double actDeg_5,
                            double actDeg_6)	//03 ??????
{
    double deg[SRV_CHANNEL_COUNT] = {0};
    deg[0] = actDeg_1; 
    deg[1] = actDeg_2; 
    deg[2] = actDeg_3; 
    deg[3] = actDeg_4; 
    deg[4] = actDeg_5; 
    deg[5] = actDeg_6; 
    
    MsgToSrv(deg, 0x02, 6);
    
    return 0;
}

// OS_U8 SaveSrvInDataPool(STRU_SRV_INFO *data);	//????????

// /***********************************************************
//  * ????????: ServoRtHandler()
//  * ????????: ???????????????????????????????????????????????????????????????????????????????????????????????
//  * 			??????????????????????????????:
//  * 			1.????????0x11		: ???????????????????????????????????????????????????????????
//  * 			2.?????????0x22		: ?????????????????????????????????????????????????????????????????????
//  * 			3.?????????0x23		: ???????
//  * ????????: <TXII-Y1 422???????????>
//  * ????:	???Z
//  ***********************************************************/
// OS_U32 SrvRtHandler(STRU_422_MSG_INFO * srvMsg)	
// {
// 	//?????????280???????????????014???????
// 	//????????????????????
// 	//?????????????????????
// 	RECV_CMD_ID recvCmdId = srvMsg->u8MsgID;
// 	//????????????????
// 	switch(recvCmdId)
// 	{
// 		case 0x11:
//             SaveSrvInDataPool((STRU_SRV_INFO *)srvMsg->au8Data);
// 			break;
	
// 		default:
// 			break;
// 	}
//     g_DeviceState.srvCountDown = 200;
// 	return 0;
// }

/**
 * @brief CAN?????????????
 * @param id CAN ID
 * @param ext_id ???????
 * @param data ????
 * @param len ???????
 */
OS_U8 CanRtServoHandler(OS_U32 id, OS_BOOL ext_id, const OS_U8* data, OS_U8 len)
{
    OS_U8 node;

    if (!ext_id) {
        return (OS_U8)-1;
    }

    if (KST_CAN_IS_SDO_RX_ID(id)) {
        node = (OS_U8)(id & 0xFFU);

        if (len >= 2U && !Servo_IsKstSdoResponse(data)) {
            double angle = PositionToAngle_CAN(data[1], data[0]);
            Servo_ApplyAngleReadback(node, angle);
            g_DeviceState.srvCountDown = 200;
            return 0;
        }

        switch (data[1]) {
        case 0x02:
            if (data[0] == 0x4B && len >= 6U) {
                Servo_ApplyAngleReadback(node, PositionToAngle_CAN(data[5], data[4]));
            }
            break;
        case 0x05:
            if (data[0] == 0x43 && len >= 7U) {
                OS_U16 current_ma = (OS_U16)((data[5] << 8) | data[4]);
                double current_a = current_ma / 100.0;
                switch (node) {
                //case SERVO_NODE_1:
                    //SETDATA(pDataPoolSrv, "Sr1A", (OS_S16)(current_a * 1000), OS_S16);
                    //break;
                //case SERVO_NODE_2:
                    //SETDATA(pDataPoolSrv, "Sr2A", (OS_S16)(current_a * 1000), OS_S16);
                  //  break;
                //case SERVO_NODE_3:
                    //SETDATA(pDataPoolSrv, "Sr3A", (OS_S16)(current_a * 1000), OS_S16);
                   // break;
                //case SERVO_NODE_4:
                  //  SETDATA(pDataPoolSrv, "Sr4A", (OS_S16)(current_a * 1000), OS_S16);
                //    break;
                case SERVO_NODE_5:
                    SETDATA(pDataPoolSrv, "Sr5A", (OS_S16)(current_a * 1000), OS_S16);
                    break;
                case SERVO_NODE_6:
                    SETDATA(pDataPoolSrv, "Sr6A", (OS_S16)(current_a * 1000), OS_S16);
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }

        g_DeviceState.srvCountDown = 200;
        return 0;
    }

    if (KST_CAN_IS_ALARM_ID(id) && len >= 2U) {
        if (data[0] == 0x0A && data[1] == 0x02) {
            g_DeviceState.srvCountDown = 0;
        }
        return 0;
    }

    if (KST_CAN_IS_GUARD_ID(id) && len >= 1U) {
        g_DeviceState.srvCountDown = 200;
        return 0;
    }

    return 0;
}

/***********************************************************
 * ????????: InsertServoTestData()
 * ????????: ???????????????????????????????????????????????????????????????????????FREQ_SCAN??
 * 			?????????????????????????????????????????5ms????????????????????????????
 * 			????????????????????????????????????????
 * ????:	???Z
 ***********************************************************/
//??????5ms??????????????
int InsertServoTestData(
	double* actuator_I,
	double* actuator_II,
	double* actuator_III,
    double* actuator_IV,
    double* actuator_V,
    double* actuator_VI,
    double* actuator_VII)
{
	static unsigned int tickOffset=0;
	//static unsigned int freqIndex = 0;
	/*???????????????????????????
	double k = freq * 2*PI;
	double x = tickOffset*fs.tickStep;
	double b = 0;
	double x0 = kx;
	double maxX = 2*PI;		//?????????2pi
	 */
	//????????0.1??????0.1???????????15
	if(fs.freq>0.049 && fs.freq < 15.001 && tickOffset*fs.tickStep *fs.freq <= cycleCount)
			//&& tickOffset*fs.tickStep*fs.freqs[freqIndex] <= fs.scanCnt[freqIndex])
	{

		*actuator_I 	 = fs.enable[0]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_II	 = fs.enable[1]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_III    = fs.enable[2]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_IV 	 = fs.enable[3]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_V	     = fs.enable[4]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_VI     = fs.enable[5]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_VII    = fs.enable[6]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		++tickOffset;
	}
	else
	{
		tickOffset=0;
		//????????????????
		g_DeviceState.workStage &=  ~((unsigned int)DOM_SIMSRVDAT);
		return -1;
	}
	return 0;
}

OS_U16 waitForStart = 0;
OS_U8 MiniLoopSimulation()	//???  // 5ms???????
{
	if((g_DeviceState.workStage & DOM_SIMSRVDAT) != DOM_SIMSRVDAT)
		return -1;
	if(waitForStart > 0)
	{
		waitForStart--;
		return 0;
	}
	double a1=0,a2=0,a3=0,a4=0,a5=0,a6=0,a7=0;
	//short mm1,mm2,mm3,mm4;,&mm1,&mm2,&mm3,&mm4
	InsertServoTestData(&a1, &a2, &a3, &a4, &a5, &a6, &a7);

	ServoCtlOnce_6Rudder(a1,a2,a3,a4,a5,a6);

    // ?????????????????????? - ??7??
    AngleServo_SetAngle(SERVO_PWM7, (float)a7);

	return 0;
}

// OS_U8 SaveSrvInDataPool(STRU_SRV_INFO *data)	//280???
// {
//     SETDATA(pDataPoolSrv, "Sr1Read", data->srv1Read / 10,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr2Read", data->srv2Read / 10,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr3Read", data->srv3Read / 10,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr4Read", data->srv4Read / 10,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr5Read", data->srv5Read / 10,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr6Read", data->srv6Read / 10,	OS_S16);
    
//     SETDATA(pDataPoolSrv, "Sr1A", data->srv1A,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr2A", data->srv2A,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr3A", data->srv3A,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr4A", data->srv4A,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr5A", data->srv5A,	OS_S16);
// 	SETDATA(pDataPoolSrv, "Sr6A", data->srv6A,	OS_S16);
//     return 0;
// }

OS_U8 StartMiniLoop(float freq, float amp, float zero, OS_U8 enable[7])
{
	fs.freq = freq;
	fs.ampAngle = amp;
	fs.zeroAngle = zero;
	memcpy(fs.enable, enable, 7*sizeof(OS_U8));
	//??????
	g_DeviceState.workStage |= DOM_SIMSRVDAT;
	return 0;
}
/***********************************************************
 * ????????: ServoCmdHandler()
 * ????????: ???????????????????????????????????M???????:
 * 			1.???????????0x42: ?????????????????????????????????????????????????????????????????????
 * 			2.??????????0x40: ??????????????????????????????????????????????????
 * 			3.??????????0x44	: ????????????????????????????????????????????????????????????0x11)??????????????
 * 							: ?????????????????????0x22??
 * ????????: <TXII-Y1 ???????????>
 * ????:	???Z
 ***********************************************************/
OS_U32 ServoCmdHandler(STRU_422_MSG_INFO * frame)//03?????????????????????????	//???
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case CMD_SRV_CTRL_REQ://0x42 ???????????
	{
		OS_S16 ctrlDegi[SRV_CHANNEL_COUNT] = {0};
        OS_DOUBLE ctrlDeg[SRV_CHANNEL_COUNT] = {0};
		OS_U8 srvCount = (OS_U8)(frame->u16Len / sizeof(OS_S16));
		if (srvCount > SRV_CHANNEL_COUNT)
			srvCount = SRV_CHANNEL_COUNT;
		if (srvCount < 6)
			srvCount = 6;
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * srvCount);
		for(int i=0;i<srvCount;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x02, srvCount);
	}
		break;
    case CMD_SRV_BOOKMODE:
    {
		OS_S16 ctrlDegi[SRV_CHANNEL_COUNT] = {0};
        OS_DOUBLE ctrlDeg[SRV_CHANNEL_COUNT] = {0};
		OS_U8 srvCount = (OS_U8)(frame->u16Len / sizeof(OS_S16));
		if (srvCount > SRV_CHANNEL_COUNT)
			srvCount = SRV_CHANNEL_COUNT;
		if (srvCount < 6)
			srvCount = 6;
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * srvCount);
		for(int i=0;i<srvCount;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x44, srvCount);
	}
		break;
	case CMD_SRV_MINLOOP_REQ://0x44 ?????????
		{
			STRU_SERVO_MinLoopTest_REQUEST* srvPtr = (STRU_SERVO_MinLoopTest_REQUEST*)frame->au8Data;
			//??????????
			OS_U8 enable[7];
			for(int i=0;i<7;i++)
			{
				enable[i] = ((srvPtr->LevelOneAcuator_Enb >> i) & 1);
			}
			waitForStart = 0;
			cycleCount = srvPtr->count;
			StartMiniLoop(
					srvPtr->Freq * 0.01,
					srvPtr->Amp * 0.1f,
					0,
					enable
					);
			//????????????????????????????????????????????????????????
		}
		break;
	}
	return 0;
}

OS_U8 SrvStatusUpdata()	//???
{
	MiniLoopSimulation();	//?????????
	//???????????????????????????????????????can???
	/*int srvIndex = g_DeviceState.CurrTick % 4;
	//if(g_DeviceState.CurrTick % 100 == 0)
	MsgToSrv(srvIndex + 1, CMD_GET_CTRL , 0);
	//
	//DoSrvProtect();
	//?????????????
	if(SrvEncp > 0)
	{
		SrvEncp--;
		if(SrvEncp <4*10 + 1)
		{
			for(int i=1;i<=4;i++)
			{
				if(SrvEncp / 10 == (i) && SrvEncp % 10 == 0)
				MsgToSrv(i, CMD_SET_ZERO, 0);
			}
		}
	}
	if(g_DeviceState.CurrTick % 10 == 0)
	{
		AutoZero();
	}

	
    */
	return 0;
}

OS_U8 DoSrvProtect()//03??????????0	//??? ??280???????
{
	if(SrvProtect > 0)
	{
		 double deg[SRV_CHANNEL_COUNT] = {0};
		MsgToSrv(deg, 0x02, SRV_CHANNEL_COUNT);
	}
	return 0;
}

OS_U16 AutoZeroCount = 0;

OS_U8 AutoZero()//03??????????0	//??? ??280???????
{
	if(BookingMode == 1)
		return 1;
	if(AutoZeroCount > 0)
	{
        double deg[SRV_CHANNEL_COUNT] = {0};
		MsgToSrv(deg, 0x02, SRV_CHANNEL_COUNT);
		AutoZeroCount--;
	}
	return 0;
}

