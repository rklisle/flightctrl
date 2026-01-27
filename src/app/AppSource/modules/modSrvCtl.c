/*
 * modSrvCtl.c
 *
 *  Created on: 2021年10月16日
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
#include "../flight/os_flight_io.h"
#include "../support/os_bufferLoop.h"

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
    
//OS_U8 InitSrv()	//MML舵机: 没人调了，这是原先串口的Init
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

OS_U16 ChkSrvFrame(OS_MEM* pmData)	//MML舵机 需要更新CAN帧检查逻辑 这个没人调了。是原先串口的。
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
}

// 配置参数
// #define PSC_VALUE       23      // 分频24
// #define ARR_VALUE       30029   // 周期30030计数
#define CNT_TIME_US     0.1f    // 每个计数0.1μs

#define PULSE_MIN_US    900     // 最小脉宽
#define PULSE_MAX_US    2100    // 最大脉宽
#define PULSE_CENTER_US 1500    // 中心脉宽（0°）
#define US_PER_DEG      10.0f   // 每度对应的μs数

#define CCR_MIN         ((uint32_t)(PULSE_MIN_US / CNT_TIME_US))      // 9000
#define CCR_MAX         ((uint32_t)(PULSE_MAX_US / CNT_TIME_US))      // 21000

uint32_t angle_to_ccr(double angle_deg) {
    // 角度转脉宽：1500μs + angle×10μs
    double pulse_us = PULSE_CENTER_US + (angle_deg * US_PER_DEG);

    // 脉宽转CCR：pulse_us / 0.1
    uint32_t ccr = (uint32_t)(pulse_us / CNT_TIME_US);
    
    // 确保在有效范围内
    if (ccr < CCR_MIN) ccr = CCR_MIN;
    if (ccr > CCR_MAX) ccr = CCR_MAX;
    
    return ccr;
}

/***********************************************************
 * 函数名称: MsgToSrv()
 * 函数功能: 伺服额外要求指令计数及发送帧计数，因此剥离一层控制数据，再发送
 * 作者:	未知
 ***********************************************************/
OS_U8 MsgToSrv(OS_DOUBLE ctrlDeg[6], OS_U8 ctrlMode/*control = 0x02, 0x44=setZero*/)//04 存入数据池 控制舵机
{
	//MML舵机
    SETDATA(pDataPoolSrv, "Sr1Cmd", ctrlDeg[0] * 100, OS_S16); 
	SETDATA(pDataPoolSrv, "Sr2Cmd", ctrlDeg[1] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr3Cmd", ctrlDeg[2] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr4Cmd", ctrlDeg[3] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr5Cmd", ctrlDeg[4] * 100, OS_S16);
    SETDATA(pDataPoolSrv, "Sr6Cmd", ctrlDeg[5] * 100, OS_S16);

    SETDATA(pDataPoolSrv, "Sr5Angle", ctrlDeg[4], OS_U16);
    SETDATA(pDataPoolSrv, "Sr6Angle", ctrlDeg[5], OS_S16);

	if (ctrlMode == 0x02) 
	{
		// 正常控制模式
		// 控制1-4号CAN舵机
		Servo_SetAngle_CAN(SERVO_NODE_1, ctrlDeg[0]);
		Servo_SetAngle_CAN(SERVO_NODE_2, ctrlDeg[1]);
		Servo_SetAngle_CAN(SERVO_NODE_3, ctrlDeg[2]);
		Servo_SetAngle_CAN(SERVO_NODE_4, ctrlDeg[3]);
		
		// 控制5-6号PWM舵机
        AngleServo_SetAngle(SERVO_PWM1, (float)ctrlDeg[4]);
        AngleServo_SetAngle(SERVO_PWM6, (float)ctrlDeg[5]);
    }
	else if (ctrlMode == 0x44) 
	{
		// 设置零点模式
        // 对于CAN舵机，发送设置中点命令
        Servo_SetMidpoint_CAN(SERVO_NODE_1);
        Servo_SetMidpoint_CAN(SERVO_NODE_2);
        Servo_SetMidpoint_CAN(SERVO_NODE_3);
        Servo_SetMidpoint_CAN(SERVO_NODE_4);
        
        // PWM舵机零点设置
        // PWM舵机通常需要机械调零，这里仅让舵机回归0°位置
        AngleServo_SetAngle(SERVO_PWM1, 0.0f);
        AngleServo_SetAngle(SERVO_PWM6, 0.0f);
    }
    return 0;
}

/**
 * @brief 设置CAN舵机中点（零点）
 */
OS_U8 Servo_SetMidpoint_CAN(ServoNodeID node)
{
    uint8_t data[8] = {
        0x22,  // 字节1：固定
        0x09,  // 字节2：设置中点索引
        0x30,  // 字节3：子索引
        0x00,  // 字节4：固定
        0x00, 0x00, 0x00, 0x00  // 保留
    };
    
    uint32_t can_id = CAN_CMD_ID_BASE | node;
    
    return Servo_SendCANFrame(can_id, data, 8);
}

/**
 * @brief 设置CAN舵机扭力为0（自由转动）
 */
OS_U8 Servo_SetTorqueZero_CAN(ServoNodeID node)
{
    uint8_t data[8] = {
        0x22,  // 字节1：固定
        0x10,  // 字节2：扭力为0命令索引
        0x30,  // 字节3：子索引
        0x00,  // 字节4：固定
        0x00, 0x00, 0x00, 0x00  // 保留
    };
    
    uint32_t can_id = CAN_CMD_ID_BASE | node;
    
    return Servo_SendCANFrame(can_id, data, 8);
}

/***********************************************************
 * 函数名称: ServoCtlOnce_6Rudder()
 * 函数功能: 单次计算及伺服控制，由飞控代码调用，传入为6个舵的偏转角度
 * 作者:	未知
 ***********************************************************/
OS_U8 ServoCtlOnce_6Rudder( double actDeg_1,
                            double actDeg_2,
                            double actDeg_3,
                            double actDeg_4,
                            double actDeg_5,
                            double actDeg_6)	//03 控制舵机
{
    double deg[6];
    deg[0] = actDeg_1; 
    deg[1] = actDeg_2; 
    deg[2] = actDeg_3; 
    deg[3] = actDeg_4; 
    deg[4] = actDeg_5; 
    deg[5] = actDeg_6; 
    
    MsgToSrv(deg, 0x02);  // 0x02为正常控制模式
    
    return 0;
}

// OS_U8 SaveSrvInDataPool(STRU_SRV_INFO *data);	//函数声明

// /***********************************************************
//  * 函数名称: ServoRtHandler()
//  * 函数功能: 伺服总线处理函数，伺服的设定是不会定时给智能控制器发送内容的，仅在收到智能控制器的指令后回复。测发控阶段，均需下传地面
//  * 			本型号智能控制器接收伺服发出的指令:
//  * 			1.伺服自检回复0x11		: 地面已取消自检命令，因此也收不到这个回复了，协议中规定的格式与控制回复相同。
//  * 			2.伺服控制回复0x22		: 存数据池，伺服的绝大多数内容来自控制回复，包括地面控制指令、小回路指令、发射后的控制指令。
//  * 			3.伺服调零回复0x23		: 存数据池
//  * 参考资料: <TXII-Y1 422箭上通信协议>
//  * 作者:	成宏璟
//  ***********************************************************/
// OS_U32 SrvRtHandler(STRU_422_MSG_INFO * srvMsg)	
// {
// 	//MML舵机，这是280用的串口的回调函数，014项目不用
// 	//首先判断是什么类型的指令
// 	//对数据区前四字节进行判断
// 	RECV_CMD_ID recvCmdId = srvMsg->u8MsgID;
// 	//根据反馈指令做处理
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
 * @brief CAN舵机接收回调函数
 * @param id CAN ID
 * @param ext_id 是否扩展帧
 * @param data 数据
 * @param len 数据长度
 */
OS_U8 CanRtServoHandler(OS_U32 id, OS_BOOL ext_id, const OS_U8* data, OS_U8 len)
{
	//MML舵机，这是新的CAN舵机的回调函数
    // 只处理扩展帧
    if (!ext_id) {
        return -1;
    }
    
    // 检查是否是舵机响应帧（0x00000580 + 节点号）
    if ((id & 0xFFFFFE00) == 0x00000580UL) {
        // 提取节点号
        ServoNodeID node = id & 0xFF;
        
        // 处理不同类型的响应
        switch (data[1]) {
            case 0x02:  // 位置读取响应
                if (data[0] == 0x4B) {
                    // 处理位置数据
                    double angle = PositionToAngle_CAN(data[5], data[4]);
                    // 保存到数据池
                    switch (node) {
                        case SERVO_NODE_1:
                            SETDATA(pDataPoolSrv, "Sr1Read", (OS_S16)(angle * 100), OS_S16);//单位0.01°
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
                        default:
                            break;
                    }
                }
                break;
                
            case 0x05:  // 状态读取响应
                if (data[0] == 0x43) {
                    // 处理电流和温度数据
                    // 电流：X1X2表示工作电流，单位10mA
                    OS_U16 current_ma = (data[5] << 8) | data[4];
                    double current_a = current_ma / 100.0;
                    // 保存到数据池
                    switch (node) {
                        case SERVO_NODE_1:
                            SETDATA(pDataPoolSrv, "Sr1A", (OS_S16)(current_a * 1000), OS_S16);//单位是A
                            break;
                        case SERVO_NODE_2:
                            SETDATA(pDataPoolSrv, "Sr2A", (OS_S16)(current_a * 1000), OS_S16);
                            break;
                        case SERVO_NODE_3:
                            SETDATA(pDataPoolSrv, "Sr3A", (OS_S16)(current_a * 1000), OS_S16);
                            break;
                        case SERVO_NODE_4:
                            SETDATA(pDataPoolSrv, "Sr4A", (OS_S16)(current_a * 1000), OS_S16);
                            break;
                        default:
                            break;
                    }
                    
                    // 温度：X3表示工作温度，补码表示
                    // int8_t temp_c = (int8_t)data[6];
                    
                    // 保存到数据池
                    // 这里可以根据需要保存电流和温度
                }
                break;
                
            // 可以添加其他响应类型的处理
            default:
                break;
        }
    }
	g_DeviceState.srvCountDown = 200;
	return 0;
}

/***********************************************************
 * 函数名称: InsertServoTestData()
 * 函数功能: 伺服小回路测试专用函数。小回路测试前，地面会通过小回路测试指令上传扫频相关参数，见FREQ_SCAN结构
 * 			并修改状态机状态为伺服小回路模式。在该模式下，飞控代码会每5ms（由状态机触发的定时器调用飞控代码）
 * 			调用本函数，计算出每次的线控舵偏长度，通过指针返回。
 * 作者:	成宏璟
 ***********************************************************/
//该函数5ms调用一次，实现扫频
int InsertServoTestData(
	double* actuator_I,
	double* actuator_II,
	double* actuator_III,
    double* actuator_IV,
    double* actuator_V,
    double* actuator_VI)
{
	static unsigned int tickOffset=0;
	//static unsigned int freqIndex = 0;
	/*算法公式计算过程，屏蔽公式推导过程
	double k = freq * 2*PI;
	double x = tickOffset*fs.tickStep;
	double b = 0;
	double x0 = kx;
	double maxX = 2*PI;		//正弦一轮是2pi
	 */
	//扫频频率最低0.1，低于0.1不处理。最大15
	if(fs.freq>0.049 && fs.freq < 15.001 && tickOffset*fs.tickStep *fs.freq <= cycleCount)
			//&& tickOffset*fs.tickStep*fs.freqs[freqIndex] <= fs.scanCnt[freqIndex])
	{

		*actuator_I 	 = fs.enable[0]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_II	 = fs.enable[1]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_III    = fs.enable[2]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
        *actuator_IV 	 = fs.enable[3]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_V	     = fs.enable[4]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		*actuator_VI     = fs.enable[5]*(fs.zeroAngle+ fs.ampAngle*sin(2*PI*fs.freq*(tickOffset*fs.tickStep)));
		++tickOffset;
	}
	else
	{
		tickOffset=0;
		//测试结束，关闭状态机
		g_DeviceState.workStage &=  ~((unsigned int)DOM_SIMSRVDAT);
		return -1;
	}
	return 0;
}

OS_U16 waitForStart = 0;
OS_U8 MiniLoopSimulation()	//MML舵机
{
	if((g_DeviceState.workStage & DOM_SIMSRVDAT) != DOM_SIMSRVDAT)
		return -1;
	if(waitForStart > 0)
	{
		waitForStart--;
		return 0;
	}
	double a1=0,a2=0,a3=0,a4=0,a5=0,a6=0;
	//short mm1,mm2,mm3,mm4;,&mm1,&mm2,&mm3,&mm4
	InsertServoTestData(&a1, &a2, &a3, &a4, &a5, &a6);

	ServoCtlOnce_6Rudder(a1,a2,a3,a4,a5,a6);
	return 0;
}

// OS_U8 SaveSrvInDataPool(STRU_SRV_INFO *data)	//280回调
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

OS_U8 StartMiniLoop(float freq, float amp, float zero, OS_U8 enable[6])
{
	fs.freq = freq;
	fs.ampAngle = amp;
	fs.zeroAngle = zero;
	memcpy(fs.enable, enable, 6*sizeof(OS_U8));
	//开启扫频
	g_DeviceState.workStage |= DOM_SIMSRVDAT;
	return 0;
}
/***********************************************************
 * 函数名称: ServoCmdHandler()
 * 函数功能: 伺服指令处理函数，本型号智能控制器接收地面发出的指令:
 * 			1.伺服测量请求0x42: 对伺服进行控制，指定伺服移动固定位置，直接转发至伺服驱动器。回复地面不由本函数控制
 * 			2.伺服零位装订0x40: 对伺服进行调零，接转发至伺服驱动器。回复地面不由本函数控制
 * 			3.小回路测试0x44	: 小回路测试有专门的状态机状态。接收指令后开始小回路测试，并立即返回（执行中0x11)，当测试完成后再
 * 							: 由其它函数返回（执行完成0x22）
 * 参考资料: <TXII-Y1 箭地通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ServoCmdHandler(STRU_422_MSG_INFO * frame)//03根据数据链过来的指令，具体干活	//MML舵机
{
	OS_U8 msgID = frame->u8MsgID;
	switch(msgID)
	{
	case CMD_SRV_CTRL_REQ://0x42 伺服控制请求
	{
		OS_S16 ctrlDegi[6];
        OS_DOUBLE ctrlDeg[6];
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * 6);
		for(int i=0;i<6;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x02);
	}
		break;
    case CMD_SRV_BOOKMODE:
    {
		OS_S16 ctrlDegi[6];
        OS_DOUBLE ctrlDeg[6];
		memcpy(ctrlDegi, frame->au8Data, sizeof(OS_S16) * 6);
		for(int i=0;i<6;i++)
		{
			ctrlDeg[i] = ctrlDegi[i] * 0.01;
		}
        MsgToSrv(ctrlDeg, 0x44);  // 零点设置模式
	}
		break;
	case CMD_SRV_MINLOOP_REQ://0x44 伺服小回路
		{
			STRU_SERVO_MinLoopTest_REQUEST* srvPtr = (STRU_SERVO_MinLoopTest_REQUEST*)frame->au8Data;
			//设置扫频参数
			OS_U8 enable[6];
			for(int i=0;i<6;i++)
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
			//开启伺服小回路模式后，由飞控软件计算并向伺服发送数据，此处不再转发伺服
		}
		break;
	}
	return 0;
}

OS_U8 SrvStatusUpdata()	//MML舵机
{
	MiniLoopSimulation();	//伺服小回路
	//伺服位置查询，每轮循环只查询一个伺服，以尽量避免can冲突
	/*int srvIndex = g_DeviceState.CurrTick % 4;
	//if(g_DeviceState.CurrTick % 100 == 0)
	MsgToSrv(srvIndex + 1, CMD_GET_CTRL , 0);
	//
	//DoSrvProtect();
	//伺服零位装订检测
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

OS_U8 DoSrvProtect()//03舵机角度控制为0	//MML舵机 （280目前不用）
{
	if(SrvProtect > 0)
	{
		 double deg[6] = {0};
		MsgToSrv(deg, 0x02);
	}
	return 0;
}

OS_U16 AutoZeroCount = 0;

OS_U8 AutoZero()//03舵机角度控制为0	//MML舵机 （280目前不用）
{
	if(BookingMode == 1)
		return 1;
	if(AutoZeroCount > 0)
	{
        double deg[6] = {0};
		MsgToSrv(deg, 0x02);
		AutoZeroCount--;
	}
	return 0;
}

