/******************************************************************************

Copyright (C), 2022-2023, SpaceTransportation Co., Ltd.

******************************************************************************/
/******************************************************************************
File Name		: os_framework.h
Version			: 2.0
Author			: 成宏璟
Created			: 2022/07/23
******************************************************************************/
#include "os_types.h"
#include <string.h>
#include <stdio.h>
#ifndef _OS_FRAMEWORK_H_
#define _OS_FRAMEWORK_H_

#define WORK			(1)
#define SAFE		    (0)

#define OS_SUCCESS		(0)
#define OS_FAILURE		(-1)

typedef enum
{
	DOM_INTERACTIVE	= 0x1,	// 与地面(有线/发射准备)交互过程 或称 射前过程，包含了调试到发射过程中的一切交互逻辑;1) 进入飞控后无效，允许响地面站“应急开伞”等指令
	DOM_SIMIMUDAT	= 0x1<<1,	// 模拟飞行模式，IMU数据从FLASH读取
	DOM_SIMSRVDAT	= 0x1<<2,	// 伺服小回路模式，伺服数据由地面给出，自动生成并替换输出
	DOM_TRIGGERON	= 0x1<<3,	// 时序开关，是否输出时序动作
	DOM_AUTOMATIC	= 0x1<<4,	// 自动过程，弹动(或起飞)后自动执行的过程，周期调用
	DOM_HILSMODE	= 0x1<<5,	// 半实物模式
	DOM_NAVON		= 0x1<<6,	// 开启导航
}FUNC_DOMAIN;

#pragma pack(1)
typedef struct
{
	OS_U8 u8HeadA;//EB
	OS_U8 u8HeadB;//90
	OS_U16 u16Len;//au8Data长度
	OS_U8 u8Seq;//
	OS_U8 u8MsgID;	// 地面站和机载之间规定好的命令
	OS_U8 au8Data[2048];
	OS_U8 u8CRCA;
	OS_U8 u8CRCB;
}STRU_422_MSG_INFO;	//422消息传递格式

typedef struct
{
	OS_U8 u8HeadA;
	OS_U8 u8HeadB;
	OS_U16 datalen;
	OS_U16 sendid;
	OS_U16 receiveid;
	OS_U8 type;
	OS_U8 u8Seq;
	OS_U8 au8Data[2048];
	OS_U8 u8CRC;
}LinkOUT_422_MSG_INFO;	//LinkOUT422消息传递格式
typedef struct
{
	OS_U8 u8RtIndex;	//总线rt索引号
	STRU_422_MSG_INFO pStand422Data;
}STRU_STANDARD_FRAME;  //定义标准帧格式，比传递422格式多一个描述422通道号的变量

typedef struct
{
	OS_U8 CanIndex;
	OS_U8 NodeIndex;	//舵机节点号，默认0x25	//舵机
	OS_U16 MsgID;
	OS_U8 MsgLen;
	OS_U8 MsgData[8];
}STRU_CAN_MSG;  //定义标准帧格式，比传递422格式多一个描述422通道号的变量

#pragma pack()

#define MathUtils_SignBit(x) (((signed char*)&x)[sizeof(x)-1]>>7|1)
#define LOBYTE(w)       ((BYTE)(w))
#define HIBYTE(w)       ((BYTE)(((UINT)(w) >> 8) & 0xFF))
//**************************************************************************

#define STANDARD_HEADA						(0xEB)
#define STANDARD_HEADB						(0x90)
#define STANDARD_HEADGPSA					(0xEB)
#define STANDARD_HEADGPSB					(0x90)

#define STANDARD_HEADGPSC					(0xFC)	//暂未使用
#define STANDARD_HEADGPSD					(0x1D)	//暂未使用

#define IMU_G0								(9.794265)	//重力加速度常数，按发射场纬度设置

#define _422_FRAME_SYNCCHAR_LEN			(0x2)
#define _422_FRAME_HEADER_LEN			(0x6)
#define _422_FRAME_TM_HEADER_LEN		(0x7)
#define _422_FRAME_FOOTER_LEN			(0x2)
#define _422_PAYLOAD_MAX_LEN			(0x0FFF)
#define  _422_LinkFRAME_HEADER_LEN			(0xA)
#define _422_LinkFRAME_FOOTER_LEN			(0x1)



//#include "../support/qspi_flash.h"
#include "../support/support.h"
//#include "../support/time.h"
#include "./os_basic.h"
#include "./os_bufferQueue.h"
//#include "./os_bufferLoop.h"
#include "./os_error.h"
//#include "./os_time.h"
#include "../controller/controller.h"

//DEV 设备号
#define CHECK_HEAD							(0x01)	 //预编译头，是否对消息头进行比对校验
#define DEV_CODE_FK						(0x01)  //设备号：DEV     ID
#define DEV_CODE_POWER						(0x0A)  //设备号：DEV     ID  时序设备
#define DEV_CODE_SX						(0x0B)  //设备号：DEV     ID  配电设备

//MSG-指令-发射
#define CMD_ENGINE_START					(0xF6)	//发动机启动
#define CMD_ENGINE_STOP                   (0xF7)	//发动机停机
#define CMD_FORE_LAUNCH_REQ				(0xF8)	//发射预起控指令	//预发射（未使用）
#define CMD_FORE_LAUNCH_RSP				(0xF9)	//发射预起控指令结果
#define CMD_LAUNCH_REQ						(0xFA)	//全部解锁，
#define CMD_LUANCH_FORCE					(0xFB)	//强制发射指令		// 0xFB 首页 - 起飞

//MSG-指令-智能控制器状态
#define CMD_MODULE_SET_REQ			(0x01)	//智能控制器模式设置
#define CMD_STATUS_REPORT				(0x02)	//智能控制器状态上报

#define CMD_BJTIME_SET					(0x0A)	//北京时设置

#define CMD_DATA_REQ					(0x03)	//诸元列表请求
#define CMD_DATA_RSP					(0x04)	//诸元列表上传
#define CMD_DATA_SET					(0x21)	//诸元列表设置

#define CMD_URGENT_LAND				(0x22)	//紧急伞降
#define CMD_URGENT_RETURN				(0x23)	//紧急返航
#define CMD_INSTANT_RECOVER			(0x24)	//紧急回收

//MSG_指令_ECU
#define	CMD_START_STOP_ENGINE			(0xC1)	// 0xC1 伺服时序 - 启动停止发动机 - 根据带的参数不同区分启停：0x11启动 0x22停止
#define	CMD_GET_RUNNING_INFO			(0xC2)	// 0xC2 伺服时序 - 左右滑块

#define	CMD_GET_RUNNING_PARAM			(0xC3)	// 0xC3 伺服时序 - 获取运行参数
#define	CMD_GET_START_PARAM			(0xC4)	// 0xC4 伺服时序 - 获取启动参数
#define	CMD_ECU_RPM_SETTING			(0xC5)	//转速设置	// 0xC5 伺服时序 - 油门设定

//MSG-指令-配电
#define CMD_POWER_REQ					(0x05)	//单机配电请求		  // 0x05 伺服时序 - 单通道时序测试 - 发送/开/关
#define CMD_SEQ_POWER_REQ				(0xEF)	//时序配电请求

//MSG-指令-惯组
#define CMD_NAV_INIT					(0xE4)	//发射点诸元送导航板
#define CMD_HOR_CALC_REQ				(0xE0)	//水平计算（对准）请求	 // 0xE0 伺服时序 - 对准
#define CMD_TO_NAV_REQ					(0xE2)	//转导航请求			// 0xE2 伺服时序 - 转导航
#define CMD_TO_AFTER_LUANCH				(0xE3)	//转导航请求			// 0xE3 伺服时序 - 转射后
#define CMD_POLAR_TEST_REQ				(0x3A)	//极性测试

//MSG 指令-星历
#define CMD_GET_EPH						(0xE7)	//星历提取
#define CMD_GET_EPH_RSP					(0xE8)	//星历提取回复
#define CMD_SET_EPH						(0xE9)	//星历装订
#define CMD_SET_EPH_RSP					(0xEA)	//星历装订回复

//MSG-指令-SD
#define CMD_SD_READ_FILE				(0x50)	//读取SD卡文件
#define CMD_SD_INIT					(0x51)	//读取SD卡文件

//MSG-指令-伺服
#define CMD_SRV_ZERO_ENCAP_REQ		(0x40)	//伺服零位装订请求
#define CMD_SRV_CTRL_REQ				(0x42)	//伺服控制请求			// 0x42 伺服时序 - 设定
#define CMD_SRV_MINLOOP_REQ			(0x44)	//伺服小回路测量请求	// 0x44 伺服时序 - 小回路测试 - 开始
#define CMD_SRV_BOOKMODE				(0x48)	//伺服装订模式使能		// 0x48 伺服时序 - 零位装订
#define CMD_SRV_GET_ID					(0x4A)
#define CMD_SRV_SAVE					(0x4C)
#define CMD_SRV_SET_ID					(0x4E)

//MSG-指令-电池
#define CMD_BATT_CMD					(0x46)	//电池指令

//MSG-指令-FLASH
#define CMD_FLASH_CTRL_REQ				(0x60)	//FLASH控制请求
#define CMD_FLASH_CTRL_RSP				(0x61)	//FLASH控制结果
#define CMD_FLASH_ENCAP_REQ				(0x62)	//FLASH数据上行请求
#define CMD_FLASH_ENCAP_RSP				(0x63)	//FLASH数据上行结果

#define CMD_FLASH_QUERY_REQ				(0x64)	//FLASH查询
#define CMD_FLASH_QUERY_RSP				(0x65)	//FLASH查询结果
#define CMD_FLASH_CHECK_REQ				(0x66)	//FLASH校验请求
#define CMD_FLASH_CHECK_RSP				(0x67)	//FLASH校验结果
#define CMD_FLASH_LOAD_REQ				(0x68)	//诸元数据加载
#define CMD_FLASH_LOAD_RSP				(0x69)	//诸元数据加载反馈
#define CMD_FLASH_CLEAR_REQ				(0x6A)	//诸元数据清除
#define CMD_FLASH_CLEAR_RSP				(0x6B)	//清除回复

#define CMD_MSN_UPDATE                  (0x13)	// 0x13 首页 - 加载任务
#define CMD_MSN_NEWPT                   (0x21)	// 0x21 可能是 任务编辑 - 飞行航点
//---------------------总线---------------------------------------------
#define BUS_SLAVER_REPORT				(0x9F)
#define BUS_SLAVER_CMD					(0x9E)

//MSG-总线-惯组
#define BUS_IMU_INFO_REPORT				(0x93)	//光纤惯组发送至飞控计算机的测试结果
#define BUS_IMU_INFO_SEND				(0x94)	//飞控计算机至光纤惯组发送的请求
#define BUS_IMU_INFO_SEND_EPH		    (0x3A)	//add by Li@20230514，飞控发送给惯组星历协议的请求
#define BUS_IMU_SIMU_GPSIMU			(0xA0)	//半实物仿真惯组与GPS数据结构体直传

#define BUS_NAV_INIT_DATA				(0xE0)	//安装模式设置
#define BUS_NAV_FOCUS					(0xE2)	//对准请求
#define BUS_NAV_START_NAV				(0x3A)	//转导航
#define BUS_NAV_IGNATION				(0x3B)	//发射
#define BUS_NAV_IMUDATA				(0x3C)	//高精度数据送导航板

//----------------------------遥测--------------------------------------------
//MSG-遥测
#define TM_FLIGHT						(0x9A)
#define TM_OTHER						(0x99)
//-----------------------------载荷--------------------------------------------
//MSG-载荷-指令

//MSG-载荷-飞控-总线
#define BUS_FLIGHT_REPORT				(0xB0)
#define BUS_FLIGHT_LUNCH				(0xBA)
#define BUS_FLIGHT_INFO				(0xBF)

//MSG-SCOUT
#define SCOUT_AFRAME            (0XAA)
#define SCOUT_BFRAME            (0XBB)
#define TEMPLATE_TXT            (0XB1)
#define IMAGE_INFO              (0XB2)
#define IMAGE_REBACK            (0XB3)
#define SOFT_UPDATE             (0XD1)
#define SOFT_REBACKE            (0XD2)
#define CMD_USER_SETTARGET    (0xD3)
#define CMD_SET_IMAGEMODE		(0xD4)
//MSG定义完-----------------------------------------------------------------------
#pragma pack(1)
typedef struct
{
	/***************
	 * 入遥测部分
	 * *************/

	 /** workStage	工作状态 */
	// DOM_INTERACTIVE	= 0x1,		// 交互过程，包含了调试到发射过程中的一切交互逻辑，为射前过程
	// DOM_SIMIMUDAT	= 0x1<<1,	// 模拟飞行模式，IMU数据从FLASH读取
	// DOM_SIMSRVDAT	= 0x1<<2,	// 伺服小回路模式，伺服数据由地面给出，自动生成并替换输出
	// DOM_TRIGGERON	= 0x1<<3,	// 时序开关，是否输出时序动作
	// DOM_AUTOMATIC	= 0x1<<4,	// 自动过程，射后自动执行的过程，周期调用
	// DOM_HILSMODE		= 0x1<<5,	// 半实物模式
	// DOM_NAVON		= 0x1<<6,	// 开启导航
	FUNC_DOMAIN workStage;	//工作阶段
	
	/** 各单机通信状态，初始设置为200tick，每个tick调用-1，每次收到数据恢复200.保证在通讯中断1秒内能够反馈到遥测 */
	OS_U8 srvCountDown;	// 设备通信状态
	OS_U8 battCountDown;	// 设备通信状态
	OS_U8 navCountDown;	// 设备通信状态
    OS_U8 powerCountDown;	// 设备通信状态
	OS_U8 ecuCountDown;	// 设备通信状态
	OS_U8 pwrStatePos;
	OS_U8 hilCountDown;	// 设备通信状态
    OS_U8 imuCountDown;	// 设备通信状态
    OS_U8 scoutCountDown;	// 设备通信状态
    OS_U8 fuseCountDown;	// 设备通信状态（是不是没用上？）
    
	//时序配电器的配电状态
	OS_U8 pwrStateB1:1;
	OS_U8 pwrStateB2:1;
	OS_U8 pwrStateB3:1;
	OS_U8 pwrStateB4:1;
	OS_U8 pwrStateB6:1;
	OS_U8 pwrStateP15:1;
	OS_U8 pwrStateN15:1;
	OS_U8 pwrState5V:1;

	OS_U8 luanchState;//未用到	
	OS_U8 detachState;//未用到
	OS_FLOAT temperature;	//飞控温度
	OS_U8 luanchStart;		//发射准备完成标识，用于解锁
	
	OS_DOUBLE flightStartTime;	// 起飞时间，即飞控初始化时，去掉起飞时刻得到飞控时间，单位 s
	OS_U64 CurrTick;	// 程序初始化时从0增加，每5ms+1；起飞时，DoIgnition函数中，赋值0，大状态机中，每5ms+1
	OS_DOUBLE currTime;// 程序初始化从0增加+0.005s；起飞时置0，重新从0增加+0.005s	// 单位 s
	OS_U64 BJTimeSecond;//组合导航输出北京时间，年月日，时分秒
	OS_U16 BJTimeMS;	//授时毫秒
}DeviceState;

typedef struct
{
	OS_DOUBLE FzOnStamp_s;	// fuse power-on time	// 单位 s
	// OS_DOUBLE sysTime_s;	// 系统时间，程序启动时0	// 单位 s
	OS_U8 msgFromGCS;	// message from Ground Control Station // 用于启动SD卡文件写入。初始化时设置为0x00，收到数据链传来数据时设置为0x01
}DeviceStatus;
#pragma pack()
extern DeviceState g_DeviceState;
extern DeviceStatus g_DeviceStatus;

#endif
