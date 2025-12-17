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
	DOM_INTERACTIVE	= 0x1,		// 交互过程，包含了调试到发射过程中的一切交互逻辑，为射前过程
	DOM_SIMIMUDAT	= 0x1<<1,	// 模拟飞行模式，IMU数据从FLASH读取
	DOM_SIMSRVDAT	= 0x1<<2,	// 伺服小回路模式，伺服数据由地面给出，自动生成并替换输出
	DOM_TRIGGERON	= 0x1<<3,	// 时序开关，是否输出时序动作
	DOM_AUTOMATIC	= 0x1<<4,	// 自动过程，射后自动执行的过程，周期调用
	DOM_HILSMODE	= 0x1<<5,	// 半实物模式
	DOM_NAVON		= 0x1<<6,	// 开启导航
}FUNC_DOMAIN;

#pragma pack(1)
typedef struct
{
	OS_U8 u8HeadA;
	OS_U8 u8HeadB;
	OS_U16 u16Len;
	OS_U8 u8Seq;
	OS_U8 u8MsgID;
	OS_U8 au8Data[2048];
	OS_U8 u8CRCA;
	OS_U8 u8CRCB;
}STRU_422_MSG_INFO;	//422消息传递格式

typedef struct
{
	OS_U8 u8RtIndex;	//总线rt索引号
	STRU_422_MSG_INFO pStand422Data;
}STRU_STANDARD_FRAME;  //定义标准帧格式，比传递422格式多一个描述422通道号的变量

typedef struct
{
	OS_U8 CanIndex;
	OS_U8 NodeIndex;	//舵机节点号，默认0x25
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

#include "../support/support.h"
//#include "../support/time.h"
#include "./os_basic.h"
#include "./os_bufferQueue.h"
//#include "./os_bufferLoop.h"
#include "./os_error.h"
//#include "./os_time.h"
#include "../controller/controller.h"

//#include "AXI_AD7606.h"
//#include "xparameters.h"
//#include "xil_io.h"
//#include "sleep.h"
//#include "xtime_l.h"
//#include "axi_spi.h"
//#include "xgpio.h"
//DEV 设备号
#define CHECK_HEAD							(0x01)	//预编译头，是否对消息头进行比对校验
#define DEV_CODE_FK							(0x01)  //设备号：DEV     ID
#define DEV_CODE_POWER						(0x0A)  //设备号：DEV     ID  时序设备
#define DEV_CODE_SX							(0x0B)  //设备号：DEV     ID  配电设备

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

#define CMD_GET_EPH						(0xE7)	//星历提取
#define CMD_GET_EPH_RSP					(0xE8)	//星历提取回复
#define CMD_SET_EPH						(0xE9)	//星历装订
#define CMD_SET_EPH_RSP					(0xEA)	//星历装订回复
//---------------------总线---------------------------------------------
//MSG-总线-惯组
#define BUS_IMU_INFO_REPORT				(0x93)	//光纤惯组发送至飞控计算机的测试结果
#define BUS_IMU_INFO_SEND				(0x94)	//飞控计算机至光纤惯组发送的请求
#define BUS_IMU_INFO_SEND_EPH		    (0x3A)	//add by Li@20230514，飞控发送给惯组星历协议的请求
#define BUS_IMU_SIMU_GPSIMU				(0xA0)	//半实物仿真惯组与GPS数据结构体直传

#define BUS_NAV_INIT_DATA				(0xE0)	//安装模式设置
#define BUS_NAV_FOCUS					(0xE2)	//对准请求
#define BUS_NAV_FLY_MODE				(0xE4)	//模式设置
#define BUS_NAV_START_NAV				(0x3A)	//转导航
#define BUS_NAV_IGNATION				(0x3B)	//发射
#define BUS_NAV_IMUDATA					(0x3C)	//高精度数据送导航板
//MGS-总线-GPS
#define BUS_GPS_INFO_REPORT				(0xC3)  //GPS测试结果
#define BUS_GPS_EPH_REPORT				(0xC4)  //GPS测试结果
#define BUS_BD_EPH_REPORT				(0xC5)  //GPS测试结果
#define EPH_GPS	(1)
#define EPH_BD	(2)
//----------------------------遥测--------------------------------------------
//MSG-遥测
#define TM_FLIGHT						(0x9A)
#define TM_OTHER						(0x99)
//-----------------------------载荷--------------------------------------------
//MSG-载荷-指令

//MSG-载荷-飞控-总线
#define BUS_FLIGHT_REPORT				(0xB0)
#define BUS_FLIGHT_LUNCH				(0xBA)
#define BUS_FLIGHT_INFO					(0xBF)

extern OS_U8 flightMode;//正式飞行模式
#endif
