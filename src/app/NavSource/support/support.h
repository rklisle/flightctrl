/******************************************************************************

Copyright (C), 2015-2018, OneSpace Co., Ltd.

File Name		: support.h
Describe		: the support is used for board driver and give user the funct-
				  on
Version			: 1.0
Author			: zhaoyeni
Created			: 2017/08/28

******************************************************************************/

#ifndef _SUPPORT_H_
#define _SUPPORT_H_

#define REG_SELFCHK_RESULT_ADDR		(0)
//422端口定义
#define ROCKET_GROUND_CONN_422_PORT	(8)
#define SERVO_CONN_422_PORT			(5)
//设备号定义
#define DEV_GROUND_ID				(2)
#define DEV_IMU_ID					(5)
#define DEV_RECORD_ID				(6)
#define DEV_SERVO_ID				(3)

#define READ_REG_U8(u8Addr)		(*(volatile unsigned char*)u8Addr)
#define READ_REG_U16(u16Addr)	(*(volatile unsigned short*)u16Addr)
#define READ_REG_U32(u32Addr)	(*(volatile unsigned int*)u32Addr)

#define WRITE_REG_U8(u8Addr, u8Value)		(*(volatile unsigned char*)u8Addr = u8Value)
#define WRITE_REG_U16(u16Addr, u16Value)	(*(volatile unsigned short*)u16Addr = u16Value)
#define WRITE_REG_U32(u32Addr, u32Value)	(*(volatile unsigned int*)u32Addr = u32Value)

#define ReadReg(BaseAddress, RegOffset)             \
		Xil_In32((BaseAddress) + (RegOffset))    //读硬件寄存器

#define WriteReg(BaseAddress, RegOffset, Data)          \
		Xil_Out32((BaseAddress) + (RegOffset), (Data))  //写硬件寄存器

int BoardInit();   //使能寄存器和取时间地址
extern void uart_mode_init(void);

#endif
