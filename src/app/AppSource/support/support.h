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


int BoardInit();   //使能寄存器和取时间地址
extern void uart_mode_init(void);

#endif
