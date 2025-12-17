/*
 * interface_gpio.h
 *
 *  Created on: 2024年11月24日
 *      Author: cheng hongjing
 */


#include "../support/os_types.h"
#ifndef _INTERFACE_GPIO_H_
#define _INTERFACE_GPIO_H_

typedef struct
{
	OS_U8 channel;
	char name[8];
	float value;
}VA_VALUE;

extern VA_VALUE vas[16];


int InitGPIO();
//AD采集函数
int ADC_GetVoltage();
//拖插检测
int GetGpioOnOff(int channel);

#endif /* _INTERFACE_GPIO_H_ */
