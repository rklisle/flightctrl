/*
 * interface.c
 *
 *
 *
 *
 *
 *
 *  Created on: 2020年9月24日
 *      Author: liyuan
 */
#include "interface_gpio.h"
#include    "stm32h7xx_hal.h"
#define GPIO_ONOFF_ENABLE   __HAL_RCC_GPIOK_CLK_ENABLE
#define GPIO_ONOFF_1_PIN    GPIO_PIN_0
#define GPIO_ONOFF_2_PIN    GPIO_PIN_1
#define GPIO_ONOFF_Port     GPIOK

// int InitGPIO()
// {
//   __HAL_RCC_GPIOB_CLK_ENABLE(); //MML舵机16 保证舵机用到的pin开启时钟
//   __HAL_RCC_GPIOC_CLK_ENABLE(); //MML舵机16 保证舵机用到的pin开启时钟
//   __HAL_RCC_GPIOD_CLK_ENABLE(); //MML舵机16 保证舵机用到的pin开启时钟
//     /*
//     GPIO_InitTypeDef GPIO_InitStruct1 = {0};
//     GPIO_InitTypeDef GPIO_InitStruct2 = {0};
//     GPIO_ONOFF_ENABLE();
    
    
//     GPIO_InitStruct1.Pin =  GPIO_ONOFF_1_PIN;
//     GPIO_InitStruct1.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct1.Pull = GPIO_NOPULL;
//     GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIO_ONOFF_Port, &GPIO_InitStruct1);
    
//     GPIO_InitStruct2.Pin =  GPIO_ONOFF_2_PIN;
//     GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct2.Pull = GPIO_NOPULL;
//     GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIO_ONOFF_Port, &GPIO_InitStruct2);
//     */
// }

int ADC_GetVoltage()
{
	
	return 0;
}

int GetGpioOnOff(int channel)
{
    GPIO_PinState pinState1 = HAL_GPIO_ReadPin(GPIO_ONOFF_Port, GPIO_ONOFF_1_PIN);
    GPIO_PinState pinState2 = HAL_GPIO_ReadPin(GPIO_ONOFF_Port, GPIO_ONOFF_2_PIN);
    if(channel == 0)
    {
        return pinState1 == GPIO_PIN_SET?1:0;
    }
    if(channel == 1)
    {
        return pinState2 == GPIO_PIN_SET?1:0;
    }
	return 0;
}


