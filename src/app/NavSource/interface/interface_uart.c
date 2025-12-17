/*
 * interface_uart.c
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */
#include "interface_uart.h"
#include "../core/BusInteract.h"

#include "Driver_USART.h"
#include "stm32h7xx_hal.h"

#include "tx_api.h"
#include "ringbuffer.h"
#include "../../uart_agent.h"


extern TX_BYTE_POOL byte_pool_0;
OS_S32 fd[9];

/*-------------------------校验模式定义----------------------------*/
typedef enum
{
	none 			= 0, 	//不产生校验
	even 			= 1,	//偶校验
	odd 			= 2,	//奇校验
} UART_VerifyMode;

/*-------------------------停止位个数----------------------------*/
typedef enum
{
	onestop 		= 1,	//1位停止位
	twostop 		= 2,	//2位停止位
} UART_StopBit;

/***************************************
*	函数名称:	UART_Setting()
*	功能：根据波特率，奇偶校验设置串口参数
*	input:
*	channel：串口号，值域0-11
*	Baud：波特率
*	Par：校验位，’n’无校验，’e’偶校验，’o’奇校验
*	返回值：0正常
*****************************************/
int UART_Setting(int channel, unsigned int Baud, char check)
{
	unsigned int mode;
	if( check == 'n' || check == 'N')			//无校验
	{
		mode = ARM_USART_PARITY_NONE;
	}
	else if(check == 'e'|| check == 'E')
	{
		mode = ARM_USART_PARITY_EVEN;
	}
	else
	{
		mode = ARM_USART_PARITY_ODD;
	}

 
	switch(channel)
	{
	case 0:
			fd[0] = fcs_uart_init(&byte_pool_0, &Driver_USART1, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 1:
			fd[1] = fcs_uart_init(&byte_pool_0, &Driver_USART2, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 2:
			fd[2] = fcs_uart_init(&byte_pool_0, &Driver_USART3, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
    /*
	case 3:
 			fd[3] = fcs_uart_init(&byte_pool_0, &Driver_USART4, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 4:
			fd[4] = fcs_uart_init(&byte_pool_0, &Driver_USART5, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 5:
			fd[5] = fcs_uart_init(&byte_pool_0, &Driver_USART6, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 6:
			fd[6] = fcs_uart_init(&byte_pool_0, &Driver_USART7, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 7:
			fd[7] = fcs_uart_init(&byte_pool_0, &Driver_USART8, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
    */
	}
    
	return 0;
}

/************************************************************
*	功能：从串口发送一个数据包
*	ch：串口号，值域0-5
*	TxBuff：发送的数据包的数组首地址
*	Length：发送的长度
*	返回值：	0
***************************************************************/
int UART_PutBuff(int channel, unsigned char * TxBuff, unsigned short Length)
{
		fcs_uart_send(fd[channel],TxBuff,Length);
		return 0;
}

/****************************************************************
*	功能：查询并接收当前fifo所有数据
*	ch：串口号，值域0-5
*	RxBuff：用于存储接收数据帧的数组首地址
*	MaxLength：接收数组的长度，即允许接收的最大长度
*	返回值：0
 *******************************************************************/
int uart_recvdata = 0;
int UART_GetFrame(int channel, unsigned char * RxBuff, unsigned short MaxLength)
{
    int recv_len = fcs_uart_recv(fd[channel], RxBuff, MaxLength);
    if (recv_len > 0)
    {
        if(channel == 1)
        {
            uart_recvdata = 1;
            unsigned char data[500] = {0};
            memcpy( data, RxBuff, recv_len);
        }
        if(channel == 2)
        {
            uart_recvdata = 1;
            unsigned char data[500] = {0};
            memcpy( data, RxBuff, recv_len);
        }
        return recv_len;
    }
    else
        return 0;
}

