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

//-------------------Drive Start---------------------
/*
#define     RS422_BLOCK_SIZE    512

struct rs422_driver_inst
{
    ARM_DRIVER_USART*   pdrv;
    uint32_t            baudrate;
    uint32_t            parity;
    uint32_t            stopbits;
    uint16_t            rx_buf_size;
    uint16_t            tx_buf_size;

    TX_EVENT_FLAGS_GROUP events;   

    ring_buffer_t   rx_buf;
    ring_buffer_t   tx_buf;
};
*/
/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
//#define     APP_STACK_SIZE         1024

//struct rs422_driver_inst rs422_inst;
//#define RX_BUFFER_SIZE 138
extern TX_BYTE_POOL byte_pool_0;
OS_S32 fd[9];
//OS_U8 rxBuf[8][RX_BUFFER_SIZE];
//OS_U8 rxBufSize[8] = {0};
/* 
static void prv_rs422_event_handle(ARM_DRIVER_USART* pdrv, uint32_t idx, uint32_t event)
{
   if ((event & ARM_USART_EVENT_RECEIVE_COMPLETE) || (event & ARM_USART_EVENT_RX_TIMEOUT) )
	 {
		 	int rx_size = pdrv->GetRxCount();
		  rxBufSize[idx - 1] = rx_size;
			pdrv->Receive(rxBuf[idx - 1], RX_BUFFER_SIZE);
   }
}

static void rs422_callback_1(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART1, 1, event);
}
static void rs422_callback_2(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART2, 2, event);
}
static void rs422_callback_3(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART3, 3, event);
}
static void rs422_callback_4(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART4, 4, event);
}
static void rs422_callback_5(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART5, 5, event);
}
static void rs422_callback_6(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART6, 6, event);
}
static void rs422_callback_7(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART7, 7, event);
}
static void rs422_callback_8(uint32_t event)
{
    prv_rs422_event_handle(&Driver_USART8, 8, event);
}

Define what the initial system looks like. 
void fcs_422_init(ARM_DRIVER_USART* pdrv, void (*callback)(uint32_t event), uint32_t baudrate, uint32_t parity, uint32_t stopbits, int uartIndex)
{
    rs422_inst.pdrv = pdrv;
    rs422_inst.baudrate = baudrate;
    rs422_inst.parity = parity;
    rs422_inst.stopbits = stopbits;

    tx_event_flags_create(&rs422_inst.events, "rs422_events");

    struct rs422_driver_inst* pinst = &rs422_inst;

    assert(pinst != NULL);
    pdrv = pinst->pdrv;
    assert(pdrv != NULL);

    pdrv->Initialize(callback);

    pdrv->PowerControl(ARM_POWER_FULL);

    pdrv->Control(  ARM_USART_MODE_ASYNCHRONOUS |
                    ARM_USART_DATA_BITS_8 |
                    pinst->parity|
                    pinst->stopbits |
                    ARM_USART_FLOW_CONTROL_NONE, pinst->baudrate);
     
 
    pdrv->Control (ARM_USART_CONTROL_TX, 1);
    pdrv->Control (ARM_USART_CONTROL_RX, 1);
		
		pdrv->Receive(rxBuf[uartIndex], RX_BUFFER_SIZE);
		return ;

} */
//-------------------Drive Over---------------------



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

//extern void fcs_422_init(ARM_DRIVER_USART* pdrv, void (*callback)(uint32_t event), uint32_t baudrate, uint32_t parity, uint32_t stopbits);
/***************************************
*	函数名称:	UART_Setting()
*	功能：根据波特率，奇偶校验设置串口参数
*	input:
*	channel：串口号，值域0-11
*	Baud：波特率
*	Par：校验位，’n’无校验，’e’偶校验，’o’奇校验
*	返回值：0正常
*****************************************/
int UART_Setting(int channel, unsigned int Baud, char check, int stopLen)
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
	case 3:
 			fd[3] = fcs_uart_init(&byte_pool_0, &Driver_USART4, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 4:
//			fd[4] = fcs_uart_init(&byte_pool_0, &Driver_USART5, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 5:
			fd[5] = fcs_uart_init(&byte_pool_0, &Driver_USART6, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 6:
//			fd[6] = fcs_uart_init(&byte_pool_0, &Driver_USART7, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
	case 7:
			fd[7] = fcs_uart_init(&byte_pool_0, &Driver_USART8, Baud, mode, ARM_USART_STOP_BITS_1);
			break;
    // case 8:
	// 		fd[8] = fcs_uart_init(&byte_pool_0, &Driver_USART21, Baud, mode, ARM_USART_STOP_BITS_1);
	// 		break;
    default:
        break;
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
    //if(channel == 3)
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
int UART_GetFrame(int channel, unsigned char * RxBuff, unsigned short MaxLength)
{
    int recv_len = fcs_uart_recv(fd[channel], RxBuff, MaxLength);
    if (recv_len > 0)
    {
        int a = 0;
        switch(channel)
        {
            case 0:
                a = 0;
                break;
            case 1:
                a = 0;
                break;
            case 2:
                a = 0;
                break;
            case 3:
                a = 0;
                break;
            case 4:
                a = 0;
                break;
            case 5:
                a = 0;
                break;
            case 6:
                a = 0;
                break;
            case 7:
                a = 0;
                break;
            case 8:
                a = 0;
                break;
            default:
                break;
        }
        return recv_len;
    }
    else
        return 0;
}

