/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>

#include    "tx_api.h"

#include    "sd_flash.h"
/*
**************************************************************
*
*     serial echo test
*
**************************************************************
*/
#include    "SPI_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
#include    "nor_flash.h"
#include    "app_can.h"
#define     NOR_STACK_SIZE         2048
static TX_THREAD   nor_tcb;
static uint32_t    nor_stack[NOR_STACK_SIZE/sizeof(uint32_t)];
const char* nor_test_data = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
uint8_t sd_flash_buf[512] = {0};


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

static void nor_task(ULONG thread_input)
{
    TX_BYTE_POOL *pmem = (TX_BYTE_POOL *) thread_input;
    uint8_t rd_mem[64] = {0};
    uint8_t can_tx_msg[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

//    __HAL_RCC_GPIOB_CLK_ENABLE();    

    
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    //GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    
    /*nor_flash_init( &Driver_SPI1 ,pmem);

    nor_read(0x00, rd_mem, sizeof(rd_mem));
    if(memcmp(rd_mem, nor_test_data, strlen(nor_test_data)) != 0)
    {
        nor_block_erase(0x00, strlen(nor_test_data));
        nor_write(0x00, (const uint8_t*)nor_test_data, strlen(nor_test_data));
    }
*/
    // sd flash
    sd_flash_init(NULL);
    fatFsFileOpen("abc.txt",0);
    fatFsFileOpen("ddd.txt",1);
    
    char file1[512] = "111113";
    char file2[512] = "222223";
    
    fatFsFileWrite(0, (char *)file1, strlen(file1));
    fatFsFileWrite(1, (char *)file2, strlen(file2));
    
    return;
    
    
    
    sd_read(0, sd_flash_buf, sizeof(sd_flash_buf));
    if(memcmp(sd_flash_buf, nor_test_data, strlen(nor_test_data)) != 0)
    {
        memcpy(sd_flash_buf,nor_test_data,strlen(nor_test_data));
        sd_block_erase(0,1);
        sd_write(0,sd_flash_buf, sizeof(sd_flash_buf));
    }

    // can test
    hfdcan1.Instance = FDCAN1;
    int32_t fd1 = app_can_init(NULL, &hfdcan1, NULL);
    
    hfdcan2.Instance = FDCAN2;
    int32_t fd2 = app_can_init(NULL, &hfdcan2, NULL);
    
    tx_thread_sleep(1000);
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

    
    while(1)
    {
        tx_thread_sleep(200);
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        app_can_send(fd1, 0x123, false, can_tx_msg, sizeof(can_tx_msg));
        app_can_send(fd2, 0x123, false, can_tx_msg, sizeof(can_tx_msg));
    }
}
/*
**************************************************************
*
*     serial echo test
*
**************************************************************
*/

#include    "UART_STM32H7xx.h"
#include    "uart_agent.h"

#define     ECHO_STACK_SIZE         2048
static TX_THREAD   echo_tcb;
static uint32_t    echo_stack[ECHO_STACK_SIZE/sizeof(uint32_t)];

#define ECHO_HELLO_STRING   "echo start\r\n"
uint32_t recv_cnt = 0;
uint32_t recv_time = 0;
static void echo_task(ULONG thread_input) 
{
    uint8_t recv_buf[64];
    TX_BYTE_POOL *pmem = (TX_BYTE_POOL *) thread_input;
    int32_t fd = -1;
    int32_t recv_len;
    
    fd = fcs_uart_init( pmem,
                        &Driver_USART1,
                        921600,
                        ARM_USART_PARITY_NONE,
                        ARM_USART_STOP_BITS_1);
    /* This thread simply sits in while-forever-sleep loop.  */
    fcs_uart_send(fd, (const uint8_t*)ECHO_HELLO_STRING, sizeof(ECHO_HELLO_STRING));
    while (1) 
    {   // do nothing in idle
        do
        {
            recv_len = fcs_uart_recv(fd,recv_buf, sizeof(recv_buf));
            // echo out
            if(recv_len > 0)
            {
                recv_time++;
                recv_cnt += recv_len;
                fcs_uart_send(fd, recv_buf, recv_len);
            }
        }while(recv_len > 0);
        tx_thread_sleep(1);
    }
}



/*
**************************************************************
*
*     test entry
*
**************************************************************
*/
#include "pressure_sensor.h"
void drv_test_init(TX_BYTE_POOL *pmem)
{   
    /* Create the application thread.  */
    tx_thread_create(   &echo_tcb, 
                        "echo", 
                        echo_task, 
                        (ULONG)pmem, 
                        echo_stack,
                        ECHO_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);

    /* Create the application thread.  */
    tx_thread_create(   &nor_tcb, 
                        "nor", 
                        nor_task, 
                        (ULONG)pmem,
                        nor_stack,
                        NOR_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
    app_pressure_init(pmem);

}
