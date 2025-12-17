/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>

#include    "tx_api.h"
#include    "UART_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
#include 	"./AppSource/stmToZynq.h"
/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
#define     APP_STACK_SIZE         12*1024
static TX_THREAD   s_app_tcb;
static uint32_t    s_app_stack[APP_STACK_SIZE/sizeof(uint32_t)];
static void app_task(ULONG thread_input);

#define     APP_TMR_FLAG    0x0800

/* Define what the initial system looks like.  */
void app_data_collector_init(TX_BYTE_POOL* pheap)
{
    /* Create the application thread.  */
    tx_thread_create(   &s_app_tcb, 
                        "data_task", 
                        app_task, 
                        (ULONG)pheap, 
                        s_app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 4,
                        TX_MAX_PRIORITIES - 4, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
}


uint32_t max_period_1ms;
static void app_task(ULONG thread_input) 
{
	Init7020Code();

    (void)thread_input;
    
    /* This thread simply sits in while-forever-sleep loop.  */
    while (1) 
    {              
        tx_thread_sleep(1);
        // do something here
        periodic_function_1ms();
    }
}
