/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>

#include    "tx_api.h"

#include    "UART_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
#include    "pressure_sensor.h"
#include    "./AppSource/modules/modSD.h"




/**
 * ****************************************************
 * 
 * IDLE task to prevent CPU stay in PSV handler
 * 
 * ****************************************************
*/
#define     IDLE_STACK_SIZE         256
TX_THREAD   idle_tcb;
uint32_t    idle_stack[IDLE_STACK_SIZE/sizeof(uint32_t)];
void idle(ULONG thread_input) 
{
//    UINT status;
    /* This thread simply sits in while-forever-sleep loop.  */
    while (1) 
    {   // do nothing in idle
        __asm("nop");
        // while((tx_time_get() - start_ts) < delay_ms)
        // {
        // }
        // start_ts = tx_time_get();
    }
}
/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
#define     APP_STACK_SIZE         4096
static TX_THREAD   app_tcb;
static uint32_t    app_stack[APP_STACK_SIZE/sizeof(uint32_t)];


#ifdef STM32H753xx

//#define LED_CLOCK_ENABLE    __HAL_RCC_GPIOI_CLK_ENABLE
#define LED_CLOCK_ENABLE    __HAL_RCC_GPIOI_CLK_ENABLE
#define LED1_Pin            GPIO_PIN_13
#define LED1_GPIO_Port      GPIOI
#define LED2_Pin            GPIO_PIN_14
#define LED2_GPIO_Port      GPIOI

#endif

#ifdef  CPU_NAVI
#define LED_CLOCK_ENABLE    __HAL_RCC_GPIOE_CLK_ENABLE
#define LED1_Pin            GPIO_PIN_8
#define LED1_GPIO_Port      GPIOE
#define LED2_Pin            GPIO_PIN_12
#define LED2_GPIO_Port      GPIOE

#define UM982_CLOCK_ENABLE  __HAL_RCC_GPIOD_CLK_ENABLE
#define UM982_RST_PIN       GPIO_PIN_14
#define UM982_RST_PORT      GPIOD          
#endif

static void led_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */
    /* GPIO Ports Clock Enable */
    /* PI13, PI14 */
    LED_CLOCK_ENABLE();
#ifdef  CPU_NAVI
    UM982_CLOCK_ENABLE();
#endif
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);//LED1_GPIO_Port,LED1_Pin
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
#ifdef  CPU_NAVI
    HAL_GPIO_WritePin(UM982_RST_PORT, UM982_RST_PIN, GPIO_PIN_SET);
#endif    

    /*Configure GPIO pin : PI12 */
    GPIO_InitStruct.Pin = LED1_Pin;//LED1_Pin ;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);//LED1_GPIO_Port

    GPIO_InitStruct.Pin =  LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
#ifdef  CPU_NAVI   
    GPIO_InitStruct.Pin =  UM982_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(UM982_RST_PORT, &GPIO_InitStruct);
#endif
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


#include "uart_agent.h"
//extern int uart_recvdata;
void app_task(ULONG thread_input) 
{
    (void) thread_input;
//    UINT status;
    led_gpio_init();
    /* This thread simply sits in while-forever-sleep loop.  */
    while (1) 
    {   // do nothing in idle
        tx_thread_sleep(500);
        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        //if(uart_recvdata == 1)
        {
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        }
       // if(i%2 == 0)
        //i++;
    }
}
/**
 * ****************************************************
 * 
 * Main entry of system
 * 
 * ****************************************************
*/
// board init from bsp
extern void board_setup(void);
int main(int argc, char **argv) 
{
    /* Setup the hardware. */
    board_setup();

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


extern int32_t fcs_uart_init(TX_BYTE_POOL* pool, ARM_DRIVER_USART* pdrv, uint32_t baudrate, uint32_t parity, uint32_t stopbits);
extern void log_init(TX_BYTE_POOL* pool, ARM_DRIVER_USART* pdrv);

extern void pressure_Init(void);

extern void app_data_collector_init(TX_BYTE_POOL* pheap);
extern void app_control_algorithm_init(TX_BYTE_POOL* pheap);
extern void drv_test_init(TX_BYTE_POOL *pmem);

/* Define what the initial system looks like.  */
TX_BYTE_POOL byte_pool_0;
extern uint32_t _eram;  // symbo from linker description
extern uint32_t _sram;  // symbo from linker description
void tx_application_define(void *first_unused_memory)
{
    /*
    uint32_t left_ram_size = (uint32_t)(&_eram) - (uint32_t)(&_sram);
    int32_t fd = -1;
    tx_byte_pool_create(&byte_pool_0, 
                        "byte pool 0", 
                        (void*)(&_sram),
                        left_ram_size);

    tx_thread_create(   &idle_tcb, 
                        "idle", 
                        idle, 
                        0, 
                        idle_stack,
                        IDLE_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 1,
                        TX_MAX_PRIORITIES - 1, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
    
    tx_thread_create(   &app_tcb, 
                        "led", 
                        app_task, 
                        fd, 
                        app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
       //drv_test_init(&byte_pool_0);
       //return;
       */
  
        //app_pressure_init(&byte_pool_0);
		app_data_collector_init(&byte_pool_0);
		//app_control_algorithm_init(&byte_pool_0);
        //sd_write_init(&byte_pool_0);


}

