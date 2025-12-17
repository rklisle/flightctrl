/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>

#include    "tx_api.h"
#include    "UART_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
#include 	"./NavSource/stmToZynq.h"
/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
#define     APP_STACK_SIZE         35*1024
static TX_THREAD   s_app_tcb;
static uint32_t    s_app_stack[APP_STACK_SIZE/sizeof(uint32_t)];
static TX_TIMER    s_app_tmr;
static TX_EVENT_FLAGS_GROUP     s_events;
static void app_task(ULONG thread_input);

#define     APP_TMR_FLAG    0x0800

static void tmr_entry(ULONG thread_input)
{
    TX_EVENT_FLAGS_GROUP* pevent = (TX_EVENT_FLAGS_GROUP*)thread_input;
    tx_event_flags_set( pevent,
                        APP_TMR_FLAG,
                        TX_OR);
}

/* Define what the initial system looks like.  */
void navi_control_algorithm_init(TX_BYTE_POOL* pheap)
{
    /* Create the application thread.  */
    tx_thread_create(   &s_app_tcb, 
                        "ctrl_task", 
                        app_task, 
                        (ULONG)pheap, 
                        s_app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 15,
                        TX_MAX_PRIORITIES - 15, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
}

uint32_t max_period;
static void app_task(ULONG thread_input) 
{
    (void)thread_input;
    uint32_t events = 0;
    uint32_t start_ts;
    uint32_t cur_ts;
    InitNavCode();
    tx_event_flags_create(&s_events, "ctrl_event");
//    UINT status;
    tx_timer_create(&s_app_tmr,"ctrl_tmr", tmr_entry, (ULONG)&s_events, 5, 5, TX_AUTO_ACTIVATE);
    /* This thread simply sits in while-forever-sleep loop.  */
    while (1) 
    {
       tx_event_flags_get( &s_events,
                            APP_TMR_FLAG,
                            TX_OR_CLEAR,
                            &events,
                            TX_WAIT_FOREVER);

        // do something here
        periodic_nav_5ms();

    }
}
