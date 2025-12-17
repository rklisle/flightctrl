/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>
#include    <assert.h>

#include    "slist.h"
#include    "ringbuffer.h"
#include    "tx_api.h"



#include "Driver_USART.h"
#include "stm32h7xx_hal.h"

#define     UART_RX_RING_SIZE       2048
#define     UART_TX_RING_SIZE       1024

#define     UART_BLOCK_SIZE         256
#define     UART_INST_MAX           9
#define     UART_STACK_SIZE         6000

#define     UART_SEND_REQ           0x00000001
#define     UART_RECV_ERR           0x00000002
#define     UART_SEND_ERR           0x00000004

#define     UART_DRV_RETRY_DELAY    1
struct  uart_rx_block_tag
{
    slist_t         node;
    uint16_t        rx_cnt;
    uint16_t        rdout_cnt;
    uint8_t         buf[UART_BLOCK_SIZE];
}uart_rx_block_t;

struct uart_driver_inst
{
    ARM_DRIVER_USART*       pdrv;
    uint32_t                baudrate;
    uint32_t                parity;
    uint32_t                stopbits;

    int32_t                 drv_rx_ret;
    int32_t                 drv_tx_ret;
    ARM_USART_SignalEvent_t event_cb;
  
    uint16_t                rdout_cnt;
    uint16_t                tx_size;
    uint8_t*                rx_block_ptr;
    uint8_t                 rx_block[UART_BLOCK_SIZE*2];
    uint8_t                 tx_block[UART_BLOCK_SIZE];
    /* ring buffer */
    ring_buffer_t           rx_ring;
    ring_buffer_t           tx_ring;
    uint8_t                 rx_ring_buf[UART_RX_RING_SIZE];
    uint8_t                 tx_ring_buf[UART_TX_RING_SIZE];

    /* RTOS related resource */
    TX_THREAD               tcb;
    TX_EVENT_FLAGS_GROUP    events;

    uint8_t                 chj_recieving;
    uint32_t                stack[UART_STACK_SIZE/sizeof(uint32_t)];
};

static void uart_callback_1(uint32_t event);
static void uart_callback_2(uint32_t event);
static void uart_callback_3(uint32_t event);
static void uart_callback_4(uint32_t event);
static void uart_callback_5(uint32_t event);
static void uart_callback_6(uint32_t event);
static void uart_callback_7(uint32_t event);
static void uart_callback_8(uint32_t event);
static void uart_callback_9(uint32_t event);

static struct uart_driver_inst* s_inst_tbl[UART_INST_MAX];
const static ARM_USART_SignalEvent_t s_drv_cb_tbl[UART_INST_MAX] = 
{
    &uart_callback_1,
    &uart_callback_2,
    &uart_callback_3,
    &uart_callback_4,
    &uart_callback_5,
    &uart_callback_6,
    &uart_callback_7,
    &uart_callback_8,
    &uart_callback_9
};

static void prv_uart_event_handle(struct uart_driver_inst* pinst, uint32_t event);

static void uart_callback_1(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[0], event);
}
static void uart_callback_2(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[1], event);
}
static void uart_callback_3(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[2], event);
}
static void uart_callback_4(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[3], event);
}
static void uart_callback_5(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[4], event);
}
static void uart_callback_6(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[5], event);
}
static void uart_callback_7(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[6], event);
}
static void uart_callback_8(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[7], event);
}

static void uart_callback_9(uint32_t event)
{
    prv_uart_event_handle(s_inst_tbl[8], event);
}
extern UART_HandleTypeDef huart8;
static void prv_uart_event_handle(struct uart_driver_inst* pinst, uint32_t event)
{
    ARM_DRIVER_USART* pdrv = pinst->pdrv;
    int32_t         rx_cnt;
    uint32_t        event_flg = 0;
    uint8_t*        pcur_rx;
    uint32_t        rdout_cnt;
    TX_INTERRUPT_SAVE_AREA
    
    TX_DISABLE
    pinst->chj_recieving = 1;
    
    if(event & (ARM_USART_EVENT_RECEIVE_COMPLETE | ARM_USART_EVENT_RX_PARITY_ERROR | 
                ARM_USART_EVENT_RX_FRAMING_ERROR | ARM_USART_EVENT_RX_TIMEOUT))
    {   // receive complete or error happen
        rx_cnt = pdrv->GetRxCount();
        pdrv->Control (ARM_USART_ABORT_RECEIVE, 0);
        // record the rxed buf
        pcur_rx = pinst->rx_block_ptr;
        pinst->rx_block_ptr =   (pcur_rx == pinst->rx_block) ? 
                                pinst->rx_block + UART_BLOCK_SIZE:
                                pinst->rx_block;
        // enable recv now
        rdout_cnt = pinst->rdout_cnt;
        pinst->rdout_cnt = 0;
        pinst->drv_rx_ret = pdrv->Receive(pinst->rx_block_ptr, UART_BLOCK_SIZE);
        // do copy
        if(rdout_cnt < rx_cnt)
        {
            ring_buffer_write(  &pinst->rx_ring,
                                &pcur_rx[rdout_cnt],
                                rx_cnt - rdout_cnt);
        }
        // enable receive again
        if(pinst->drv_rx_ret != ARM_DRIVER_OK)
        {   // set flag
            event_flg = UART_RECV_ERR;
        }
    }
    if(event & ARM_USART_EVENT_SEND_COMPLETE)
    {   // get any tx data from ring
        pinst->tx_size = ring_buffer_read(&pinst->tx_ring, pinst->tx_block, sizeof(pinst->tx_block));
        if( pinst->tx_size != 0)
        {   /* send out to UART */
            pinst->drv_tx_ret = pdrv->Send(pinst->tx_block, pinst->tx_size);    
        } 
        if(pinst->drv_tx_ret != ARM_DRIVER_OK)
        {   // set flag
            event_flg |= UART_SEND_ERR;
        }
    }
    TX_RESTORE
    tx_event_flags_set( &pinst->events,
                        event_flg,
                        TX_OR);
}

/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
void uart_drv_task(ULONG thread_input) 
{
    struct uart_driver_inst* pinst = (void*)thread_input;
    ARM_DRIVER_USART* pdrv;
    // struct  uart_rx_block* rx_blk;
    // slist_t *pnode;
    uint32_t        actual_flag;
    uint32_t        delay_tick = TX_WAIT_FOREVER;
    TX_INTERRUPT_SAVE_AREA

    pdrv = pinst->pdrv;
    /*Initialize the USART driver */
    pdrv->Initialize(pinst->event_cb);
    /*Power up the USART peripheral */
    pdrv->PowerControl(ARM_POWER_FULL);
    /*Configure UART */
    pdrv->Control(  ARM_USART_MODE_ASYNCHRONOUS |
                    ARM_USART_DATA_BITS_8 |
                    pinst->parity|
                    pinst->stopbits |
                    ARM_USART_FLOW_CONTROL_NONE, pinst->baudrate);
     
    /* Enable Receiver and Transmitter lines */
    pdrv->Control (ARM_USART_CONTROL_TX, 1);
    pdrv->Control (ARM_USART_CONTROL_RX, 1);
    /* load an empty rx block */
    // pinst->cur_rx_blk = slist_entry(slist_pop(&pinst->blk_free_list),uart_rx_block_t, node);
    /* Get byte from UART */
    pinst->rx_block_ptr = pinst->rx_block;
    pinst->drv_rx_ret = pdrv->Receive(pinst->rx_block_ptr, UART_BLOCK_SIZE); 
    pinst->chj_recieving = 0;
    while (1)
    {
        actual_flag = 0;
        delay_tick = ((pinst->drv_rx_ret != ARM_DRIVER_OK) || (pinst->drv_tx_ret != ARM_DRIVER_OK)) ? UART_DRV_RETRY_DELAY : TX_WAIT_FOREVER ;
        if(pinst->chj_recieving == 0 && delay_tick == TX_WAIT_FOREVER)
        {
            delay_tick = 100;
        }
        tx_event_flags_get( &pinst->events, 
                            (UART_SEND_REQ | UART_RECV_ERR | UART_SEND_ERR),     // any flag
                            TX_OR_CLEAR, 
                            &actual_flag, 
                            delay_tick);
        if(actual_flag & UART_RECV_ERR)
        {   // try again
            pinst->drv_rx_ret = pdrv->Receive(pinst->rx_block_ptr, UART_BLOCK_SIZE);
        }
        if(actual_flag & UART_SEND_ERR)
        {   // try again
            pinst->drv_rx_ret = pdrv->Send(pinst->tx_block, pinst->tx_size);
        }        
        if((actual_flag & UART_SEND_REQ) && (pinst->tx_size == 0))
        {   // trigger send again
            TX_DISABLE
            pinst->tx_size = ring_buffer_read(&pinst->tx_ring, pinst->tx_block, sizeof(pinst->tx_block));
            TX_RESTORE
            if( pinst->tx_size != 0)
            {   /* send out to UART */
                pinst->drv_tx_ret = pdrv->Send(pinst->tx_block, pinst->tx_size);    
            } 
        }
        if(pinst->chj_recieving == 0)
        {
            pinst->drv_rx_ret = pdrv->Receive(pinst->rx_block_ptr, UART_BLOCK_SIZE);
        }

    }
}

int32_t fcs_uart_recv(int32_t fd, uint8_t* pdata, uint32_t len)
{    
    TX_INTERRUPT_SAVE_AREA
    int32_t ret = 0;
    uint32_t data_in_cache;
    uint32_t cpy_size;
    int32_t cur_rx_cnt;
    struct uart_driver_inst* pinst = s_inst_tbl[fd];

    if((fd < 0) || (fd >= UART_INST_MAX) || (pdata == NULL) || (pinst == NULL))
    {
        return -1;
    }

    TX_DISABLE
    // get data from ring buffer
    ret = ring_buffer_read(    &pinst->rx_ring,
                                pdata,
                                len);
    if(ret < len) 
    {
        cur_rx_cnt = pinst->pdrv->GetRxCount();
        if(cur_rx_cnt > pinst->rdout_cnt)
        {   // read from current recv buf
            data_in_cache = cur_rx_cnt - pinst->rdout_cnt;
            cpy_size = (len - ret) > data_in_cache ? data_in_cache : (len - ret);
            memcpy( pdata + ret, 
                    (pinst->rx_block_ptr + pinst->rdout_cnt), 
                    cpy_size);
            // update rd size
            ret += cpy_size;
            pinst->rdout_cnt = cur_rx_cnt;
        }
    }
    TX_RESTORE
    return ret;

}

int32_t fcs_uart_send(int32_t fd, const uint8_t* pdata, uint32_t len)
{
    TX_INTERRUPT_SAVE_AREA
    int32_t ret = 0;
    struct uart_driver_inst* pinst = s_inst_tbl[fd];

    if((fd < 0) || (fd >= UART_INST_MAX) || (pdata == NULL) || (pinst == NULL))
    {
        return -1;
    }

    TX_DISABLE
    // put data to ring buffer
    ret = ring_buffer_write(    &pinst->tx_ring,
                                pdata,
                                len);
    TX_RESTORE
    // notify task to send
    tx_event_flags_set( &pinst->events,
                        UART_SEND_REQ,
                        TX_OR);
    return ret;
}

/* Define what the initial system looks like.  */
int32_t fcs_uart_init(TX_BYTE_POOL* pool, ARM_DRIVER_USART* pdrv, uint32_t baudrate, uint32_t parity, uint32_t stopbits)
{
    uint32_t k;
    struct uart_driver_inst* pinst = NULL;
    TX_INTERRUPT_SAVE_AREA
    // alloc resource
    tx_byte_allocate(pool, (void**)&pinst, sizeof(struct uart_driver_inst), TX_NO_WAIT);
    if(pinst == NULL)
    {
        return -1;
    }
    // clr memory
    memset(pinst, 0x00, sizeof(struct uart_driver_inst));
    // find a free slot
    TX_DISABLE
    for(k=0; k<UART_INST_MAX; k++)
    {
        if(s_inst_tbl[k] == NULL)
        {
            s_inst_tbl[k] = pinst;
            pinst->event_cb = s_drv_cb_tbl[k];
            break;
        }
    }
    TX_RESTORE
    if(k >= UART_INST_MAX)
    {   // no enough slot
        tx_byte_release(pinst);
        return -1;
    }
    // clr 
    // pinst->cur_rx_blk = NULL;
   /* inst list initialize */
    // slist_init(&pinst->blk_rdy_list);
    // slist_init(&pinst->blk_free_list);
    // for(k=0; k<UART_RX_BLOCK_CNT; k++)
    // {
    //     slist_init(&pinst->rx_block[k].node);
    //     // link to free list
    //     slist_append(&pinst->blk_free_list, &pinst->rx_block[k].node);
    // }
    // ring buffer initialize
    ring_buffer_init(&pinst->rx_ring, pinst->rx_ring_buf, sizeof(pinst->rx_ring_buf));
    ring_buffer_init(&pinst->tx_ring, pinst->tx_ring_buf, sizeof(pinst->tx_ring_buf));

    // assign resource
    pinst->pdrv = pdrv;
    pinst->baudrate = baudrate;
    pinst->parity = parity;
    pinst->stopbits = stopbits;
    // init event
    tx_event_flags_create(&pinst->events, "uart_events");
    /* Create the application thread.  */
    tx_thread_create(   &pinst->tcb, 
                        "uart", 
                        uart_drv_task, 
                        (ULONG)pinst, 
                        pinst->stack,
                        UART_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 15,
                        TX_MAX_PRIORITIES - 15, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
    return k;
}

#include "./AppSource/support/os_framework.h"
void ReConnectUart()
{
    if(g_DeviceState.CurrTick % 200 == 0 && g_DeviceState.ecuCountDown == 0)
    {
        s_inst_tbl[5]->chj_recieving = 0;
    }
    
}