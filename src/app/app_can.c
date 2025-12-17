#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_can.h"
#include "tx_api.h"
#include "stm32h7xx_hal.h"

int fdCan[2];
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define     APP_STACK_SIZE         20480

#define     APP_CAN_RX_EVENT        0x01
#define     APP_CAN_TX_CPLT_EVENT   0x02
#define     APP_CAN_TX_REQ_EVENT    0x04

struct app_can_rx_msg    
{
    uint32_t    id;
    uint16_t    len;
    uint8_t     frm_type;
    bool        ext_id;
    uint8_t     buf[8];
};

typedef struct fdcan_agent_tag
{
    FDCAN_HandleTypeDef*    phdl;
    TX_THREAD               tcb;
    TX_QUEUE                queue;
    TX_EVENT_FLAGS_GROUP    event;
    pcan_recv_cb_t          pcb;
    struct app_can_rx_msg   msg_buf[8];
    uint32_t                app_stack[APP_STACK_SIZE/sizeof(uint32_t)];
}fdcan_agent_t;

fdcan_agent_t   s_inst[2];
struct app_can_rx_msg canCurMsg[2];
static TX_MUTEX       can_mutex_write[2];  

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
  /* Prevent unused argument(s) compilation warning */
    TX_EVENT_FLAGS_GROUP *pevent;
    UNUSED(TxEventFifoITs);

    pevent = (hfdcan == s_inst[0].phdl)? (&s_inst[0].event) : (&s_inst[1].event);

    /* In case of ADC error, call main error handler */
    tx_event_flags_set(pevent, APP_CAN_TX_CPLT_EVENT, TX_OR);
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    TX_EVENT_FLAGS_GROUP *  pevent;
    TX_QUEUE             *  pqueue;

    struct app_can_rx_msg   msg;
    FDCAN_RxHeaderTypeDef   rxHeader;
    int index = 0;
    if(hfdcan == s_inst[0].phdl)
    {
        pevent = &s_inst[0].event;
        pqueue = &s_inst[0].queue;
        index = 0;
    }
    else
    {
        pevent = &s_inst[1].event;
        pqueue = &s_inst[1].queue;
        index = 1;
    }

    if( ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) && 
        (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg.buf) == HAL_OK))
    {   /* Retrieve Rx messages from RX FIFO0 */
        msg.id = rxHeader.Identifier;
        msg.len = rxHeader.DataLength;
        tx_mutex_get(&can_mutex_write[index], TX_WAIT_FOREVER);
        memcpy(&canCurMsg[index], &msg, sizeof(msg));
        tx_mutex_put(&can_mutex_write[index]);
        /* put recv msg to queue */
        //tx_queue_send(pqueue, &msg, TX_NO_WAIT);
        /* set event to wakeup task */
        tx_event_flags_set(pevent, APP_CAN_RX_EVENT, TX_OR);
    }
}
// void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
// {
//     TX_EVENT_FLAGS_GROUP *  pevent;
//     TX_QUEUE             *  pqueue;

//     struct app_can_rx_msg   msg;
//     FDCAN_RxHeaderTypeDef   rxHeader;

//     pevent = &s_inst[1].event;
//     pqueue = &s_inst[1].queue;

//     if( ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0) && 
//         (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader, msg.buf) == HAL_OK))
//     {   /* Retrieve Rx messages from RX FIFO0 */
//         msg.id = rxHeader.Identifier;
//         msg.len = rxHeader.DataLength;
//         /* put recv msg to queue */
//         tx_queue_send(pqueue, &msg, TX_NO_WAIT);
//         /* set event to wakeup task */
//         tx_event_flags_set(pevent, APP_CAN_RX_EVENT, TX_OR);
//     }
// }

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void prv_FDCAN_Init(FDCAN_HandleTypeDef* phdl, int32_t fd)
{
    /* USER CODE BEGIN FDCAN1_Init 0 */

    FDCAN_FilterTypeDef sFilterConfig;
    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

        /* Bit time configuration:
            fdcan_ker_ck               = 64 MHz
            Time_quantum (tq)          = 15.628 ns
            Synchronization_segment    = 1 tq
            Propagation_segment        = 39 tq
            Phase_segment_1            = 12 tq
            Phase_segment_2            = 12 tq 
            Synchronization_Jump_width = 8 tq
            Bit_length                 = 64 tq = 1us
            Bit_rate                   = 1 MBit/s
        */
    /* USER CODE END FDCAN1_Init 1 */
    //hfdcan2.Instance = FDCAN2;
    phdl->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    phdl->Init.Mode = FDCAN_MODE_NORMAL;
    phdl->Init.AutoRetransmission = DISABLE;
    phdl->Init.TransmitPause = DISABLE;
    phdl->Init.ProtocolException = DISABLE;
    phdl->Init.NominalPrescaler = 2;
    phdl->Init.NominalSyncJumpWidth = 7;
    phdl->Init.NominalTimeSeg1 = 23;
    phdl->Init.NominalTimeSeg2 = 8; // 500K = 64M/(1/4/(1+Seg1 + Seg2))

    phdl->Init.DataPrescaler = 2;
    phdl->Init.DataSyncJumpWidth = 7;
    phdl->Init.DataTimeSeg1 = 23;
    phdl->Init.DataTimeSeg2 = 8;
    phdl->Init.MessageRAMOffset = 0;
    phdl->Init.StdFiltersNbr = 16;
    phdl->Init.ExtFiltersNbr = 0;

    phdl->Init.RxFifo0ElmtsNbr = 2;
    phdl->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    phdl->Init.RxFifo1ElmtsNbr = 2;
    phdl->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    phdl->Init.RxBuffersNbr = 0;
    phdl->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    phdl->Init.TxEventsNbr = 0;
    phdl->Init.TxBuffersNbr = 0;
    phdl->Init.TxFifoQueueElmtsNbr = 2;
    phdl->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    phdl->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(phdl) != HAL_OK)
    {
        while(1){;}
    }
    /* USER CODE BEGIN FDCAN1_Init 2 */

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = fd;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
    HAL_FDCAN_ConfigFilter(phdl, &sFilterConfig);

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(phdl, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* Configure Rx FIFO 0 watermark to 2 */
    HAL_FDCAN_ConfigFifoWatermark(phdl,  FDCAN_CFG_RX_FIFO0, 2);

    /* Activate Rx FIFO 0 watermark notification */
    HAL_FDCAN_ActivateNotification(phdl, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* Start the FDCAN module */
    HAL_FDCAN_Start(phdl);
    /* USER CODE END FDCAN1_Init 2 */
}

int32_t app_can_send(int32_t fd, uint32_t id, bool ext_id, const uint8_t* pdata, uint32_t datalen)
{
    FDCAN_TxHeaderTypeDef tx_hdr;
    //TX_EVENT_FLAGS_GROUP *pevent;

    if((fd < 0) || (fd >= 2) || (s_inst[fd].phdl == NULL)) 
    {
        return -1;
    }
    
    /* init tx header */
    tx_hdr.Identifier = id;
    tx_hdr.IdType = (ext_id == false) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = datalen;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker = 0;
    
    int result = HAL_FDCAN_AddMessageToTxFifoQ(s_inst[fd].phdl, &tx_hdr, pdata);
    return datalen;
}

struct app_can_rx_msg   recvCanMsg[2] = {0};
void app_can_demo (ULONG thread_input) 
{
    uint32_t    actual_flags = 0;
    struct app_can_rx_msg   msg = {0};
    fdcan_agent_t* pinst = &s_inst[thread_input];
    
   // init can driver
    prv_FDCAN_Init(pinst->phdl,thread_input);

    /* activate tmr */
    //tx_timer_activate(&s_tmr);
    while (1) 
    {  
        tx_event_flags_get( &pinst->event, 
                            (APP_CAN_RX_EVENT), 
                            TX_OR_CLEAR, 
                            &actual_flags,
                            TX_WAIT_FOREVER);
        if(actual_flags & APP_CAN_RX_EVENT)
        { 
            tx_mutex_get(&can_mutex_write[thread_input], TX_WAIT_FOREVER);
            memcpy(&recvCanMsg[thread_input], &canCurMsg[thread_input], sizeof(struct app_can_rx_msg));
            tx_mutex_put(&can_mutex_write[thread_input]);
            pinst->pcb(recvCanMsg[thread_input].id, recvCanMsg[thread_input].ext_id, recvCanMsg[thread_input].buf, recvCanMsg[thread_input].len);

            /*
            // receive some message
            while(tx_queue_receive(&pinst->queue, &msg, TX_NO_WAIT) == TX_SUCCESS)
            {
                if(pinst->pcb != NULL)
                {
                    pinst->pcb(msg.id, msg.ext_id, msg.buf, msg.len);
                }
                tx_thread_sleep(1);
            }*/
        }
        else
        {
            tx_thread_sleep(1);
        }
    }
}

int32_t app_can_init(TX_BYTE_POOL *pmem, FDCAN_HandleTypeDef* phdl,pcan_recv_cb_t pcb )
{
    (void)pmem;
    int32_t k;
    fdcan_agent_t *pinst;
    
    for(k=0; k<(sizeof(s_inst)/sizeof(fdcan_agent_t)); k++)
    {
        pinst  = &s_inst[k];
        if(pinst->phdl == NULL)
        {
            break;
        }
        pinst = NULL;
    }

    if(pinst == NULL)
    {
        return -1;
    }
    pinst->phdl = phdl;
    pinst->pcb = pcb;

    tx_mutex_create(&can_mutex_write[k], "sd_write_mutex", TX_INHERIT);
    
    tx_queue_create(    &pinst->queue, 
                        "can mbox", 
                        sizeof( struct app_can_rx_msg  ), 
                        &pinst->msg_buf, 
                        sizeof(pinst->msg_buf));

    /* create event flag */
    tx_event_flags_create(&pinst->event, "can_event");

    /* Create the application thread.  */
    tx_thread_create(   &pinst->tcb, 
                        "can", 
                        app_can_demo, 
                        k, 
                        pinst->app_stack,
                        sizeof(pinst->app_stack), 
                        TX_MAX_PRIORITIES - 10,
                        TX_MAX_PRIORITIES - 10, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
    return k;
}
