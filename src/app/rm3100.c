/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>
#include    <assert.h>

#include    "tx_api.h"

#include    "SPI_STM32H7xx.h"
#include    "stm32h7xx_hal.h"

enum RM3100_REGISTER {
    RM3100_REG_POLL =   0x00, // polls for a single measurement
    RM3100_REG_CMM =    0x01, // initiates continuous measurement mode
    RM3100_REG_CCX =    0x04, // cycle counts -- X axis
    RM3100_REG_CCY =    0x06, // cycle counts -- Y axis
    RM3100_REG_CCZ =    0x08, // cycle counts -- Z axis
    RM3100_REG_TMRC =   0x0B, // sets continuous mode data rate
    RM3100_REG_MX =     0x24, // measurement results -- X axis
    RM3100_REG_MY =     0x27, // measurement results -- Y axis
    RM3100_REG_MZ =     0x2A, // measurement results -- Z axis
    RM3100_REG_BIST =   0x33, // built-in self test
    RM3100_REG_STATUS = 0x34, // status of DRDY
    RM3100_REG_HSHAKE = 0x35, // handshake register
    RM3100_REG_REVID =  0x36, // MagI2C revision identification
};

struct rm3100_inst
{
    // spi driver
    ARM_DRIVER_SPI* drv;
    // output calcuation ratio
    float       ratio_x;
    float       ratio_y;
    float       ratio_z;
    /* data */
    int32_t     raw_x;
    int32_t     raw_y;
    int32_t     raw_z;

    float       mag_x;
    float       mag_y;
    float       mag_z;
    float       temp;
};

#define     DEFAULT_CCR         100  //200 cycles take 8ms, 100 cycle take 4ms

#define     SMM_AXIS_X      (1 << 4)
#define     SMM_AXIS_Y      (1 << 5)
#define     SMM_AXIS_Z      (1 << 6)

#define     SPI_TIMEOUT     50      // ms

#define     RM3100_REG_READ 0x80
#define     RM3100_REVID    0x22
/**
 * ****************************************************
 * 
 * Application static var
 * 
 * ****************************************************
*/
#define     APP_STACK_SIZE      1024
static TX_THREAD                app_tcb;
static uint32_t                 app_stack[APP_STACK_SIZE/sizeof(uint32_t)];
static TX_EVENT_FLAGS_GROUP     s_events;

static struct rm3100_inst       s_rm3100 = {0};


void GetMagXYZDIR(double *x, double *y, double *z)
{
    *x = s_rm3100.mag_x;
    *y = s_rm3100.mag_y;
    *z = s_rm3100.mag_z;
}

/**
 * ****************************************************
 * HAL porting layer
 * ****************************************************
 */
static void prv_drdy_init(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    
    /* Enable GPIOC clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /*Configure GPIO pin : PC4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    //HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
    //HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

// static void prv_drdy_ctrl(bool en)
// {
//     if(en)
//     {   // clear pending flag
//         __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
//         // enable data ready
//         HAL_NVIC_EnableIRQ(EXTI4_IRQn);
//     }
//     else
//     {
//         HAL_NVIC_DisableIRQ(EXTI4_IRQn);
//         // clear pending flag
//         __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
//     }

// }

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
#define     RM3100_DRDY_EVENT       0x80
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4)
    {
        tx_event_flags_set( &s_events,
                            RM3100_DRDY_EVENT,
                            TX_OR);
    }
}

static void prv_spi_callback(uint32_t event)
{
    tx_event_flags_set( &s_events,
                        event,
                        TX_OR);
}


static int32_t prv_rm3100_read(ARM_DRIVER_SPI* drv, uint8_t reg_addr, uint8_t* rd_buf, uint32_t rd_cnt)
{
    uint8_t tx_buf[16] = {0};
    uint8_t rx_buf[16] = {0};
    uint32_t events = 0;
    
    // overflow check
    assert((rd_cnt + 1) <= sizeof(tx_buf));
    // assign register address
    tx_buf[0] = RM3100_REG_READ | reg_addr;

    // clear spi event here
    tx_event_flags_get( &s_events,
                        (ARM_SPI_EVENT_TRANSFER_COMPLETE | ARM_SPI_EVENT_DATA_LOST | ARM_SPI_EVENT_MODE_FAULT),
                        TX_OR_CLEAR,
                        &events,
                        0);
    // now sendout register address
    assert(ARM_DRIVER_OK == drv->Transfer(tx_buf, rx_buf, rd_cnt + 1));
    if(TX_SUCCESS != tx_event_flags_get( &s_events,
                                        (ARM_SPI_EVENT_TRANSFER_COMPLETE | ARM_SPI_EVENT_DATA_LOST | ARM_SPI_EVENT_MODE_FAULT),
                                        TX_OR_CLEAR,
                                        &events,
                                        SPI_TIMEOUT))
    {
        return -1;
    }
    // check event result
    if(events & ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        memcpy(rd_buf, &rx_buf[1], rd_cnt);
        return rd_cnt;
    }
    return 0;
}

static int32_t prv_rm3100_write(ARM_DRIVER_SPI* drv, uint8_t reg_addr, uint8_t* wt_buf, uint32_t wt_cnt)
{
    uint8_t tx_buf[16] = {0};
//    uint8_t rx_buf[16] = {0};
    uint32_t events = 0;
    
    // overflow check
    assert((wt_cnt + 1) <= sizeof(tx_buf));
    // assign register address
    tx_buf[0] = reg_addr;
    memcpy(&tx_buf[1], wt_buf, wt_cnt);
    // clear spi event here
    tx_event_flags_get( &s_events,
                        (ARM_SPI_EVENT_TRANSFER_COMPLETE | ARM_SPI_EVENT_DATA_LOST | ARM_SPI_EVENT_MODE_FAULT),
                        TX_OR_CLEAR,
                        &events,
                        0);
    // now sendout register address
    assert(ARM_DRIVER_OK == drv->Send(tx_buf, wt_cnt + 1));
    if(TX_SUCCESS != tx_event_flags_get( &s_events,
                                        (ARM_SPI_EVENT_TRANSFER_COMPLETE | ARM_SPI_EVENT_DATA_LOST | ARM_SPI_EVENT_MODE_FAULT),
                                        TX_OR_CLEAR,
                                        &events,
                                        SPI_TIMEOUT))
    {
        return -1;
    }
    return wt_cnt;
}

/**
 * ****************************************************
 * 
 * RM3100 driver
 * 
 * ****************************************************
*/

/**
 * Gets the current single measurement mode config, POLL(0x00)
 *
 * @param smm pointer to read config into
 * @return 0 on success
 */
static int32_t prv_rm3100_smm_get(struct rm3100_inst* pinst, uint8_t* psmm)
{
    return prv_rm3100_read(pinst->drv, RM3100_REG_POLL, (uint8_t *)psmm, 1);
}


/**
 * Sets the current single measurement mode config
 *
 * @return 0 on success
 */
static int32_t prv_rm3100_smm_set(struct rm3100_inst* pinst, uint8_t mode)
{
    uint8_t smm = mode;
    return prv_rm3100_write(pinst->drv, RM3100_REG_POLL, (uint8_t *)&smm, sizeof(smm));
}

/**
 * Gets the current cycle counts, CCX, CCY, CCZ (0x04 -- 0x09)
 *
 * @param cc pointer to read into
 * @return 0 on success
 */
static int32_t prv_rm3100_cyclecount_get(struct rm3100_inst* pinst,  uint16_t* px, uint16_t* py, uint16_t* pz)
{
    uint8_t buffer[6];
    int ret = prv_rm3100_read(pinst->drv, RM3100_REG_CCX, buffer, sizeof(buffer));
    if (ret != sizeof(buffer)) {
        return -1;
    }
    *px = (buffer[0] << 8) | buffer[1];
    *py = (buffer[2] << 8) | buffer[3];
    *pz = (buffer[4] << 8) | buffer[5];
    return sizeof(buffer);
}


/*
 * Sets the current cycle counts (x, y, z)
 *
 * @param x cycle counts for X axis
 * @param y cycle counts for Y axis
 * @param z cycle counts for Z axis
 * @return 0 on success
 */
static int32_t prv_rm3100_cyclecount_set(struct rm3100_inst* pinst, uint16_t x, uint16_t y, uint16_t z)
{
    // convert to BE
    uint8_t buffer[6] = {
        (x >> 8), (x & 0xFF),
        (y >> 8), (y & 0xFF),
        (z >> 8), (z & 0xFF),
    };

    pinst->ratio_x = 1.0f / ((x * 0.3627f) + 1.85f);
    pinst->ratio_y = 1.0f / ((y * 0.3627f) + 1.85f);
    pinst->ratio_z = 1.0f / ((z * 0.3627f) + 1.85f);
    
    return prv_rm3100_write(pinst->drv, RM3100_REG_CCX, buffer, sizeof(buffer));
}

/**
 * Gets the current status
 *
 * @param status pointer to read into
 * @return 0 on success
 */
static int32_t prv_rm3100_status_get(struct rm3100_inst* pinst)
{
    uint8_t status      = 0;
    prv_rm3100_read(pinst->drv, RM3100_REG_STATUS, &status, sizeof(status));
    return status;
}

/**
 * Gets the current status
 *
 * @param status pointer to read into
 * @return 0 on success
 */
static int32_t prv_rm3100_revision_get(struct rm3100_inst* pinst, uint8_t* prevision)
{
    prv_rm3100_read(pinst->drv, RM3100_REG_REVID, prevision, sizeof(uint8_t));
    return 0;
}




/**
 * Gets the current measurement data (24-bit signed)
 *
 * @param m pointer to read into
 * @return 0 on success
 */
static int32_t prv_rm3100_xyz_get(struct rm3100_inst* pinst)
{
    uint8_t buffer[9] = { 0 };

    if(prv_rm3100_read(pinst->drv, RM3100_REG_MX, buffer, sizeof(buffer)) != sizeof(buffer))
    {
        return -1;
    }
    // load raw data
    pinst->raw_x = (((int8_t)buffer[0]) << 16) | (buffer[1] << 8) | (buffer[2]);
    pinst->raw_y = (((int8_t)buffer[3]) << 16) | (buffer[4] << 8) | (buffer[5]);
    pinst->raw_z = (((int8_t)buffer[6]) << 16) | (buffer[7] << 8) | (buffer[8]);
    // convert to output
    pinst->mag_x = (float)pinst->raw_x * pinst->ratio_x;
    pinst->mag_y = (float)pinst->raw_y * pinst->ratio_y;
    pinst->mag_z = (float)pinst->raw_z * pinst->ratio_z;
    return 0;
}


uint32_t    measure_ts;
static void rm3100_task(ULONG thread_input) 
{
    struct rm3100_inst* pinst = (void*)thread_input;
    ARM_DRIVER_SPI* drv = pinst->drv;
    uint8_t     revision;
    uint32_t    pre_ts;

    /* Initialize the SPI driver */
    assert( drv->Initialize(prv_spi_callback) == ARM_DRIVER_OK);
    /* Power up the SPI peripheral */
    assert( drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    /* Configure the SPI to Master, 8-bit mode @1000 kBits/sec */
    assert( drv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8), 1000000) == ARM_DRIVER_OK);


    /* check hardware revision */
    assert(prv_rm3100_revision_get(pinst, &revision) == 0);
    assert(revision == RM3100_REVID);

    /* set scale */
    prv_rm3100_cyclecount_set(pinst, DEFAULT_CCR, DEFAULT_CCR, DEFAULT_CCR);
    /* config drdy as input */
    prv_drdy_init();
    tx_thread_sleep(1000);
    while (1)
    {
        pre_ts = tx_time_get();

        prv_rm3100_smm_set(pinst, SMM_AXIS_X | SMM_AXIS_Y | SMM_AXIS_Z);

        while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            tx_thread_sleep(1);
        }
        measure_ts = tx_time_get()- pre_ts;
        prv_rm3100_xyz_get(pinst);
        // delay for next sample
        tx_thread_sleep(5);
    }
}

/* Define what the initial system looks like.  */
void rm3100_init(void)
{
    // rs422_inst.pdrv = pdrv;
    // rs422_inst.baudrate = baudrate;
    // rs422_inst.parity = parity;
    // rs422_inst.stopbits = stopbits;
    
    // rm3100 initialize */
    s_rm3100.drv = &Driver_SPI2;

    tx_event_flags_create(&s_events, "rm3100_events");
    /* Create the application thread.  */
    tx_thread_create(   &app_tcb, 
                        "rm3100", 
                        rm3100_task, 
                        (uint32_t)&s_rm3100, 
                        app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);

}

