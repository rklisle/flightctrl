/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>
#include    <assert.h>

#include    "tx_api.h"


#include "SPI_STM32H7xx.h"


#define     GYRO_LSB    0.025   // -2, 0.0.00625 --> -1, 0.1 ---> -3// uint deg/sec
#define     ACCL_LSB    12.25	//2.45   // 12.25mm/s2 12.25
#define     TEMP_LSB    0.1     // 0.1 degree

struct adis16507_output
{
    bool      data_valid;
    
    /* data */
    float       gyro_x;
    float       gyro_y;
    float       gyro_z;
    float       accl_x;
    float       accl_y;
    float       accl_z;
    float       temp;
};


struct adis16507_output     out;

void GetImuOut(struct adis16507_output *out1)
{
    memcpy(out1, &out, sizeof(struct adis16507_output));
}

/**
 * ****************************************************
 * 
 * Application task
 * 
 * ****************************************************
*/
#define     APP_STACK_SIZE         1024
static TX_THREAD   app_tcb;
static uint32_t    app_stack[APP_STACK_SIZE/sizeof(uint32_t)];

static void mySPI_callback(uint32_t event)
{
    // tx_event_flags_set( &rs422_inst.events,
    //                     event,
    //                     TX_OR);
}

static uint16_t prv_check_sum(void* pdata, uint32_t size)
{
    uint32_t k;
    uint8_t* data = pdata;
    uint16_t ret = 0;
    for( k=0;k<size;k++)
    {
        ret += data[k];
    }
    return ret;
} 

static inline float    prv_scale_calc(int16_t  raw, float factor)
{
    return ((float)raw) * factor;
}

 
static void adis16507_task(ULONG thread_input) 
{
    ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1;
    int16_t          rx_buf[30];
    // uint16_t         tx_buf[30] = { 0x0200,0x0400,0x0600,0x0800,0x0A00,0x0C00,0x0E00,0x1000,
    //                                 0x1200,0x1400,0x1600,0x1800,0x1A00,0x1C00,0x1E00,0x0000};
    uint16_t         tx_buf[30] = { 0x6800};    // MUST use this to read
    uint16_t         rd_pid[2] = {0x7200,0x0200};

    /* Initialize the SPI driver */
    SPIdrv->Initialize(NULL);
    /* Power up the SPI peripheral */
    SPIdrv->PowerControl(ARM_POWER_FULL);
    /* Configure the SPI to Master, 8-bit mode @1000 kBits/sec */
    SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(16), 1000000);
 
    /* SS line = INACTIVE = HIGH */
    // SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
 

    //pdrv->Receive(rx_buf, sizeof(rx_buf));          /* Get byte from UART */
    while (1)
    {
        SPIdrv->Transfer(tx_buf, rx_buf, 11);
        tx_thread_sleep(5);
        if(prv_check_sum(&rx_buf[1], 18) != rx_buf[10])
        {
           // while(1);
        }
        // gyro calc
        out.gyro_x = prv_scale_calc(rx_buf[2], GYRO_LSB);
        out.gyro_y = prv_scale_calc(rx_buf[3], GYRO_LSB);
        out.gyro_z = prv_scale_calc(rx_buf[4], GYRO_LSB);
        // accl calc
        out.accl_x = prv_scale_calc(rx_buf[5], ACCL_LSB);
        out.accl_y = prv_scale_calc(rx_buf[6], ACCL_LSB);
        out.accl_z = prv_scale_calc(rx_buf[7], ACCL_LSB);

        out.temp = prv_scale_calc(rx_buf[8], TEMP_LSB);

    }
}

/* Define what the initial system looks like.  */
void adis16507_Init(void)
{
    // rs422_inst.pdrv = pdrv;
    // rs422_inst.baudrate = baudrate;
    // rs422_inst.parity = parity;
    // rs422_inst.stopbits = stopbits;

    // tx_event_flags_create(&rs422_inst.events, "rs422_events");
    /* Create the application thread.  */
    tx_thread_create(   &app_tcb, 
                        "app1", 
                        adis16507_task, 
                        0, 
                        app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);

}

