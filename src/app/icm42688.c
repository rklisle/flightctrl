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

#include    "icm42688_def.h"

#define     IMU_REG_READ        0x80

#define     SPI_TIMEOUT         500     // 500 ticks

struct imu_data
{
    /* raw value from imu */
    int16_t  raw_x;
    int16_t  raw_y;
    int16_t  raw_z;

    /* output */
    float    x;
    float    y;
    float    z;
};

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


static void prv_spi_callback(uint32_t event)
{
    tx_event_flags_set( &s_events,
                        event,
                        TX_OR);
}

/**
 * ****************************************************
 * 
 * ICM42688 APIs
 * 
 * ****************************************************
*/

static int32_t prv_imu_read(ARM_DRIVER_SPI* drv, uint8_t reg_addr, uint8_t* rd_buf, uint32_t rd_cnt)
{
    uint8_t tx_buf[16] = {0};
    uint8_t rx_buf[16] = {0};
    uint32_t events = 0;
    
    // overflow check
    assert((rd_cnt + 1) <= sizeof(tx_buf));
    // assign register address
    tx_buf[0] = IMU_REG_READ | reg_addr;

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

static int32_t prv_imu_write(ARM_DRIVER_SPI* drv, uint8_t reg_addr, uint8_t* wt_buf, uint32_t wt_cnt)
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

static int32_t prv_icm42688_getdata(ARM_DRIVER_SPI* pdrv, struct imu_data *pgyro, struct imu_data* paccel, float* ptemp)
{
    uint8_t rd_out[14];
    int16_t raw;
    prv_imu_read(pdrv, UB0_REG_TEMP_DATA1, rd_out, sizeof(rd_out));//42688 1d
    raw = (((int8_t)rd_out[0]) << 8) | (rd_out[1]);
   *ptemp =  (raw / 132.48f) + 25.0f;

    raw = (((int8_t)rd_out[2]) << 8) | (rd_out[3]);
    paccel->raw_x = raw;
    paccel->x = LSB_ACC_16G * raw;
    raw = (((int8_t)rd_out[4]) << 8) | (rd_out[5]);
    paccel->raw_y = raw;
    paccel->y = LSB_ACC_16G * raw;
    raw = (((int8_t)rd_out[6]) << 8) | (rd_out[7]);
    paccel->raw_z = raw;
    paccel->z = LSB_ACC_16G * raw;


    raw = (((int8_t)rd_out[8]) << 8) | (rd_out[9]);
    pgyro->raw_x = raw;
    pgyro->x = LSB_GYRO_2000DPS * raw;
    raw = (((int8_t)rd_out[10]) << 8) | (rd_out[11]);
    pgyro->raw_y = raw;
    pgyro->y = LSB_GYRO_2000DPS * raw;
    raw = (((int8_t)rd_out[12]) << 8) | (rd_out[13]);
    pgyro->raw_z = raw;
    pgyro->z = LSB_GYRO_2000DPS * raw;
    return 0;
}

static int32_t prv_icm45686_getdata(ARM_DRIVER_SPI* pdrv, struct imu_data *pgyro, struct imu_data* paccel, float* ptemp)
{
    uint8_t rd_out[14];
    int16_t raw;
    prv_imu_read(pdrv, 0x00, rd_out, sizeof(rd_out));//45686 0C

    raw = (((int8_t)rd_out[0]) << 8) | (rd_out[1]);
    paccel->raw_x = raw;
    paccel->x = LSB_ACC_16G * raw;
    raw = (((int8_t)rd_out[2]) << 8) | (rd_out[3]);
    paccel->raw_y = raw;
    paccel->y = LSB_ACC_16G * raw;
    raw = (((int8_t)rd_out[4]) << 8) | (rd_out[5]);
    paccel->raw_z = raw;
    paccel->z = LSB_ACC_16G * raw;

    raw = (((int8_t)rd_out[6]) << 8) | (rd_out[7]);
    pgyro->raw_x = raw;
    pgyro->x = LSB_GYRO_2000DPS * raw;
    raw = (((int8_t)rd_out[8]) << 8) | (rd_out[9]);
    pgyro->raw_y = raw;
    pgyro->y = LSB_GYRO_2000DPS * raw;
    raw = (((int8_t)rd_out[10]) << 8) | (rd_out[11]);
    pgyro->raw_z = raw;
    pgyro->z = LSB_GYRO_2000DPS * raw;
    
    raw = (((int8_t)rd_out[12]) << 8) | (rd_out[13]);
    *ptemp =  (raw / 132.48f) + 25.0f;
    return 0;
}

static int32_t prv_icm42688_init(ARM_DRIVER_SPI* drv)
{
    uint8_t reg_val = 0;
    uint8_t reg_stored = 0;

    /* verify AM I */
    prv_imu_read(drv, UB0_REG_WHO_AM_I, &reg_val, sizeof(reg_val));//42688 0X75
    //prv_imu_read(drv, 0x72, &reg_val, sizeof(reg_val));//45686 0x72
    
    assert(reg_val == ICM42688_DEFAULT_ID);    // 0x47 as default ID 42688
   // assert(reg_val == 0xE9);    // 0xE9 as default ID 45686
    
    /* set operation at REG bank 0 */
    reg_val = 0;
    prv_imu_write(drv, REG_BANK_SEL,  &reg_val , sizeof(reg_val));     
    reg_val = 40;    //Stream-to-FIFO Mode(page63)
    prv_imu_write(drv, UB0_REG_FIFO_CONFIG,  &reg_val , sizeof(reg_val)); 

    /* set interrupt  */
    prv_imu_read(drv, UB0_REG_INT_SOURCE0, &reg_stored, sizeof(reg_stored));
    reg_val = 0;
    prv_imu_write(drv, UB0_REG_INT_SOURCE0,  &reg_val ,     sizeof(reg_val)); 
    reg_val = 0;
    prv_imu_write(drv, UB0_REG_FIFO_CONFIG2, &reg_val,      sizeof(reg_val)); 
    reg_val = 0x00;
    prv_imu_write(drv, UB0_REG_FIFO_CONFIG3, &reg_val,      sizeof(reg_val)); 

    prv_imu_write(drv, UB0_REG_INT_SOURCE0,  &reg_stored ,  sizeof(reg_stored)); 
    
    /* Disable the accel and gyro to the FIFO */
    reg_val = 0x00;
    prv_imu_write(drv, UB0_REG_FIFO_CONFIG2, &reg_val,      sizeof(reg_val)); 

    //prv_imu_write(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = 0x00;
    prv_imu_write(drv, UB0_REG_INT_CONFIG,  &reg_val,       sizeof(reg_val));

    //prv_imu_write(ICM42688_REG_BANK_SEL, 0x00);
    //icm42688_readReg(ICM42688_INT_SOURCE0, &reg_val);
    reg_stored = 0; 
    prv_imu_write(drv, UB0_REG_INT_SOURCE0,  &reg_stored ,  sizeof(reg_stored)); 

    /* Acce setting */  
    reg_val = ACCEL_FS_SEL_16G | ACCEL_ODR_SEL_1KHZ;                           
    prv_imu_write(drv, UB0_REG_ACCEL_CONFIG0, &reg_val,     sizeof(reg_val));

    /* GYRO setting */  
    reg_val = GYRO_FS_SEL_2000DPS | GYRO_ODR_SEL_1KHZ;  
    prv_imu_write(drv, UB0_REG_GYRO_CONFIG0,  &reg_val,     sizeof(reg_val));

    /* Power management setting */  
    reg_val = PWR_MGMT0_ACCEL_LN | PWR_MGMT0_GYRO_LN ;  
    prv_imu_write(drv, UB0_REG_PWR_MGMT0,     &reg_val,     sizeof(reg_val));
    return 0;
}




/**
 * ****************************************************
 * 
 * application
 * 
 * ****************************************************
*/
struct imu_data gyro42688;
struct imu_data accel42688;
static void icm42688_task(ULONG thread_input) 
{
    ARM_DRIVER_SPI* drv = (void*)thread_input;

    float           temp;
    uint32_t    pre_ts;

    /* Initialize the SPI driver */
    assert( drv->Initialize(prv_spi_callback) == ARM_DRIVER_OK);
    /* Power up the SPI peripheral */
    assert( drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    /* Configure the SPI to Master, 8-bit mode @1000 kBits/sec */
    assert( drv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8), 1000000) == ARM_DRIVER_OK);

    prv_icm42688_init(drv);

    tx_thread_sleep(10);
    while (1)
    {
        
        // delay for next sample
        tx_thread_sleep(2);
        prv_icm42688_getdata(drv, &gyro42688, &accel42688, &temp);
       // prv_icm45686_getdata(drv, &gyro42688, &accel42688, &temp);
    }
}

/* Define what the initial system looks like.  */
void icm42688_Init(void)
//void adis16507_Init(void)
{
    // rs422_inst.pdrv = pdrv;
    // rs422_inst.baudrate = baudrate;
    // rs422_inst.parity = parity;
    // rs422_inst.stopbits = stopbits;

    tx_event_flags_create(&s_events, "icm42688__events");
    /* Create the application thread.  */
    tx_thread_create(   &app_tcb, 
                        "icm42688_", 
                        icm42688_task, 
                        (uint32_t)&Driver_SPI6, 
                        app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);

}

