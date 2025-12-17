/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>
#include    <assert.h>

#include    "tx_api.h"

#include "pressure_sensor.h"

#include "I2C_STM32H7xx.h"

#define I2C_EVENT_FLAG_ALL  \
                (   ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE | ARM_I2C_EVENT_SLAVE_TRANSMIT |\
                    ARM_I2C_EVENT_SLAVE_RECEIVE | ARM_I2C_EVENT_ADDRESS_NACK       | ARM_I2C_EVENT_GENERAL_CALL    |\
                    ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR       | ARM_I2C_EVENT_BUS_CLEAR)


#define DRIVER_TIMEOUT_MS   2

static TX_EVENT_FLAGS_GROUP     s_events;
/* I2C Signal Event function callback */
static void prv_i2c_event_cb (uint32_t event) 
{ 
    tx_event_flags_set( &s_events,
                        event,
                        TX_OR);
}


static int32_t prv_i2c_read(ARM_DRIVER_I2C* pdrv, uint8_t slv_addr, uint8_t* pdata, uint32_t len)
{
    uint32_t events = 0;
    // clear spi event here
    tx_event_flags_get( &s_events,
                        I2C_EVENT_FLAG_ALL,
                        TX_OR_CLEAR,
                        &events,
                        0);
    // now sendout register address
    assert(ARM_DRIVER_OK == pdrv->MasterReceive(slv_addr, pdata, len, false));
    if(TX_SUCCESS != tx_event_flags_get( &s_events,
                                        I2C_EVENT_FLAG_ALL,
                                        TX_OR_CLEAR,
                                        &events,
                                        DRIVER_TIMEOUT_MS))
    {
        return -1;
    }
    // check event result
    if(events & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
    {   // return actual data size
        return pdrv->GetDataCount();
    }
    return len;
}

static int32_t prv_i2c_write(ARM_DRIVER_I2C* pdrv, uint8_t slv_addr, const uint8_t* pdata, uint32_t len)
{
    uint32_t events = 0;
    // clear spi event here
    tx_event_flags_get( &s_events,
                        I2C_EVENT_FLAG_ALL,
                        TX_OR_CLEAR,
                        &events,
                        0);
    // now sendout register address
    assert(ARM_DRIVER_OK == pdrv->MasterTransmit(slv_addr, pdata, len, false));
    if(TX_SUCCESS != tx_event_flags_get( &s_events,
                                        I2C_EVENT_FLAG_ALL,
                                        TX_OR_CLEAR,
                                        &events,
                                        DRIVER_TIMEOUT_MS))
    {
        return -1;
    }
    // check event result
    if(events & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
    {   // return actual data size
        return pdrv->GetDataCount();
    }
    return len;
}


/*
**************************************************************
*
*     MS4525 operations
*
**************************************************************
*/

#define     MS4525_CHIP_I2C_ADDR       0x28    // I
#define     P_MAX       (1.0f)
#define     P_MIN       (-1.0f)

#define     TEMP_LSB    (200.0f/2048)
#define     TEMP_MIN    -50.0f

#define RAW_CALC_COUNT		(10)
short calibration_raw = 0;
short calibration_raw_calc[RAW_CALC_COUNT] = {0};
int calibration_raw_calc_count = RAW_CALC_COUNT;


static int32_t prv_ms4525_load_data(ARM_DRIVER_I2C* pdrv, float* ppsi, float *ptemp)
{
    uint8_t  recv_buf[4];
    uint16_t raw;
    if(prv_i2c_read(pdrv, MS4525_CHIP_I2C_ADDR, recv_buf, sizeof(recv_buf)) != sizeof(recv_buf))
    {   // i2c operation failed
        return -1;
    }
    // decode pressure
    raw = recv_buf[0] & 0x3F;
    raw = (raw << 8) + recv_buf[1];

    int dp_raw = raw;
    
    
    if(calibration_raw_calc_count > 0)
	{
		calibration_raw_calc[calibration_raw_calc_count - 1] = dp_raw - 8192;
		calibration_raw_calc_count--;
		if(calibration_raw_calc_count == 0)
		{
			long long all = 0;
			for(int i=0;i<RAW_CALC_COUNT;i++)
			{
				all += calibration_raw_calc[i];
			}
			calibration_raw = (all * 1.0)/ RAW_CALC_COUNT;
		}
	}
    dp_raw = dp_raw - calibration_raw;
    
    *ppsi = ((dp_raw - 0.1f * 16383) * (P_MAX - P_MIN) / (0.8f * 16383) + P_MIN);

    raw = recv_buf[2];
    raw = (raw << 3) + ((recv_buf[3] >> 5) & 0x07);

    *ptemp = TEMP_MIN + TEMP_LSB * raw;
    return 0;
}

/*
**************************************************************
*
*     MS56110 operations
*
**************************************************************
*/

#define     MS5611_CHIP_I2C_ADDR        0x77    // I
#define     MS5611_CMD_PROM             0xA0
#define     MS5611_CMD_D1_CONV          0x44    // OSR 1024, t = 2.08ms, RES = 0.027 mbar
#define     MS5611_CMD_D2_CONV          0x52    // OSR 512, t = 1.06ms, RES = 0.008 C
#define     MS5611_CMD_READ_DATA        0x00

#define     MS5611_CMD_SIZE             1
#define     MS5611_DATA_SIZE            3

static int32_t prv_ms5611_load_calib_data(ARM_DRIVER_I2C* pdrv, uint16_t* pcalib)
{
    uint8_t  buf[2];
    uint32_t k;
    // send cmd
    for(k=0;k<8;k++)
    {
        buf[0] = MS5611_CMD_PROM +  (k<<1);
        if(prv_i2c_write(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_CMD_SIZE) != MS5611_CMD_SIZE)
        {   // i2c operation failed
            return -1;
        }
        // then read
        if(prv_i2c_read(pdrv, MS5611_CHIP_I2C_ADDR, buf, sizeof(buf)) != sizeof(buf))
        {   // i2c operation failed
            return -1;
        }
        // copy out
        
        pcalib[k] = (buf[0] << 8) + buf[1];
        //memcpy(&pcalib[k], &buf, sizeof(buf));
        tx_thread_sleep(5);
    }
    return 0;
}

static int32_t prv_ms5611_load_data(ARM_DRIVER_I2C* pdrv, uint32_t* pbar_raw, uint32_t *ptemp_raw)
{
    uint8_t  buf[4] = {0};
    // send D1 cmd
    buf[0] = MS5611_CMD_D1_CONV;
    if(prv_i2c_write(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_CMD_SIZE) != MS5611_CMD_SIZE)
    {   // i2c operation failed
        return -1;
    }
    tx_thread_sleep(3);
    buf[0] = MS5611_CMD_READ_DATA;
    if(prv_i2c_write(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_CMD_SIZE) != MS5611_CMD_SIZE)
    {   // i2c operation failed
        return -1;
    }
    if(prv_i2c_read(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_DATA_SIZE) != MS5611_DATA_SIZE)
    {   // i2c operation failed
        return -1;
    }
   // memcpy(pbar_raw, buf, sizeof(buf));
    *pbar_raw = (buf[0] << 16) + (buf[1]<< 8) + (buf[2]);
    // send D1 cmd
    buf[0] = MS5611_CMD_D2_CONV;
    if(prv_i2c_write(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_CMD_SIZE) != MS5611_CMD_SIZE)
    {   // i2c operation failed
        return -1;
    }
    tx_thread_sleep(2);
    buf[0] = MS5611_CMD_READ_DATA;
    if(prv_i2c_write(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_CMD_SIZE) != MS5611_CMD_SIZE)
    {   // i2c operation failed
        return -1;
    }
    if(prv_i2c_read(pdrv, MS5611_CHIP_I2C_ADDR, buf, MS5611_DATA_SIZE) != MS5611_DATA_SIZE)
    {   // i2c operation failed
        return -1;
    }

    *ptemp_raw = (buf[0] << 16) + (buf[1]<< 8) + (buf[2]);
    //memcpy(ptemp_raw, buf, sizeof(buf));
    return 0;
}

/*
**************************************************************
*
*     sample task
*
**************************************************************
*/
pressure_status_t pressureData;
static void pressure_sensor_task(ULONG thread_input) 
{
    ARM_DRIVER_I2C* pdrv = &Driver_I2C2;
    float psi;
    float temp;
    uint32_t bar_raw = 0;
    uint32_t temp_raw = 0;
    uint16_t calib[8];
    pdrv->Initialize    (prv_i2c_event_cb);
    pdrv->PowerControl  (ARM_POWER_FULL);
    pdrv->Control       (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    pdrv->Control       (ARM_I2C_BUS_CLEAR, 0);
    //pdrv->Receive(rx_buf, sizeof(rx_buf));          /* Get byte from UART */
    prv_ms5611_load_calib_data(pdrv, calib);

    while (1)
    {
        tx_thread_sleep(2);
        prv_ms4525_load_data(pdrv, &psi, &temp);
        tx_thread_sleep(2);
        prv_ms5611_load_data(pdrv, &bar_raw, &temp_raw);
        // TODO: calculate
        
        int d1,d2,c1,c2,c3,c4,c5,c6;
        d1 = bar_raw;
        d2 = temp_raw;
        c1 = calib[1];
        c2 = calib[2];
        c3 = calib[3];
        c4 = calib[4];
        c5 = calib[5];
        c6 = calib[6];
        
        
        float Aux;
//        int rst=-1;
        int dT;
        float TEMP,P;

        double OFF,SENS;
        double T2=0,OFF2=0,SENS2=0;
        
        dT=d2- (c5*256);
        TEMP = 2000 + dT * (((float)c6)/8388608.0);
        
        
        OFF = (((double)c2 )* 65536) + (((double)c4 * dT) )/128;
        SENS = (((double)c1 )* 32768) + (((double)c3 * dT ))/256;
        
        //P =( ( (d1 * ((double)(SENS /2097152))) - OFF)/32768);
        
        P= ((float)(d1/(256.0)))*((float)(SENS/(268435456.0)))- (double)(OFF/32768.0);
        if(TEMP<2000)
        {
            
            T2 = ((double)dT)*((double)dT)/(1<<31);
            
            Aux = (TEMP-2000)*(TEMP-2000);
            
            OFF2 = 2.5*Aux;
            SENS2 =1.25*Aux;	
            
            if(TEMP<-1500)
            {		
                Aux = (TEMP + 1500)*(TEMP + 1500);			
                OFF2 = OFF2 + 7*Aux;
                SENS2 = SENS2 + 5.5*Aux;
            }		
		
        }
        else
        {
             T2=0;
             OFF2=0;
             SENS2=0;
            
        }
            
        SENS = SENS - SENS2;
        OFF = OFF - OFF2;
        TEMP = TEMP - T2;	
        //printf("COMPENSATION %lld,%lld,%d\n",SENS,OFF,TEMP);
        
        P= ((float)(d1/(256.0)))*((float)(SENS/(268435456.0)))- (double)(OFF/32768.0);
        
        pressureData.temperature = (float)TEMP/100.0f;	
        pressureData.abs_pressure = P;
        
        const float PSI_to_Pa = 6894.757f;	
        pressureData.diff_pressure = psi * PSI_to_Pa;
    }
}

int32_t pressure_get_status(pressure_status_t * pout)
{
    *pout = pressureData;
    return 0;
}

/* Define what the initial system looks like.  */

#define     APP_STACK_SIZE         2048
static TX_THREAD   app_tcb;
static uint32_t    app_stack[APP_STACK_SIZE/sizeof(uint32_t)];

int32_t app_pressure_init(TX_BYTE_POOL *pmem)
{
    (void)pmem;
    tx_event_flags_create(&s_events, "sensor_event");
    /* Create the application thread.  */
    tx_thread_create(   &app_tcb, 
                        "sensor", 
                        pressure_sensor_task, 
                        0, 
                        app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 5,
                        TX_MAX_PRIORITIES - 5, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
    return 0;
}

