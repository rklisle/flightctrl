

#include <stdint.h>
#include "tx_api.h"
#include "stm32h7xx_hal.h"
#include "./AppSource/drive/ff.h"
#include "sd_flash.h"

static FIL fil[FILE_COUNT] = {0};
static FATFS fs;

SD_HandleTypeDef hsd1;

static TX_MUTEX                 s_mutex;   
static TX_EVENT_FLAGS_GROUP     s_events;

#define     SD_EVENT_RX_DONE        0x01
#define     SD_EVENT_TX_DONE        0x02
#define     SD_EVENT_ERROR          0x10

#define     SD_EVNETS_ALL           (SD_EVENT_RX_DONE | SD_EVENT_TX_DONE | SD_EVENT_ERROR)

#define     Error_Handler()         while(1){;}
/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    tx_event_flags_set( &s_events,
                        SD_EVENT_RX_DONE,
                        TX_OR);
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    tx_event_flags_set( &s_events,
                        SD_EVENT_TX_DONE,
                        TX_OR);
}

/**
  * @brief SD error callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    tx_event_flags_set( &s_events,
                        SD_EVENT_ERROR,
                        TX_OR);
}
/**
  * @brief  Wait SD Card ready status
  * @param  None
  * @retval None
  */
static uint8_t prv_wait_sd_ready(uint32_t to_ms)
{
  uint32_t loop = to_ms;
  
  /* Wait for the Erasing process is completed */
  /* Verify that SD card is ready to use after the Erase */
  while(loop > 0)
  {
    loop--;
    if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
    {
        return HAL_OK;
    }
    tx_thread_sleep(1);
  }
  return HAL_ERROR;
}


/**
 * @brief  nor read data
 *@param  flash_address
 *@param  destination
 *@param  words
 *@return UINT
 */
int32_t  sd_read(uint32_t block_addr, uint8_t *dest, uint32_t size)
{
    uint32_t events = 0;
    if((size & (BLOCKSIZE - 1 )) != 0)
    {
        return -1;
    }

    tx_mutex_get(&s_mutex, TX_WAIT_FOREVER);

    tx_event_flags_get( &s_events,
                        SD_EVNETS_ALL,
                        TX_OR_CLEAR,
                        &events,
                        0);
    /*##- 7 - Initialize Reception buffer #####################*/
    if(HAL_SD_ReadBlocks_DMA(&hsd1, dest, block_addr, size/BLOCKSIZE) != HAL_OK)
    {
        Error_Handler();
    }
    // wait rx complete
    tx_event_flags_get( &s_events,
                        (SD_EVENT_RX_DONE | SD_EVENT_ERROR),
                        TX_OR_CLEAR,
                        &events,
                        3000);
    if(events & SD_EVENT_ERROR)
    {
        Error_Handler();
    }
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t*)dest, size);
    }

    tx_mutex_put(&s_mutex);
    return size;
}

/**
 * @brief  write data to nor
 *@param  flash_address
 *@param  source
 *@param  words
 *@return UINT
 */
int32_t  sd_write(uint32_t block_addr, const uint8_t *source, uint32_t size)
{
    uint32_t events = 0;
    if((size & (BLOCKSIZE - 1 )) != 0)
    {
        return -1;
    }

    tx_mutex_get(&s_mutex, TX_WAIT_FOREVER);

    tx_event_flags_get( &s_events,
                        SD_EVNETS_ALL,
                        TX_OR_CLEAR,
                        &events,
                        0);

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t*)source, size);
    }
    
    if (HAL_SD_GetState(&hsd1) != HAL_SD_STATE_READY)
    {
        return -1;
    }
    if(HAL_SD_WriteBlocks_DMA(&hsd1, source, block_addr, size/BLOCKSIZE) != HAL_OK)
    {
        Error_Handler();
    }
    // wait tx complete
    int result = tx_event_flags_get( &s_events,
                        (SD_EVENT_TX_DONE | SD_EVENT_ERROR),
                        TX_OR_CLEAR,
                        &events,
                        3000);
    if(result == TX_SUCCESS)
    {
        int a = 0;
    }
    if(result == TX_NO_EVENTS)
    {
        int a = 0;
    }
    if(result == TX_WAIT_ABORTED)
    {
        int a = 0;
    }
    if(events & SD_EVENT_ERROR)
    {
        Error_Handler();
    }
    // wait program done
    if(prv_wait_sd_ready(1000) != HAL_OK)
    {
        Error_Handler();
    }

    tx_mutex_put(&s_mutex);
    return size;
}

int32_t sd_block_erase(uint32_t block_addr, uint32_t erase_blk_size)
{

    tx_mutex_get(&s_mutex, TX_WAIT_FOREVER);

    if(HAL_SD_Erase(&hsd1, block_addr, block_addr+erase_blk_size) != HAL_OK)
    {
        Error_Handler();
    }

    if(prv_wait_sd_ready(1000) != HAL_OK)
    {
        Error_Handler();
    }
    tx_mutex_put(&s_mutex);

    return (int32_t )erase_blk_size;
}

int32_t  sd_flash_init(TX_BYTE_POOL *pmem)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_SD_CardCIDTypedef pCID;
    HAL_SD_CardCSDTypedef pCSD;
    /*##- 1- power on sd card power #####################*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // power on SD
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    /*##- 2 - create mutex and event group #####################*/
    tx_mutex_create(&s_mutex, "sd_mutex", TX_INHERIT);
    tx_event_flags_create(&s_events, "sd_event");

    /*##-3- Initialize SD instance #####################*/
    hsd1.Instance = SDMMC1;
    HAL_SD_DeInit(&hsd1);
        
    /* if CLKDIV = 0 then SDMMC Clock frequency = SDMMC Kernel Clock
        else SDMMC Clock frequency = SDMMC Kernel Clock / [2 * CLKDIV]. 
        SDMMC Kernel Clock = 120MHz, SDMMC Clock frequency = 60MHz  */
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_FALLING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 1;

    if(HAL_SD_Init(&hsd1) != HAL_OK)
    {
        return 1;
        //while(1){;};
    }
    
    HAL_SD_GetCardCID(&hsd1, &pCID);
    HAL_SD_GetCardCSD(&hsd1, &pCSD);
    
    
    FRESULT res = f_mount(&fs, "0:/", 0);  
    if (res != FR_OK) {
        return 2;  // ????
    }
    return 0;
}

int fatFsFileOpen(char *name, int adcId)
{
	FRESULT res;

	res = f_open(&fil[adcId], name, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if(res != FR_OK)
	{
		return 1;
	}
	return 0;
}

int fatFsFileWrite(int adcId, char *dataBuf, int dataLen)
{
	FRESULT res;
	unsigned int br;

	res = f_write(&fil[adcId], dataBuf, dataLen, &br) ;
	if(res != FR_OK)
	{
		return 1;
	}
	f_sync(&fil[adcId]);
	return 0;
}
