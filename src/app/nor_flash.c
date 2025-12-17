

#include <stdint.h>
#include "tx_api.h"
#include "Driver_SPI.h"

// NOR configuration define
#define NOR_SECTOR_SIZE             (512)
#define NOR_BLOCK_SIZE              (32*1024)
#define NOR_TOTAL_SIZE              (16*1024*1024)
#define NOR_PROGRAM_PAGE_SIZE       (256)

#define NOR_BUSY_FLAG               (0x01)

#define NOR_SPI_ACCESS_TIMEOUT_MS   (500)
#define NOR_PROGRAM_TIMEOUT_MS      (1000)

/* SPI Driver */
static ARM_DRIVER_SPI* s_SPIdrv = NULL;
/* nor sem */
static  TX_SEMAPHORE    s_nor_sem;

static void prv_SPI_callback(uint32_t event)
{
    if( (event & ARM_SPI_EVENT_TRANSFER_COMPLETE) !=  0)
    {    /* Success: Wakeup Thread */
        tx_semaphore_ceiling_put(&s_nor_sem, 1);
    }
}
/**
 * @brief  read nor jedec id
 *@param  pjedec
 *@return int32_t
 */
static  int32_t  nor_JEDECID_read(uint32_t* pjedec)
{
    uint32_t        status;
    uint32_t    rdout = 0xABCDEF68;
    uint8_t     cmd[4] = {0x9F,0x00,0x00,0x00}; //JEDEC ID read command(9Fh)

    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    s_SPIdrv->Transfer(cmd, &rdout, sizeof(cmd));

    status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);

    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    //
    if(status == TX_SUCCESS)
    {
        *pjedec = (rdout >> 8);
    }

    return (status == TX_SUCCESS) ? 0 : -1;
}
/**
 * @brief  wait chip to idle state
 *@param  to_ms
 *@return int32_t
 */
static int32_t   prv_nor_wait_idle( uint32_t to_ms)
{
    int32_t     ret = 0;
    uint32_t        status;
    uint8_t     cmd[2] = {0x05, 0x00};
    uint8_t     rdout[2];
    uint32_t    delay_tick = to_ms;

    do
    {
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
        s_SPIdrv->Transfer(cmd, &rdout, sizeof(cmd));
        status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

        // check error
        if(status != TX_SUCCESS)
        {
            ret = -1;
            break;
        }
        // busy flag
        if( (rdout[1] & NOR_BUSY_FLAG) == 0)
        {   // chip idle
            ret = 0;
            break;
        }
        // still in busy, sleep 1 ms
        tx_thread_sleep(5);
    } while(delay_tick > 0);

    return ret;
}

static int32_t   prv_nor_write_enable(void)
{
    uint8_t     cmd = 0x06;// Write-Enable command(06H)
    int32_t     ret = 0;

    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    s_SPIdrv->Send(&cmd, sizeof(cmd));

    if(tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS) != TX_SUCCESS)
    {
        ret = -1;
    }
    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    return ret;
}

// static int32_t   prv_nor_setup(void)
// {
//     uint8_t     cmd[2] = {0x06, 0x80}; // Write-Status command(01H) // Set Status Register to with hardware write protect mode
//     int32_t     ret = 0;
//     // enable wirte
//     prv_nor_write_enable();
//
//     s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
//     s_SPIdrv->Send(cmd, sizeof(cmd));

//     if(tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS) != TX_SUCCESS)
//     {
//         ret = -1;
//     }
//     s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

//     return ret;
// }


/**
 * @brief  nor read data
 *@param  flash_address
 *@param  destination
 *@param  words
 *@return UINT
 */
int32_t  nor_read(uint32_t flash_address, uint8_t *dest, uint32_t size)
{
    uint32_t    addr = (uint32_t)flash_address;
    uint32_t    status;
    uint8_t     cmd[5];

    cmd[0] = 0x0B;                          // High-Speed-Read command(0BH)
    cmd[1] = ((addr & 0xFF0000) >> 16);     // address
    cmd[2] = ((addr & 0xFF00) >> 8);
    cmd[3] = (addr & 0xFF);
    cmd[4] = 0xFF;                          // dummy byte
    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    // clear sem
    tx_semaphore_get(&s_nor_sem, 0);
    s_SPIdrv->Send(cmd, sizeof(cmd));

    if(tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS) != TX_SUCCESS)
    {
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
        return -1;
    }
    // no start load data
    s_SPIdrv->Receive(dest, size);
    /* wait done read flash.  */
    status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);

    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    /* Loop to read flash.  */
    if(status != TX_SUCCESS)
    {
        return -1;
    }
    return size;
}

/**
 * @brief  write data to nor
 *@param  flash_address
 *@param  source
 *@param  words
 *@return UINT
 */
int32_t  nor_write(uint32_t flash_address, const uint8_t *source, uint32_t size)
{
    uint8_t     cmd[4];
    uint32_t    status;
    uint32_t    wtsize;
    uint32_t    addr = (uint32_t)flash_address;
    uint32_t    byteLeft = size;
    uint8_t*    pSndbuf = (uint8_t*)source;
    int32_t     ret = 0;

    //cli_printf("nor write addr %08x\r\n", addr);
    /* Loop to write flash.  */
    while (byteLeft > 0)
    {
        // enable write
        ret = prv_nor_write_enable();
        if( ret < 0)
        {
            return ret;
        }

        // How many bytes do we have to write in the current page?
        wtsize = NOR_PROGRAM_PAGE_SIZE - (addr & (NOR_PROGRAM_PAGE_SIZE - 1));

        // How many bytes can we write in the current page?
        wtsize = (wtsize >= byteLeft) ? byteLeft : wtsize;

        cmd[0] = 0x02;                          // Byte-Program command(02H)
        cmd[1] = (uint8_t)((addr & 0xFF0000) >> 16);     // Send Address (3 bytes)
        cmd[2] = (uint8_t)((addr & 0xFF00) >> 8);
        cmd[3] = (uint8_t)(addr & 0xFF);

        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);    // clear sem
        tx_semaphore_get(&s_nor_sem, 0);
        s_SPIdrv->Send(cmd, sizeof(cmd));
        if(tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS) != TX_SUCCESS)
        {
            s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
            return -1;
        }

        s_SPIdrv->Send(pSndbuf, wtsize);
        /* Loop to read flash.  */
        status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);

        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
        // wait operation done
        if( (status != TX_SUCCESS) || (prv_nor_wait_idle(NOR_PROGRAM_TIMEOUT_MS) < 0))
        {
            return -1;
        }

        // Update data length and address for the next page
        byteLeft  -= wtsize;
        addr  += wtsize;
        pSndbuf += wtsize;
    }

    return size;
}

int32_t nor_write_deepseek(uint32_t flash_address, const uint8_t *source, uint32_t size)
{
    uint8_t     cmd[4];
    uint32_t    status;
    uint32_t    wtsize;
    uint32_t    addr = (uint32_t)flash_address;
    uint32_t    byteLeft = size;
    uint8_t*    pSndbuf = (uint8_t*)source;
    int32_t     ret = 0;

    /* Loop to write flash.  */
    while (byteLeft > 0)
    {
        // Enable write
        ret = prv_nor_write_enable();
        if (ret < 0)
        {
            return ret; // ?????
        }

        // Calculate the number of bytes to write in the current page
        wtsize = NOR_PROGRAM_PAGE_SIZE - (addr & (NOR_PROGRAM_PAGE_SIZE - 1));
        wtsize = (wtsize >= byteLeft) ? byteLeft : wtsize;

        // Prepare the write command and address
        cmd[0] = 0x02; // Byte-Program command (02H)
        cmd[1] = (uint8_t)((addr & 0xFF0000) >> 16); // Address high byte
        cmd[2] = (uint8_t)((addr & 0xFF00) >> 8);    // Address middle byte
        cmd[3] = (uint8_t)(addr & 0xFF);             // Address low byte

        // Activate chip select
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

        // Send the write command and address
        s_SPIdrv->Send(cmd, sizeof(cmd));
        if (tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS) != TX_SUCCESS)
        {
            s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
            return -1; // SPI ????
        }

        // Send the data
        s_SPIdrv->Send(pSndbuf, wtsize);
        status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);

        // Deactivate chip select
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

        // Wait for the write operation to complete
        if ((status != TX_SUCCESS) || (prv_nor_wait_idle(NOR_PROGRAM_TIMEOUT_MS) < 0))
        {
            return -1; // SPI ?????????
        }

        // Update data length and address for the next page
        byteLeft -= wtsize;
        addr += wtsize;
        pSndbuf += wtsize;
    }

    return size; // ????
}

int32_t nor_block_erase(uint32_t flash_address, uint32_t erase_size)
{
    uint8_t     cmd[4];
    uint32_t    status;
    uint32_t    erased_size = 0;
    uint32_t    addr = flash_address ;//& (NOR_BLOCK_SIZE - 1);
    uint32_t    dst_addr = (flash_address + erase_size);// & (NOR_BLOCK_SIZE - 1);
    /* Loop to write flash.  */
    while (addr < dst_addr)
    {
        // enable write
        if(prv_nor_write_enable() < 0)
        {
            return -2;
        }
        //cli_printf("nor erase addr %08x\r\n", addr);

        cmd[0] = 0x52;                           // 32K erase command(52H)
        cmd[1] = (uint8_t)((addr & 0xFF0000) >> 16);     // Send Address (3 bytes)
        cmd[2] = (uint8_t)((addr & 0xFF00) >> 8);
        cmd[3] = (uint8_t)(addr & 0xFF);

        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
        s_SPIdrv->Send(cmd, sizeof(cmd));
        status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

        // wait operation done
        if( (status != TX_SUCCESS) || (prv_nor_wait_idle(NOR_PROGRAM_TIMEOUT_MS) < 0))
        {
            return -3;
        }

        addr += NOR_BLOCK_SIZE;
        erased_size += NOR_BLOCK_SIZE;
    }
    return (int32_t )erased_size;
}
int32_t nor_block_erase_deepseek(uint32_t flash_address, uint32_t erase_size) {
    uint8_t cmd[4];
    uint32_t status;
    uint32_t erased_size = 0;
    uint32_t addr = flash_address & ~(NOR_BLOCK_SIZE - 1); // ??? 32KB ??
    uint32_t dst_addr = (flash_address + erase_size + NOR_BLOCK_SIZE - 1) & ~(NOR_BLOCK_SIZE - 1); // ??? 32KB ??

    // ??????
    if (flash_address % NOR_BLOCK_SIZE != 0) {
        return -1; // ?????
    }

    // ????
    while (addr < dst_addr) {
        // ???????
        if (prv_nor_write_enable() < 0) {
            return -2; // ?????
        }

        // ?????????
        cmd[0] = 0x52; // 32KB ?????
        cmd[1] = (uint8_t)((addr & 0xFF0000) >> 16); // ????
        cmd[2] = (uint8_t)((addr & 0xFF00) >> 8);    // ????
        cmd[3] = (uint8_t)(addr & 0xFF);             // ????

        // ?? Flash ??
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

        // ?????????
        s_SPIdrv->Send(cmd, sizeof(cmd));
        status = tx_semaphore_get(&s_nor_sem, NOR_SPI_ACCESS_TIMEOUT_MS);

        // ???? Flash ??
        s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

        // ?? SPI ??????
        if (status != TX_SUCCESS) {
            return -3; // SPI ????
        }

        // ????????
        if (prv_nor_wait_idle(NOR_PROGRAM_TIMEOUT_MS) < 0) {
            return -4; // ??????
        }

        // ??????????
        addr += NOR_BLOCK_SIZE;
        erased_size += NOR_BLOCK_SIZE;
    }

    return (int32_t)erased_size; // ????????
}

int32_t  nor_flash_init(ARM_DRIVER_SPI* pdrv, TX_BYTE_POOL *pmem)
{
    uint32_t    jedec = 0;

    if((s_SPIdrv != NULL) || (pdrv == NULL))
    {   // already inited
        return -1;
    }

    // init sem
    tx_semaphore_create(&s_nor_sem, "spi_access",0);
    s_SPIdrv = pdrv;
    /* Initialize the SPI driver */
    s_SPIdrv->Initialize(prv_SPI_callback);
    /* Power up the SPI peripheral */
    s_SPIdrv->PowerControl(ARM_POWER_FULL);
    /* Configure the SPI to Master, 8-bit mode @10000 kBits/sec */
    s_SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 8000000);
    /* SS line = INACTIVE = HIGH */
    s_SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    // read jedec for testing
    while(jedec != 0x1840c8)
    {
        jedec = 0;
        nor_JEDECID_read(&jedec);
    }
    return 0;
}
