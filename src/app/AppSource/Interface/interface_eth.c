#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_eth.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include "EMAC_STM32H7xx.h"
#include "interface_eth.h"

// 定义全局变量
extern ETH_HandleTypeDef heth;
uint8_t MACAddr[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00}; // 自定义MAC地址

// 写寄存器：通过 HAL_ETH_WritePHYRegister 实现 MDIO 写
HAL_StatusTypeDef LAN9303_WriteReg(uint8_t phy_addr, uint8_t reg_addr, uint32_t data)
{
    return HAL_ETH_WritePHYRegister(&heth, phy_addr, reg_addr, data);
}

// 读寄存器
HAL_StatusTypeDef LAN9303_ReadReg(uint8_t phy_addr, uint8_t reg_addr, uint32_t *data)
{
    return HAL_ETH_ReadPHYRegister(&heth, phy_addr, reg_addr, data);
}

/**
  * @brief 初始化LAN9303i交换机（完整版）
  * @retval HAL状态
  */
HAL_StatusTypeDef LAN9303_Init(void) 
{
    ARM_DRIVER_ETH_MAC *DrvEth = &Driver_ETH_MAC0;
     /* Initialize the SPI driver */
    DrvEth->Initialize(NULL);
    /* Power up the SPI peripheral */
    DrvEth->PowerControl(ARM_POWER_FULL);
    /* Configure the SPI to Master, 8-bit mode @1000 kBits/sec */
 
    /*
    #define RCC_PERIPHCLK_ETH ((uint64_t)(0x0400000000U))
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ETH ;
    PeriphClkInit.EthClockSelection = RCC_ETHCLKSOURCE_RMII;
   // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
   // {
   //     Error_Handler();
   // }
    
    HAL_StatusTypeDef status;

    // ==================== 1. 配置ETH初始化结构体 ====================
    heth.Instance = ETH;
    heth.Init.MACAddr = MACAddr; // 设置MAC地址
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE; // RMII模式
    heth.Init.RxBuffLen = ETH_MAX_PACKET_SIZE; // RX缓冲区长度

    // 初始化ETH外设
    status = HAL_ETH_Init(&heth);
    if (status != HAL_OK) {
        Error_Handler();
    }

    // ==================== 5. 启动ETH通信 ====================
    status = HAL_ETH_Start(&heth);
    if (status != HAL_OK) 
    {
        Error_Handler();
    }

    // ==================== 6. 配置交换机端口 ====================
   // LAN9303_WritePHYReg(&heth, 0x14, 0x2100); // Port1: 100M全双工
   // LAN9303_WritePHYReg(&heth, 0x18, 0x2100); // Port2: 100M全双工
    uint8_t phy_addr = 0x01;
    LAN9303_WriteReg(phy_addr, 0x00, 0x1200);  // 自协商、全双工、100M
    */
    return HAL_OK;
    
}

