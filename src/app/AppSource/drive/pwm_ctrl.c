/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
 threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
 byte pool, and block pool.  */
#include    <stdint.h>
#include    <stdbool.h>
#include    <string.h>
#include    <assert.h>

#include    "tx_api.h"
#include    "stm32h7xx_hal.h"

extern void Error_Handler(void);
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define TIM_CLK_PRESCALER   (23)  // MCU主频240MHz，分频24，得到TIM主频是10MHz
#define TIM_CLK_FREQ        (10000000)//  240MHz/（TIM_CLK_PRESCALER + 1）= 240MHz/（23 + 1） = 10MHz

struct pwm_channel
{
    TIM_HandleTypeDef  * phtim;
    uint32_t             chn;
    uint16_t             duty;
    bool                 enabled;
};
struct pwm_channel  s_pwm_chn[] = 
{
    {&htim3, TIM_CHANNEL_1, 0, false},
    {&htim3, TIM_CHANNEL_2, 0, false},
    {&htim3, TIM_CHANNEL_3, 0, false},
    {&htim3, TIM_CHANNEL_4, 0, false},
    {&htim4, TIM_CHANNEL_1, 0, false},
    {&htim4, TIM_CHANNEL_2, 0, false},
    {&htim4, TIM_CHANNEL_3, 0, false},
    {&htim4, TIM_CHANNEL_4, 0, false},
    {&htim5, TIM_CHANNEL_1, 0, false},
    {&htim5, TIM_CHANNEL_2, 0, false},
    {&htim5, TIM_CHANNEL_3, 0, false},
    {&htim5, TIM_CHANNEL_4, 0, false}
};

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(uint32_t period)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
// 1. 基本定时器初始化（但不启动）
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM_CLK_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period;  //
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  // 2. 直接初始化PWM通道
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 3. 配置PWM通道
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 15000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. 启动PWM输出
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(uint32_t period)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  // 1. 基本定时器初始化（但不启动）
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM_CLK_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // 2. 直接初始化PWM通道
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 3. 配置PWM通道
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 15000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. 启动PWM输出
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	//MML舵机16
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	//MML舵机16

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(uint32_t period)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = TIM_CLK_PRESCALER;   // clock 240MHz
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = period;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = 10;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
//   if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//   {
//     Error_Handler();
//   }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

int pwm_ctrl_init(unsigned int pwm_freq)
{
	uint32_t period;  // ARR

	if(pwm_freq == 333)	// 特殊除不尽的数字处理
	{
		// 舵机精度为0，01°，对应0.1us
		// 需要让计数器的精度为0.1us
		// 得到计数器的时钟至少为	1/0.1us = 10MHz
		// ARR = ( 计数器时钟 / PWM频率 ) - 1 = (10MHz / 333Hz) - 1 ≈ 30029
		period = 30029;
	}
	else
	{
		period = (TIM_CLK_FREQ/pwm_freq) - 1;
	}

    MX_TIM3_Init(period);	//MML舵机16
    MX_TIM4_Init(period);
    return 0;
};

int pwm_ctrl_pulse(unsigned char chn, int pulse_width)
{
    struct pwm_channel *pchn;
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t duty_int;

    if(chn >= 12)
    {
        return -1;
    }
    
    pchn = &s_pwm_chn[chn];

    duty_int = pulse_width/10;
    if(duty_int == 0)
    {   // disable pwm output
        HAL_TIM_PWM_Stop(pchn->phtim, pchn->chn) ;
    }
    else
    {
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = duty_int;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
        HAL_TIM_PWM_ConfigChannel(pchn->phtim, &sConfigOC, pchn->chn);
        HAL_TIM_PWM_Start(pchn->phtim, pchn->chn);
    }
    pchn->duty = duty_int;
    return 0;
}
