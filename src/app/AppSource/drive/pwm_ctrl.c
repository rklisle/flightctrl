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

#define TIM_CLK_PRESCALER   (2399)  // 100K @ 240MHz
#define TIM_CLK_FREQ        (100000)

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
uint32_t            s_pwm_period = 0;
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
//  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM_CLK_PRESCALER;   // clock 240MHz to 100K
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period;  //
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
//  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM_CLK_PRESCALER;   // clock 240MHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
    uint32_t period;

    period = (TIM_CLK_FREQ/pwm_freq) - 1;
    MX_TIM3_Init(period);
    MX_TIM4_Init(period);
    MX_TIM5_Init(period);
    s_pwm_period = period;
    return 0;
};

// pwm duty rang 0~100% mapped  0~1000 
int pwm_ctrl_dutyCycle(unsigned char chn, float duty)
{
    struct pwm_channel *pchn;
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t duty_int;

    if(chn >= 12)
    {
        return -1;
    }
    
    pchn = &s_pwm_chn[chn];

    duty_int = (uint32_t)(duty * (float)s_pwm_period);
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
