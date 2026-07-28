#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_gpio.h"

#include "driver_pwm.h"
#include <string.h>

extern void Error_Handler(void);
extern uint32_t SystemCoreClock;
#define MCU_MAIN_FREQ SystemCoreClock//240000000

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim17;

/** 舵机和TIM的映射表 */
typedef struct {
    TIM_HandleTypeDef* htim;     // TIM句柄
    TIM_TypeDef* TIMx;
    uint32_t channel;           // TIM通道
    GPIO_TypeDef* gpio_port;    // GPIO端口
    uint16_t gpio_pin;          // GPIO引脚
    uint16_t alternate_func;    // GPIO复用功能
} Servo_Hardware_Mapping_t;

/** 全局硬件映射表 */
#ifdef SERVO_CAN
// 只需要定义2个翅膀的舵机，1个开伞舵机，1个驱动发动机用的PWM
static const Servo_Hardware_Mapping_t hardware_map[PWM_CH_MAX] = {
    [SERVO_PWM7] = {&htim4, TIM4, TIM_CHANNEL_3, GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4},
    [ECU_PWM8]  = {&htim17, TIM17, TIM_CHANNEL_1, GPIOB, GPIO_PIN_9, GPIO_AF1_TIM17},
};
#endif

#ifdef SERVO_PWM
// 需要定义全部6个舵机，1个开伞舵机，1个驱动发动机用的PWM
static const Servo_Hardware_Mapping_t hardware_map[PWM_CH_MAX] = {
    [SERVO_PWM1] = {&htim3, TIM3,  TIM_CHANNEL_1, GPIOC, GPIO_PIN_6,  GPIO_AF2_TIM3},
    [SERVO_PWM2] = {&htim3, TIM3,  TIM_CHANNEL_2, GPIOC, GPIO_PIN_7,  GPIO_AF2_TIM3},
    [SERVO_PWM3] = {&htim3, TIM3,  TIM_CHANNEL_3, GPIOB, GPIO_PIN_0,  GPIO_AF2_TIM3},
    [SERVO_PWM4] = {&htim3, TIM3,  TIM_CHANNEL_4, GPIOB, GPIO_PIN_1,  GPIO_AF2_TIM3},
    [SERVO_PWM5] = {&htim4, TIM4,  TIM_CHANNEL_1, GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4},
    [SERVO_PWM6] = {&htim4, TIM4,  TIM_CHANNEL_2, GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4},
    [SERVO_PWM7] = {&htim4, TIM4,  TIM_CHANNEL_3, GPIOB, GPIO_PIN_8,  GPIO_AF2_TIM4},
    [ECU_PWM8]  = {&htim17, TIM17, TIM_CHANNEL_1, GPIOB, GPIO_PIN_9,  GPIO_AF1_TIM17},
};
#endif

/** PWM通道状态结构体，针对ST的专属配置 */
typedef struct {
    bool initialized;
    PWM_Config_t config;
    uint32_t timer_input_clock_hz;
    uint32_t timer_clock_hz;     // 定时器时钟频率(Hz)
    uint32_t prescaler;          // 预分频值
    uint32_t period;             // 自动重载值(ARR)
    float current_duty_ratio;    // 当前占空比
} PWM_Channel_State_t;

/** PWM通道状态实例化 */
static PWM_Channel_State_t s_channel_states[PWM_CH_MAX] = {0};

/** 函数声明 */

static void prv_MX_GPIO_Init(Servo_ID_t servo_id);
static bool prv_calculate_timer_psc_arr(Servo_ID_t servo_id);
static void prv_MX_TIM_Init(Servo_ID_t servo_id, Servo_Hardware_Mapping_t* hw);
static void prv_HAL_TIM_MspPostInit(Servo_Hardware_Mapping_t* hw);
static void prv_set_channel_ccr(Servo_ID_t servo_id, float duty_ratio);

/** 功能：初始化
 * 参数1：哪个舵机
 * 参数2：底层配置结构体
 */
bool Driver_PWM_Init(Servo_ID_t servo_id, PWM_Config_t* pconfig)
{
    Servo_Hardware_Mapping_t* hw;
    PWM_Channel_State_t* pchn;

    // 基础参数检查
    if (servo_id < 0 || servo_id >= PWM_CH_MAX || pconfig == NULL) {
        return false;
    }
    
    prv_MX_GPIO_Init(servo_id);

    hw = &hardware_map[servo_id];
    if (hw->htim == NULL) 
    {
        return false;
    }
    
    pchn = &s_channel_states[servo_id];
    // 保存配置
    //s_channel_states[servo_id].config = *pconfig;
    memcpy( &pchn->config,
            pconfig,
            sizeof(PWM_Config_t));

    // 获取定时器时钟频率
    if (servo_id == ECU_PWM8) {
        // TIM17挂载在APB2上
        s_channel_states[servo_id].timer_input_clock_hz = HAL_RCC_GetPCLK2Freq() * 2;
    } else {
        // TIM3/TIM4挂载在APB1上
        s_channel_states[servo_id].timer_input_clock_hz = HAL_RCC_GetPCLK1Freq() * 2;
    }

    // 计算定时器参数
    if (!prv_calculate_timer_psc_arr(servo_id)){
        return false;
    }

    prv_MX_TIM_Init(servo_id, hw);
    
    HAL_TIM_PWM_Start(hw->htim, hw->channel);
    
    pchn->initialized = true;
    pchn->current_duty_ratio = pchn->config.init_duty_ratio;

    return true;
}

/** 设置占空比
 * 参数2：占空比，取值0~1
 */
bool Driver_PWM_SetDutyRatio(Servo_ID_t servo_id, float duty_ratio)
{
    PWM_Channel_State_t* pchn;
    pchn = &s_channel_states[servo_id];
    // 入参检查
    if (servo_id < 0 || servo_id >= PWM_CH_MAX) { return false; }
    if(!pchn->initialized) { return false; }
    // 边界检查
    if (duty_ratio < pchn->config.min_duty_ratio) {
        duty_ratio = pchn->config.min_duty_ratio;
    }
    if (duty_ratio > pchn->config.max_duty_ratio) {
        duty_ratio = pchn->config.max_duty_ratio;
    }
    prv_set_channel_ccr(servo_id, duty_ratio);
    pchn->current_duty_ratio = duty_ratio;
    return true;
}

/** 反初始化
 * 功能：PWMStop
 * 参数：哪个舵机
 */
bool Driver_PWM_Deinit(Servo_ID_t servo_id)
{
    const Servo_Hardware_Mapping_t* hw ;

    if (servo_id < 0 || servo_id >= PWM_CH_MAX || !s_channel_states[servo_id].initialized) 
    {
        return false;
    }
    hw = &hardware_map[servo_id];
    
    HAL_TIM_PWM_Stop(hw->htim, hw->channel);
    
    s_channel_states[servo_id].initialized = false;
    
    return true;
}

/** 开启GPIO时钟 */
static void prv_MX_GPIO_Init(Servo_ID_t servo_id)
{
#ifdef SERVO_CAN
    switch (servo_id)
    {
    case SERVO_PWM7:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case ECU_PWM8:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    default:
        break;
    }
#endif

#ifdef SERVO_PWM
    switch (servo_id)
    {
    case SERVO_PWM1:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case SERVO_PWM2:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case SERVO_PWM3:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case SERVO_PWM4:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case SERVO_PWM5:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case SERVO_PWM6:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case SERVO_PWM7:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case ECU_PWM8:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    default:
        break;
    }
#endif

}

/** 计算PSC和ARR */
static bool prv_calculate_timer_psc_arr(Servo_ID_t servo_id)
{
    s_channel_states[servo_id].prescaler = s_channel_states[servo_id].timer_input_clock_hz * s_channel_states[servo_id].config.resolution_us / 1000000 - 1;//240-1 = 239
    s_channel_states[servo_id].period = (s_channel_states[servo_id].timer_input_clock_hz / (s_channel_states[servo_id].prescaler + 1) )/ s_channel_states[servo_id].config.frequency_hz - 1;//240Mhz/240/333 = 3003 so ARR=3002
    return true;
}

/** 配置定时器参数 */
static void prv_MX_TIM_Init(Servo_ID_t servo_id, Servo_Hardware_Mapping_t* hw)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  hw->htim->Instance = hw->TIMx;
  hw->htim->Init.Prescaler = s_channel_states[servo_id].prescaler;
  hw->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  hw->htim->Init.Period = s_channel_states[servo_id].period;
  hw->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hw->htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(hw->htim) != HAL_OK)
  {
    Error_Handler();
  }
    if(hw->htim != &htim17)
    {
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(hw->htim, &sClockSourceConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
  if (HAL_TIM_PWM_Init(hw->htim) != HAL_OK)
  {
    Error_Handler();
  }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)(s_channel_states[servo_id].config.init_duty_ratio * (s_channel_states[servo_id].period + 1));
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(hw->htim, &sConfigOC, hw->channel) != HAL_OK)
  {
    Error_Handler();
  }

  prv_HAL_TIM_MspPostInit(hw);
}

// void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
// {
//   if(tim_baseHandle->Instance==TIM3)
//   {
//     __HAL_RCC_TIM3_CLK_ENABLE();
//   }
//   else if(tim_baseHandle->Instance==TIM4)
//   {
//     __HAL_RCC_TIM4_CLK_ENABLE();
//   }
//   else if(tim_baseHandle->Instance==TIM17)
//   {
//     __HAL_RCC_TIM17_CLK_ENABLE();
//   }
// }

/** 配置管脚 */
static void prv_HAL_TIM_MspPostInit(Servo_Hardware_Mapping_t* hw)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch (hw->gpio_pin)
    {
    case GPIO_PIN_6:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case GPIO_PIN_13:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    default:
        __HAL_RCC_GPIOB_CLK_ENABLE();   // PIN8 AND PIN9
        break;
    }
    GPIO_InitStruct.Pin = hw->gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = hw->alternate_func;
    HAL_GPIO_Init(hw->gpio_port, &GPIO_InitStruct);
}

/** 设置占空比 */
static void prv_set_channel_ccr(Servo_ID_t servo_id, float duty_ratio) {
    const Servo_Hardware_Mapping_t* hw = &hardware_map[servo_id];
    
    uint32_t ccr_value = (uint32_t)(duty_ratio * (s_channel_states[servo_id].period + 1));
    
    __HAL_TIM_SET_COMPARE(hw->htim, hw->channel, ccr_value);
}
