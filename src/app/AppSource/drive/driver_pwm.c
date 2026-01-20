#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_gpio.h"

#include "driver_pwm.h"
#include <string.h>

#define MCU_MAIN_FREQ 240000000

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim17;

/** 舵机和TIM的映射表 */
typedef struct {
    TIM_HandleTypeDef* htim;     // TIM句柄
    uint32_t channel;           // TIM通道
    GPIO_TypeDef* gpio_port;    // GPIO端口
    uint16_t gpio_pin;          // GPIO引脚
    uint16_t alternate_func;    // GPIO复用功能
} Servo_Hardware_Mapping_t;

/** 全局硬件映射表 */
static const Servo_Hardware_Mapping_t hardware_map[PWM_CH_MAX] = {
    [SERVO_PWM1] = {&htim3, TIM_CHANNEL_1, GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3},
    [SERVO_PWM6] = {&htim4, TIM_CHANNEL_2, GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4},
    [SERVO_PWM7] = {&htim4, TIM_CHANNEL_3, GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4},
    [ECU_PWM8]  = {&htim17, TIM_CHANNEL_1, GPIOB, GPIO_PIN_9, GPIO_AF1_TIM17},
};

/** PWM通道状态结构体，针对ST的专属配置 */
typedef struct {
    bool initialized;
    PWM_Config_t config;
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
        pchn->timer_clock_hz = MCU_MAIN_FREQ;
    } else {
        // TIM3/TIM4挂载在APB1上
        pchn->timer_clock_hz = MCU_MAIN_FREQ;
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

/** 开启时钟 */
static void prv_MX_GPIO_Init(Servo_ID_t servo_id)
{
    switch (servo_id)
    {
    case SERVO_PWM1:
        __HAL_RCC_GPIOC_CLK_ENABLE();
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
}

/** 计算PSC和ARR */
static bool prv_calculate_timer_psc_arr(Servo_ID_t servo_id)
{
    s_channel_states[servo_id].prescaler = MCU_MAIN_FREQ * s_channel_states[servo_id].config.resolution_us / 1000000 - 1;//240-1 = 239
    s_channel_states[servo_id].period = (MCU_MAIN_FREQ / (s_channel_states[servo_id].prescaler + 1) )/ s_channel_states[servo_id].config.frequency_hz - 1;//240Mhz/240/333 = 3003 so ARR=3002
    return true;
}

/** 配置定时器参数 */
static void prv_MX_TIM_Init(Servo_ID_t servo_id, Servo_Hardware_Mapping_t* hw)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  hw->htim->Init.Prescaler = s_channel_states[servo_id].prescaler;
  hw->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  hw->htim->Init.Period = s_channel_states[servo_id].period;
  hw->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hw->htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(hw->htim) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = s_channel_states[servo_id].config.init_duty_ratio;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(hw->htim, &sConfigOC, hw->channel) != HAL_OK)
  {
    Error_Handler();
  }

  prv_HAL_TIM_MspPostInit(hw);
}

/** 配置管脚 */
static void prv_HAL_TIM_MspPostInit(Servo_Hardware_Mapping_t* hw)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
