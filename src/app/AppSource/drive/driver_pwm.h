#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H

//#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/** 舵机编号定义 */
typedef enum {
    SERVO_PWM1 = 0, //左翅膀
    SERVO_PWM6,     //右翅膀
    SERVO_PWM7,     //开伞舵机
    ECU_PWM8,       //发动机启动信号
    PWM_CH_MAX,
} Servo_ID_t;

/** PWM配置结构体 */
typedef struct {
    float frequency_hz;   // PWM频率(Hz)    333
    float resolution_us;     // TIMER的精度（us）  1
    float init_duty_ratio;     // 占空比（0.0~1.0）  初始0.4995
    float min_duty_ratio; // 最小占空比(0.0~1.0)    0.333
    float max_duty_ratio; // 最大占空比(0.0~1.0)    0.666
} PWM_Config_t;

// 驱动层API
bool Driver_PWM_Init(Servo_ID_t servo_id, PWM_Config_t* pconfig);
bool Driver_PWM_SetDutyRatio(Servo_ID_t servo_id, float duty_ratio);
bool Driver_PWM_Deinit(Servo_ID_t servo_id);

#endif  // DRIVER_PWM_H
