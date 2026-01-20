#ifndef INTERFACE_TIMER_H
#define INTERFACE_TIMER_H

#include "driver_pwm.h"
#include <stdint.h>
#include <stdbool.h>

/** 中间层错误码 */
typedef enum {
    SERVO_DEVICE_OK = 0,
    SERVO_DEVICE_ERR_INVALID_PARAM,
    SERVO_DEVICE_ERR_INVALID_ID,
    SERVO_DEVICE_ERR_OUT_OF_RANGE,
    SERVO_DEVICE_ERR_HARDWARE,
    SERVO_DEVICE_ERR_ALREADY_USED,
    SERVO_DEVICE_ERR_NO_MEMORY,
    SERVO_DEVICE_ERR_NOT_INIT,
} Servo_ErrorCode_t;

// ==================== 角度舵机API ====================
/** 初始化角度舵机 */
Servo_ErrorCode_t AngleServo_Init(Servo_ID_t servo_id, float init_angle_deg);  // 333Hz, 0.01度

/** 设置角度 */
Servo_ErrorCode_t AngleServo_SetAngle(Servo_ID_t servo_id, float angle_deg);

/** 反初始化 */
Servo_ErrorCode_t AngleServo_Deinit(Servo_ID_t servo_id);

// ==================== 脉冲舵机API ====================
/** 初始化脉冲舵机 */
Servo_ErrorCode_t PulseServo_Init(Servo_ID_t servo_id, float init_pulse_ms);

/** 设置脉冲宽度 */
Servo_ErrorCode_t PulseServo_SetPulseWidth(Servo_ID_t servo_id, float pulse_ms);

/** 反初始化 */
Servo_ErrorCode_t PulseServo_Deinit(Servo_ID_t servo_id);

#endif // INTERFACE_TIMER_H
