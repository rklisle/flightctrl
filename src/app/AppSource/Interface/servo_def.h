#ifndef SERVO_DEF_H
#define SERVO_DEF_H

#include <stdint.h>
#include <stdbool.h>

/** ==================== 角度舵机配置 ==================== */
typedef struct {
    float frequency_hz;          // PWM频率(Hz) - 必须配置        333
    float resolution_deg;        // 角度分辨率(度) - 必须配置      0.1
    float min_angle_deg;         // 最小角度(度) - 必须配置        -50
    float max_angle_deg;         // 最大角度(度) - 必须配置        +50
    float min_pulse_us;          // 最小角度对应脉冲(us) - 可配置       1000
    float max_pulse_us;          // 最大角度对应脉冲(us) - 可配置       2000
} Angle_Servo_Config_t;

/** ==================== 脉冲舵机配置 ==================== */
typedef struct {
    float frequency_hz;          // PWM频率(Hz) - 必须配置          50
    float resolution_ms;         // 时间分辨率(ms) - 必须配置       0.1
    float min_pulse_ms;          // 最小脉冲宽度(ms) - 必须配置     0.8
    float max_pulse_ms;          // 最大脉冲宽度(ms) - 必须配置     2.2
} Pulse_Servo_Config_t;

/** ==================== 舵机状态结构体 ==================== */
typedef struct {
    bool initialized;
    uint8_t ID;
    void *config;
    void *ctx;  // store angle private status information//角度舵机：为记录K1K2的结构体；脉冲舵机：直接记录float（因为float也是4字节，直接类型强转）
    float current_value;//角度，单位°；脉宽，单位ms
} Servo_State_t;

extern Servo_State_t g_servo_state[PWM_CH_MAX];
extern Servo_ErrorCode_t check_servo_id(Servo_ID_t servo_id);

#endif // SERVO_DEF_H
