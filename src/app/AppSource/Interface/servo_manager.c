#include "interface_timer.h"
#include "servo_def.h"
#include <stddef.h>
Servo_State_t g_servo_state[PWM_CH_MAX] = {0}; // [*][*][*][*]

//extern TX_BYTE_POOL byte_pool_0;
static Servo_ErrorCode_t prv_validate_angle_config(Angle_Servo_Config_t *config);
static Servo_ErrorCode_t prv_validate_pulse_config(Pulse_Servo_Config_t *config);

extern Servo_ErrorCode_t AngleServo_Setup(Servo_ID_t servo_id, float init_angle_deg);  // 333Hz, 0.01度
extern Servo_ErrorCode_t PulseServo_Setup(Servo_ID_t servo_id, float init_pulse_ms);

Servo_ErrorCode_t check_servo_id(Servo_ID_t servo_id);

// 根据协议，配置 Angle_Servo_Config_t 结构体
static const Angle_Servo_Config_t s_angle_config = {
    .frequency_hz = 333.0f,
    .resolution_deg = 0.1f,      // 0.1度精度
    .min_angle_deg = -50.0f,
    .max_angle_deg = 50.0f,
    .min_pulse_us = 1000.0f,      // -50度对应1000us
    .max_pulse_us = 2000.0f,      // 50度对应2000us
};

// 根据协议，配置 Pulse_Servo_Config_t 结构体
static const Pulse_Servo_Config_t s_pulse_config = {
    .frequency_hz = 50.0f,
    .resolution_ms = 0.1f,      // 0.1ms精度
    .min_pulse_ms = 0.8f,      // 0.8ms
    .max_pulse_ms = 2.2f,      // 2.2ms
};

/** 初始化角度舵机 */
Servo_ErrorCode_t AngleServo_Init(Servo_ID_t servo_id, float init_angle_deg)  // 333Hz, 0.01度
{
    Servo_ErrorCode_t errCode;
    Angle_Servo_Config_t* pcfg = NULL; // []
    
    pcfg = &s_angle_config;

    // 参数检查
    errCode = prv_validate_angle_config(pcfg);
    if(SERVO_DEVICE_OK != errCode) {return errCode;}

    // 入参检查
    errCode= check_servo_id(servo_id);
    if(SERVO_DEVICE_OK != errCode) {
        return errCode;
    }

    if((init_angle_deg < pcfg->min_angle_deg)||
       (init_angle_deg > pcfg->max_angle_deg)) {
        return SERVO_DEVICE_ERR_OUT_OF_RANGE;
    }

    if(g_servo_state[servo_id].initialized == true)
    {   // already used 
        return SERVO_DEVICE_ERR_ALREADY_USED; 
    }
    
    // alloc resource
    // tx_byte_allocate(&byte_pool_0, (void**)&pcfg, sizeof(Angle_Servo_Config_t), TX_NO_WAIT);
    // if(pcfg == NULL)
    // {   // no memory
    //     return SERVO_DEVICE_ERR_NO_MEMORY;
    // }
    //
    // memcpy(pcfg, &s_angle_config, sizeof(s_angle_config));

    /** 用state记录 Angle_Servo_Config_t 结构体 */
    g_servo_state[servo_id].ID = servo_id;
    g_servo_state[servo_id].config = pcfg;

    /** 继续调下面的执行过程 */
    return AngleServo_Setup(servo_id, init_angle_deg); 
}

/** 初始化脉冲舵机 */
Servo_ErrorCode_t PulseServo_Init(Servo_ID_t servo_id, float init_pulse_ms)
{
    Servo_ErrorCode_t errCode;
    Pulse_Servo_Config_t* pcfg = NULL;

    pcfg = &s_pulse_config;

    // 参数检查
    errCode = prv_validate_pulse_config(pcfg);
    if(SERVO_DEVICE_OK != errCode) {return errCode;}

    // 入参检查
    errCode= check_servo_id(servo_id);
    if(SERVO_DEVICE_OK != errCode) {
        return errCode;
    }
    
    if((init_pulse_ms < pcfg->min_pulse_ms)||
       (init_pulse_ms > pcfg->max_pulse_ms)) {
        return SERVO_DEVICE_ERR_OUT_OF_RANGE;
    }

    if(g_servo_state[servo_id].initialized == true)
    {   // already used 
        return SERVO_DEVICE_ERR_ALREADY_USED; 
    }
    /** 用state记录 Angle_Servo_Config_t 结构体 */
    g_servo_state[servo_id].ID = servo_id;
    g_servo_state[servo_id].config = pcfg;

    /** 继续调下面的执行过程 */
    return PulseServo_Setup(servo_id, init_pulse_ms); 
}

/** 检查 Angle_Servo_Config_t 参数 */
static Servo_ErrorCode_t prv_validate_angle_config(Angle_Servo_Config_t *config)
{
    if (config == NULL) return SERVO_DEVICE_ERR_INVALID_PARAM;
    // 检查必须参数
    if (config->frequency_hz <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->resolution_deg <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->max_angle_deg <= config->min_angle_deg) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->min_pulse_us <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->max_pulse_us <= config->min_pulse_us) return SERVO_DEVICE_ERR_INVALID_PARAM;
    return SERVO_DEVICE_OK;
}

/** 检查 Pulse_Servo_Config_t 参数 */
static Servo_ErrorCode_t prv_validate_pulse_config(Pulse_Servo_Config_t *config)
{
    if (config == NULL) return SERVO_DEVICE_ERR_INVALID_PARAM;
    // 检查必须参数
    if (config->frequency_hz <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->resolution_ms <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->min_pulse_ms <= 0.0f) return SERVO_DEVICE_ERR_INVALID_PARAM;
    if (config->max_pulse_ms <= config->min_pulse_ms) return SERVO_DEVICE_ERR_INVALID_PARAM;
    return SERVO_DEVICE_OK;
}

/** 检查ID */
Servo_ErrorCode_t check_servo_id(Servo_ID_t servo_id)
{
    if (servo_id < 0 || servo_id >= PWM_CH_MAX) {
        return SERVO_DEVICE_ERR_INVALID_ID;
    }
    return SERVO_DEVICE_OK;
}
