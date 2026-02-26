#include "interface_timer.h"
#include "servo_def.h"
#include "tx_api.h"
#include <math.h>
#include <string.h>
#include <assert.h>

struct pulse_servo_status {
    float K1;
};

extern TX_BYTE_POOL byte_pool_0;

// /** 功能：脉宽ms -> 占空比 */
// static void prv_status_init(Pulse_Servo_Config_t *pcfg, struct pulse_servo_status* pstatus)
// {
//     pstatus->K1 = pcfg->frequency_hz / 1000.0f;
// }

#define PULSE_MS_TO_DUTY_RATION( pulse_ms,  K1) (pulse_ms*K1)


/** 初始化脉宽舵机 */
Servo_ErrorCode_t PulseServo_Setup(Servo_ID_t servo_id, float init_pulse_ms)
{
    Pulse_Servo_Config_t *pcfg;
    float K1;
    // struct pulse_servo_status* pstatus = NULL; // []

    pcfg = (Pulse_Servo_Config_t *)g_servo_state[servo_id].config;

    //alloc status resource
    // tx_byte_allocate(&byte_pool_0, (void**)&pstatus, sizeof(struct pulse_servo_status), TX_NO_WAIT);
    // if(pstatus == NULL)
    // {   // no memory
    //     return SERVO_DEVICE_ERR_NO_MEMORY;
    // }
    // initial local status K1
    // prv_status_init(pcfg, pstatus);
    K1 = pcfg->frequency_hz / 1000.0f;
    // pstatus->K1 = pcfg->frequency_hz / 1000.0f;
    // pstatus = &K1;

    // 设置下层的 PWM_Config_t 结构体
    PWM_Config_t pwm_config = {
        .frequency_hz = pcfg->frequency_hz,              //50Hz
        .resolution_us = pcfg->resolution_ms * 1000,    //精度0.1ms对应100us
        .init_duty_ratio = PULSE_MS_TO_DUTY_RATION(init_pulse_ms, K1),  //1ms / 20ms = 0.05
        .min_duty_ratio = PULSE_MS_TO_DUTY_RATION(pcfg->min_pulse_ms, K1),//0.8ms / 20ms = 0.04
        .max_duty_ratio = PULSE_MS_TO_DUTY_RATION(pcfg->max_pulse_ms, K1),//2.2ms / 20ms = 0.11
    };
    // 调用驱动层接口
    if(!Driver_PWM_Init(servo_id, &pwm_config)) {
        // tx_byte_release(pstatus);
        return SERVO_DEVICE_ERR_HARDWARE;
    }

    /** state记录 */
		uint32_t ctx_int = 0;
		memcpy(&ctx_int, &K1, sizeof(K1));
    g_servo_state[servo_id].ctx = (void*)(ctx_int);
    // memcpy(&g_servo_state[servo_id].ctx, &K1, 4);
    g_servo_state[servo_id].initialized = true;    
    g_servo_state[servo_id].current_value = init_pulse_ms;

    return SERVO_DEVICE_OK;
}

/** 功能：设置脉冲宽度
 * 参数1：哪个舵机
 * 参数2：脉冲宽度
 * 返回值：中间层错误码
 * 备注：调用此函数前，需先调用舵机初始化函数 PulseServo_Init
 */
Servo_ErrorCode_t PulseServo_SetPulseWidth(Servo_ID_t servo_id, float pulse_ms)
{
    Pulse_Servo_Config_t *pcfg;
    Servo_ErrorCode_t errCode;
	  float target_pulse = pulse_ms;
    
    // 入参1检查
    errCode= check_servo_id(servo_id);
    if(SERVO_DEVICE_OK != errCode) {return errCode;}
    // 要操作的舵机还未初始化，报错
    if (!g_servo_state[servo_id].initialized) {
        return SERVO_DEVICE_ERR_NOT_INIT;
    }
    pcfg = (Pulse_Servo_Config_t *)g_servo_state[servo_id].config;
    // 入参2检查
    // if((pulse_ms < pcfg->min_pulse_ms) ||
    //    (pulse_ms > pcfg->max_pulse_ms)) {
    //     return SERVO_DEVICE_ERR_OUT_OF_RANGE;
    // }
    if(target_pulse < pcfg->min_pulse_ms)
    {
        target_pulse = pcfg->min_pulse_ms;
    }
    if(target_pulse > pcfg->max_pulse_ms)
    {
        target_pulse = pcfg->max_pulse_ms;
    }

    // 脉宽 -> 占空比
    float ctx_int;
    memcpy(&ctx_int, &g_servo_state[servo_id].ctx, sizeof(g_servo_state[servo_id].ctx));
    float duty_ratio = PULSE_MS_TO_DUTY_RATION(target_pulse, (float)ctx_int);
    if (!Driver_PWM_SetDutyRatio(servo_id, duty_ratio)) {
        return SERVO_DEVICE_ERR_HARDWARE;
    }
    // 更新状态
    g_servo_state[servo_id].current_value = target_pulse;
    return SERVO_DEVICE_OK;
}

/** 反初始化 */
Servo_ErrorCode_t PulseServo_Deinit(Servo_ID_t servo_id)
{
    Servo_ErrorCode_t errCode;
    // 入参1检查
    errCode= check_servo_id(servo_id);
    if(SERVO_DEVICE_OK != errCode) {return errCode;}
    // 要操作的舵机还未初始化，报错
    if (!g_servo_state[servo_id].initialized) {
        return SERVO_DEVICE_ERR_NOT_INIT;
    }
    // 调用驱动层接口
    if (!Driver_PWM_Deinit(servo_id)) {
    return SERVO_DEVICE_ERR_HARDWARE;
    }
    // 更新状态
    g_servo_state[servo_id].initialized = false;
    return SERVO_DEVICE_OK;
}

