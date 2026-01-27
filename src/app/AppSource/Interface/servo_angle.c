#include "interface_timer.h"
#include "servo_def.h"
#include "tx_api.h"
#include <math.h>
#include <string.h>
#include <assert.h>

struct angle_servo_status {
    float K1;
    float K2;
};

extern TX_BYTE_POOL byte_pool_0;

/** 功能：根据协议，将需要的精度，转换成TIMER的精度，即TIMER每计一个数，增加的时间，单位为us
 * 参数1：设备管理层，协议强相关的结构体（里面包含需要转换的精度，比如角度舵机精度可能是0.1°）
 * 返回值：TIMER的精度，us
 */
static float prv_resolution_deg_to_TIMER_resolution(Angle_Servo_Config_t* config)
{
    assert(config != NULL);

    return config->resolution_deg
            * (config->max_pulse_us - config->min_pulse_us)
            / (config->max_angle_deg - config->min_angle_deg);
}

/** 功能：角度 -> 占空比
 *                   根据协议
 * （4）舵机角度为x度 →→→→→→→→→ 占空比(不是百分数) = （1500 + 10x）/3003
 * SetDuty = (SetDeg * (pulse_max_us - pulse_min_us)/(max_angle_deg - min_angle_deg) + pulse_zero_us) * frequency_hz / 1000000
 * 参数1：需要转换的角度
 * 参数2：设备管理层，协议强相关的结构体
 * 返回值：占空比（0.0~1.0）
 */
static void prv_status_init(Angle_Servo_Config_t *pcfg, struct angle_servo_status* pstatus)
{
    float pulse_range_us = pcfg->max_pulse_us - pcfg->min_pulse_us;
    float angle_range_deg = pcfg->max_angle_deg - pcfg->min_angle_deg;
    float pulse_midpoint_us = (pulse_range_us / 2.0f) + pcfg->min_pulse_us;
    float freq_factor = pcfg->frequency_hz / 1000000.0f;
    
    pstatus->K1 = (pulse_range_us / angle_range_deg) * freq_factor;
    pstatus->K2 = pulse_midpoint_us * freq_factor;
}

static inline float prv_deg_to_duty_ratio(float angle_deg, struct angle_servo_status * pstatus)
{
    return angle_deg * pstatus->K1 + pstatus->K2;
}

/** 初始化角度舵机 */
Servo_ErrorCode_t AngleServo_Setup(Servo_ID_t servo_id, float init_angle_deg)  // 333Hz, 0.01度
{
    Angle_Servo_Config_t *pcfg;
    struct angle_servo_status* pstatus = NULL; // []

    pcfg = (Angle_Servo_Config_t *)g_servo_state[servo_id].config;

    //alloc status resource
    tx_byte_allocate(&byte_pool_0, 
                    (void**)&pstatus, 
                    sizeof(struct angle_servo_status), 
                    TX_NO_WAIT);
    if(pstatus == NULL)
    {   // no memory
        return SERVO_DEVICE_ERR_NO_MEMORY;
    }
    // initial local status K1, K2
    prv_status_init(pcfg, pstatus);

    // 设置下层的 PWM_Config_t 结构体
    PWM_Config_t pwm_config = {
        .frequency_hz = pcfg->frequency_hz,              //333Hz
        .resolution_us = prv_resolution_deg_to_TIMER_resolution(pcfg),    //精度0.1°对应1us
        .init_duty_ratio = prv_deg_to_duty_ratio(init_angle_deg, pstatus), //0°对应0.4995
        .min_duty_ratio = prv_deg_to_duty_ratio(pcfg->min_angle_deg, pstatus),//-50°对应0.333
        .max_duty_ratio = prv_deg_to_duty_ratio(pcfg->max_angle_deg, pstatus),//50°对应0.666
    };
    // 调用驱动层接口
    if(!Driver_PWM_Init(servo_id, &pwm_config)) {
        tx_byte_release(pstatus);
        return SERVO_DEVICE_ERR_HARDWARE;
    }

    /** state记录 */
    g_servo_state[servo_id].ctx = pstatus;
    g_servo_state[servo_id].initialized = true;    
    g_servo_state[servo_id].current_value = init_angle_deg;

    return SERVO_DEVICE_OK;
}

/** 功能：调节舵机到多少角度
 * 参数1：哪个舵机
 * 参数2：调节角度
 * 返回值：中间层错误码
 * 备注：调用此函数前，需先调用舵机初始化函数 AngleServo_Init
 */
Servo_ErrorCode_t AngleServo_SetAngle(Servo_ID_t servo_id, float angle_deg)
{
    Angle_Servo_Config_t *pcfg;
    Servo_ErrorCode_t errCode;
    float duty_ratio;
    
    // 入参1检查
    errCode= check_servo_id(servo_id);
    if(SERVO_DEVICE_OK != errCode) 
    {
        return errCode;
    }
    // 要操作的舵机还未初始化，报错
    if (!g_servo_state[servo_id].initialized) 
    {
        return SERVO_DEVICE_ERR_NOT_INIT;
    }
    pcfg = (Angle_Servo_Config_t *)g_servo_state[servo_id].config;
    // 入参2检查
    // if((angle_deg < pcfg->min_angle_deg) ||
    //    (angle_deg > pcfg->max_angle_deg)) 
    // {
    //     return SERVO_DEVICE_ERR_OUT_OF_RANGE;
    // }
    if(angle_deg < pcfg->min_angle_deg)
    {
        angle_deg = pcfg->min_angle_deg;
    }
    if(angle_deg > pcfg->max_angle_deg)
    {
        angle_deg = pcfg->max_angle_deg;
    }
    // 角度 -> 占空比
    duty_ratio = prv_deg_to_duty_ratio(angle_deg, (struct angle_servo_status *)g_servo_state[servo_id].ctx);
    // 调用驱动层接口
    if (!Driver_PWM_SetDutyRatio(servo_id, duty_ratio)) 
    {
        return SERVO_DEVICE_ERR_HARDWARE;
    }
    // 更新状态
    g_servo_state[servo_id].current_value = angle_deg;
    return SERVO_DEVICE_OK;
}

/** 反初始化 */
Servo_ErrorCode_t AngleServo_Deinit(Servo_ID_t servo_id)
{
    Servo_ErrorCode_t errCode;
    struct angle_servo_status* pstatus = NULL;

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
    // 释放内存
    pstatus = (struct angle_servo_status *)g_servo_state[servo_id].ctx;
    tx_byte_release(pstatus);
    // 更新状态
    g_servo_state[servo_id].initialized = false;
    return SERVO_DEVICE_OK;
}
