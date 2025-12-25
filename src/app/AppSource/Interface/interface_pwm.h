/*
 * interface_power.h
 * Created on: 2022年8月7日
 *      Author: 成宏璟
 */


#include "../support/os_types.h"
#include "../drive/pwm_ctrl.h"

#ifndef INTERFACE_PWM_H_
#define INTERFACE_PWM_H_

/*控制具体PWM通道*/
extern int Drv_SetPwm(int PwmPort,int pwm_en,int pwm_freq_sel,int pulse_width_data);
/*初始化PWM频率*/
extern int SetPwmgpio_Enable(int hz);
#endif /* INTERFACE_PWM_H_ */
