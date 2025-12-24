/*
 * interface_PWM.c
 *  Created on: 2022年8月7日
 *      Author: 成宏璟
 */
#include "interface_pwm.h"
#include "../drive/pwm_ctrl.h"

int SetPwmgpio_Enable(int hz)//flag=1 enable;flag=0,disable
{
	pwm_ctrl_init(hz);//set pwm 200hz
	return 0;
}

int Drv_SetPwm(int PwmPort, int pwm_en, int pwm_freq_sel, int pulse_width_data)
{
    if(pwm_en)
    {
        pwm_ctrl_pulse(PwmPort, pulse_width_data);
    }
    else
    {
        pwm_ctrl_pulse(PwmPort, 0);
    }
    
    return 0;
}
