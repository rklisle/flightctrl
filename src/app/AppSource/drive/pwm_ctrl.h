#ifndef APP_PWM_CTRL_H
#define APP_PWM_CTRL_H

// pwm init, freq range from 50Hz to 1000Hz
int pwm_ctrl_init(unsigned int pwm_freq);

// pwm duty rang 0~100% mapped  0~1000 
int pwm_ctrl_dutyCycle(unsigned char chn, float duty);


int pwm_ctrl_pulse(unsigned char chn, int pulse_width);
#endif

