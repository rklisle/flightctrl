#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../Interface/interface_uart.h"
#include "../Interface/interface_gpio.h"
#include "interface_timer.h"
#include "../modules/modSrvCtl.h"
#include "../../sd_flash.h"

void uart_mode_init(void)
{
   // tx_thread_sleep(20);
	for(int i=0;i<MODULE_COUNT;i++)
	{
       // if(rtList[i].chIndex != 0)
       //     continue;
        char check;
        if(rtList[i].oddCheckEnable)
            check = 'o';
        else if(rtList[i].evenCheckEnable)
            check = 'e';
        else
            check = 'n';
        
		UART_Setting(rtList[i].chIndex, rtList[i].devBuad, check, rtList[i].devStopLen);
	}
}

OS_S32 BoardInit()
{
	BufferInit();	
	//��ʼ�����ݳ�
	InitDataPool();
	//��ʼ������
	InitRts();

	InitCanRts();

    AngleServo_Init(SERVO_PWM7, 0.0f);
#ifdef SERVO_PWM
    AngleServo_Init(SERVO_PWM1, 0.0f);
    AngleServo_Init(SERVO_PWM2, 0.0f);
    AngleServo_Init(SERVO_PWM3, 0.0f);
    AngleServo_Init(SERVO_PWM4, 0.0f);
    AngleServo_Init(SERVO_PWM5, 0.0f);
    AngleServo_Init(SERVO_PWM6, 0.0f);
#endif
    return OS_SUCCESS;
}

