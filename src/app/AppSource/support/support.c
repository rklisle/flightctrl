#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../Interface/interface_uart.h"
#include "../Interface/interface_gpio.h"
#include "interface_timer.h"
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
	//初始化数据池
	InitDataPool();
	//初始化总线
	InitRts();

	InitCanRts();

    AngleServo_Init(SERVO_PWM1, 0.0f); //左翅膀舵机初始化
    AngleServo_Init(SERVO_PWM6, 0.0f); //右翅膀舵机初始化
    AngleServo_Init(SERVO_PWM7, 0.0f); //开伞舵机初始化

#ifdef SERVO_PWM
    AngleServo_Init(SERVO_PWM2, 0.0f);	// 左副翼舵
    AngleServo_Init(SERVO_PWM3, 0.0f);	// 左俯仰舵
    AngleServo_Init(SERVO_PWM4, 0.0f);	// 右俯仰舵
    AngleServo_Init(SERVO_PWM5, 0.0f);	// 右副翼舵
#endif

    return OS_SUCCESS;
}

