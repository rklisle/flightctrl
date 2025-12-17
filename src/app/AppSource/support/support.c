#include "../core/BusInteract.h"
#include "../core/DataPool.h"
#include "../Interface/interface_uart.h"
#include "../Interface/interface_gpio.h"
#include "../Interface/interface_pwm.h"
#include "../Interface/interface_eth.h"
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
    
    InitGPIO();
    
    LAN9303_Init();
    
    SetPwmgpio_Enable(200);

    return OS_SUCCESS;
}

