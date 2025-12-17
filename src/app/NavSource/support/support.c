//#include "xil_cache.h"

#include "os_framework.h"
#include "os_bufferQueue.h"
#include "support.h"
//#include "time.h"
//#include "os_time.h"
//#include "../drive/fq_kzq.h"
#include "../core/BusInteract.h"
#include "../interface/interface_uart.h"
//#include "../interface/interface_board.h"
#include "../modules/modFlash.h"
#include "../modules/modGps.h"
#include "../modules/modImu.h"
//#include "qspi_flash.h"
#include "../Interface/Interface_flash.h"
void uart_mode_init(void)
{
	for(int i=0;i<MODULE_COUNT;i++)
	{
		UART_Setting(rtList[i].chIndex, rtList[i].devBuad, rtList[i].oddCheckEnable == FALSE?'n':'o');
	}
}

OS_S32 BoardInit()
{
    BufferInit();	
	//初始化数据池
	InitDataPool();
	//初始化总线
	InitRts();
	return OS_SUCCESS;
}

