#include "StateMachine.h"
#include "./controller/controller.h"
#include "./modules/modFlash.h"
#include "./modules/modImu.h"
#include "./interface/interface_uart.h"
#include "./controller/controller.h"
#include "./core/DataPool.h"
#include "./core/Telecontrol.h"
#include "./core/BusInteract.h"
#include "./support/os_framework.h"
#include "./navSupport.h"


#include "stmToZynq.h"
#include    "tx_api.h"
#include    "UART_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
void RunInitialInit()
{
    BoardInit();
    ADC3_Init();
    uart_mode_init();
	//初始化数据池
	InitDataPool();
}
/***********************************************************
 * 函数名称:RunStageMachineStep()
 * 函数功能:状态机执行主函数，每一次时间片轮询（5ms）需要执行一次
 * 作者:	成宏璟
 ***********************************************************/
void RunStageMachineStep()
{
    
	// 1.本机状态更新
	SelfStatusUpdata();
	// 2.准备给导航的数据
	NavInputGenerate();
	// 3.空速校准
	//DoAirSpdCalibration();
	//NavSendToCpu1();
	// 4.调用CPU0导航算法
	DoNavRun();
	// 5.处理导航输出(舵控，时序开关设置)
	NavOutputHandle();
	// 6.输出到串口，给智能控制器
	SelfFrameOut();
}


