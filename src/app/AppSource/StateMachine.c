#include "StateMachine.h"
#include "./controller/controller.h"
#include "./support/support.h"
#include "./FlightSupport.h"
#include "./modules/modPwrSeqCtl.h"
#include "./modules/modSrvCtl.h"
#include "./modules/modNav.h"
#include "./modules/modEngine.h"
#include "./modules/modOnceBattery.h"
#include "./modules/modSD.h"
#include "./modules/modFlash.h"
#include "./mission/mission.h"
#include "./controller/controller.h"
#include "./core/DataPool.h"
#include "./core/Telemetry.h"
#include "./core/Telecontrol.h"
#include "./core/BusInteract.h"
#include "./support/os_framework.h"
#include "flight/os_flight_io.h"
#include "payload/payloadController.h"
#include "./support/support.h"
#include "./Interface/Interface_flash.h"
#include "./Interface/Interface_power.h"
#include "stmToZynq.h"
#include    "tx_api.h"
#include    "UART_STM32H7xx.h"
#include    "stm32h7xx_hal.h"
#include "flightPort.h"

DeviceState g_DeviceState={0};
DeviceStatus g_DeviceStatus = {0};
CMathControlMain *g_pControl = NULL;

void RunInitialInit()   // MML 初始化调用一次
{
    g_pControl = newCMathControlMain();

	//根据箭上协议，配置422串口对应的设备，设置422串口校验方式，比特率
	BoardInit();        //初始化buffer，数据池，串口，CAN, 两个PWM333Hz

    uart_mode_init();   //初始化9路串口，并开启各自串口的收发任务
        
    //初始化遥测结构体
    InitTelemetry();    //各种遥测参数
     
    //控制初始化函数
    // FlightInit();       //飞行航点描述
    //TODO
ControlInitial(g_pControl);

    FlashInit();        //把NorFlash准备好，读了一遍JEDEC

    //初始化载荷数据池
    PayloadInit();      //目前没东西
       
    //初始化定时上报的变量
    InitReportParam();  //往数据池里存数据

    //初始化任务机
    MissionInit();      //设置炮ID, 管ID
    
    
    //读取app信息
    // ReloadAppInfoFromFlash();
    ADC3_Init();        //测MCU温度
 
    
    
    //启动地面控制
    g_DeviceState.workStage = (DOM_INTERACTIVE | DOM_TRIGGERON);
}
/***********************************************************
 * 函数名称:RunStageMachineStep()
 * 函数功能:状态机执行主函数，每一次时间片轮询（5ms）需要执行一次
 * 作者:	成宏璟
 ***********************************************************/
float calcTimeCpu0;
float maxCalcTime = 0;
void RunStageMachineStep()  // 5ms运行一次
{
	g_DeviceState.CurrTick++;
   // unsigned long current_time0 = tx_time_get();
			
            
	// 1.智能控制器状态更新
	ControllerStatusUpdata();
    
	// 3.准备给飞控的数据
	FlightInputGenerate();
	// 4.调用飞控算法
	DoFlightRun();
	// 5.时序触发条件判断
	SeqCalc();
	// 6.处理飞控输出(舵控，时序开关设置)
	FlightOutputHandle();   // 控舵机、发动机、伞降要求
	// 7.时序执行
	SeqHandle();
    // 12.输出到遥测，组帧发送
	TelemetryFrameOut();
	// 8.载荷处理
	PayloadHandle();        //MML 配电板→飞控 上报实时电压电流数据
	// 9.伺服定时处理（伺服小回路，伺服位置查询)
	SrvStatusUpdata();
	// 10.电调发动机定时处理
	AutoDriveEnginePwm();
    // 11.
    AutoLuanchProcess();    //MML 飞控→给各个设备上电、加载任务信息、惯组对准、转导航、发动机启动、预发射、发射、
     // 2.
	RunMissionTask(10);   
    //unsigned long current_time1 = tx_time_get();
    //calcTimeCpu0 = (current_time1 - current_time0);
    //if(maxCalcTime < calcTimeCpu0)
    //    maxCalcTime = calcTimeCpu0;
    
    //OS_U8 testP900[10] = {1,2,3,4,5,6,7,8,9,10};
   // UART_PutBuff(rtList[RT_P900].chIndex, testP900, 10);
}


