#include "stmToZynq.h"
#include "StateMachine.h"
#include "./core/BusInteract.h"

double curTime;	// 初始化时 置0；每5ms，+0.005	// 系统时间，单位 s
// double step = 0.005;
#define TIME_STEP 0.005
void Init7020Code()
{
	 RunInitialInit();
	 ResetCurTime();
}

// 鐨?
void periodic_function_1ms()
{
    BusDataHandle();
		//ADC_GetVoltage();
}

void periodic_function_5ms()
{
    curTime += TIME_STEP;
    RunStageMachineStep();
}

// 获取时间，参照对象是：程序初始化为0时，每5ms，+0.005
double GetCurTime()
{
	return curTime;
}

// 未使用
void SetCurTime(double time)
{
	curTime = time;
}

// 初始化调用一次
void ResetCurTime()
{
	curTime = 0;
}