#include "stmToZynq.h"
#include "StateMachine.h"
#include "./core/BusInteract.h"

double curTime;
double step = 0.005;
void Init7020Code()
{
	 RunInitialInit();
	 ResetCurTime();
}

// 的
void periodic_function_1ms()
{
    BusDataHandle();
		//ADC_GetVoltage();
}

void periodic_function_5ms()
{
    curTime += step;
    RunStageMachineStep();
}


double GetCurTime()
{
	return curTime;
}

void SetCurTime(double time)
{
	curTime = time;
}

void ResetCurTime()
{
	curTime = 0;
}