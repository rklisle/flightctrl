#include "stmToZynq.h"
#include "StateMachine.h"
#include "./core/BusInteract.h"

double curTime;
void InitNavCode()
{
	 RunInitialInit();
	 ResetCurTime();
}

// 的
void periodic_nav_1ms()
{
	//ImuSourceDataUpdate();
    
}
#include <tx_api.h>
float calcTimeCpu1;
float maxCalcTime1 = 0;
void periodic_nav_5ms()
{
    curTime += 0.005;
    BusDataHandle();
    RunStageMachineStep();
    ImuSourceDataUpdate();
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