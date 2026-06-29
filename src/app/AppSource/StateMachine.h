/*
 * StateMachine.h
 *
 *  Created on: 2021年10月14日
 *      Author: QL
 */

#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

#include "flightPort.h"

extern void *pControl;

extern void RunInitialInit();//流程调度，初始化
extern void RunStageMachineStep();//流程调度，运行

#endif /* SRC_STATEMACHINE_H_ */
