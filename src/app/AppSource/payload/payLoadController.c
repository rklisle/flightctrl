/*
 * payLoadController.c
 *
 *  Created on: 2023年4月23日
 *      Author: lenovo
 */
#include "PayloadController.h"
#include "scout.h"
#include "../Modules/modPwrSeqCtl.h"
#include "../Modules/modOnceBattery.h"

OS_U8 PayloadHandle()
{
	if(IsLoadImage == 0)
	{
   		ScoutAutoSend();
	}
    SaveOnceBattery();
    SavePwrSeq();        //MML 配电板→飞控 上报的实时电压电流数据 存入数据池
    return 0;
}

OS_U8 PayloadInit()
{
	return 0;
}
