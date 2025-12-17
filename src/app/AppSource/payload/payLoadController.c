/*
 * payLoadController.c
 *
 *  Created on: 2023Äê4ÔÂ23ÈÕ
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
    SavePwrSeq();
    return 0;
}

OS_U8 PayloadInit()
{
	return 0;
}
