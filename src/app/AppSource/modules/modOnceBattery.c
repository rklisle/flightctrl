/*
 * modOnceBattery.c
 *
 *  Created on: 2024Äê6ÔÂ16ÈÕ
 *      Author: lenovo
 */

#include "modOnceBattery.h"
#include "../core/DataPool.h"
#include "../core/BusInteract.h"
#include "../Interface/interface_can.h"
#include "modEngine.h"
OS_U8 battPowerCmd = 0x00;
OS_U8 batt24VCmd = 0x00;

OnceBattStru onceBattStru = {0};
OS_U8 OnceBatteryInit()
{
	SaveOnceBattery();
	return 0;
}

OS_U8 SaveOnceBattery()
{
	SETDATA(pDataPoolPwr, "Batt28V", onceBattStru.mcu28V,	OS_U16);
	SETDATA(pDataPoolPwr, "Batt28A", onceBattStru.mcu28A,	OS_U16);
	SETDATA(pDataPoolPwr, "Eng24V", onceBattStru.mcu24V,	OS_U16);
	SETDATA(pDataPoolPwr, "Eng24A", onceBattStru.mcu24A,	OS_U16);
	SETDATA(pDataPoolPwr, "BattTemp", onceBattStru.battTemp,OS_U16);
	SETDATA(pDataPoolPwr, "mcuTemp", onceBattStru.mcuTemp,	OS_U16);
	return 0;
}

void CanRtBattHandler(long unsigned int id, bool ext_id, const OS_U8* pdata, long unsigned int datalen)
{
    unsigned long long data;
	memcpy(&data, pdata, 8);
	onceBattStru.mcu28V =    (OS_U16)((data >> 0 ) & 0x7FF);
	onceBattStru.mcu28A =    (OS_U16)((data >> 11 ) & 0x7FF);
	onceBattStru.mcu24V =  (OS_U16)((data >> 22 ) & 0x1FF);
	onceBattStru.mcu24A =  (OS_U16)((data >> 31 ) & 0x1FF);
  onceBattStru.battTemp = (OS_U16)(((data >> 40 ) & 0xFFF));
	onceBattStru.mcuTemp = (OS_U16)(((data >> 52 ) & 0xFFF));
	//SaveOnceBattery();
	g_DeviceState.battCountDown = 200;
}


