/*
 * interface_power.c
 *  Created on: 2022年8月7日
 *      Author: 成宏璟
 */
#include "interface_can.h"
#include "../../app_can.h"

/* 原有标准帧发送函数 */
int SendCanFrame(OS_U8 canIndex, OS_U16 id, OS_U8 length, OS_U8 * TxData)
{
	int Status = 0;
    app_can_send(fdCan[canIndex], id, false, TxData, length);
	return Status;
}

/* 新增扩展帧发送函数 */
int SendCanFrameExt(OS_U8 canIndex, OS_U32 id, OS_U8 length, OS_U8 * TxData)
{
	int Status = 0;
    app_can_send(fdCan[canIndex], id, true, TxData, length);
	return Status;
}
