/*
 * DataPool.c
 *
 *  Created on: 2022年1月22日
 *      Author: ChengHongjing
 */

#include "DataPool.h"
#include "../support/os_error.h"
struct DataPool DataPoolALL[MAX_MODULE_COUNT];
p_DataPool pDataPoolNav;
p_DataPool pDataPoolFly;
p_DataPool pDataPoolMcu;
p_DataPool pDataPoolSrv;
p_DataPool pDataPoolPwr;
p_DataPool pDataPoolSelf;
p_DataPool pDataPoolSD;
p_DataPool pDataPoolMsn;
p_DataPool pDataPoolImu;

static OS_U32 GetData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue* const value, DataPoolType * const type );
static OS_U32 SetData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type);
static OS_U32 AddData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type);
static OS_U32 ResetItor(struct DataPool* const this_obj);
static OS_U32 GetCurData(struct DataPool* const this_obj, DataPoolValue* value, DataPoolType *type);
static void GetPacketByKeys(struct DataPool* const this_obj, char (*key)[8], OS_U8* buf, OS_U16 *len);

void InitDataPool()
{
	for(int i=0;i<MAX_MODULE_COUNT;i++)
	{
		DataPoolALL[i].initialized = TRUE;
		DataPoolALL[i].dataCount = 0;
		DataPoolALL[i].itor = NULL;
		DataPoolALL[i].ptr_GetData = GetData;
		DataPoolALL[i].ptr_SetData = SetData;
		DataPoolALL[i].ptr_AddData = AddData;
		DataPoolALL[i].ptr_ResetItor = ResetItor;
		DataPoolALL[i].ptr_GetCurData = GetCurData;
		DataPoolALL[i].ptr_GetPacketByKeys = GetPacketByKeys;
	}
	pDataPoolNav = &(DataPoolALL[NAV_DATAPOOL_INDEX]);
	pDataPoolFly = &(DataPoolALL[FLY_DATAPOOL_INDEX]);
	pDataPoolMcu = &(DataPoolALL[MCU_DATAPOOL_INDEX]);
	pDataPoolSrv = &(DataPoolALL[SRV_DATAPOOL_INDEX]);
	pDataPoolPwr = &(DataPoolALL[PWR_DATAPOOL_INDEX]);
	pDataPoolSelf= &(DataPoolALL[SELF_DATAPOOL_INDEX]);
	pDataPoolSD= &(DataPoolALL[SD_DATAPOOL_INDEX]);
	pDataPoolMsn = &(DataPoolALL[MSN_DATAPOOL_INDEX]);
    pDataPoolImu = &(DataPoolALL[IMU_DATAPOOL_INDEX]);
}

static void GetPacketByKeys(struct DataPool* const this_obj, char (*key)[8], OS_U8* buf ,OS_U16 *len)
{
	char (*pKey)[8] = key;
	OS_U8 *p = buf;
	OS_U16 pktLen = 0;
	while(*pKey != NULL && *pKey[0] != 0)
	{
		DataPoolKey dkey;
		memcpy(dkey.keyStr, *pKey, 8);
		DataPoolValue value;
		DataPoolType type = 1;
		GetData(this_obj, dkey, &value, &type);
		memcpy(p, &value, type);
		p += type;
		pktLen += type;
		pKey ++;
	}
	if(len != NULL)
		*len = pktLen;
	return;
}

static OS_U32 ResetItor(struct DataPool* const this_obj)
{
	if(this_obj == NULL)
		return -1;
	if(this_obj->dataCount == 0)
		return -1;
	this_obj->itor = &(this_obj->dataNodes[0]);
	return 0;
}

static OS_U32 GetCurData(struct DataPool* const this_obj, DataPoolValue* const value, DataPoolType* const type)
{
	if(this_obj == NULL)
		return -1;
	if(this_obj->dataCount == 0)
		return -1;
	*value = (this_obj->itor->DataValue);
	*type = (this_obj->itor->DataType);
	//迭代器向后移位
	if(this_obj->itor == &(this_obj->dataNodes[this_obj->dataCount - 1]))
	{
		this_obj->itor = &(this_obj->dataNodes[0]);
	}
	else
	{
		this_obj->itor ++;
	}
	return 0;
}

void GetDataFast(struct DataPool* const this_obj, const char* key, void* const value)
{
	DataPoolKey key1;
	memcpy(key1.keyStr, key, 8);
	DataPoolValue value1;
	DataPoolType type;
	//GetData(this_obj, key1, &value1, &type);
	OS_U32 res = GetData(this_obj, key1, &value1, &type);
	if(res == (OS_U32)(-1))
	{
		char errInfo[128];
		sprintf(errInfo,"Key [%s] didn't found!\r\n",key);
		EnterError(errInfo);
	}
	else
	{
		//正常情况
		memcpy(value, &value1, type);
	}

}

static OS_U32 GetData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue* const value, DataPoolType * const type )
{
	if(this_obj == NULL)
		return -1;
	if(this_obj->dataCount == 0)
	{
		return -1;
	}
	DPNode * startPos = this_obj->itor;
	do
	{
		if(this_obj->itor->DataKey.keyLong == key.keyLong)
		{
			//查找到
			*value = this_obj->itor->DataValue;
			*type = this_obj->itor->DataType;
			return 0;
		}
		//迭代器向后移位
		if(this_obj->itor == &(this_obj->dataNodes[this_obj->dataCount - 1]))
		{
			this_obj->itor = &(this_obj->dataNodes[0]);
		}
		else
		{
			this_obj->itor ++;
		}
	}
	while(this_obj->itor != startPos);
	return -1;
}



void SetDataFast(struct DataPool* const this_obj, const char* key, const void *const value, DataPoolType type)
{
	if(this_obj == NULL)
		return;
	DataPoolKey key1;
	memcpy(key1.keyStr, key, 8);
	DataPoolValue value1;
	memcpy(&value1, value, type);
	SetData(this_obj, key1, value1, type);
}

static OS_U32 SetData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type)
{
	if(this_obj == NULL)
		return -1;
	if(this_obj->itor == NULL)
	{
		return AddData(this_obj, key, value, type);
	}
	DPNode * startPos = this_obj->itor;
	do
	{
		if(this_obj->itor->DataKey.keyLong == key.keyLong)
		{
			//查找到
			this_obj->itor->DataValue.u64Value = value.u64Value;
			return 0;
		}
		//迭代器向后移位
		if(this_obj->itor == &(this_obj->dataNodes[this_obj->dataCount - 1]))
		{
			this_obj->itor = &(this_obj->dataNodes[0]);
		}
		else
		{
			this_obj->itor ++;
		}
	}
	while(this_obj->itor != startPos);

	//不存在则新增
	return AddData(this_obj, key, value, type);
}

static OS_U32 AddData(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type)
{
	//新增
	if(this_obj->dataCount >= MAX_PARAM_COUNT)
	{
		//数据池已满
		return -1;
	}
	this_obj->dataNodes[this_obj->dataCount].DataKey.keyLong = key.keyLong;
	this_obj->dataNodes[this_obj->dataCount].DataValue.u64Value = value.u64Value;
	this_obj->dataNodes[this_obj->dataCount].DataType = type;
	this_obj->itor = &(this_obj->dataNodes[this_obj->dataCount]);
	this_obj->dataCount += 1;
	return 0;
}
