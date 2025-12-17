/*
 * DataPool.h
 *
 *  Created on: 2022年1月22日
 *      Author: ChengHongjing
 *  Function : 支持以整数或8字符以内的字符串作为key,查找对应的value的方式,实现数据队列.数据内容最大8字节
 */

#ifndef SRC_CORE_DATAPOOL_H_
#define SRC_CORE_DATAPOOL_H_

#include "../support/os_framework.h"

#define MAX_MODULE_COUNT (9)
#define PARAM_CODE_MAXLEN		(8)
#define MAX_PARAM_COUNT 		(80)

#define ALL_DATAPOOL			(0xFF)//暂不支持
#define NAV_DATAPOOL_INDEX 		(0)
#define FLY_DATAPOOL_INDEX		(1)
#define MCU_DATAPOOL_INDEX		(2)
#define SRV_DATAPOOL_INDEX		(3)
#define PWR_DATAPOOL_INDEX		(4)
#define SELF_DATAPOOL_INDEX		(5)
#define SD_DATAPOOL_INDEX		(6)
#define MSN_DATAPOOL_INDEX		(7)
#define IMU_DATAPOOL_INDEX		(8)

#define SETDATA(thisPt,key,value,type)					\
		do{												\
			type temp = (value);						\
            char keystr[8] = {0};                       \
            memcpy(keystr,key,strlen(key));                          \
			SetDataFast(thisPt,keystr,&temp,sizeof(type));	\
		}while(0);

#define INITDATA(thisPt,key,len)						\
		do{												\
			OS_U64 temp = 0;							\
			SetDataFast(thisPt,key,&temp,len);			\
		}while(0);
/*
typedef enum
{
	u_8int = 1,
	u_16int = 2,
	u_32int = 4,
	u_64int = 8,
	s_8int = 1,
	s_16int = 2,
	s_32int = 4,
	s_64int = 8,
	s_32float = 4,
	s_64double = 8,
}DataPoolType;*/

typedef unsigned char DataPoolType;

#pragma pack(8)
typedef union DataPoolValue
{
	OS_U8 u8Value;
	OS_S8 s8Value;
	OS_U16 u16Value;
	OS_S16 s16Value;
	OS_U32 u32Value;
	OS_S32 s32Value;
	OS_FLOAT floatValue;
	OS_DOUBLE doubleValue;
	unsigned long long u64Value;
	long long s64Value;
}DataPoolValue;

typedef union DataPoolKey
{
	char keyStr[PARAM_CODE_MAXLEN];
	unsigned long long keyLong;
}DataPoolKey;
#pragma pack()

typedef struct DataPoolKeyValue
{
	DataPoolKey DataKey;
	DataPoolValue DataValue;
	DataPoolType DataType;
}DPNode;//每个KeyValue节点16字节

typedef struct DataPool
{
	OS_BOOL initialized;
	OS_U32 dataCount;
	DPNode dataNodes[MAX_PARAM_COUNT];
	DPNode *itor;
	OS_U32 (*ptr_GetData)(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue* const value, DataPoolType * const type );
	OS_U32 (*ptr_SetData)(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type);
	OS_U32 (*ptr_AddData)(struct DataPool* const this_obj, DataPoolKey key, DataPoolValue value, DataPoolType type);
	OS_U32 (*ptr_ResetItor)(struct DataPool* const this_obj);
	OS_U32 (*ptr_GetCurData)(struct DataPool* const this_obj, DataPoolValue* value, DataPoolType *type);
	void (*ptr_GetPacketByKeys)(struct DataPool* const this_obj, char (*key)[8], OS_U8* buf, OS_U16 *len);
}*p_DataPool;

void SetDataFast(struct DataPool* const this_obj, const char* key, const void* value, DataPoolType type);
void GetDataFast(struct DataPool* const this_obj, const char* key, void* const value);

extern struct DataPool DataPoolALL[MAX_MODULE_COUNT];

extern p_DataPool pDataPoolFly;
extern p_DataPool pDataPoolMcu;
extern p_DataPool pDataPoolNav;
extern p_DataPool pDataPoolSrv;
extern p_DataPool pDataPoolPwr;
extern p_DataPool pDataPoolSelf;
extern p_DataPool pDataPoolSD;
extern p_DataPool pDataPoolMsn;
extern p_DataPool pDataPoolImu;
extern void InitDataPool();

#endif /* SRC_CORE_DATAPOOL_H_ */
