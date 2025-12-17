#include "os_framework.h"

#ifdef _WORK_MODE_REPORT_ERROR_

static OS_S32 g_as32ErrorList[MAX_ERROR_LIST_SIZE];
static OS_S32 g_s32ListUsedNum = 0;

inline OS_S32 Push2ErrorList(OS_S32 s32ErrorCode)
{
	if (s32ErrorCode > (MAX_ERROR_LIST_SIZE - 2))
	{
		g_as32ErrorList[MAX_ERROR_LIST_SIZE - 1] = ERROR_LIST_FULL;
		return s32ErrorCode;
	}
	g_as32ErrorList[g_s32ListUsedNum] = s32ErrorCode;
	g_s32ListUsedNum++;
	return s32ErrorCode;
}

OS_S32 PopAllError(OS_S32* pu32ErrorList)
{
	pu32ErrorList = g_as32ErrorList;
	OS_S32 temp = g_s32ListUsedNum;
	g_s32ListUsedNum = 0;
	return temp;
}

#endif

#include <stdio.h>
#include <stdlib.h>

OS_U8 EnterError(char *errorInfo)
{
	char errorStr[128];
	sprintf(errorStr, "%s", errorInfo);
	//while(1)
	//{
	//	int a = 0;
	//}
	return 0;
}
