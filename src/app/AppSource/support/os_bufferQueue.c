#include "os_framework.h"
#include "os_error.h"
#include "../core/BusInteract.h"

#define MAX_BUFFER_SIZE		(2048)
#define MAX_BLOCK_NUM		(8)

typedef enum
{
	EN_BLOCK_EMPTY = 0,
	EN_BLOCK_FULL
}EN_BLOCK_STA;

typedef enum
{
	EN_BLOCK_NO_READ = 0,
	EN_BLOCK_HAVE_READ
}EN_BLOCK_READ_STA;

typedef struct
{
	EN_BLOCK_STA enBlockSta;
	EN_BLOCK_READ_STA enReadSta;
	OS_MEM* pmStart;
	OS_U16 u16BlockLength;
}STRU_BLOCK;

typedef struct
{
	OS_MEM* pmBufferAddr;
	STRU_BLOCK astBlock[MAX_BLOCK_NUM];
	OS_U16 u16BufferLength;
	OS_U16 u16BufferUsed;
}STRU_BUFFER;

static STRU_BUFFER g_stBuffer;
static OS_MEM g_mBuffer[MAX_BUFFER_SIZE];

STRU_BLOCK* get_available_block(OS_U16 u16Length);
OS_S32 write_block(const OS_MEM* pmFrom, OS_U16 u16Length, STRU_BLOCK* pstBlock);


OS_S32 BufferInit()
{
	memset(&g_stBuffer, 0, sizeof(STRU_BUFFER));
	g_stBuffer.pmBufferAddr = g_mBuffer;
	g_stBuffer.u16BufferLength = MAX_BUFFER_SIZE;
	g_stBuffer.u16BufferUsed = 0;

#ifdef _WORK_MODE_PRINTF_
	printf("buffer have inited\n");
#endif

	return OS_SUCCESS;
}

OS_S32 PushBuffer(const OS_MEM * pmData, const OS_U16 u16Length)
{
	STRU_BLOCK* pstBlock = NULL;

	if (pmData == NULL)
		return ERROR_INPUT_POINT_IS_NULL;

	if (u16Length <= 0)
		return ERROR_LENGTH_LESS_ZERO;

	pstBlock = get_available_block(u16Length);

	if (pstBlock == NULL)
		return ERROR_NO_AVAILIBLE_BLOCK;

	return write_block(pmData, sizeof(OS_MEM)*u16Length, pstBlock);
}

OS_MEM* PopBuffer(OS_U16 * pu16Length)
{
	OS_S32 i = 0;
	if (pu16Length == NULL)
		return NULL;

	for (i = 0; i < MAX_BLOCK_NUM; i++)
	{
		if ((g_stBuffer.astBlock[i].enBlockSta == EN_BLOCK_FULL)
			&& (g_stBuffer.astBlock[i].enReadSta == EN_BLOCK_NO_READ))
		{
			g_stBuffer.astBlock[i].enReadSta = EN_BLOCK_HAVE_READ;
			*pu16Length = g_stBuffer.astBlock[i].u16BlockLength;
			return g_stBuffer.astBlock[i].pmStart;
		}
	}

	memset(g_stBuffer.astBlock, 0, sizeof(STRU_BLOCK)*MAX_BLOCK_NUM);
	memset(g_stBuffer.pmBufferAddr, 0, sizeof(OS_MEM)*g_stBuffer.u16BufferLength);
	g_stBuffer.u16BufferUsed = 0;

	return NULL;
}

STRU_BLOCK* get_available_block(OS_U16 u16Length)
{
	OS_S32 i = 0;

	if ((g_stBuffer.u16BufferUsed + u16Length) > g_stBuffer.u16BufferLength)
		return NULL;

	for (i = 0; i < MAX_BLOCK_NUM; i++)
	{
		if (g_stBuffer.astBlock[i].enBlockSta == EN_BLOCK_EMPTY)
		{
			g_stBuffer.astBlock[i].pmStart = g_stBuffer.pmBufferAddr + g_stBuffer.u16BufferUsed;
			return &g_stBuffer.astBlock[i];
		}
	}

	return NULL;
}

OS_S32 write_block(const OS_MEM* pmFrom, OS_U16 u16Length, STRU_BLOCK* pstBlock)
{
	if ((pmFrom == NULL)
		|| (pstBlock == NULL))
		return ERROR_INPUT_POINT_IS_NULL;

	if (u16Length <= 0)
		return ERROR_LENGTH_LESS_ZERO;

	memcpy(pstBlock->pmStart, pmFrom, u16Length);

	pstBlock->enBlockSta = EN_BLOCK_FULL;

	pstBlock->u16BlockLength = u16Length;

	g_stBuffer.u16BufferUsed += u16Length;

	return OS_SUCCESS;
}






