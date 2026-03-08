/*
 * modHil.c
 *
 *  Created on: 2024年4月6日
 *      Author: lenovo
 */

#include "modSD.h"
#include "../drive/ff.h"
#include "../../sd_flash.h"
#include "../core/DataPool.h"

int sd_card_fault = 0;  // 0：SD卡初始化成功； 1：初始化失败
unsigned int SD_BLOCK = 8192;
unsigned int BUFF_BLOCK = 0x8000;
typedef struct
{
	int fileOpenRes;
	OS_S32 fileCurPos;
    OS_U8 fileBuffer[0x8000];
}SD_FILE;

unsigned char SD_MountOK = FALSE;    // TRUE：SD卡已挂载（延迟挂载）；FALSE：SD卡未挂载
unsigned char SD_Enable = FALSE;    // TRUE：SD卡初始化成功；FALSE：SD卡初始化失败

SD_FILE sdFile[FILE_COUNT];
char fileName[FILE_COUNT][13] = {"TMData.dat","FireA.csv","GPS.csv","Mems.csv","navZ.csv","TMFlight.dat"};

static TX_MUTEX                 s_mutex_write[FILE_COUNT];   

OS_U8 DoWriteToSD(OS_U8 fileIndex, const OS_U8 *buf, OS_U32 length)
{
    if(sd_card_fault != 0)
    {
        return 0;
    }
	return fatFsFileWrite((OS_S32)fileIndex, (char *)buf, (OS_S32)length);
}

/** 1ms调用一次，当需要写入SD卡的数据，多到一定程度，这里是>SD_BLOCK时，再集中将其写入 */
OS_U8 JudgeWriteToSD()
{
    for(int fileIndex = 0; fileIndex < FILE_COUNT; fileIndex++)
    {
        if(sdFile[fileIndex].fileCurPos >= SD_BLOCK)
        {
            DoWriteToSD(fileIndex, sdFile[fileIndex].fileBuffer, SD_BLOCK);
            tx_mutex_get(&s_mutex_write[fileIndex], TX_WAIT_FOREVER);
            if(sdFile[fileIndex].fileCurPos - SD_BLOCK < BUFF_BLOCK)
            {
                memmove(sdFile[fileIndex].fileBuffer,
                    &sdFile[fileIndex].fileBuffer[SD_BLOCK],
                    sdFile[fileIndex].fileCurPos - SD_BLOCK);
            }
            sdFile[fileIndex].fileCurPos -= SD_BLOCK;
            tx_mutex_put(&s_mutex_write[fileIndex]);
        }
    }
    return 0;
}

/** 将数据安全地复制到指定文件的缓冲区中，准备后续批量写入SD卡。 */
OS_U8 WriteToSD(OS_U8 fileIndex, OS_U8 *buf, OS_U32 length)
{
	if(SD_Enable == FALSE)
		return 1;
	if(fileIndex >= FILE_COUNT)
		return 1;
    if(sdFile[fileIndex].fileCurPos + length > BUFF_BLOCK)
    {
        return 1;
    }
	if(sdFile[fileIndex].fileOpenRes == FR_OK)
	{
        tx_mutex_get(&s_mutex_write[fileIndex], TX_WAIT_FOREVER);
		memcpy(&sdFile[fileIndex].fileBuffer[sdFile[fileIndex].fileCurPos], buf, length);
		sdFile[fileIndex].fileCurPos += length;
        tx_mutex_put(&s_mutex_write[fileIndex]);
		return 0;
	}
	return 1;
}

void MountSD()
{
    if(SD_MountOK == TRUE)
        return;
    if(sd_flash_init(NULL) == 0)
    {
        SD_MountOK = TRUE;
    }
    return;
}

/** 初始化SD卡存储系统，包括：
 * 初始化SD卡硬件
 * 创建多个数据文件并写入CSV表头
 * 创建线程同步机制 
 */
OS_U8 InitSD()
{
    if(SD_Enable == TRUE)
		return 0xAA;
	OS_U32 sdInitState = FR_DISK_ERR;
	// if(sd_flash_init(NULL) == FR_OK)
    if(SD_MountOK == TRUE)
    {
		SD_Enable = TRUE;
        sdInitState = FR_OK;
        sd_card_fault = 0;
        
        
		for(int i=0;i<FILE_COUNT;i++)
		{
			sdFile[i].fileOpenRes = fatFsFileOpen(fileName[i],i);
		}

		char memsHead[100];
		char fireHead[100];
		char navZHead[100];
		char GPSHead[120];
		sprintf(fireHead, "time,BatV,BatA,CombinV,CombinA,SrvV,SrvA\n");
		sprintf(GPSHead, "time,lon,lat,high,gpsvn,gpsvs,gpsve,sacount,locaState,pdop,dir,dirmark,updatemark,dirEffect,gpsTrac\n");
		sprintf(memsHead, "time,ax,ay,az,wx,wy,wz\n");
		sprintf(navZHead, "time,x,y,z,vx,vy,vz, pitch,yaw,roll,lon,lat,high,mode,navvn,navvs,navve\n");
		WriteToSD(1, (OS_U8 *)fireHead, strlen(fireHead) );
		WriteToSD(2, (OS_U8 *)GPSHead, strlen(GPSHead) );
		WriteToSD(3, (OS_U8 *)memsHead, strlen(memsHead) );
		WriteToSD(4, (OS_U8 *)navZHead, strlen(navZHead) );

		for(int i=0;i<sizeof(sdFile)/sizeof(SD_FILE);i++)
		{
			if(sdFile[i].fileOpenRes != FR_OK)
			{
				sdInitState = FR_DISK_ERR;
			}
		}
	}
    else
    {
        sd_card_fault = 1;
    }
    for(int fileIndex = 0; fileIndex < FILE_COUNT; fileIndex++)
    {
        tx_mutex_create(&s_mutex_write[fileIndex], "sd_write_mutex", TX_INHERIT);
    }
	return sdInitState == FR_OK?0xAA:0xC0;
}

///////////////////////////SD THREAD/////////////////////////////////////////

#define     APP_STACK_SIZE         20480
static TX_THREAD   s_app_tcb;
static uint32_t    s_app_stack[APP_STACK_SIZE/sizeof(uint32_t)];
static void sd_task(ULONG thread_input);

#define     APP_TMR_FLAG    0x0800

/* Define what the initial system looks like.  */
void sd_write_init(TX_BYTE_POOL* pheap)
{

    /* Create the application thread.  */
    tx_thread_create(   &s_app_tcb, 
                        "sd_task", 
                        sd_task, 
                        (ULONG)pheap, 
                        s_app_stack,
                        APP_STACK_SIZE, 
                        TX_MAX_PRIORITIES - 2,
                        TX_MAX_PRIORITIES - 2, 
                        TX_NO_TIME_SLICE, 
                        TX_AUTO_START);
}

static void sd_task(ULONG thread_input) 
{

    /* This thread simply sits in while-forever-sleep loop.  */
    while (1) 
    {              
        tx_thread_sleep(1);
        // do something here
        JudgeWriteToSD();
    }
}
