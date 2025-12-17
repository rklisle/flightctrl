/*
 * modFlash.c
 *
 *  Created on: 2021年10月22日
 *      Author: Admin
 */
#include "modFlash.h"
#include "../Interface/interface_flash.h"
#include "../core/Telecontrol.h"
#include "../core/DataPool.h"
#include "stddef.h"

#define BURNING	(0x11)
#define UNBURN	(0x22)
#define MAXPACKETLEN	(2000)

#define FILE_INDEX_SIMU	(0xA0)
#define FILE_INDEX_CPU0	(0xB0)
#define FILE_INDEX_CPU1	(0xB1)

static OS_U8 fileContent[FLASH_FILE_MAX] = {0};
OS_BOOL FlashProgramming = FALSE;
OS_U8 ephRecvReadyFlag = 0;

FLASHFILE flashFiles[FLASHFILE_COUNT]={
	//  MagicCode 			 fileID    baseAddr             		fileLen 	CRC 	ptrOffset 	fileName
		{FLASH_MAGIC_CODE,    0      ,(void*)0x600000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    1      ,(void*)0x640000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    2      ,(void*)0x680000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    3      ,(void*)0x6C0000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    4      ,(void*)0x700000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    5      ,(void*)0x740000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    6      ,(void*)0x780000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    7      ,(void*)0x7C0000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    0xA0   ,(void*)0x800000,				0,			0,		0x100,		{0}},
		{FLASH_MAGIC_CODE,    0xB0   ,(void*)0x410000,				0,			0,		0x10000,	{0}},
		{FLASH_MAGIC_CODE,    0xB1   ,(void*)0x510000,				0,			0,		0x10000,	{0}},

};
char* getNextsptr(char* src, int num)
{
	static char * pch = NULL;
	if (num < 1)
		return pch=NULL;
	if (!pch)
		pch = strtok(src, " :,\n\t\r");
	do
	{
		pch = strtok(NULL, " :,\n\t\r");
	} while (--num&&pch);
	return pch;
}

OS_U8 fileBufStatic[FLASH_FILE_MAX];
char* AllocFileBuffer(char* filepath, char* mode)
{
	memset(fileBufStatic, 0, FLASH_FILE_MAX);
	ReadFileByName(filepath, fileBufStatic);
	return (char *)fileBufStatic;
}

void FreeFileBuffer(char* pbuf)
{
	return ;
}

/***********************************************************
 * 函数名称: ReadFileByName()
 * 函数功能: 通过文件名称读取文件。文件名称需要在文件信息表中对查得到文件索引号，再通过
 * 			索引号读取文件内容。
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ReadFileByName(char* fileName, OS_U8* buf)
{
	OS_U8 index = 0;
	FLASHFILE tmpFileInfo={0};
	FLASHFILE *p = flashFiles;
	while(p && p->MagicCode == FLASH_MAGIC_CODE)
	{
		QspiFlashRead((OS_U32)p->baseAddr,(OS_U32)&tmpFileInfo,sizeof(FLASHFILE));
		if(!strcmp(tmpFileInfo.fileName, fileName))
		{
			break;
		}
		index++;
		p++;
	}
	if(index == FLASHFILE_COUNT)
		return -1;
	return ReadFileByIndex(index, buf);
}

/***********************************************************
 * 函数名称: ReadFileByIndex()
 * 函数功能: 通过文件索引号读取文件。因为文件都是在FLASH中按照固定大小写入固定位置的，
 * 			因此，通过索引号可以快速计算出文件在FALSH中存储的地址.
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 ReadFileByIndex(OS_U8 fileIndex, OS_U8* buf)
{
	FLASHFILE* fptr = &flashFiles[fileIndex];
	if(QspiFlashRead((OS_U32)(fptr->baseAddr), (OS_U32)fptr, sizeof(FLASHFILE)) == 0)
	{
		if(QspiFlashRead((OS_U32)(fptr->baseAddr + fptr->ptrOffset), (OS_U32)buf, fptr->fileLen) == 0)
		{
			return fptr->fileLen;
		}
		else
			return -1;
	}
	return -1;
}

OS_U32 ReadFileInfoByIndex(OS_U8 fileIndex)
{
	FLASHFILE* fptr = &flashFiles[fileIndex];
	FLASHFILE temp;
	if(QspiFlashRead((OS_U32)fptr->baseAddr, (OS_U32)&temp, sizeof(FLASHFILE)) == 0)
	{
		if(temp.MagicCode == FLASH_MAGIC_CODE)
		{
			memcpy(fptr, &temp, sizeof(FLASHFILE));
		}
		return 0;
	}
	return -1;
}

OS_U32 UpdateFlashFileInfo()
{
	for(int fileIndex = 0; fileIndex < FLASHFILE_COUNT; fileIndex++)
	{
		FLASHFILE* fptr = &flashFiles[fileIndex];
		QspiFlashRead((OS_U32)(fptr->baseAddr), (OS_U32)fptr, sizeof(FLASHFILE));
	}
	return 0;
}

OS_U32 ClearFlashDatFile()
{
	for(int fileIndex = 0; fileIndex < FLASHFILE_COUNT; fileIndex++)
	{
		FLASHFILE* fptr = &flashFiles[fileIndex];
		if(fptr->fileID != 0xA0 && fptr->fileID != 0xB0 && fptr->fileID != 0xB1)
		{
			fptr->MagicCode = 0;
			QspiFlashProgram(
								(OS_U32)fptr,
								(OS_U32)(fptr->baseAddr),
								(OS_U32)(sizeof(FLASHFILE))
								);
			fptr->MagicCode = FLASH_MAGIC_CODE;
		}
	}
	return 0;
}

static OS_U8 FlashWriteReadTest(int fileIndex)
{
	static FLASHFILE* pflashFile = NULL;
	pflashFile = &flashFiles[fileIndex];//文件ID

	memset(fileContent, 0, 8);
	fileContent[0] = 0x01;
	fileContent[1] = 0x02;
	fileContent[2] = 0x03;
	fileContent[3] = 0x04;
	OS_U8 writeRes,readRes;
	writeRes = QspiFlashProgram(
						(OS_U32)fileContent,
						(OS_U32)(pflashFile->baseAddr),
						(OS_U32)((int)4)
						);
	if (writeRes == 0)
	{
		readRes = QspiFlashRead(
						(OS_U32)(pflashFile->baseAddr),
						(OS_U32)(fileContent + 4),
						4);
	}
	else
	{
		return writeRes;
	}
	if(fileContent[0] == fileContent[4] &&
		fileContent[1] == fileContent[5] &&
		fileContent[2] == fileContent[6] &&
		fileContent[3] == fileContent[7])
		return 0;
	else
		return readRes;
	//return readRes;
}

OS_U8 AutoTestFlash()
{
	for(int i=0;i<FLASHFILE_COUNT;i++)
	{
		OS_U8 res;
		if((res = FlashWriteReadTest(i)) != 0)
			return res;
	}
	return 0;
}

/***********************************************************
 * 函数名称:FlashCmdHandler()
 * 函数功能: flash指令处理函数，本型号智能控制器接收地面发出的指令:
 * 			1.flash烧写请求0x12	:对flash做的任何操作都需要先执行本指令，将状态置为烧写中。本指令可:
 * 								:(1)进入烧写,并将烧写文件名及时间写入固定位置
 * 								:(2)退出烧写,退出后不能够进行烧写及校验
 * 			2.flash烧写内容0x62	:flash烧写过程如果文件大于一次传输上限（250字节）时，需要分多次发送，多次为连续发送
 * 								:(1)当本次发送未发完时，写flash，并将计数增加250，并返回（烧写中0x11）给地面
 * 								:(2)本次发送完成时，写flash，并返回（烧写完成0x22)给地面,并开始将内存中的数据写往FLASH
 * 			3.flash内容校验		:地面上传文件的CRC32校验和，智能控制器计算指定文件的CRC32并比较，返回结果给地面
 * 								:校验需要在发送烧写控制（退出烧写）前进行。
 * 			4.诸元数据加载			:读取指定的文件，获取发射地经纬高等数据，下传地面
 * 参考资料: <TXII-Y1 箭地通信协议>
 * 作者:	成宏璟
 ***********************************************************/
OS_U32 FlashCmdHandler(STRU_422_MSG_INFO * frame)
{
	OS_U8 msgId = frame->u8MsgID;
	static FLASHFILE* pflashFile = NULL;
	//三条指令均需要回复
	static OS_U8 burnState = UNBURN;
	static OS_U16 lastPacketIndex = 0;
	static OS_U16 lastPacketCount = 0;
	OS_U8 resBuf[1024];
	switch(msgId)
	{
	case CMD_FLASH_CTRL_REQ:		//flahs烧写请求
	{
		burnState = frame->au8Data[0];
		if(burnState == BURNING)
		{
			OS_U8 fileIndex = frame->au8Data[1];
			if(fileIndex == FILE_INDEX_SIMU)
				fileIndex = FLASHFILE_COUNT - 3;
			if(fileIndex == FILE_INDEX_CPU0)
				fileIndex = FLASHFILE_COUNT - 2;
			if(fileIndex == FILE_INDEX_CPU1)
				fileIndex = FLASHFILE_COUNT - 1;
			pflashFile = &flashFiles[fileIndex];//文件ID
			pflashFile->fileLen = *(OS_U32*)(frame->au8Data + 2);	//2-5字节为文件长度
			pflashFile->CRC1 = *(OS_U32*)(frame->au8Data + 6);//6-9字节为文件CRC
			memcpy(pflashFile->fileName, frame->au8Data + 10, 128);	//10字节开始前64字节为文件名，后64字节为创建时间
			memset(fileContent, 0, FLASH_FILE_MAX);
			memcpy(fileContent, pflashFile, sizeof(FLASHFILE));
			lastPacketIndex = 0;
			lastPacketCount = 0;
			FlashProgramming = TRUE;
		}
		else if(burnState == UNBURN)
		{
			pflashFile = NULL;
			FlashProgramming = FALSE;
		}
		resBuf[0] = 0xAA;
		resBuf[1] = burnState;
		CmdResponseHandler(CMD_FLASH_CTRL_RSP, 2, resBuf);
	}
		break;
	case CMD_FLASH_ENCAP_REQ:		//flash烧写内容
	{

		OS_U16* totalPacketCount = (OS_U16*)frame->au8Data;
		OS_U16* curPacketIndex = (OS_U16*)(frame->au8Data + 2);
		OS_U16*	effectByteCount = (OS_U16*)(frame->au8Data + 4);
		OS_U8* payloadPtr = (OS_U8*)(frame->au8Data + 6);
		//OS_U8 temp[256];
		//memcpy(temp,payloadPtr, 250);
		OS_U8 writeRes;
		if(*curPacketIndex > *totalPacketCount)
		{
			resBuf[0] = 0x33;
			resBuf[1] = 0b01;	//序号异常
			FlashProgramming = FALSE;
		}
		else if(*curPacketIndex != 0 && lastPacketIndex != *curPacketIndex - 1)
		{
			resBuf[0] = 0x33;
			resBuf[1] = 0b10;	//序号不连续
			FlashProgramming = FALSE;
		}
		else if(lastPacketCount != 0 && lastPacketCount != * totalPacketCount)
		{
			resBuf[0] = 0x33;
			resBuf[1] = 0b100;	//总包数异常
			FlashProgramming = FALSE;
		}
		else if(BURNING != burnState)
		{
			resBuf[0] = 0x33;
			resBuf[1] = 0b1000;	//非烧写状态
			FlashProgramming = FALSE;
		}
		else
		{
			resBuf[1] = 0x00;	//数据接收正常
			lastPacketCount = * totalPacketCount;
			lastPacketIndex = *curPacketIndex;
			memcpy((OS_U8 *)(fileContent + (int)pflashFile->ptrOffset + (int)(*curPacketIndex * MAXPACKETLEN)),
					(OS_U8*)payloadPtr,
					(size_t)*effectByteCount);
			if(*curPacketIndex == *totalPacketCount - 1)
			{
				//先回复地面收到数据，再进行flash烧写工作
				resBuf[0] = 0x11;
				*(OS_U16*)(resBuf + 2) = *totalPacketCount;
				*(OS_U16*)(resBuf + 4) = *curPacketIndex;
				CmdResponseHandler(CMD_FLASH_ENCAP_RSP, 6, resBuf);
				//对文件进行数据校验,并反馈校验结果给地面
				OS_U32 crc = CalCRC32(fileContent + pflashFile->ptrOffset, pflashFile->fileLen,0,0);
				if(crc == pflashFile->CRC1)
				{
					writeRes = QspiFlashProgram(
					(OS_U32)fileContent,
					(OS_U32)(pflashFile->baseAddr),
					(OS_U32)((int)pflashFile->ptrOffset + (int)*curPacketIndex * MAXPACKETLEN + (int)*effectByteCount)
					);
					resBuf[0] = writeRes == 0?0x22:0x33;	//烧写成功0x22,失败0x33
					if((OS_U32)pflashFile->fileID == FILE_INDEX_CPU0)
					{
						OS_U32 Cpu1Len;
						QspiFlashRead(
							(OS_U32)(0x500004),
							(OS_U32)&Cpu1Len,
							sizeof(OS_U32));
						OS_U32 len[2];
						len[0] = pflashFile->fileLen;
						len[1] = Cpu1Len;
						writeRes = QspiFlashProgram(
							(OS_U32) (&len),
							(OS_U32)(0x500000),
							(OS_U32)(sizeof(OS_U32) * 2)
							);
					}
					if((OS_U32)pflashFile->fileID == FILE_INDEX_CPU1)
					{
						OS_U32 Cpu0Len;
						QspiFlashRead(
											(OS_U32)(0x500000),
											(OS_U32)&Cpu0Len,
											sizeof(OS_U32));
						OS_U32 len[2];
						len[0] = Cpu0Len;
						len[1] = pflashFile->fileLen;
						writeRes = QspiFlashProgram(
							(OS_U32) (len),
							(OS_U32)(0x500000),
							(OS_U32)(sizeof(OS_U32)*2)
							);
					}
				}
				else
				{
					resBuf[0] = 0x33;	//校验失败0x33
				}
				lastPacketIndex = 0;
				lastPacketCount = 0;
			}
			else
			{
				resBuf[0] = 0x11;	//烧写过程中

			}
		}
		*(OS_U16*)(resBuf + 2) = *totalPacketCount;
		*(OS_U16*)(resBuf + 4) = *curPacketIndex;
		CmdResponseHandler(CMD_FLASH_ENCAP_RSP, 6, resBuf);
	}
		break;
	case CMD_FLASH_QUERY_REQ:
		{
			FLASHFILE tmpFileInfo={0};
			FLASHFILE *p = flashFiles;
			while(p && p->MagicCode == FLASH_MAGIC_CODE)
			{
				QspiFlashRead((OS_U32)p->baseAddr,(OS_U32)&tmpFileInfo,sizeof(FLASHFILE));
				if(tmpFileInfo.MagicCode == FLASH_MAGIC_CODE)
				{
					OS_U8 fileInfo[200];
					fileInfo[0] = tmpFileInfo.fileID;
					memcpy(fileInfo + 1, &tmpFileInfo.fileLen, 4);
					memcpy(fileInfo + 5, tmpFileInfo.fileName, 128);
					CmdResponseHandler(CMD_FLASH_QUERY_RSP, 133, fileInfo);
				}
				p++;
			}
		}
		break;
	case CMD_FLASH_CLEAR_REQ:
		{
			ClearFlashDatFile();
			OS_U8 data[10];
			CmdResponseHandler(CMD_FLASH_CLEAR_RSP, 0, data);
		}
		break;
	case CMD_FLASH_CHECK_REQ:		//flash烧写校验
		if(BURNING == burnState)	//进入烧写态可校验
		{
			memset(fileContent, 0, FLASH_FILE_MAX);
			OS_U32* checkSum = (OS_U32 *)frame->au8Data;
			OS_U8 readRes = QspiFlashRead(
					(OS_U32)(pflashFile->baseAddr),
					(OS_U32)fileContent,
					sizeof(FLASHFILE));
			readRes = QspiFlashRead(
					(OS_U32)(pflashFile->baseAddr + pflashFile->ptrOffset),
					(OS_U32)(fileContent + pflashFile->ptrOffset),
					pflashFile->fileLen);
			OS_U32 fileLen = ((FLASHFILE*)fileContent)->fileLen;
			OS_U32 calcCheckSum = CalCRC32(fileContent + pflashFile->ptrOffset, fileLen,0,0);
			if(readRes == 0)
			{
				if(*checkSum == calcCheckSum)
				{
					resBuf[0] = 0;	//校验正常
					if(!strcmp(pflashFile->fileName, "9.Ephemeris0.dat"))
					{
						ephRecvReadyFlag = TRUE;
					}
				}
				else
				{
					resBuf[0] = 2;	//校验错误
				}
			}
			else
			{
				resBuf[0] = readRes;	//文件读取错误或其它错误
			}
			*(OS_U32*)(resBuf + 1) = fileLen;//诸元数据长度
			//32位校验和
			*(OS_U32*)(resBuf + 5) = calcCheckSum;
			CmdResponseHandler(CMD_FLASH_CHECK_RSP, 9, resBuf);
			FlashProgramming = FALSE;
		}
		break;
	}

	return 0;
}

