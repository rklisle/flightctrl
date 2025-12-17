/*
 * modFlash.h
 *
 *  Created on: 2021年10月22日
 *      Author: Admin
 */

#ifndef SRC_MODFLASH_H_
#define SRC_MODFLASH_H_
// 用于上传下传、烧写的结构体和函数声明

#include "../support/os_framework.h"
#define FLASH_FILE_MAX		(0x500)	//1280
#define FLASH_MAGIC_CODE (0xFEDCBA98)
#define FLASHFILE_COUNT	(7)	//6个诸元文件，1个程序文件

typedef struct tagFlashFile{
	unsigned int MagicCode;
	unsigned int fileID;
	void * baseAddr;
	unsigned int fileLen;
	unsigned int CRC1;
	unsigned int ptrOffset;
	char fileName[128];
    unsigned char updateMark;
}FLASHFILE;

extern FLASHFILE flashFiles[];
extern OS_BOOL FlashProgramming;
extern OS_U8 ephRecvReadyFlag;

extern OS_U32 FlashCmdHandler(STRU_422_MSG_INFO * frame);
extern OS_U32 ReadFileByName(char* fileName, OS_U8* buf);
extern OS_U32 ReadFileByIndex(OS_U8 fileIndex, OS_U8* buf);
extern OS_U32 ReadFileInfoByIndex(OS_U8 fileIndex);
extern char* getNextsptr(char* src, int num);
extern char* AllocFileBuffer(char* filepath, char* mode);
extern void FreeFileBuffer(char* pbuf);
extern OS_U8 AutoTestFlash();
extern void LoadLunchFile();
extern double BinFileToDouble(char *p, char **end);
//extern int FreeFileBuffer(char* pbuf);
#endif /* SRC_MODFLASH_H_ */
