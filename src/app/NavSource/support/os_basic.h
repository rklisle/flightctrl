#ifndef _OS_BASIC_H_
#define _OS_BASIC_H_

#include "./os_framework.h"

OS_BOOL Chk2in3U16(OS_U16 u16Data1, OS_U16 u16Data2, OS_U16 u16Data3, OS_U16* pu16DataOut);

OS_VOID SetBit_U32(OS_U32* pu32In, OS_U8 u8Index);

OS_VOID ClearBit_U32(OS_U32* pu32In, OS_U8 u8Index);

OS_VOID SetBit_U16(OS_U16* pu16In, OS_U8 u8Index);

OS_VOID ClearBit_U16(OS_U16* pu16In, OS_U8 u8Index);

OS_VOID SetBit_U8(OS_U8* pu8In, OS_U8 u8Index);

OS_VOID ClearBit_U8(OS_U8* pu8In, OS_U8 u8Index);

OS_BOOL Chk16CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16 CRCValue);

unsigned short crc16_ccitt(const unsigned char *data, int length) ;
OS_VOID Insert16CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16* pu16CrcAddr);

OS_VOID Insert32CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U32* pu32CrcAddr);

OS_U32 CalCRC32(OS_U8 * array, OS_U32 nDatalen, unsigned int lastCrc, unsigned int useLastCrc);

OS_U8 CalCRC8(OS_U8 * data, OS_U32 len);

OS_U16 CheckSum16(OS_U16 *buff, OS_U16 len);

OS_U8 CheckSum8(OS_U8 *buff, OS_U16 len);

OS_U8 XorSum8(OS_U8 *buff, OS_U16 len);

OS_U16 CalCKS(unsigned short *pData, unsigned int dwNumOfBytes);

OS_U16 ROR( const unsigned short data, unsigned int wordIndex);

OS_U16 ROL(const unsigned short data, unsigned int wordIndex);

OS_VOID Insert16CKS_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16* pu16CrcAddr);

OS_U8* SwapBuff_4Byte(OS_U8* buf);

OS_U8* SwapBuff_2Byte(OS_U8* buf);

void ConSys_atmosphere(double H, double par[]);

#endif
