#include "os_framework.h"
#include "os_basic.h"
#include <math.h>

static OS_U16 crc16_table[] = 
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

unsigned char crc8_itu_table[] = 
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77,
    0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,
    0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B,
    0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88,
    0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74,
    0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E,
    0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
    0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55,
    0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91,
    0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
    0xFA, 0xFD, 0xF4, 0xF3
};
static inline OS_U16 crc16_byte(OS_U16 crc, const OS_U8 data)
{
	return (crc << 8) ^ crc16_table[((crc >> 8) ^ data) & 0xff];
}

OS_BOOL Chk16CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16 CRCValue)
{
	if((pu8Start == NULL)
		||(u32Length <= 0))
		return OS_FALSE;

	OS_U16 wCRC = 0x0000;

	while ( 0 != (u32Length--) )
		wCRC = crc16_byte(wCRC, *pu8Start++);	

	if(CRCValue == wCRC)
		return OS_TRUE;
	else
		return OS_FALSE;
}
unsigned char reflect_byte(unsigned char byte) {
    unsigned char reflected = 0;
    for (int i = 0; i < 8; i++) {
        reflected |= ((byte >> i) & 0x01) << (7 - i);
    }
    return reflected;
}
unsigned short reflect_uint16(unsigned short value) {
    unsigned short reflected = 0;
    for (int i = 0; i < 16; i++) {
        reflected |= ((value >> i) & 0x01) << (15 - i);
    }
    return reflected;
}
unsigned short crc16_ccitt(const unsigned char *data, int length) {
    unsigned short crc = 0x0000; 
    for (size_t i = 0; i < length; i++) {
        unsigned char byte = data[i];
        byte = reflect_byte(byte);
        crc ^= (unsigned short)byte << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; 
            } else {
                crc <<= 1;
            }
        }
    }
    crc = reflect_uint16(crc); 
    return crc; 
}
OS_U8 CalCRC8(OS_U8 * data, OS_U32 len)
{
    unsigned char crca = data[0];
    unsigned char crcb;
    for(int i = 0; i < len; i++)
    {
        if(i == 0)
        {
            crca = crc8_itu_table[(data[i] ^ 0x00)];
            crcb = crca;
        }
        else
        {
            crca = crc8_itu_table[(data[i] ^ crcb)];
            crcb = crca;
        }
    }
    return (crcb ^ 0x55);
}

OS_VOID Insert16CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16* pu16CrcAddr)
{
	if((pu8Start == NULL)
		||(pu16CrcAddr == NULL)
		||(u32Length <= 0))
		return;

	OS_U16 wCRC = 0x0000;

	while ( 0 != (u32Length--) )
		wCRC = crc16_byte(wCRC, *pu8Start++);

	*pu16CrcAddr = wCRC;
}

OS_VOID Insert16CKS_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U16* pu16CrcAddr)
{
	if((pu8Start == NULL)
		||(pu16CrcAddr == NULL)
		||(u32Length <= 0))
		return;

	OS_U16 cksCheckCalc = CalCKS((OS_U16*)pu8Start, u32Length/2);

	*pu16CrcAddr = cksCheckCalc;
}

static OS_U32 crc32_tab[256];

unsigned int fun_Reflect(unsigned int ref, unsigned char ch)
{
	unsigned int value = 0;
	int i = 0;
	for (i = 1; i<(ch + 1); i++)
	{
		if (ref & 1)
			value |= 1 << (ch - i);

		ref >>= 1;
	}
	return value;
}

void fun_Init_CRC32()
{
	unsigned int ulPolynomial = 0x04c11db7;
	unsigned int i = 0, j = 0;

	for (i = 0; i <= 0xFF; i++) /*256 Values representing ascii character ades*/
	{
		crc32_tab[i] = fun_Reflect(i, 8) << 24;
		for (j = 0; j<8; j++)
		{
			crc32_tab[i] = (crc32_tab[i] << 1) ^ (crc32_tab[i] & (1 << 31) ? ulPolynomial : 0);
		}
		crc32_tab[i] = fun_Reflect(crc32_tab[i], 32);
	}
	return;
}

unsigned int CalCRC32(unsigned char *array, unsigned int nDatalen, unsigned int lastCrc, unsigned int useLastCrc)
{
	static unsigned char s_bInitFlag = 0;
	unsigned int dGain = 0xFFFFFFFF;
    if(useLastCrc)
    {
        dGain = lastCrc;
    }
	int i = 0;

	if(s_bInitFlag == 0)
	{
		s_bInitFlag = 1;
		for (i = 0; i<256; i++)
		{
			crc32_tab[i] = 0x00000000;
		}
		fun_Init_CRC32();
	}

	while (nDatalen--)
    {
		dGain = (dGain >> 8) ^ crc32_tab[(dGain & 0xFF) ^ ((*array++) & 0xFF)];
    }
	return dGain ^ 0xFFFFFFFF;;
}

OS_VOID Insert32CRC_U8(OS_U8* pu8Start, OS_U32 u32Length, OS_U32* pu32CrcAddr)
{
	OS_U32 temp = CalCRC32(pu8Start, u32Length,0,0);

	*pu32CrcAddr = temp;
}

OS_BOOL Chk2in3U16(OS_U16 u16Data1, OS_U16 u16Data2, OS_U16 u16Data3, OS_U16* pu16DataOut)
{
	if (u16Data1 == u16Data2)
	{
		if (pu16DataOut != NULL)
			*pu16DataOut = u16Data1;
		return OS_TRUE;
	}
	else if(u16Data2 == u16Data3)
	{
		if (pu16DataOut != NULL)
			*pu16DataOut = u16Data2;
		return OS_TRUE;
	}
	else if (u16Data1 == u16Data3)
	{
		if (pu16DataOut != NULL)
			*pu16DataOut = u16Data1;
		return OS_TRUE;
	}

	return OS_FALSE;
}

OS_VOID SetBit_U32(OS_U32* pu32In, OS_U8 u8Index)
{
	if ((pu32In == NULL)
		||(u8Index > 31))
		return;

	*pu32In |= 1 << u8Index;

	return;

}

OS_VOID ClearBit_U32(OS_U32* pu32In, OS_U8 u8Index)
{
	if ((pu32In == NULL)
		|| (u8Index > 31))
		return;

	{
		OS_U32 temp = 1;

		*pu32In &= ~(temp << u8Index);
	}

	return;
}

OS_VOID SetBit_U16(OS_U16 * pu16In, OS_U8 u8Index)
{
	if ((pu16In == NULL)
		||(u8Index > 15))
		return;

	*pu16In |= 1 << u8Index;

	return;
}

OS_VOID ClearBit_U16(OS_U16 * pu16In, OS_U8 u8Index)
{
	if ((pu16In == NULL)
		|| (u8Index > 15))
		return;

	{
		OS_U16 temp = 1;

		*pu16In &= ~(temp << u8Index);
	}

	return;
}

OS_VOID SetBit_U8(OS_U8* pu8In, OS_U8 u8Index)
{
	if ((pu8In == NULL)
		||(u8Index > 7))
		return;

	*pu8In |= 1 << u8Index;

	return;	
}

OS_VOID ClearBit_U8(OS_U8* pu8In, OS_U8 u8Index)
{
	if ((pu8In == NULL)
		|| (u8Index > 7))
		return;

	{
		OS_U8 temp = 1;

		*pu8In &= ~(temp << u8Index);
	}

	return;
}

OS_U16 CheckSum16(OS_U16 *buff, OS_U16 len)
{
	OS_U16 i;
	OS_U16 sum = 0;
	for (i=0; i<len; i++)
	{
		sum += *buff;
		buff++;
	}
	return (OS_U16)sum;
}

OS_U8 CheckSum8(OS_U8 *buff, OS_U16 len)
{
	OS_U8 sum = 0;
	int i = 0;
	for(i=0; i<len; i++)
	{
		sum += *buff;
		buff++;
	}
	return sum;
}

OS_VOID RangeValue(OS_FLOAT* pValue, OS_FLOAT left, OS_FLOAT right)
{
	if(pValue == NULL)
		return;

	if(*pValue >= right)
		*pValue = right;

	if(*pValue <= left)
		*pValue = left;
}

OS_U16 CalCKS(unsigned short *pData, unsigned int dwNumOfBytes)
{
	unsigned short wCKS = 0x0000;
	unsigned short *pbDataBuf = pData;
	int wordIndex = 0;
	while (wordIndex < dwNumOfBytes)
		wCKS ^= ROR( *pbDataBuf++, wordIndex++);

	return ROL(wCKS, wordIndex);
}

OS_U16 ROR( const unsigned short data, unsigned int wordIndex)
{
	wordIndex = wordIndex % 16;
    return data >> wordIndex | data << (16 - wordIndex);
}

OS_U16 ROL(const unsigned short data, unsigned int wordIndex)
{
	wordIndex = wordIndex % 16;
	return data << wordIndex | data >> (16 - wordIndex);
}

OS_U8* SwapBuff_4Byte(OS_U8* buf)
{
	OS_U8 temp = buf[0];
	buf[0] = buf[3];
	buf[3] = temp;
	temp = buf[1];
	buf[1] = buf[2];
	buf[2] = temp;
	return buf;
}

OS_U8* SwapBuff_2Byte(OS_U8* buf)
{
	OS_U8 temp = buf[0];
	buf[0] = buf[1];
	buf[1] = temp;
	return buf;
}

OS_U8 XorSum8(OS_U8 *buff, OS_U16 len)
{
	OS_U8 checksum = 0;
	OS_U16 i = 0;
	while(i<len)
	{
		checksum = checksum ^ buff[i];
		i++;
	}
	return checksum;
}

void ConSys_atmosphere(double H, double par[])
{

	//---------------------------大气数据计算函数--------------------------------
	// GRAM大气密度模型, 参数通过诸元装订获取, 优先级1
	// 美国1976标准大气文件,参考文件《远程火箭弹道学》,适用范围0~91km, 优先级2
	//
	// 输入：
	// H       几何高度  m (WGS-84高度近似)
	//
	// 输出：par
	// par[0]  声速      m/s
	// par[1]  大气密度  kg/m3
	// par[2]  压强      Pa
	// par[3]  大气温度  K
	//
	// 调用格式:
	// ConSys_atmosphere(H, par);
	//--------------------------------------------------------------------------

	H = H / 1000;
	double h = H / (1 + H / 6356.766);
	double W, T, p, rou, a=340;
	double psl = 101325;      // 海平面压强，单位Pa
	double rousl = 1.225;     // 海平面空气密度，单位kg/m^3

	if (H < 0)
	{
		W = 1 - h / 44.3308;
		T = 288.15 * W;
		p = psl;
		rou = rousl;
	}
	else if (0 <= H && H <= 11.0191)
	{
		W = 1 - h / 44.3308;
		T = 288.15 * W;
		p = psl * pow(W, 5.2559);
		rou = rousl * pow(W, 4.2559);
	}
	else if (11.0191 < H && H <= 20.0631)
	{
		W = exp((14.9647 - h) / 6.3416);
		T = 216.650;
		p = psl * 1.1953e-1 * W;
		rou = rousl * 1.5898e-1 * W;
	}
	else if (20.0631 < H && H <= 32.1619)
	{
		W = 1 + (h - 24.9021) / 221.552;
		T = 221.552 * W;
		p = psl * 2.5158e-2 * pow(W, -34.1629);
		rou = rousl * 3.2722e-2 * pow(W, -35.1629);
	}
	else if (32.1619 < H && H <= 47.3501)
	{
		W = 1 + (h - 39.7499) / 89.4107;
		T = 250.350 * W;
		p = psl * 2.8338e-3 * pow(W, -12.2011);
		rou = rousl * 3.2618e-3 * pow(W, -13.2011);
	}
	else if (47.3501 < H && H <= 51.4125)
	{
		W = exp((48.6252 - h) / 7.9223);
		T = 270.654;
		p = psl * 8.9155e-4 * W;
		rou = rousl * 9.4920e-4 * W;
	}
	else if (51.4125 < H && H <= 71.8020)
	{
		W = 1 - (h - 59.4390) / 88.2218;
		T = 247.021 * W;
		p = psl * 2.1671e-4 * pow(W, 12.2011);
		rou = rousl * 2.5280e-4 * pow(W, 11.2011);
	}
	else if (71.8020 < H && H <= 86.0)
	{
		W = 1 - (h - 78.0303) / 100.2950;
		T = 200.590 * W;
		p = psl * 1.2274e-5 * pow(W, 17.0816);
		rou = rousl * 1.7632e-5 * pow(W, 16.0816);
	}
	else if (86.0 < H && H <= 91.0)
	{
		W = exp((87.2848 - h) / 5.4700);
		T = 186.870;
		p = psl * (2.2730 + 1.042e-3 * h) * pow(10.0, -6.0) * W;
		rou = rousl * 3.6411e-6 * W;
	}
	else
	{
		T = 186.870;
		p = 0;
		rou = 0;
	}

	par[0] = a;
	par[1] = rou;
	par[2] = p;
	par[3] = T;

}
