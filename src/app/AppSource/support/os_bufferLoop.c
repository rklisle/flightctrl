/*
 * os_bufferLoop.c
 *
 *  Created on: 2022年4月29日
 *      Author: Lenovo
 */

#include "os_bufferLoop.h"
#include "os_error.h"

BUFF_LOOP buffLoop[MODULE_COUNT] = {0};

OS_U8 InitBuffLoop(OS_U8 rtIndex)
{
	buffLoop[rtIndex].syncHead_A = STANDARD_HEADA;
	buffLoop[rtIndex].syncHead_B = STANDARD_HEADB;
	buffLoop[rtIndex].lenExtern = 8;	//普通422消息，头部6字节，校验和2字节不算入长度字段
	buffLoop[rtIndex].head = 0;
	buffLoop[rtIndex].tail = 0;
	buffLoop[rtIndex].lenPos = 0;	//同步头后第n个字节为长度
	buffLoop[rtIndex].fixedLen = 0;
	buffLoop[rtIndex].inited = TRUE;
	return 0;

}

/**函数功能：
 * 串口数据压入循环缓冲的内存池
 * 参数1：哪一串口号，对应哪个内存池
 * 参数2：数据指针
 * 参数3：数据长度
 * 返回值：0成功 -1失败
 */
OS_U8 PushLoopMem(OS_U8 rtIndex, OS_U8 *mem, OS_U16 len)
{
	if(buffLoop[rtIndex].inited == FALSE)
		InitBuffLoop(rtIndex);
	if(len > BUFFER_LOOP_POOL_MAX)
	{//单次压入的内存长度已经大于循环缓存的大小
		return -1;
	}
	for (int idx = 0; idx < len; idx++)//将本次接收数据存入循环缓存
	{
		buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].tail++] = mem[idx];
		if(buffLoop[rtIndex].tail == BUFFER_LOOP_POOL_MAX)
		{
			buffLoop[rtIndex].tail = 0;
		}

	}
	//OS_U16 count = (buffLoop[rtIndex].tail + BUFFER_LOOP_POOL_MAX - buffLoop[rtIndex].head) % BUFFER_LOOP_POOL_MAX ;
	//char info[50] = {0};
	//sprintf(info,"\nrecv info, lenth is %d, head is:%x, head1 is %x",count,buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head],
	//		buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head + 1]);
	//PrintDebug(info);
	return 0;
}

/**函数功能：
 * 从内存池中弹出一帧数据
 * 参数1：根据串口号选择哪个内存池
 * 参数2：数据存到哪里
 * 参数3：数据长度
 * 返回值：0成功 负值：各种失败
 */
OS_S32 PopFrameLoopMem(OS_U8 rtIndex, OS_U8 *memFrame, OS_U16* u16len)
{
	//在地面测发控过程中，如果使用了无线方式，则测控前端与测控后端之间使用无线数据通信
	//这个过程使得地面的上行指令可能无法在5ms内发送至箭上，当箭上每5ms调用一次422接口
	//数据时，导致收取的数据不是一整帧，因此需要做循环缓存以解决断帧问题。

	//数据接收完成后，计算头尾指针的差值，获取多次累积的包长度
	int len = (buffLoop[rtIndex].tail + BUFFER_LOOP_POOL_MAX - buffLoop[rtIndex].head) % BUFFER_LOOP_POOL_MAX;



	if(len < FRAME_MIN_LEN)
	{//未获得一个整帧，等待下一次数据
		return ERROR_LENGTH_LESS_ZERO;
	}
    
    if(rtIndex == RT_ENGINE)
    {
		// 014应该不会进入这里
        while(len > 28)
        {
            OS_U16 msgLen = 0;
            if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == 0xFF))	// 数据帧头
            {
                msgLen = 53;	// 协议中对应此数据帧头的数据长度
            }
            else if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == 0xFE))
            {
                msgLen = 29;
            }
            else if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == 0xFD))
            {
                msgLen = 41;
            }
            if(len < msgLen)	// ringbuffer中数据少于一帧数据
            {
                return ERROR_LENGTH_LESS_ZERO;//head，tail指针保留位置不动，等待下一帧
            }
            if(msgLen > 0)
            {
				// 找到了数据帧头
				// 将完整的一帧数据从ringbuffer中拷贝出来，memFrame，长度 u16len
                for(int idx = 0; idx < msgLen; idx++)
                {
                    memFrame[idx] = buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head++];
                    if(buffLoop[rtIndex].head == BUFFER_LOOP_POOL_MAX)
                    {
                        buffLoop[rtIndex].head = 0;
                    }
                }
                *u16len = msgLen;
                return 0;
            }
            else
            {
				// 没有找到数据帧头
				// 待处理长度len减去1，head指向下一字节
                len--;
                buffLoop[rtIndex].head = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
            }
        }
        return -1;
    }
    if(rtIndex == RT_FUSE)
    {
        while(len >= FRAME_MIN_LEN)
        {
            int head_1 = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
            int temp_len_low = (buffLoop[rtIndex].head + 2 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;
  
            if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == buffLoop[rtIndex].syncHead_A) && (buffLoop[rtIndex].BUFFER[head_1] == buffLoop[rtIndex].syncHead_B))//0xAA55
            {
                OS_U16 msgLen = 4 + buffLoop[rtIndex].BUFFER[temp_len_low];	// 4 + (0x0B = 11 Byte) = 总共15Byte
                if(len < msgLen)//return NULL的重要标志，说明本次接收仍未接收够整包数
                {
                    return ERROR_LENGTH_LESS_ZERO;//head，tail指针保留位置不动，等待下一帧
                }
                for(int idx = 0; idx < msgLen; idx++)
                {
                    memFrame[idx] = buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head++];
                    if(buffLoop[rtIndex].head == BUFFER_LOOP_POOL_MAX)
                    {
                        buffLoop[rtIndex].head = 0;
                    }
                }
                *u16len = msgLen;
                return 0;
            }
            else
            {
                //找不到0x55AA头
                //正常情况下不会找不到的，进到本分支说明数据已经错乱，此时不要想补救了，需要立刻清空状态。
                //buffLoop[rtIndex].head = buffLoop[rtIndex].tail = 0;
                //return ERROR_LENGTH_LESS_ZERO;
                len--;
                buffLoop[rtIndex].head = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;

            }
        }
        return -1;
    }
    if(rtIndex == RT_DATA_LINK)
	{
		while(len >= FRAME_MIN_LEN)
		{
			int head_1 = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
			int temp_len_low = (buffLoop[rtIndex].head + 2 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;
			int temp_len_high = (buffLoop[rtIndex].head + 3 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;

			if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == 0xEB) && (buffLoop[rtIndex].BUFFER[head_1] == 0x90))//0x55AA
			{
				OS_U16 msgLen;

				OS_U8 temp_len[2];
				temp_len[0] = buffLoop[rtIndex].BUFFER[temp_len_low];
				temp_len[1] = buffLoop[rtIndex].BUFFER[temp_len_high];
				memcpy(&msgLen, temp_len, 2);
				msgLen += 4;
				if(len < msgLen)
				{
					return ERROR_LENGTH_LESS_ZERO;
				}
				for(int idx = 0; idx < msgLen; idx++)
				{
					memFrame[idx] = buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head++];
					if(buffLoop[rtIndex].head == BUFFER_LOOP_POOL_MAX)
					{
						buffLoop[rtIndex].head = 0;
					}
				}
				*u16len = msgLen;
				return 0;
			}
			else
			{
				len--;
				buffLoop[rtIndex].head = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
			}
		}
        return -1;
	}
	while(len >= FRAME_MIN_LEN)
	{
		int head_1 = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
		int temp_len_low = (buffLoop[rtIndex].head + 2 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;
		int temp_len_high = (buffLoop[rtIndex].head + 3 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;

		if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == buffLoop[rtIndex].syncHead_A) && (buffLoop[rtIndex].BUFFER[head_1] == buffLoop[rtIndex].syncHead_B))//0x55AA
		{
			// ************************** 根据协议解析出一帧数据的长度msgLen *************************************
			OS_U16 msgLen;
			if(buffLoop[rtIndex].fixedLen == 0)//正常情况，计算长度,2字节长度
			{
				OS_U8 temp_len[2];
				temp_len[0] = buffLoop[rtIndex].BUFFER[temp_len_low];
				temp_len[1] = buffLoop[rtIndex].BUFFER[temp_len_high];
               
				memcpy(&msgLen, temp_len, 2);
				msgLen += buffLoop[rtIndex].lenExtern;
				//msgLen = buffLoop[rtIndex].BUFFER[temp_len] + buffLoop[rtIndex].lenExtern;
			}
			else if(buffLoop[rtIndex].fixedLen == -1)//无长度，按国科环宇上报
			{
				msgLen = len;
			}
			else							//固定长度
			{
				msgLen = buffLoop[rtIndex].fixedLen;
			}
			// ************************* 导航板发过来的数据是169个字节 **************************************
            if(msgLen > 300)
			{
				memset(buffLoop[rtIndex].BUFFER, 0, BUFFER_LOOP_POOL_MAX);
				buffLoop[rtIndex].head = 0;
				buffLoop[rtIndex].tail = 0;
				return -1;
			}
			//测发控发出的整包数据长度
			if(len < msgLen)//return NULL的重要标志，说明本次接收仍未接收够整包数
			{
				return ERROR_LENGTH_LESS_ZERO;//head，tail指针保留位置不动，等待下一帧
			}
			for(int idx = 0; idx < msgLen; idx++)
			{
				memFrame[idx] = buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head++];	// 从数据帧头开始，拷贝整帧数据
				if(buffLoop[rtIndex].head == BUFFER_LOOP_POOL_MAX)
				{
					buffLoop[rtIndex].head = 0;
				}
			}
			*u16len = msgLen;
			return 0;
		}
		else
		{
			//找不到0x55AA头
			//正常情况下不会找不到的，进到本分支说明数据已经错乱，此时不要想补救了，需要立刻清空状态。
			//buffLoop[rtIndex].head = buffLoop[rtIndex].tail = 0;
			//return ERROR_LENGTH_LESS_ZERO;
			len--;
			buffLoop[rtIndex].head = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
		}
	}
	return -1;
}

