/*
 * os_bufferLoop.c
 *
 *  Created on: 2022年4月29日
 *      Author: Lenovo
 */
#include <stdbool.h>
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

static bool	prv_is_gps_msg_hdr(OS_U8 msg)
{
	return ((msg == '$') || (msg == '#')) ? true : false;
}
static inline OS_U16 prv_idx_next(OS_U16 cur_idx)
{
	return ((cur_idx+1) >= BUFFER_LOOP_POOL_MAX) ? 0 : (cur_idx + 1);
}
#define		GPS_MSG_TAIL	0x0D0A
OS_S32 PopFrameLoopMem(OS_U8 rtIndex, OS_U8 *memFrame, OS_U16* u16len)
{
	//在地面测发控过程中，如果使用了无线方式，则测控前端与测控后端之间使用无线数据通信
	//这个过程使得地面的上行指令可能无法在5ms内发送至箭上，当箭上每5ms调用一次422接口
	//数据时，导致收取的数据不是一整帧，因此需要做循环缓存以解决断帧问题。

	//数据接收完成后，计算头尾指针的差值，获取多次累积的包长度
	OS_U16	msg_tail = 0;
	int msg_idx;
	int copy_idx = 0;
	int search_len;
	int len = (buffLoop[rtIndex].tail + BUFFER_LOOP_POOL_MAX - buffLoop[rtIndex].head) % BUFFER_LOOP_POOL_MAX;

	if(len < 1)
	{//未获得一个整帧，等待下一次数据
		return ERROR_LENGTH_LESS_ZERO;
	}


	if(rtIndex == RT_GPS_1 || rtIndex == RT_GPS_2)
	{
		msg_idx = buffLoop[rtIndex].head;
		/* missing header */
			//此时有个情况未考虑，即接收了1.5包时，tail必不是尾
		if(	!prv_is_gps_msg_hdr(buffLoop[rtIndex].BUFFER[msg_idx]))
			{
			search_len = len;
			while(search_len > 0)
				{
				if(prv_is_gps_msg_hdr(buffLoop[rtIndex].BUFFER[msg_idx]))
					{
					len = search_len;
					buffLoop[rtIndex].head = msg_idx;
					break;
					}
				msg_idx = prv_idx_next(msg_idx);
				search_len --;
						}
			if(!prv_is_gps_msg_hdr(buffLoop[rtIndex].BUFFER[msg_idx]))
						{
				return ERROR_LENGTH_LESS_ZERO;

								}
							}
		/* when arrived here, we should get a header*/
		if(len <= 3)
		{	// not enough message in buffer

            return ERROR_LENGTH_LESS_ZERO;
        }
        //////////////////////////////////////
		search_len = len;
		while(search_len > 0)
		{	// combine hdr
			msg_tail = (msg_tail << 8);
			msg_tail |= buffLoop[rtIndex].BUFFER[msg_idx];
			memFrame[copy_idx] = buffLoop[rtIndex].BUFFER[msg_idx];
			msg_idx = prv_idx_next(msg_idx);
			if(msg_tail == GPS_MSG_TAIL)
			{
				*u16len = len - search_len;
				buffLoop[rtIndex].head = msg_idx;
				return 0;
			}
			copy_idx++;
			search_len --;
		}
		return ERROR_LENGTH_LESS_ZERO;
	}


    /*
	if(rtIndex == RT_GPS_2)
	{//星历数据，该数据没有头和尾，需要收到至收不到为止
		OS_U16 msgLen = (buffLoop[rtIndex].tail + BUFFER_LOOP_POOL_MAX - buffLoop[rtIndex].head) % BUFFER_LOOP_POOL_MAX;
		//前两字节放长度
		memcpy(memFrame, &msgLen, 2);
		for(int idx = 2; idx < msgLen + 2; idx++)
		{
			memFrame[idx] = buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head++];
			if(buffLoop[rtIndex].head == BUFFER_LOOP_POOL_MAX)
			{
				buffLoop[rtIndex].head = 0;
			}
		}
		*u16len = msgLen + 2;
		return 0;
	}
*/
	while(len >= FRAME_MIN_LEN)
	{
		int head_1 = (buffLoop[rtIndex].head + 1) % BUFFER_LOOP_POOL_MAX;
		int temp_len_low = (buffLoop[rtIndex].head + 2 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;
		int temp_len_high = (buffLoop[rtIndex].head + 3 + buffLoop[rtIndex].lenPos) % BUFFER_LOOP_POOL_MAX;
		if((buffLoop[rtIndex].BUFFER[buffLoop[rtIndex].head] == buffLoop[rtIndex].syncHead_A) && (buffLoop[rtIndex].BUFFER[head_1] == buffLoop[rtIndex].syncHead_B))//0x55AA
		{
			OS_U16 msgLen;
			if(buffLoop[rtIndex].fixedLen == 0)//正常情况，计算长度
			{
				OS_U8 temp_len[2];
				temp_len[0] = buffLoop[rtIndex].BUFFER[temp_len_low];
				temp_len[1] = buffLoop[rtIndex].BUFFER[temp_len_high];
				memcpy(&msgLen, temp_len, 2);
				msgLen += buffLoop[rtIndex].lenExtern;
			}
			else if(buffLoop[rtIndex].fixedLen == -1)//无长度，按国科环宇上报
			{
				msgLen = len;
			}
			else							//固定长度
			{
				msgLen = buffLoop[rtIndex].fixedLen;
			}
			//测发控发出的整包数据长度
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

