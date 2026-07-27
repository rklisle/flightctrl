/*
 * common.c
 *
 *  Created on: 2022年4月29日
 *      Author: Lenovo
 */

#include "../core/BusInteract.h"
//#include "../drive/can_zynq.h"
//#include "../modules/modEngineEle.h"
#include "../interface/interface_uart.h"
#include "common.h"
#include "os_bufferLoop.h"
#include <math.h>
#include "log_ctrl.h"

ERROR_RESULT RecvMessageS422FPGA(OS_U8 recvBuf[MAX_SIMUL_FRAME][RECV_422_MAX_LEN + 1], OS_U8 rtIndex, OS_U16 pu16Length[MAX_SIMUL_FRAME]);
STRU_STANDARD_FRAME recvStandFrame;

/********************************************************************************
 * 函数名:ChkStandardFrame()
 *
 * 函数功能:对接收的标准通信格式进行crc校验，注意，并非所有的设备传递的均是标准格式
 * 		在GPS模块及其它载荷模块中，使用的协议并非标准协议，因此，不仅要根据协议选择相应
 * 		的校验模式、根据不同协议理解对于长度的定义，还需要调整字节序以适配标准通信协议
 * 		具体实现请参考ChkGpsStandardFrame及ChkXgdStandardFrame
 * 		不同的校验函数在总线初始化函数中绑定到对应的RT设备中，请参考BusInteract.c的InitRts()
 * 		关注其中对于函数指针ptr_ChkFrameSum的赋值
 * 	返回值：返回值为调整为标准格式后的长度，包含数据区长度+6字节标准通信头+2字节校验和+1字节rt编号
 * ******************************************************************************/
OS_U16 ChkStandardFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	//pmData[1]开始为标准协议格式，pmData[0]为rtIndex。
	//pmData[3]在标准格式协议中以1字节表示有效数据长度
	if(pmData[1] != STANDARD_HEADA || pmData[2] != STANDARD_HEADB)
	{
		return 0;
	}
	OS_U16 payloadLen = pmData[3] + pmData[4]*0x100;
	OS_U16 u16Crc = *(pmData + 7 + payloadLen) + (*(pmData + 7 + payloadLen + 1))*256;
	if(OS_TRUE != Chk16CRC_U8((OS_U8 *)(pmData + 3), payloadLen + 4, u16Crc))
	{
		memset(pmData, 0, sizeof(STRU_STANDARD_FRAME));
		return 0;
	}
	return payloadLen + 9;
}

OS_U16 ChkDataLinkFrame(OS_MEM* pmData)
{
	if(pmData == PTR_NULL)
		return 0;
	if(pmData[1] != STANDARD_HEADA || pmData[2] != STANDARD_HEADB)
	{
		return 0;
	}
	OS_U8 data[500];
	memcpy(data, pmData, 500);
	OS_U16 payloadLen = pmData[3] + pmData[4]*0x100;
	OS_U8 xorRecv = *(pmData + 4 + payloadLen);
	OS_U8 xorCalc = XorSum8((OS_U8 *)(data + 5), payloadLen - 1);
	if(xorRecv != xorCalc)
	{
		memset(pmData, 0, sizeof(STRU_STANDARD_FRAME));
		return 0;
	}
	memmove(pmData + 1, pmData + 11, payloadLen - 7);
	memcpy(data, pmData, 500);
	return payloadLen - 6;
}

/********************************************************************************
 * 函数名:PeekStandardMessage()
 * 函数功能: 函数的需求为：获取一个已经经过校验的，含rt编号的422标准协议帧。
 * 			为了实现该需求，本函数从底层接收本轮收取的原始帧，并通过校验，压队列，并弹出队列第一个
 * 			元素返回。
 * 注意事项: 对接收帧数据的断包、粘包处理在循环缓存中完成。
 * 			对帧数据的消息队列处理，在队列缓存中完成。在本函数中，PushBuff的调用在循环中完成，可能会
 * 			完成多次压队列，但是弹队列的处理本函数只执行一次，因此每次对本函数的调用仅会最多返回一帧。
 * 本次修改内容：1.调取的RecvMessageS422FPGA()函数返回的结构由单内存对象，变为内存指针数组
 * 				2.编辑为标准帧后再压栈，保证弹栈后的标准帧可被直接引用
 * ******************************************************************************/
const STRU_STANDARD_FRAME* PeekStandardMessage()
{
	OS_MEM pmData[MAX_SIMUL_FRAME][RECV_422_MAX_LEN + 1] = {0};//多1字节存储422端口索引号

	ERROR_RESULT result;
	
	//遍历所有422口，接收底层在本轮查询之间收到的所有帧
	for(int rtIndex=0; rtIndex<MODULE_COUNT; rtIndex++)//
	{
		if(rtIndex == RT_ENGINE)
			continue;
		OS_U16 u16Length[MAX_SIMUL_FRAME] = {0};
		result = RecvMessageS422FPGA(pmData,  rtIndex, u16Length);

		//result = RecvMessageS1553B(pmData,  rtIndex, u16Length);
		if(result != ERROR_NOERROR)
			continue;

		//到此处时，pmData数据格式：首字节rt号，后续为422整帧的数据(按各个外设协议的整帧数据)
		//可能会有多个帧
		// 以下这个for是对pmData中数据进行整形。整形后为标准帧。
		for(int i=0; i<MAX_SIMUL_FRAME; i++)
		{
			if((u16Length[i] > 0) && ( rtList[rtIndex].ptr_ChkFrameSum != NULL))
			{
				// if(rtIndex == RT_FUSE)
				// {
				// 	LOG_HEX16(pmData[i], "pmData[0] = ");
				// }
				//先做帧校验检查，再压栈
				//调整pmData[n]的内部排列，使其满足标准帧要求
				OS_U16 standFrameLen = rtList[rtIndex].ptr_ChkFrameSum(pmData[i]);
				PushBuffer(pmData[i], standFrameLen);
			}
			else
			{
				break;
			}
		}
	}
	
	//无论本次轮询所有422接口共获得了多少完整帧，本次函数调用仅返回最前面1帧
	OS_U16 oneFrameLen = 0;
	OS_MEM *p = PopBuffer(&oneFrameLen);
	if ((p != NULL) && (oneFrameLen > 0))
	{
		memcpy(&recvStandFrame, p, oneFrameLen);
		STRU_STANDARD_FRAME *s = &recvStandFrame;
		s->pStand422Data.u8CRCA = s->pStand422Data.au8Data[s->pStand422Data.u16Len];
		s->pStand422Data.u8CRCB = s->pStand422Data.au8Data[s->pStand422Data.u16Len + 1];
		memset(&(s->pStand422Data.au8Data[s->pStand422Data.u16Len]), 0, 2048 - s->pStand422Data.u16Len);
		return s;//返回标准帧格式
	}
	return NULL;
}

/********************************************************************************
 * 函数名:RecvMessageS422FPGA()
 * 函数功能: 尝试从底层接收数据帧，如果接收到不足长度的数据，则保存等待；如果接收到超出一帧的数据，
 * 			则返回一个完整帧，存储剩余的数据；如果接收到多帧数据，则返回所有的完整帧，存储剩余的数据
 * 本次修改内容：1.修改循环缓存调用方式，将循环缓存功能做出单独模块
 * 				2.返回所有完整帧，由调用函数再做后续处理
 * ******************************************************************************/

ERROR_RESULT RecvMessageS422FPGA(OS_U8 recvBuf[MAX_SIMUL_FRAME][RECV_422_MAX_LEN + 1], OS_U8 rtIndex, OS_U16 pu16Length[MAX_SIMUL_FRAME])
{
	OS_U16 MaxLength = RECV_422_MAX_LEN;
	
	OS_S32 uart_len = 0;
	OS_U8 byRecvData[RECV_422_MAX_LEN]={0};
	uart_len  = UART_GetFrame(rtList[rtIndex].chIndex, byRecvData, MaxLength);
	if(uart_len <= 0)
	{
		return ERROR_LENGTH_LESS_ZERO;
	}
	if(rtIndex == RT_DATA_LINK)
	{
		int a = 0;
	}
	//先将本次收到的数据全部压入循环缓存
	PushLoopMem(rtIndex, byRecvData, uart_len);

	//调用pop函数时判断帧的完整性
	OS_U8 oneFrame[RECV_422_MAX_LEN] = {0};
	OS_U16 frameLen = 0;
	int i=0;
	while(PopFrameLoopMem(rtIndex, oneFrame, &frameLen) == 0)
	{
		recvBuf[i][0] = rtIndex;
		memcpy(&recvBuf[i][0] + 1, oneFrame, frameLen);
		pu16Length[i] = frameLen;

		i++;
		if(i >= MAX_SIMUL_FRAME)
			return ERROR_LENGTH_LESS_ZERO;
		//char info[50] = {0};
		//sprintf(info,"\nrecv gps data, len is %d",frameLen);
		//PrintDebug(info);
	}
	return ERROR_NOERROR;
}

/********************************************************************************
 * 函数名:SendCRCed422Message()
 * 函数功能: 发送数据至底层422缓冲区。发送前完成数据校验并填入固定位置。
 * 			时序配电器因协议错误需要做特殊处理，此处按协议长度进行判断，可能会有风险，如果
 * 			有其它设备协议长度与时序配电器一样，则必会出错。需要尽快修复时序配电器。
 * ******************************************************************************/
OS_S32 SendCRCed422Message(OS_U8 ck, OS_U8 ch, OS_MEM* pmData, OS_S16 crcStartOffset, OS_U16 u16Length)
{
	if ((u16Length == 0) || (pmData == NULL) || (u16Length > (_422_PAYLOAD_MAX_LEN + _422_FRAME_HEADER_LEN + _422_FRAME_FOOTER_LEN)))
		return ERROR_INPUT_POINT_IS_NULL;

	if(ck >= USR_UART_PL_CK_COUNT || ch >= USR_UART_PL_CK_422MAX)
		return ERROR_UART_PORT_ILLEGAL;

	pmData[0] = STANDARD_HEADA;
	pmData[1] = STANDARD_HEADB;

	Insert16CRC_U8(pmData+crcStartOffset, u16Length-crcStartOffset-2, (OS_U16*)(pmData + u16Length - 2));
    
    if(ch == rtList[RT_DATA_LINK].chIndex)
	{//透传处理
       
        OS_U8 paoID,guanID;
        GetDataFast(pDataPoolMsn, "msnPaoID", &paoID);
        GetDataFast(pDataPoolMsn, "msnGuaID", &guanID);
        
        static OS_U8 frameID = 05;
		memmove(pmData + 10, pmData, u16Length);
		u16Length += 7;
		pmData[0] = 0xEB;
		pmData[1] = 0x90;
		pmData[2] = (u16Length & 0xFF);
		pmData[3] = ((u16Length >> 8) & 0xFF);
		pmData[4] = paoID;
		pmData[5] = guanID;
        pmData[6] = 0xCB;
		pmData[7] = 0x00;
        pmData[8] = 0x01;
        pmData[9] = frameID++;
        u16Length += 4;
        pmData[u16Length - 1] = XorSum8(pmData + 4, u16Length - 4);
	}
    
	UART_PutBuff(ch,pmData,u16Length);
	return OS_SUCCESS;
}


/********************************************************************************
 * 函数名:PeekCanMessage()
 * 函数功能: 函数的需求为：获取一个以CAN结构描述的数据。
 * 			为了实现该需求，本函数从底层接收本轮收取的原始帧，并通压队列，并弹出队列第一个
 * 			元素返回。
 * ******************************************************************************/
OS_U32 SendCanData[8];
OS_U32 RecvCanData[8];
OS_U8 ConvertData[8] = {0};

const STRU_CAN_MSG* PeekCanMessage()
{
	return PTR_NULL;
}


const double ae = (6378137.0);
const double e2 = (6.69437999014e-3);
const double PI = (3.141592653589793);
const double DTR = (PI / 180.0);

void ConSys_EarthWGS84_To_Launch(double * stateWGS84, double * state,double startLon, double startLat, double startHigh, double startPos);
void ConSys_M3x3_transpose(double M_in[3][3], double M_out[3][3]);
void ConSys_EarthLBH_To_EarthFixed(double * stateLBH, double * stateEg);
void ConSys_cx(double th, double M[3][3]);
void ConSys_cy(double th, double M[3][3]);
void ConSys_cz(double th, double M[3][3]);
void ConSys_Matrix3x3(double Matrix1[3][3], double Matrix2[3][3], double Matrixout[3][3]);
void ConSys_Matrix3x1(double Matrix1[3][3], double Matrix2[3], double Matrixout[3]);
void DoCalcXYZ(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double *x, double *y, double *z, double *vx, double *vy, double *vz);

void DoCalcXYZ(double startLon, double startLat, double startHigh, double startPos, double targetLon, double targetLat, double targetHigh, double vn, double vs, double ve, double *x, double *y, double *z, double *vx, double *vy, double *vz)
{
	startLon *= DTR; startLat *= DTR; startPos *= DTR;
	targetLon *= DTR; targetLat *= DTR;

	double stateWGS84[6];
	stateWGS84[0] = targetLon;
	stateWGS84[1] = targetLat;
	stateWGS84[2] = targetHigh;
	stateWGS84[3] = vn;
	stateWGS84[4] = vs;
	stateWGS84[5] = ve;

	double state[6];
	ConSys_EarthWGS84_To_Launch(stateWGS84, state, startLon, startLat, startHigh, startPos);

	*x = state[0];
	*y = state[1];
	*z = state[2];
	*vx = state[3];
	*vy = state[4];
	*vz = state[5];

}


void ConSys_Matrix3x1(double Matrix1[3][3], double Matrix2[3], double Matrixout[3])
{

	//------------矩阵乘法运算函数------------
	// 矩阵相乘运算 Matrix1*Matrix2 输出矩阵 Matrixout 为3行1列
	//
	// 输入：
	// Matrix1、Matrix2
	//
	// 输出：
	// Matrixout
	//
	// 调用格式:
	// ConSys_Matrix3x1(Matrix1, Matrix2, Matrixout);
	//----------------------------------------

	double sum = 0;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			sum = sum + Matrix1[i][j]*Matrix2[j];
		}
		Matrixout[i] = sum;
		sum = 0;
	}

}

void ConSys_Matrix3x3(double Matrix1[3][3], double Matrix2[3][3], double Matrixout[3][3])
{

	//------------矩阵乘法运算函数------------
	// 矩阵相乘运算 Matrix1*Matrix2 输出矩阵 Matrixout 为3行3列
	//
	// 输入：
	// Matrix1、Matrix2
	//
	// 输出：
	// Matrixout
	//
	// 调用格式:
	// ConSys_Matrix3x1(Matrix1, Matrix2, Matrixout);
	//----------------------------------------

	double sum = 0;
	for(int i=0;i<3;i++)
	{
		for(int k=0;k<3;k++)
		{
			for(int j=0;j<3;j++)
			{
				sum = sum + Matrix1[i][j]*Matrix2[j][k];
			}
			Matrixout[i][k] = sum;
		    sum = 0;
		}
	}

}

void ConSys_cx(double th, double M[3][3])
{

	//--------------X轴旋转函数---------------
	// 三维坐标绕X轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cx(th, M);
	//----------------------------------------

	M[0][0] = 1;
	M[0][1] = 0;
	M[0][2] = 0;
	M[1][0] = 0;
	M[1][1] = cos(th);
	M[1][2] = sin(th);
	M[2][0] = 0;
	M[2][1] = -sin(th);
	M[2][2] = cos(th);

}

void ConSys_cy(double th, double M[3][3])
{

	//--------------Y轴旋转函数---------------
	// 三维坐标绕Y轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cy(th, M);
	//----------------------------------------

	M[0][0] = cos(th);
	M[0][1] = 0;
	M[0][2] = -sin(th);
	M[1][0] = 0;
	M[1][1] = 1;
	M[1][2] = 0;
	M[2][0] = sin(th);
	M[2][1] = 0;
	M[2][2] = cos(th);

}

void ConSys_cz(double th, double M[3][3])
{

	//--------------Z轴旋转函数---------------
	// 三维坐标绕Z轴旋转 生成旋转矩阵
	//
	// 输入：
	// th   rad
	//
	// 输出：
	// M    旋转矩阵
	//
	// 调用格式:
	// ConSys_cz(th, M);
	//----------------------------------------

	M[0][0] = cos(th);
	M[0][1] = sin(th);
	M[0][2] = 0;
	M[1][0] = -sin(th);
	M[1][1] = cos(th);
	M[1][2] = 0;
	M[2][0] = 0;
	M[2][1] = 0;
	M[2][2] = 1;

}

void ConSys_EarthLBH_To_EarthFixed(double * stateLBH, double * stateEg)
{

	//---------------------经纬高计算地固系状态函数--------------------------
	// 输入：
	// stateLBH  经纬高 0-2经纬高
	//
	// 输出：
	// stateEg   地固系(WGS-84)状态 0-2位置
	//
	// 调用格式
	// ConSys_EarthLBH_To_EarthFixed(stateLBH, stateEg);
	//-----------------------------------------------------------------------

	double Lon, Be, H;

	Lon = stateLBH[0];
	Be  = stateLBH[1];
	H   = stateLBH[2];

	double RN = ae / sqrt(1 - e2*sin(Be)*sin(Be));

	double x, y, z;
	x = (RN + H)*cos(Be)*cos(Lon);
	y = (RN + H)*cos(Be)*sin(Lon);
	z = (RN*(1 - e2) + H)*sin(Be);

	*(stateEg + 0) = x;
	*(stateEg + 1) = y;
	*(stateEg + 2) = z;

}

void ConSys_M3x3_transpose(double M_in[3][3], double M_out[3][3])
{

	//------------三维矩阵转置函数------------
	// 3x3矩阵转置函数
	//
	// 输入：
	// M_in
	//
	// 输出：
	// M_out
	//
	// 调用格式:
	// ConSys_M3x3_transpose(M_in, M_out);
	//----------------------------------------

	int i,j;
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			M_out[i][j] = M_in[j][i];
		}
	}

}

void CalcRoxyz(double lon, double lat, double high, double Ge[3][3], double *rox, double *roy, double *roz)
{
	double stateLBH0[3];
	double stateR0[3];
	stateLBH0[0] = lon;
	stateLBH0[1] = lat;
	stateLBH0[2] = high;
	ConSys_EarthLBH_To_EarthFixed(stateLBH0, stateR0);
	double ConSys_R0Fa[3];
	ConSys_Matrix3x1(Ge, stateR0, ConSys_R0Fa);
	*rox = ConSys_R0Fa[0];
	*roy = ConSys_R0Fa[1];
	*roz = ConSys_R0Fa[2];
}

void ConSys_EarthWGS84_To_Launch(double * stateWGS84, double * state,double startLon, double startLat, double startHigh, double startPos)
{

	//---------------------WGS-84系转发射系计算函数--------------------------
	// 输入：
	// stateLBH   地固系(WGS-84)状态 0-2经纬高 3-5北天东速度
	//
	// 输出：
	// state      发射系状态 0-2位置 3-5速度
	//
	// 调用格式
	// ConSys_EarthWGS84_To_Launch(stateWGS84, state);
	//-----------------------------------------------------------------------

	double Lon, Be, H, Vn[3];
	Lon   = stateWGS84[0];
	Be    = stateWGS84[1];
	H     = stateWGS84[2];
	Vn[0] = stateWGS84[3];
	Vn[1] = stateWGS84[4];
	Vn[2] = stateWGS84[5];

	double RN = ae / sqrt(1 - e2*sin(Be)*sin(Be));

	// 地固系位置坐标
	double R_Eg[3];
	R_Eg[0] = (RN + H)*cos(Be)*cos(Lon);
	R_Eg[1] = (RN + H)*cos(Be)*sin(Lon);
	R_Eg[2] = (RN*(1 - e2) + H)*sin(Be);

	double Ge[3][3], Gex[3][3], Gey[3][3], Gez[3][3], Mtemp[3][3];
	ConSys_cy(-(PI / 2 + startPos), Gey);
	ConSys_cx(startLat, Gex);
	ConSys_cz(-(PI / 2 - startLon), Gez);

	ConSys_Matrix3x3(Gey, Gex, Mtemp);
	ConSys_Matrix3x3(Mtemp, Gez, Ge);

	// 发射系状态 但原点在地心
	double statefr[3];
	ConSys_Matrix3x1(Ge, R_Eg, statefr);

	// 北天东坐标系下速度计算
	double MB0[3][3], MA0[3][3], M_EL[3][3];
	ConSys_cz(startLat, MB0);
	ConSys_cy(startPos, MA0);
	ConSys_Matrix3x3(MB0, MA0, M_EL);

	double delta_L = Lon - startLon;

	double MBE[3][3], MdL[3][3];
	ConSys_cz(-Be, MBE);
	ConSys_cx(delta_L, MdL);

	double M_NE[3][3], M_NL[3][3];
	ConSys_Matrix3x3(MBE, MdL, M_NE);
	ConSys_Matrix3x3(M_NE, M_EL, M_NL);

	double M_LN[3][3];
	ConSys_M3x3_transpose(M_NL, M_LN);   // 正交矩阵逆和转置相同

	double V_vector[3];
	ConSys_Matrix3x1(M_LN, Vn, V_vector);

	double ConSys_Rox = 0, ConSys_Roy = 0, ConSys_Roz = 0;
    CalcRoxyz(startLon, startLat, startHigh, Ge, &ConSys_Rox, &ConSys_Roy, &ConSys_Roz);


	*(state + 0) = statefr[0] - ConSys_Rox;
	*(state + 1) = statefr[1] - ConSys_Roy;
	*(state + 2) = statefr[2] - ConSys_Roz;
	*(state + 3) = V_vector[0];
	*(state + 4) = V_vector[1];
	*(state + 5) = V_vector[2];

}

double EcllipesToAltitude(double lon, double lat, double ecllipseHigh)
{
	double a = 6378137;
	double b = 6356752.3142;
	double f = 1/298.257223563;
	double e2 = 2*f - f*f;
	double n = a/sqrt(1-e2*sin(lat)*sin(lat));
	double x = (n+ecllipseHigh)*cos(lat)*cos(lon);
	double y = (n+ecllipseHigh)*cos(lat)*sin(lon);
	double z = (n*(1-e2) + ecllipseHigh)*sin(lat);
	double p = sqrt(x*x + y*y);
	double theta = atan2(z*a, p*b);
	double n1 = a/sqrt(1-e2*sin(theta)*sin(theta));
	double h1 = p/cos(theta) - n1;
	return h1;
}

//根据海拔高度，计算空气密度
OS_U8 uav_density(float alt, float *ru)
{
	float	H,T, PP, RR;
	float	K = 34.163195,
			RL = 1.225,
			C1 = 0.001;//,
			//AL = 340.294;

	H = C1 * alt / (1 + C1 * alt / 6356.766);
	//dH = C1 / (1 + C1 * alt / 6356.766) + (C1*alt*(-1) / pow((1 + C1 * alt / 6356.766),2))*C1 / 6356.766;

	if (H<11)
	{
		T = 288.15 - 6.5 * H;
	PP = pow((288.15 / T) , (-K / 6.5));
	RR = PP / (T / 288.15);
	//dT = -6.5;
	//dRu = RL*((-PP*288.15 / pow(T,2)) + (288.15 / T)*pow((288.15 / T) ,(-K/6.5-1))*(-K / 6.5)*(-288.15 / pow(T,2)))*dT*dH;
	}
		else if (H < 20){
			T = 216.65;
		PP = 0.22336*exp(-K*(H - 11) / 216.65);
		RR = PP / (T / 288.15);
		//dT = 0;
		//dRu = RL*(288.15 / T)*0.22336*exp(-K*(H - 11) / 216.65)*(-K / 216.65)*dH;
	}
		else if (H < 32){
			T = 216.65 + (H - 20);
		PP = 0.054032 * pow((216.65 / T),K);
		RR = PP / (T / 288.15);
		//dT = 1;
		//dRu = RL*((-PP*288.15 / pow(T,2)) + (288.15 / T)*0.054032 * pow((216.65 / T),(K - 1))*K*(-216.65 / pow(T,2)))*dT*dH;
	}
		else if (H < 47){
			T = 228.65 + 2.8 * (H - 32);
		PP = 0.0085666 * pow((228.65 / T),(K / 2.8));
		RR = PP / (T / 288.15);
		//dT = 2.8;
		//dRu = RL*((-PP*288.15 / pow(T , 2)) + (288.15 / T)*0.0085666 * pow((228.65 / T) , (K / 2.8 - 1))*(K / 2.8)*(-228.65 / pow(T,2)))*dT*dH;
	}
		else if(H < 51) {
		T = 270.65;
		PP = 0.0010945 *exp(-K*(H - 47) / 270.65);
		RR = PP / (T / 288.15);
		//dT = 0;
		//dRu = RL*(288.15 / T)*0.0010945 *exp(-K*(H - 47) / 270.65)*(-K / 270.65)*dH;
	}
	else if(H < 71) {
		T = 270.65 - 2.8*(H - 51);
		PP = 0.00066063*pow((270.65 / T),(-K / 2.8));
		RR = PP / (T / 288.15);
		//dT = -2.8;
		//dRu = RL*((-PP*288.15 / pow(T,2)) + (288.15 / T)*0.00066063*pow((270.65 / T) ,(-K / 2.8 - 1))*(-K / 2.8)*(-270.65 / pow(T ,2)))*dT*dH;
	}
	else {
		T = 214.65 - 2 * (H - 71);
		PP = 3.9046e-05 * pow((214.65 / T),(-K / 2));
		RR = PP / (T / 288.15);
		//dT = -2;
		//dRu = RL*((-PP*288.15 / pow(T , 2)) + (288.15 / T)*3.9046e-05 * pow((214.65 / T),(-K / 2 - 1))*(-K / 2)*(-214.65 / pow(T,2)))*dT*dH;
	}


	//TS = T / 288.15;
	//sonic = AL *  sqrt(TS);
	*ru = RL * RR;
	return 0;
}
