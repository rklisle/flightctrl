#ifndef FLIGHTPORT_H
#define FLIGHTPORT_H

#include "../data_protocol.h"
//==================================================================================
//
//						接口，以便C中调用C++函数
//
//==================================================================================
#ifdef __cplusplus
extern "C"{
#endif

// 输入
extern Stru_Data_INS_To_Controller        g_ins_data;       // 控制输入——获取导航信息
extern Stru_Data_Engine_To_Controller     g_engine_data;    // 控制输入——获取转速、发动机状态
extern Stru_Data_Baro_To_Controller       g_baro_data;      // 控制输入——获取空速管信息
// extern Stru_Data_Seeker_To_Controller     g_seeker_data;
// extern Stru_Data_Datalink_To_Controller   g_datalink_data;
// extern Stru_Data_RadioAlt_To_Controller   g_radioalt_data;

// 输出
extern Stru_Data_Controller_To_Engine           g_controller_to_engine;     // 控制输出——控油门
extern Stru_Data_Controller_To_Actuator         g_controller_to_actuator;   // 控制输出——控舵机
extern Stru_Data_Controller_To_Switch_Output    g_controller_to_switch;     // 控制输出——标志位：起飞、关发动机、开伞\MML新增引信电激活
extern Stru_Data_Controller_To_DatalinkTel      g_CtrltoDL_tel;             // 控制输出——存数据池
// extern Stru_Data_Controller_To_Seeker           g_controller_to_seeker;
// extern Stru_Data_Controller_To_Datalink         g_controller_to_datalink;

// 航点
extern Stru_Route_Data     g_route_data;    // 初始预装订航点信息
extern Stru_Initial_Data   g_initial_data;  // 发射点/起飞点信息
extern Stru_Data_Datalink_To_ControllerSig  g_DLtoCtrl_sig; // 在线更新多个航点

// typedef void(*pre_ctrl_run_callback_t)(void);
// typedef void(*post_ctrl_run_callback_t)(void);

// 函数声明
//void * ControlInitial(pre_ctrl_run_callback_t pre_cb, post_ctrl_run_callback_t post_cb);
void * ControlInitial(void);
void ControlRun(void *v);
void deleteCMathControlMain(void *v);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHTPORT_H */

