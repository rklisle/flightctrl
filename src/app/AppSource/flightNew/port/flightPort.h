#ifndef FLIGHTPORT_H
#define FLIGHTPORT_H

#include "../data_protocol.h"
//==================================================================================
//
//						做接口，以便C中调用C++的函数
//
//==================================================================================
#ifdef __cplusplus
extern "C"{
#endif

// 输入
extern Stru_Data_Seeker_To_Controller     g_seeker_data;
extern Stru_Data_INS_To_Controller        g_ins_data;
// extern Stru_Data_Datalink_To_Controller   g_datalink_data;
extern Stru_Data_Engine_To_Controller     g_engine_data;
extern Stru_Data_Baro_To_Controller       g_baro_data;
extern Stru_Data_RadioAlt_To_Controller   g_radioalt_data;

// 输出
extern Stru_Data_Controller_To_Seeker      g_controller_to_seeker;
// extern Stru_Data_Controller_To_Datalink    g_controller_to_datalink;
extern Stru_Data_Controller_To_Engine      g_controller_to_engine;
extern Stru_Data_Controller_To_Actuator    g_controller_to_actuator;
extern Stru_Data_Controller_To_Switch_Output g_controller_to_switch;

// 函数声明
typedef struct CMathControlMain CMathControlMain;
CMathControlMain* newCMathControlMain();
void ControlInitial(CMathControlMain *v);
void ControlRun(CMathControlMain *v);
void deleteCMathControlMain(CMathControlMain *v);  // 新增：销毁对象

#ifdef __cplusplus
}
#endif

#endif /* FLIGHTPORT_H */

