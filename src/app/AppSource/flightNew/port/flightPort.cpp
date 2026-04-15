#include <cstddef>
#include "../src_control_unit_math/control_main.h"
#include "flightPort.h"

// 定义全局结构体变量
// Stru_Data_Seeker_To_Controller     g_seeker_data = {0};
Stru_Data_INS_To_Controller        g_ins_data = {0};
//Stru_Data_Datalink_To_Controller   g_datalink_data = {0};
Stru_Data_Engine_To_Controller     g_engine_data = {0};
Stru_Data_Baro_To_Controller       g_baro_data = {0};
// Stru_Data_RadioAlt_To_Controller   g_radioalt_data = {0};

// Stru_Data_Controller_To_Seeker        g_controller_to_seeker = {0};
//Stru_Data_Controller_To_Datalink      g_controller_to_datalink = {0};
Stru_Data_Controller_To_Engine        g_controller_to_engine = {0};
Stru_Data_Controller_To_Actuator      g_controller_to_actuator = {0};
// Stru_Data_Controller_To_Switch_Output g_controller_to_switch = {0};

Stru_Route_Data     g_route_data = {0};
Stru_Initial_Data   g_initial_data = {0};

//static pre_ctrl_run_callback_t s_pre_cb = NULL;
//static post_ctrl_run_callback_t s_post_cb = NULL;

// // 全局指针变量（指向这些结构体，方便赋值给 CMathControlMain）
// Stru_Data_Seeker_To_Controller     *p_g_seeker_data = &g_seeker_data;
// Stru_Data_INS_To_Controller        *p_g_ins_data = &g_ins_data;
// Stru_Data_Datalink_To_Controller   *p_g_datalink_data = &g_datalink_data;
// Stru_Data_Engine_To_Controller     *p_g_engine_data = &g_engine_data;
// Stru_Data_Baro_To_Controller       *p_g_baro_data = &g_baro_data;
// Stru_Data_RadioAlt_To_Controller   *p_g_radioalt_data = &g_radioalt_data;

extern "C"{
    //void* ControlInitial(pre_ctrl_run_callback_t pre_cb, post_ctrl_run_callback_t post_cb)
    void* ControlInitial(void)
    {
        CMathControlMain* p = new CMathControlMain;
        if(p == NULL)
        {
            return NULL;
        }
        // record control function callback
        // s_pre_cb = pre_cb;
        // s_post_cb = post_cb;
        

        // 将全局变量的地址赋给控制模块的指针
//        p->p_st_data_seeker_to_controller = &g_seeker_data;
        p->p_st_data_ins_to_controller = &g_ins_data;
//        p->p_st_data_datalink_to_controller = &g_datalink_data;
        p->p_st_data_engine_to_controller = &g_engine_data;
        p->p_st_data_baro_to_controller = &g_baro_data;
//        p->p_st_data_radioalt_to_controller = &g_radioalt_data;
        
//        p->p_st_data_controller_to_seeker = &g_controller_to_seeker;
//        p->p_st_data_controller_to_datalink = &g_controller_to_datalink;
        p->p_st_data_controller_to_engine = &g_controller_to_engine;
        p->p_st_data_controller_to_actuator = &g_controller_to_actuator;
        // p->p_st_data_controller_to_switch_output = &g_controller_to_switch;
        
        // 航路和初始数据也需要实例化
        p->p_st_route_data_preflight = &g_route_data;
        p->p_st_initial_data = &g_initial_data;
        
        p->Initial();
    }
    
    void ControlRun(void *v){
        CMathControlMain *pinst = (CMathControlMain *)v;
        if(pinst == NULL)
        {   // empty
            return;
        }
        // 
//        if(s_pre_cb != NULL)
//        {   // prepare control input data
//            s_pre_cb();
//        }
        
        pinst->Run();

//        if(s_post_cb != NULL)
//        {   // do control output data
//            s_post_cb();
//        }
    }

    void deleteCMathControlMain(void *v){
        CMathControlMain *pinst = (CMathControlMain *)v;
        if(v) delete v;
    }
}

