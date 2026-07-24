#include <cstddef>
#include "../src_control_unit_math/control_main.h"
#include "flightPort.h"

// ����
Stru_Data_INS_To_Controller        g_ins_data = {0};
Stru_Data_Engine_To_Controller     g_engine_data = {0};
Stru_Data_Baro_To_Controller       g_baro_data = {0};
Stru_Data_Seeker_To_Controller     g_seeker_data = {0};
//Stru_Data_Datalink_To_Controller   g_datalink_data = {0};
// Stru_Data_RadioAlt_To_Controller   g_radioalt_data = {0};

// ���
Stru_Data_Controller_To_Engine        g_controller_to_engine = {0};
Stru_Data_Controller_To_Actuator      g_controller_to_actuator = {0};
Stru_Data_Controller_To_Switch_Output g_controller_to_switch = {0};
Stru_Data_Controller_To_DatalinkTel     g_CtrltoDL_tel = {0};
// Stru_Data_Controller_To_Seeker        g_controller_to_seeker = {0};
// Stru_Data_Controller_To_Datalink      g_controller_to_datalink = {0};

// ����
Stru_Initial_Data   g_initial_data = {0};//��ʼ�������ݣ�
Stru_Route_Data     g_route_data = {0};//��ʼԤװ��������Ϣ
Stru_Data_Datalink_To_ControllerSig     g_DLtoCtrl_sig = {0};//�����������߸��º�����Ϣ

// ������ʱ��ȫ�ֱ�������
double   g_flight_time = 0.0;
int      g_time_tick = 0;

//static pre_ctrl_run_callback_t s_pre_cb = NULL;
//static post_ctrl_run_callback_t s_post_cb = NULL;

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

        // ����
        p->p_st_data_ins_to_controller      = &g_ins_data;
        p->p_st_data_engine_to_controller   = &g_engine_data;
        p->p_st_data_baro_to_controller     = &g_baro_data;
        // p->p_st_data_seeker_to_controller = &g_seeker_data;
        // p->p_st_data_datalink_to_controller = &g_datalink_data;
        // p->p_st_data_radioalt_to_controller = &g_radioalt_data;
        
        // ���
        p->p_st_data_controller_to_engine       = &g_controller_to_engine;
        p->p_st_data_controller_to_actuator     = &g_controller_to_actuator;
        p->p_st_data_controller_to_switch_output= &g_controller_to_switch;
        p->p_st_data_controller_to_datalinktel  = &g_CtrltoDL_tel;
        // p->p_st_data_controller_to_seeker = &g_controller_to_seeker;
        // p->p_st_data_controller_to_datalink = &g_controller_to_datalink;
        
        // ����
        p->p_st_route_data_preflight = &g_route_data;
        p->p_st_initial_data = &g_initial_data;
        p->p_st_data_datalink_to_controllersig = &g_DLtoCtrl_sig;

        // ��������ʱ�������ֱ��ʹ��ȫ�ֱ�����ַ��
        p->flight_time = g_flight_time;  // ע�⣺��ֻ�ǳ�ʼ��ʱ��ֵ
        p->time_tick = g_time_tick;
        
        p->Initial();

		return p;
    }
    
    void ControlRun(void *v){
        CMathControlMain *pinst = (CMathControlMain *)v;
        if(pinst == NULL)
        {   
            return;
        }
        // �ؼ���ÿ������ǰ��ȫ�ֱ���ͬ����ʵ��
        pinst->flight_time = g_flight_time;
        pinst->time_tick = g_time_tick;
        
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

