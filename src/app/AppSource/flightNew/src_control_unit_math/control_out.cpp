#include <cstring>
#include "control_out.h"
//#include "../../timer.h"
#include "../global_function.h"
//#include "../../sim_monitor.h"
//extern CSimMonitor sim_monitor;
#include "../port/flightPort.h"

CMathControlOut::CMathControlOut()
{
	p_st_control_out_input = NULL;
	p_st_control_out_output = NULL;
	// p_st_debug_monitor = NULL;
	memset(m_u_control, 0, sizeof(double) * 4);
	m_gama = 0.0;
	m_urg = 0.0;
	m_urg_record = 0.0;
	m_urh = 0.0;
	m_urf = 0.0;
	m_u25g = 0.0;
	m_u4g = 0.0;
	m_ug_adrc = 0.0;
	m_u2f = 0.0;
	m_u5f = 0.0;
	m_u5h = 0.0;
	m_ugf = 0.0;
	m_urf_zd = 0.0;
	m_urh_zd = 0.0;
	m_time_control_on = MAX_TIME;
	m_time_separate_booster = MAX_TIME;
	m_time_combat_status = MAX_TIME;
	m_time_combat_delay = MAX_TIME;
}
void CMathControlOut::Initial()
{

}
void CMathControlOut::Run()
{
	Get_Data();
	Calc_Data();
	Send_Data();
	Monitor_Data();
}

void CMathControlOut::Get_Data()
{
	m_gama = p_st_control_out_input->gama;
	m_u25g = p_st_control_out_input->u25g;
	m_u4g = p_st_control_out_input->u4g;
	m_ug_adrc = p_st_control_out_input->ug_adrc;
	m_u2f = p_st_control_out_input->u2f;
	m_u5f = p_st_control_out_input->u5f;
	m_u5h = p_st_control_out_input->u5h;
	m_ugf = p_st_control_out_input->ugf;
	m_urf_zd = p_st_control_out_input->urf_zd;
	m_urh_zd = p_st_control_out_input->urh_zd;
	m_time_control_on = p_st_control_out_input->time_control_on;
	m_time_separate_booster = p_st_control_out_input->time_separate_booster;
	m_time_combat_status = p_st_control_out_input->time_combat_status;
	m_time_combat_delay = p_st_control_out_input->time_combat_delay;
}

void CMathControlOut::Send_Data()
{
	p_st_control_out_output->u1 = m_u_control[0];
	p_st_control_out_output->u2 = m_u_control[1];
	p_st_control_out_output->u3 = m_u_control[2];
	p_st_control_out_output->u4 = m_u_control[3];
}
void CMathControlOut::Calc_Data()
{
	//启控前,无控
	if (flight_time < m_time_control_on)
	{
		m_u_control[0] = 0.0;
		m_u_control[1] = 0.0;
		m_u_control[2] = 0.0;
		m_u_control[3] = 0.0;

//		sim_monitor.flag_stage1 = false;
//		sim_monitor.flag_stage2 = false;
//		sim_monitor.flag_stage3 = false;
	} 
	//助推器分离前
	else if (flight_time < m_time_separate_booster)
	{
		//滚转姿态增稳，内环PD，无外环
		m_urg = m_u25g;
		m_urg = CFlightGlobalFun::Range(m_urg, 5.0);
		//航向姿态增稳，内环D阻尼，无外环
		m_urh = m_u5h;
		//俯仰姿态增稳，内环PD，无外环
		m_urf = m_u2f * cos(m_gama / RTOA) + m_u5f;
		m_urf = CFlightGlobalFun::Range(m_urf,(6.0 - fabs(m_urg)));
		m_urg = CFlightGlobalFun::Range(m_urg, (6.0 - fabs(m_urf)));//无效

		m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);
		m_u_control[1] = CFlightGlobalFun::Range((m_urg + m_urh + m_urf), 7.5);
		m_u_control[2] = CFlightGlobalFun::Range((m_urg - m_urh + m_urf), 7.5);
		m_u_control[3] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);

//		sim_monitor.flag_stage1 = true;
//		sim_monitor.flag_stage2 = false;
//		sim_monitor.flag_stage3 = false;
	} 
	//导引前，巡航段
	else if (flight_time < m_time_combat_status)
	{
		//滚转姿态PD + 姿态I
		m_urg = m_u25g + m_u4g;
		m_urg = CFlightGlobalFun::Range(m_urg, 5.0);
		m_urg_record = m_urg;//更新进入制导前扰动，认为进入制导前为稳态

		//航向: 伪侧滑角三通道过载控制
		m_urh = m_urh_zd;
		
		//俯仰姿态PD + 外回路高度控制
		m_urf = m_u2f + m_u5f + m_ugf;
		m_urf = CFlightGlobalFun::Range(m_urf, (15.0 - fabs(m_urg)));
		
		m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);	//俯仰、滚转
		m_u_control[1] = CFlightGlobalFun::Range((m_urg - m_urf), 15.0);
		m_u_control[2] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[3] = CFlightGlobalFun::Range((m_urg + m_urf), 15.0);
		m_u_control[4] = CFlightGlobalFun::Range(m_urh, 15.0);				//航向控制
		m_u_control[5] = CFlightGlobalFun::Range(m_urh, 15.0);	

		// sim_monitor.flag_stage1 = false;
		// sim_monitor.flag_stage2 = true;
		// sim_monitor.flag_stage3 = false;
	} 
	//导引段，过渡
	else if (flight_time < (m_time_combat_status + m_time_combat_delay))
	{	
		//滚转PD+ADRC
		m_urg = m_u25g + m_ug_adrc;
		//过渡: 巡航扰动过渡到ADRC
		if(flight_time < (m_time_combat_status + 1.0))
		{
			m_urg = (1.0 - (flight_time - m_time_combat_status)) * m_urg_record 
				+ (flight_time - m_time_combat_status) * m_urg;
		}
		m_urg = CFlightGlobalFun::Range(m_urg, 5.0);
		//航向: 伪侧滑角三通道过载控制
		m_urh = m_urh_zd;
		
		//俯仰姿态PD + 外回路高度控制
		m_urf = m_u2f + m_u5f + m_ugf;
		m_urf = CFlightGlobalFun::Range(m_urf, (7.5 - fabs(m_urg)));

		m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);
		m_u_control[1] = CFlightGlobalFun::Range((m_urg + m_urh + m_urf), 7.5);
		m_u_control[2] = CFlightGlobalFun::Range((m_urg - m_urh + m_urf), 7.5);
		m_u_control[3] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);
	} 
	//导引段，正式
	else
	{
		//滚转同上
		m_urg = m_u25g + m_ug_adrc;
		if(flight_time < (m_time_combat_status + 1.0))
		{
			m_urg = (1.0 - (flight_time - m_time_combat_status)) * m_urg_record 
				+ (flight_time - m_time_combat_status) * m_urg;
		}
		//俯仰，伪攻角三通道过载控制
		m_urf = m_urf_zd;
		//限幅，保滚转
		m_urf = CFlightGlobalFun::Range(m_urf, (7.5 - fabs(m_urg)));
		
		//航向
		m_urh = m_urh_zd;
		//限幅
		m_urg = CFlightGlobalFun::Range(m_urg, 6.0);
		
		//舵控分配
		m_u_control[0] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);
		m_u_control[1] = CFlightGlobalFun::Range((m_urg + m_urh + m_urf), 7.5);
		m_u_control[2] = CFlightGlobalFun::Range((m_urg - m_urh + m_urf), 7.5);
		m_u_control[3] = CFlightGlobalFun::Range((m_urg - m_urf), 7.5);

		// sim_monitor.flag_stage1 = false;
		// sim_monitor.flag_stage2 = false;
		// sim_monitor.flag_stage3 = true;
	}
}

// void CMathControlOut::Monitor_Data()
// {
// 	if (sim_monitor.flag_monitor2_valid)
// 	{
// 		sim_monitor.Get_Variable(m_u25g,"u25g",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_u4g,"u4g",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_ug_adrc,"ugADRC",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_u2f,"u2f",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_u5f,"u5f",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_ugf,"ugf",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_u5h,"u5h",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_urh_zd,"unh",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_urg,"urg",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_urh,"urh",ENUM_FILE_CONTROL1);
// 		sim_monitor.Get_Variable(m_urf,"urf",ENUM_FILE_CONTROL1);
// 	}
// }