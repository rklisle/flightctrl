/********************************************************************************/
/*Write By: Fig                                                                 */
/********************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include "os_flight_io.h"
#include "Ctrl_Law_Typedef.h"

extern ControlPara control_para;

#define Climb_Ctrl 1
#define Level_Ctrl 2
#define Glide_Ctrl 3
#define Guidance_Ctrl 4
#define Climb_Angle 8
#define Glide_Angle -10
#define Climb_Angle_Launch 8 //爬升角度
#define Omega_Limit 3
/******控制增益转换时间**********/
#define HighGainTime 30.0
#define LowGainTime 80.0
void Control_init(FLIGHT_INPUT *Flight_Input);
void SetControlPara(FLIGHT_INPUT *Flight_Input);
void FlyStabilityControl();
void FlyEngineControl();
void EngineMaxRpm();
void ESO_RollAutopilot(double omega_ESO,double u0_ESO);
void RudderTrans(); 
void DftFreqIdentify(float flight_state);
#endif
//
