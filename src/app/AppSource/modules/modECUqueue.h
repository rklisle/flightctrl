#ifndef __MODECUQUEUE_H__
#define __MODECUQUEUE_H__

#include <stdint.h>

/** ������״̬����״̬���� */
typedef enum {
    ENGINE_STOPED = 0,      // ͣ��
    ENGINE_WARMUP = 1,      // ������
    ENGINE_RUNNING = 5,     // ����
    ENGINE_SHUTTING_DOWN = 2,// ɢ��
    ENGINE_ERROR = 3, //����ʧ�ܣ�����
} SM_Engine_t;

/** �����������붨�� */
typedef enum {
    NO_ERROR = 0,
    WARMUP_FAIL = 1,
} SM_EngineError_t;

/** ������״̬���� */
struct EngineStatus
{
    /* electrical parameters */
    uint16_t UMainPwr;      // CMD96 [0~9999] X0.01
    uint16_t IIgnition1;    // CMD92 [0~9999] X0.1
    uint16_t IIgnition2;    // CMD93 [0~9999] X0.1
    /* fuel supply */
    uint16_t Actual_fuel_pressure; // CMD9 [0~9999]
    uint16_t Fuel_pump_duty_cycle; // CMD17 [0~100]
    uint16_t Actual_jet1_duty_cycle;// CMD19 [0~9999]
    uint16_t Actual_jet2_duty_cycle;// CMD39 [0~9999]
    /* temp & air pressure */
    uint16_t Air_pressure; // CMD8 [0~9999]
    uint16_t Ambient_temperatur; // CMD6 [0~9999]
    uint16_t CH_Temperature1; // CMD87 [0~9999]
    uint16_t CH_Temperature2; // CMD88 [0~9999]
    uint16_t CH_Temperature3; // CMD89 [0~9999]
    uint16_t CH_Temperature4; // CMD90 [0~9999]
    /* control */
    uint16_t Ignition1_on_off_flag; // CMD1 [0~1]
    uint16_t Ignition2_on_off_flag; // CMD5 [0~1]
    uint16_t Pump_on_off_flag; // CMD2 [0~1]
    uint16_t RPM_Regulator_on_off_flag; // CMD3 [0~1]
    uint16_t Choke_on_off_flag; // CMD4 [0~1]
    /* rpm */
    uint16_t Actual_RPM;   // CMD69 [0~9999]
    uint16_t The_2nd_RPM_value;   // CMD144 [0~9999]
    
    uint16_t throttle_state;    // CMD35 [0~1]
    uint16_t feedback_throttle;   // CMD86 [0~1000] X0.1
    uint16_t battA;             // CMD91 [0~9999] X0.1
    SM_Engine_t CurState;      //
    SM_EngineError_t ecuError; //
};

/** �ͱ����� */
void modECU_pumpOn(void);

/** �ͱ�ͣ�� */
void modECU_pumpOff(void);

/** ���������� */
void modECU_startEngine(void);

/** ������ͣ�� */
void modECU_stopEngine(void);

/** �������ſ���
 * @param:[0-100.0]
 */
void modECU_setThrottle_percent(float percent);

/** ��ȡ�������������ݴ���ṹ��ȫ�ֱ��� */
void modECU_GetEngineStatus(struct EngineStatus *pstatus);

/** �������� */
void modECU_EngineInit(void);

#endif  // __MODECUQUEUE_H__
