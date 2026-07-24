#ifndef __MODECUQUEUE_H__
#define __MODECUQUEUE_H__

#include <stdint.h>

/** 发动机状态机的状态定义 */
typedef enum {
    ENGINE_STOPED = 0,      // 停机
    ENGINE_WARMUP = 1,      // 启动中
    ENGINE_RUNNING = 5,     // 运行
    ENGINE_SHUTTING_DOWN = 2,// 散热
} SM_Engine_t;

/** 发动机错误码定义 */
typedef enum {
    NO_ERROR = 0,           // 无异常
    ERROR_FUEL_PRESSURE = 1,// 启动流程中，油压异常，油压<=2800即为异常
} SM_EngineError_t;

// HACK: TEST ECU 参数定义
/** 发动机状态参数 */
struct EngineStatus
{
    int16_t ambient_temp;      // 指令6 ，环境温度，整数，有符号实际数值，[0~9999]
    uint16_t air_pressure;      // 指令8 ，环境气压，无符号，单位mbar，[0~9999]
    uint16_t fuel_pressure; //*指令9 ，实际油压，单位mbar，[0~9999]

    uint16_t jet1_duty;     // 指令19，实际喷油1脉宽，单位us，[0~9999]
    uint16_t jet2_duty;     // 指令39，实际喷油2脉宽，单位us，[0~9999]

    uint16_t throttle_state;    //*指令35，风门状态，0001为关闭，0000未关闭
    uint16_t maxTemp;           // cmd59：最高温度超此门限，以最高气缸为准调节
    uint16_t expect_rpm;        // 指令64，期望风门位置百分比*10倍，定速模式下期望的转速值，[0~9999]
    uint16_t rpm;           //*指令69，实际转速，[0~9999]
    uint16_t expect_throttle;   // 指令86，期望的油门位置，[0~1000]

    uint16_t ch1_temp;      // 指令87，通道1实际温度值，单位℃，[0~9999]
    uint16_t ch2_temp;      // 指令88，通道2实际温度值，单位℃，[0~9999]
    uint16_t ch3_temp;      // 指令89，通道3实际温度值，单位℃，[0~9999]
    uint16_t ch4_temp;      // 指令90，通道4实际温度值，单位℃，[0~9999]

    uint16_t battA;             // cmd91：系统输入电流
    uint16_t battV;             // cmd96：系统输入电压
    uint16_t actual_throttle;// 指令97，实际风门舵机位置，%，[0~100]
    uint16_t actual_air_choke;  // 指令98，实际挡风板位置，%，[0~100]
    uint16_t version;           // cmd100:系统固件版本
    uint16_t totalMinite;       // cmd117：系统总时间（min）
    uint16_t runningMinite;     // cmd119：油泵总时间（min）
    SM_Engine_t CntState;       //*当前发动机的状态
    SM_EngineError_t ecuError; // 发动机错误码
};

/** 油泵启动 */
void modECU_pumpOn(void);

/** 油泵停机 */
void modECU_pumpOff(void);

/** 发动机启动 */
void modECU_startEngine(void);

/** 发动机停机 */
void modECU_stopEngine(void);

/** 设置油门开度
 * @param:油门百分比，0.0~100.0，对应0%~100%
 */
void modECU_setThrottle_percent(float percent);

/** 获取参数，解析数据存入结构体全局变量 */
void modECU_GetEngineStatus(struct EngineStatus *pstatus);

/** 拉起任务 */
void modECU_EngineInit(void);

#endif  // __MODECUQUEUE_H__
