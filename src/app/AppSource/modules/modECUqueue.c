#include "modECUqueue.h"
#include "UART_STM32H7xx.h"
#include "tx_api.h"
#include "../../uart_agent.h"
#include "../Interface/interface_timer.h"
#include "BusInteract.h"
#include <assert.h>
// #include "../support/os_framework.h"

/************      宏定义    ***********/ //FIXME: 宏定义
#define TASK_STACK_SIZE 2048
#define TASK_PRIORITY   (TX_MAX_PRIORITIES-4)
#define WAIT_50MS 50
#define WAIT_1S 1000
#define WAIT_1MIN 60000

#define MAX_START_RETRY 3   // 启动流程，最大重试次数
#define MAX_STOP_RETRY 3    // 关机流程，最大重试次数
#define PWM_1MS_DURATION   (2 * WAIT_1S) //  启动流程，输出1ms的PWM波形时间
#define TIMEOUT_FUEL_PRESSURE   WAIT_1MIN //  启动流程，判油压的最大等待时间
#define TIMEOUT_CHECK_RPM   (10 * WAIT_1S)  // 启动流程，判转速的最大等待时间
#define TIMEOUT_CHECK_THO   (10 * WAIT_1S)  // 关机流程，判风门的最大等待时间

/******************************** 发动机命令 ************************************** */
/* 格式：地址 + 数据高 + 数据低 + 校验和 + 0x0D + 0x0A */

const uint8_t CMD_PUMP_ON[6] = {0x02, 0x00, 0x01, 0x03, 0x0D, 0x0A};           // 打开燃油泵
const uint8_t CMD_PUMP_OFF[6] = {0x02, 0x00, 0x00, 0x02, 0x0D, 0x0A};          // 关闭燃油泵

const uint8_t CMD_IGNITION1_ON[6] = {0x01, 0x00, 0x01, 0x02, 0x0D, 0x0A};      // 打开点火器1
const uint8_t CMD_IGNITION1_OFF[6] = {0x01, 0x00, 0x00, 0x01, 0x0D, 0x0A};     // 关闭点火器1

const uint8_t CMD_IGNITION2_ON[6] = {0x05, 0x00, 0x01, 0x06, 0x0D, 0x0A};      // 打开点火器2
const uint8_t CMD_IGNITION2_OFF[6] = {0x05, 0x00, 0x00, 0x05, 0x0D, 0x0A};     // 关闭点火器2

const uint8_t CMD_CHOKE_ON[6] = {0x04, 0x00, 0x01, 0x05, 0x0D, 0x0A};          // 打开强制喷油
const uint8_t CMD_CHOKE_OFF[6] = {0x04, 0x00, 0x00, 0x04, 0x0D, 0x0A};         // 关闭强制喷油

const uint8_t CMD_RPM_MODE_OFF[6] = {0x03, 0x00, 0x00, 0x03, 0x0D, 0x0A};      // 关闭定速模式（手动模式）
const uint8_t CMD_RPM_MODE_ON[6] = {0x03, 0x00, 0x01, 0x04, 0x0D, 0x0A};       // 开启定速模式

const uint8_t CMD_STOP_ENGINE[6] = {0x41, 0x00, 0x01, 0x42, 0x0D, 0x0A};       // 停车指令 指令号65

const uint8_t CMD_CLOSE_THROTTLE[6] = {0x23, 0x00, 0x01, 0x24, 0x0D, 0x0A};    //关闭风门 指令号35
const uint8_t CMD_OPEN_THROTTLE[6] = {0x23, 0x00, 0x00, 0x23, 0x0D, 0x0A};     //打开风门

/************************************** 变量定义 ******************************************* */
enum ecu_cmd_type
{
    ECU_CMD_ENGIN_START = 1,
    ECU_CMD_ENGIN_STOP = 2,
    ECU_CMD_ENGIN_THO = 3,
    ECU_CMD_PUMP_ON = 4,
    ECU_CMD_PUMP_OFF = 5,
};

struct ecu_cmd
{
    enum ecu_cmd_type cmd;
    // 3 dump bytes
    union {
        uint32_t retry_cnt;
        float    tho_val;
    } cmd_para;
};

static TX_QUEUE s_ecu_cmd_queue;
static uint8_t s_ecu_cmd_queue_buffer[8 * sizeof(struct ecu_cmd)];

static struct EngineStatus s_engineStatus = {0};

extern TX_BYTE_POOL byte_pool_0;

/************************************** 私有函数声明 ******************************************* */

static void prv_engine_state_machine(struct ecu_cmd *pcmd);
static void prv_engine_warmup_state_machine();
static void prv_engine_shutdown_state_machine();
static void prv_Generate_Throttle_Cmd(float percent, uint8_t *buffer);
static bool prv_check_sum(uint8_t *pbuf);
static void prv_analyse_data(uint8_t *pbuf);
static int32_t prv_analyse(uint8_t *pbuf, int32_t len);
static void prv_engine_task(ULONG thread_input);
static void prv_Set_Throttle_Percent(float percent);

/************************************** 私有函数 ******************************************* */
static TX_THREAD engine_task_tcb;
static UCHAR engine_task_stack[TASK_STACK_SIZE];

static void prv_engine_task(ULONG thread_input)
{
    // 如果不使用参数，可以添加这行避免编译警告
    (void)thread_input;
    uint8_t msg_buf[128];
    int32_t wt_idx = 0;
    int32_t rxlen = 0;
    struct ecu_cmd cmd;
    struct ecu_cmd * pcmd;
    // int32_t fd;
    // fd = fcs_uart_init( &byte_pool_0,
    //                     &Driver_USART7,
    //                     115200,
    //                     ARM_USART_PARITY_NONE,
    //                     ARM_USART_STOP_BITS_1);
    while(1)
    {
        if(TX_SUCCESS == tx_queue_receive(&s_ecu_cmd_queue, &cmd, 5))
        {
            pcmd = &cmd;
        }
        else
        {
            pcmd = NULL;
        }
        prv_engine_state_machine(pcmd);

        rxlen = fcs_uart_recv(RT_ENGINE, &msg_buf[wt_idx], sizeof(msg_buf) - wt_idx);

        if(rxlen > 0)
        {
            g_DeviceState.ecuCountDown = 200;

            // 此时数组里有 (rxlen + wt_idx) 个数
            wt_idx = prv_analyse(msg_buf, (rxlen + wt_idx));
        }
    }
}

/** 发动机状态机
 * 处理发动机启动、停机、油泵启动、油泵停止、设置油门指令
 * */
static void prv_engine_state_machine(struct ecu_cmd *pcmd)
{
    /*********** 发动机状态机 ***************** */
    switch(s_engineStatus.CurState)
    {
        case ENGINE_STOPED:
            {
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_START))
                {
                    s_engineStatus.CurState = ENGINE_WARMUP;
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_PUMP_ON))
                {
                    fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_PUMP_ON, sizeof(CMD_PUMP_ON));
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_PUMP_OFF))
                {
                    fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_PUMP_OFF, sizeof(CMD_PUMP_OFF));
                }
            }
            break;
        case ENGINE_WARMUP:
            prv_engine_warmup_state_machine();
            break;
        case ENGINE_RUNNING:
            {
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_STOP))
                {
                    s_engineStatus.CurState = ENGINE_SHUTTING_DOWN;
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_THO))
                {
                    prv_Set_Throttle_Percent(pcmd->cmd_para.tho_val);
                }
            }
            break;
        case ENGINE_SHUTTING_DOWN:
            prv_engine_shutdown_state_machine();
            break;
        case ENGINE_ERROR:
            break;
    }
}

/*********** 启动状态机 ***************** */
static void prv_engine_warmup_state_machine()
{
    static uint8_t s_StartEngineRetryCnt = 0;
    static uint32_t s_startTime = 0;
    static uint32_t s_sleepTime = 0;
    static bool isfirstFail_fuel_pressure = true;
    static uint32_t s_firstFail_fuel_pressure = 0;
    static bool s_isfirstFail_rpm = true;
    static bool s_isfirstRunToHere = true;
    static uint32_t s_firstFail_rpm = 0;

    /** 启动状态机的状态定义 */
    typedef enum {
        INIT_PWM = 0,
        OUTPUT_PWM_1MS,//********* */
        SEND_CMD_PUMP_ON,
        SLEEP,
        SEND_CMD_IGNITION1_ON,
        SEND_CMD_IGNITION2_ON,
        SEND_CMD_CHOKE_ON,
        OUTPUT_PWM_2MS,
        CHECK_RPM,//*********** */
        START_SUCCESS,
        START_FAILED,
    } SM_StartEngine_t;
    static SM_StartEngine_t s_current_state = INIT_PWM;
    static SM_StartEngine_t s_next_state;

    switch (s_current_state)
    {
        case INIT_PWM:
            s_engineStatus.ecuError = NO_ERROR;
            PulseServo_Init(ECU_PWM8, 1);
        case OUTPUT_PWM_1MS:
            PulseServo_SetPulseWidth(ECU_PWM8, 1);
            s_startTime = tx_time_get();
            s_sleepTime = PWM_1MS_DURATION;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_PUMP_ON;
            break;
        case SEND_CMD_PUMP_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_PUMP_ON, sizeof(CMD_PUMP_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_IGNITION1_ON;
            break;
        case SEND_CMD_IGNITION1_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_IGNITION1_ON, sizeof(CMD_IGNITION1_ON));

            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_IGNITION2_ON;
            break;
        case SEND_CMD_IGNITION2_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_IGNITION2_ON, sizeof(CMD_IGNITION2_ON));

            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_CHOKE_ON;
            break;
        case SEND_CMD_CHOKE_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_CHOKE_ON, sizeof(CMD_CHOKE_ON));
            s_current_state = OUTPUT_PWM_2MS;
            break;
        case OUTPUT_PWM_2MS:
            PulseServo_SetPulseWidth(ECU_PWM8, 2);
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = CHECK_RPM;
            break;
        case CHECK_RPM:
            if(s_engineStatus.Actual_RPM > 2000)// 检查转速是否2000以上
            {
                s_isfirstFail_rpm = true;
                if(s_isfirstRunToHere)
                {
                    s_startTime = tx_time_get();
                    s_isfirstRunToHere = false;
                }
                else
                {
                    // 判断时间是否大于10s
                    if((tx_time_get() - s_startTime) > TIMEOUT_CHECK_RPM)
                    {
                        s_current_state = START_SUCCESS;
                        s_isfirstRunToHere = true;
                        s_isfirstFail_rpm = true;
                    }
                }
			}
            else
            {
                // 转速不满足条件
                s_isfirstRunToHere = true;
                if(s_isfirstFail_rpm)
                {
                    s_firstFail_rpm = tx_time_get();
                    s_isfirstFail_rpm = false;
                }

                if((tx_time_get() - s_firstFail_rpm) > TIMEOUT_CHECK_RPM)
                {
                    if(s_StartEngineRetryCnt < (MAX_START_RETRY - 1))
                    {
                        s_StartEngineRetryCnt++;
                        s_current_state = OUTPUT_PWM_1MS;
                    }
                    else
                    {
                        //失败次数过多，启动失败
                        s_current_state = START_FAILED;
                    }
                    s_isfirstFail_rpm = true;
                    s_isfirstRunToHere = true;
                }
            }
            break;
        case SLEEP:
            if((tx_time_get() - s_startTime) > s_sleepTime) {
                s_current_state = s_next_state;
            }
            break;
        case START_SUCCESS:
            PulseServo_Deinit(ECU_PWM8);
            s_current_state = INIT_PWM;//为下次做准备
            s_StartEngineRetryCnt = 0;
            s_engineStatus.CurState = ENGINE_RUNNING;
            break;
        case START_FAILED://发动机状态置为stoped
            PulseServo_Deinit(ECU_PWM8);
            s_current_state = INIT_PWM;//为下次做准备
            s_StartEngineRetryCnt = 0;
            s_engineStatus.CurState = ENGINE_ERROR;
            s_engineStatus.ecuError = WARMUP_FAIL;
            break;
    }
}

/*********** 停机状态机 ***************** */
static void prv_engine_shutdown_state_machine()
{
    static uint8_t s_StopEngineRetryCnt = 0;
    static uint32_t s_startTime = 0;
    static uint32_t s_sleepTime = 0;
    static bool isfirstFail = true;
    static uint32_t s_firstFail = 0;

    /** 停机状态机的状态定义 */
    typedef enum {
        SEND_CMD_STOP_ENGINE,//********* */
        CHECK_THROTTLE,
        STOP_SUCCESS,
        STOP_FAILED,
    } SM_StopEngine_t;
    static SM_StopEngine_t s_current_state = SEND_CMD_STOP_ENGINE;

    switch (s_current_state)
    {
        case SEND_CMD_STOP_ENGINE:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_STOP_ENGINE, sizeof(CMD_STOP_ENGINE));
            s_current_state = CHECK_THROTTLE;
            break;
        case CHECK_THROTTLE:
            if(s_engineStatus.throttle_state == 1)// 检查风门是否关闭 35号指令，1为关闭，0未关闭
            {
                s_current_state = STOP_SUCCESS;
                isfirstFail = true;
            }
            else    // 关闭异常
            {
                if(isfirstFail)
                {
                    s_firstFail = tx_time_get();
                    isfirstFail = false;
                }

                if((tx_time_get() - s_firstFail) > TIMEOUT_CHECK_THO)
                {
                    if(s_StopEngineRetryCnt < (MAX_STOP_RETRY-1))
                    {
                        s_StopEngineRetryCnt++;
                        s_current_state = SEND_CMD_STOP_ENGINE;
                    }
                    else
                    {
                        // 启动失败次数太多。
                        s_current_state = STOP_FAILED;
                    }
                    isfirstFail = true;
                }
            }
            break;
        case STOP_SUCCESS:
            s_current_state = SEND_CMD_STOP_ENGINE;//为下次做准备
            s_StopEngineRetryCnt = 0;
            s_engineStatus.CurState = ENGINE_STOPED;
            break;
        case STOP_FAILED:
            s_current_state = SEND_CMD_STOP_ENGINE;//为下次做准备
            s_StopEngineRetryCnt = 0;
            s_engineStatus.CurState = ENGINE_RUNNING;
            break;
    }
}


/** @brief 动态生成油门命令（任意百分比）
 * @param percent 油门百分比（0-100.0，支持一位小数）
 * @param buffer 输出缓冲区（至少6字节）
 */
static void prv_Generate_Throttle_Cmd(float percent, uint8_t *buffer) {
    /* 将百分比转换为0-1000的值 */
    uint16_t value = (uint16_t)(percent * 10.0f);  // 25.5% -> 255
    uint8_t data_h = (value >> 8) & 0xFF;         // 高字节
    uint8_t data_l = value & 0xFF;                // 低字节
    
    /* 构建命令 */
    buffer[0] = 0x40;                   // 地址64
    buffer[1] = data_h;                 // 数据高位
    buffer[2] = data_l;                 // 数据低位
    buffer[3] = 0x40 + data_h + data_l; // 校验和
    buffer[4] = 0x0D;                   // 结束符1
    buffer[5] = 0x0A;                   // 结束符2
}

/** 根据发动机协议，检查校验和 */
static bool prv_check_sum(uint8_t *pbuf)
{
    return (pbuf[3] == (pbuf[0] + pbuf[1] + pbuf[2])) ? true : false;
}

/** 根据发动机协议，数据解析，存入发动机状态参数结构体 */
static void prv_analyse_data(uint8_t *pbuf)
{
    switch (pbuf[0])
    {
    case 1:
        s_engineStatus.Ignition1_on_off_flag        = (pbuf[1] << 8) + pbuf[2];
        break;
    case 2:
        s_engineStatus.Pump_on_off_flag             = (pbuf[1] << 8) + pbuf[2];
        break;
    case 3:
        s_engineStatus.RPM_Regulator_on_off_flag    = (pbuf[1] << 8) + pbuf[2];
        break;
    case 4:
        s_engineStatus.Choke_on_off_flag            = (pbuf[1] << 8) + pbuf[2];
        break;
    case 5:
        s_engineStatus.Ignition2_on_off_flag        = (pbuf[1] << 8) + pbuf[2];
        break;
    case 6:
        s_engineStatus.Ambient_temperatur           = (pbuf[1] << 8) + pbuf[2];
        break;
    case 8:
        s_engineStatus.Air_pressure                 = (pbuf[1] << 8) + pbuf[2];
        break;
    case 9:
        s_engineStatus.Actual_fuel_pressure         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 17:
        s_engineStatus.Fuel_pump_duty_cycle         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 19:
        s_engineStatus.Actual_jet1_duty_cycle       = (pbuf[1] << 8) + pbuf[2];
        break;
    case 35:
        s_engineStatus.throttle_state               = (pbuf[1] << 8) + pbuf[2];
        break;
    case 39:
        s_engineStatus.Actual_jet2_duty_cycle       = (pbuf[1] << 8) + pbuf[2];
        break;
    case 69:
        s_engineStatus.Actual_RPM                   = (pbuf[1] << 8) + pbuf[2];
        break;
    case 86:
        s_engineStatus.feedback_throttle            = (pbuf[1] << 8) + pbuf[2];
        break;
    case 87:
        s_engineStatus.CH_Temperature1              = (pbuf[1] << 8) + pbuf[2];
        break;
    case 88:
        s_engineStatus.CH_Temperature2              = (pbuf[1] << 8) + pbuf[2];
        break;
    case 89:
        s_engineStatus.CH_Temperature3              = (pbuf[1] << 8) + pbuf[2];
        break;
    case 90:
        s_engineStatus.CH_Temperature4              = (pbuf[1] << 8) + pbuf[2];
        break;
    case 91:
        s_engineStatus.battA                        = (pbuf[1] << 8) + pbuf[2];
        break;          
    case 92:            
        s_engineStatus.IIgnition1                   = (pbuf[1] << 8) + pbuf[2];
        break;          
    case 93:            
        s_engineStatus.IIgnition2                   = (pbuf[1] << 8) + pbuf[2];
        break;          
    case 96:            
        s_engineStatus.UMainPwr                     = (pbuf[1] << 8) + pbuf[2];
        break;
    case 144:
        s_engineStatus.The_2nd_RPM_value            = (pbuf[1] << 8) + pbuf[2];
        break;
    default:
        break;
    }
}

/** 解析数据 
 * @param 接收到的数据
 * @param 数组长度
 * @return 还有多少个未处理的数据
*/
static int32_t prv_analyse(uint8_t *pbuf, int32_t len)
{
    uint16_t unprocess_bytes = 0;
    uint16_t end_key = 0;
	  uint8_t *pfrm;
    int i;
    if(len < 6)
    {
        return len;
    }
    
    for(i = 0; i < len; i++)
    {
        unprocess_bytes++;
        end_key = (end_key<<8) + pbuf[i];       // 0x00 0D  --> 0x0D 0A

        if(end_key == 0x0D0A)
        {
            if(unprocess_bytes <6)
            {   //broken frame, restart again
                unprocess_bytes = 0;
                continue;
            }
            // length pass, now check sum
            pfrm = &pbuf[i-5];
            if((prv_check_sum(pfrm) == true) && ((*pfrm <= 245)))
            {
                prv_analyse_data(pfrm);
            }
            unprocess_bytes = 0;
        }
    }
    memmove(pbuf, &pbuf[len - unprocess_bytes], (unprocess_bytes));
    
    return unprocess_bytes;
}

/** @brief 从queue中取到3号命令时，去设置油门
 * @param percent 油门百分比（0-100.0，支持一位小数）
 */
static void prv_Set_Throttle_Percent(float percent)
{
    uint8_t throttle_cmd[6];
    
    /* 动态生成油门命令 */
    prv_Generate_Throttle_Cmd(percent, throttle_cmd);
    
    /* 发送命令 */
    fcs_uart_send(RT_ENGINE, (const uint8_t *)&throttle_cmd, sizeof(throttle_cmd));
}

/************************************* 对外接口部分 ******************************************** */
void modECU_pumpOn(void)
{
    struct ecu_cmd cmd;
    cmd.cmd = ECU_CMD_PUMP_ON;
    tx_queue_send(&s_ecu_cmd_queue, &cmd, TX_NO_WAIT);
}

void modECU_pumpOff(void)
{
    struct ecu_cmd cmd;
    cmd.cmd = ECU_CMD_PUMP_OFF;
    tx_queue_send(&s_ecu_cmd_queue, &cmd, TX_NO_WAIT);
}

void modECU_startEngine(void)
{
    struct ecu_cmd cmd;
    cmd.cmd = ECU_CMD_ENGIN_START;
    tx_queue_send(&s_ecu_cmd_queue, &cmd, TX_NO_WAIT);
}

void modECU_stopEngine(void)
{
    struct ecu_cmd cmd;
    cmd.cmd = ECU_CMD_ENGIN_STOP;
    tx_queue_send(&s_ecu_cmd_queue, &cmd, TX_NO_WAIT);
}

/** @brief 发油门设置的信号
 * @param percent 油门百分比（0-100.0，支持一位小数）
 */
void modECU_setThrottle_percent(float percent)
{
    struct ecu_cmd cmd;
    cmd.cmd = ECU_CMD_ENGIN_THO;
    cmd.cmd_para.tho_val = percent;
    tx_queue_send(&s_ecu_cmd_queue, &cmd, TX_NO_WAIT);
}

void modECU_GetEngineStatus(struct EngineStatus *pstatus)
{
    memcpy(pstatus, &s_engineStatus, sizeof(s_engineStatus));
}

void modECU_EngineInit(void)
{
    tx_queue_create(&s_ecu_cmd_queue,
                    "ECU Command Queue",
                    sizeof(struct ecu_cmd),
                    s_ecu_cmd_queue_buffer,
                    sizeof(s_ecu_cmd_queue_buffer));

    tx_thread_create(
        &engine_task_tcb,                    // 线程控制块
        "Engine Task",                  // 线程名称
        prv_engine_task,                    // 线程入口函数
        0,                                // 线程输入参数
        engine_task_stack,                  // 堆栈起始地址
        TASK_STACK_SIZE,        // 堆栈大小
        TASK_PRIORITY,            // 线程优先级（较高优先级）
        TASK_PRIORITY,            // 抢占阈值
        TX_NO_TIME_SLICE,                 // 时间片
        TX_AUTO_START                     // 自动启动
    );
}
