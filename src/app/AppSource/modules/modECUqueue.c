#include "modECUqueue.h"
#include "UART_STM32H7xx.h"
#include "tx_api.h"
#include "../../uart_agent.h"
#include "../Interface/interface_timer.h"
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

static void prv_engine_state_machine(int32_t fd, struct ecu_cmd *pcmd);
static void prv_engine_warmup_state_machine(int32_t fd);
static void prv_engine_shutdown_state_machine(int32_t fd);
static void prv_Generate_Throttle_Cmd(float percent, uint8_t *buffer);
static bool prv_check_sum(uint8_t *pbuf);
static void prv_analyse_data(uint8_t *pbuf);
static int32_t prv_analyse(uint8_t *pbuf, int32_t len);
static void prv_engine_task(ULONG thread_input);
static void prv_Set_Throttle_Percent(int32_t fd, float percent);

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
    int32_t fd;
    fd = fcs_uart_init( &byte_pool_0,
                        &Driver_USART7,
                        115200,
                        ARM_USART_PARITY_NONE,
                        ARM_USART_STOP_BITS_1);
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
        prv_engine_state_machine(fd, pcmd);

        rxlen = fcs_uart_recv(fd, &msg_buf[wt_idx], sizeof(msg_buf) - wt_idx);

        if(rxlen > 0)
        {
            // 此时数组里有 (rxlen + wt_idx) 个数
            wt_idx = prv_analyse(msg_buf, (rxlen + wt_idx));
        }
    }
}

/** 发动机状态机
 * 处理发动机启动、停机、油泵启动、油泵停止、设置油门指令
 * */
static void prv_engine_state_machine(int32_t fd, struct ecu_cmd *pcmd)
{
    /*********** 发动机状态机 ***************** */
    switch(s_engineStatus.CntState)
    {
        case ENGINE_STOPED:
            {
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_START))
                {
                    s_engineStatus.CntState = ENGINE_WARMUP;
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_PUMP_ON))
                {
                    fcs_uart_send(fd, (const uint8_t *)&CMD_PUMP_ON, sizeof(CMD_PUMP_ON));
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_PUMP_OFF))
                {
                    fcs_uart_send(fd, (const uint8_t *)&CMD_PUMP_OFF, sizeof(CMD_PUMP_OFF));
                }
            }
            break;
        case ENGINE_WARMUP:
            prv_engine_warmup_state_machine(fd);
            break;
        case ENGINE_RUNNING:
            {
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_STOP))
                {
                    s_engineStatus.CntState = ENGINE_SHUTTING_DOWN;
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_THO))
                {
                    prv_Set_Throttle_Percent(fd, pcmd->cmd_para.tho_val);
                }
            }
            break;
        case ENGINE_SHUTTING_DOWN:
            prv_engine_shutdown_state_machine(fd);
            break;
    }
}

/*********** 启动状态机 ***************** */
static void prv_engine_warmup_state_machine(int32_t fd)
{
    static uint8_t s_StartEngineRetryCnt = 0;
    static uint32_t s_startTime = 0;
    static uint32_t s_sleepTime = 0;
    static bool isfirstFail_fuel_pressure = true;
    static uint32_t s_firstFail_fuel_pressure = 0;
    static bool isfirstFail_rpm = true;
    static uint32_t s_firstFail_rpm = 0;

    /** 启动状态机的状态定义 */
    typedef enum {
        INIT_PWM = 0,
        OUTPUT_PWM_1MS,//********* */
        SEND_CMD_PUMP_ON,
        SLEEP,
        CHECK_FUEL_PRESSURE,
        SEND_CMD_IGNITION1_ON,
        SEND_CMD_IGNITION2_ON,
        SEND_CMD_CHOKE_ON,
        SEND_CMD_RPM_MODE_OFF,
        SEND_CMD_Throttle_Percent,
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
            fcs_uart_send(fd, (const uint8_t *)&CMD_PUMP_ON, sizeof(CMD_PUMP_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = CHECK_FUEL_PRESSURE; // 正式代码（试车用）
            // s_next_state = SEND_CMD_IGNITION1_ON;   // FIXME: 桌面测试代码
            break;
        case CHECK_FUEL_PRESSURE:
            if(s_engineStatus.fuel_pressure > 2800)
            // && (s_engineStatus.fuel_pressure < 3200))   // 指令9：实际油压
            {
                s_current_state = SEND_CMD_IGNITION1_ON;
                isfirstFail_fuel_pressure = true;
            }
            else    // 油压异常
            {
                // 加一个超时判断
                // 如果是第一次进入else分支，记录时间
                if(isfirstFail_fuel_pressure)
                {
                    s_firstFail_fuel_pressure = tx_time_get();
                    isfirstFail_fuel_pressure = false;
                }

                // 检查是否超时，最多等待1分钟
                if((tx_time_get() - s_firstFail_fuel_pressure) > TIMEOUT_FUEL_PRESSURE)
                {
                    // 上报油压异常
                    s_engineStatus.ecuError = ERROR_FUEL_PRESSURE;
                    fcs_uart_send(fd, (const uint8_t *)&CMD_PUMP_OFF, sizeof(CMD_PUMP_OFF));
                    // 确认超时，进入失败状态
                    s_current_state = START_FAILED;
                    // 重置标志，为下一次启动做准备
                    isfirstFail_fuel_pressure = true;
                }
            }
            break;
        case SEND_CMD_IGNITION1_ON:
            fcs_uart_send(fd, (const uint8_t *)&CMD_IGNITION1_ON, sizeof(CMD_IGNITION1_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_IGNITION2_ON;
            break;
        case SEND_CMD_IGNITION2_ON:
            fcs_uart_send(fd, (const uint8_t *)&CMD_IGNITION2_ON, sizeof(CMD_IGNITION2_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_CHOKE_ON;
            break;
        case SEND_CMD_CHOKE_ON:
            fcs_uart_send(fd, (const uint8_t *)&CMD_CHOKE_ON, sizeof(CMD_CHOKE_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_RPM_MODE_OFF;
            break;
        case SEND_CMD_RPM_MODE_OFF:
            fcs_uart_send(fd, (const uint8_t *)&CMD_RPM_MODE_OFF, sizeof(CMD_RPM_MODE_OFF));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = SEND_CMD_Throttle_Percent;
            break;
        case SEND_CMD_Throttle_Percent://FIXME 启动油门25%
            prv_Set_Throttle_Percent(fd, 25.0f);
            s_current_state = OUTPUT_PWM_2MS;
            break;
        case OUTPUT_PWM_2MS:
            PulseServo_SetPulseWidth(ECU_PWM8, 2);
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SLEEP;
            s_next_state = CHECK_RPM; // 正式代码（试车用）
            // s_next_state = START_SUCCESS;   // FIXME: 桌面测试代码
            break;
        case CHECK_RPM:
            if(s_engineStatus.rpm > 2000)// 检查转速是否2000以上
            {
                s_current_state = START_SUCCESS;
                isfirstFail_rpm = true;
            }
            else    // 转速异常
            {
                if(isfirstFail_rpm)
                {
                    s_firstFail_rpm = tx_time_get();
                    isfirstFail_rpm = false;
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
                        //失败次数过多，进入5min冷却时间(20260204会议记录：人工控制下次启动按钮，by刘强)
                        // s_startTime = tx_time_get();
                        // s_sleepTime = WAIT_1MIN * 5;
                        s_current_state = START_FAILED; //SLEEP;
                        // s_next_state = START_FAILED;
                    }
                    isfirstFail_rpm = true;
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
            s_engineStatus.CntState = ENGINE_RUNNING;
            prv_Set_Throttle_Percent(fd, 0.0f);//FIXME 启动成功，油门0%
            break;
        case START_FAILED://发动机状态置为stoped
            PulseServo_Deinit(ECU_PWM8);
            s_current_state = INIT_PWM;//为下次做准备
            s_StartEngineRetryCnt = 0;
            s_engineStatus.CntState = ENGINE_STOPED;
            break;
    }
}

/*********** 停机状态机 ***************** */
static void prv_engine_shutdown_state_machine(int32_t fd)
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
            fcs_uart_send(fd, (const uint8_t *)&CMD_STOP_ENGINE, sizeof(CMD_STOP_ENGINE));
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
            s_engineStatus.CntState = ENGINE_STOPED;
            break;
        case STOP_FAILED:
            s_current_state = SEND_CMD_STOP_ENGINE;//为下次做准备
            s_StopEngineRetryCnt = 0;
            s_engineStatus.CntState = ENGINE_RUNNING;
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
// static bool prv_check_sum(uint8_t *pbuf)
// {
//     uint8_t sum = pbuf[0] + pbuf[1] + pbuf[2];
//     if(pbuf[3] == sum)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }
static bool prv_check_sum(uint8_t *pbuf)
{
    return (pbuf[3] == (pbuf[0] + pbuf[1] + pbuf[2])) ? true : false;
}

/** 根据发动机协议，数据解析，存入发动机状态参数结构体 */
static void prv_analyse_data(uint8_t *pbuf)
{
    switch (pbuf[0])
    {
    case 6:
        s_engineStatus.ambient_temp     = (pbuf[1] << 8) + pbuf[2];
        break;
    case 8:
        s_engineStatus.air_pressure     = (pbuf[1] << 8) + pbuf[2];
        break;
    case 9:
        s_engineStatus.fuel_pressure    = (pbuf[1] << 8) + pbuf[2];
        break;
    case 19:
        s_engineStatus.jet1_duty        = (pbuf[1] << 8) + pbuf[2];
        break;
    case 35:
        s_engineStatus.throttle_state   = (pbuf[1] << 8) + pbuf[2];
        break;
    case 39:
        s_engineStatus.jet2_duty        = (pbuf[1] << 8) + pbuf[2];
        break;
    case 59:
        s_engineStatus.maxTemp          = (pbuf[1] << 8) + pbuf[2];
        break;
    case 64:
        s_engineStatus.expect_rpm       = (pbuf[1] << 8) + pbuf[2];
        break;
    case 69:
        s_engineStatus.rpm              = (pbuf[1] << 8) + pbuf[2];
        break;
    case 86:
        s_engineStatus.expect_throttle  = (pbuf[1] << 8) + pbuf[2];
        break;
    case 87:
        s_engineStatus.ch1_temp         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 88:
        s_engineStatus.ch2_temp         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 89:
        s_engineStatus.ch3_temp         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 90:
        s_engineStatus.ch4_temp         = (pbuf[1] << 8) + pbuf[2];
        break;
    case 91:
        s_engineStatus.battA            = (pbuf[1] << 8) + pbuf[2];
        break;
    case 96:
        s_engineStatus.battV            = (pbuf[1] << 8) + pbuf[2];
        break;
    case 97:
        s_engineStatus.actual_throttle  = (pbuf[1] << 8) + pbuf[2];
        break;
    case 98:
        s_engineStatus.actual_air_choke = (pbuf[1] << 8) + pbuf[2];
        break;
    case 100:
        s_engineStatus.version          = (pbuf[1] << 8) + pbuf[2];
        break;
    case 117:
        s_engineStatus.totalMinite      = (pbuf[1] << 8) + pbuf[2];
        break;
    case 119:
        s_engineStatus.runningMinite    = (pbuf[1] << 8) + pbuf[2];
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
    
    for(i = 0; i < len-1; i++)
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
            // lenght pass, now check sum
            pfrm = &pbuf[i-5];
            if((prv_check_sum(pfrm) == true) && ((*pfrm <= 245)))
            {
                prv_analyse_data(pfrm);
            }
            unprocess_bytes = 0;
        }
    }
    memmove(pbuf, &pbuf[len - unprocess_bytes - 1], (unprocess_bytes));
    
    return unprocess_bytes;
}

/** @brief 从queue中取到3号命令时，去设置油门
 * @param fd 串口
 * @param percent 油门百分比（0-100.0，支持一位小数）
 */
static void prv_Set_Throttle_Percent(int32_t fd, float percent)
{
    uint8_t throttle_cmd[6];
    
    /* 动态生成油门命令 */
    prv_Generate_Throttle_Cmd(percent, throttle_cmd);
    
    /* 发送命令 */
    fcs_uart_send(fd, (const uint8_t *)&throttle_cmd, sizeof(throttle_cmd));
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
