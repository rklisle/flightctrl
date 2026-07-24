#include "modECUqueue.h"
#include "UART_STM32H7xx.h"
#include "tx_api.h"
#include "../../uart_agent.h"
#include "../Interface/interface_timer.h"
#include "BusInteract.h"
#include <assert.h>
// #include "../support/os_framework.h"

/************      锟疥定锟斤拷    ***********/ //FIXME: 锟疥定锟斤拷
#define TASK_STACK_SIZE 2048
#define TASK_PRIORITY   (TX_MAX_PRIORITIES-4)
#define WAIT_50MS 50
#define WAIT_1S 1000
#define WAIT_1MIN 60000

#define MAX_START_RETRY 3   // 锟斤拷锟斤拷锟斤拷锟教ｏ拷锟斤拷锟斤拷锟斤拷源锟斤拷锟�
#define MAX_STOP_RETRY 3    // 锟截伙拷锟斤拷锟教ｏ拷锟斤拷锟斤拷锟斤拷源锟斤拷锟�
#define PWM_1MS_DURATION   (2 * WAIT_1S) //  锟斤拷锟斤拷锟斤拷锟教ｏ拷锟斤拷锟�1ms锟斤拷PWM锟斤拷锟斤拷时锟斤拷
#define TIMEOUT_FUEL_PRESSURE   WAIT_1MIN //  锟斤拷锟斤拷锟斤拷锟教ｏ拷锟斤拷锟斤拷压锟斤拷锟斤拷锟饺达拷时锟斤拷
#define TIMEOUT_CHECK_RPM   (10 * WAIT_1S)  // 锟斤拷锟斤拷锟斤拷锟教ｏ拷锟斤拷�??锟劫碉拷锟斤拷锟饺达拷时锟斤�??
#define TIMEOUT_CHECK_THO   (10 * WAIT_1S)  // 锟截伙拷锟斤拷锟教ｏ拷锟�??凤拷锟脚碉拷锟斤拷锟饺达拷时锟斤�??

/******************************** 锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?? ************************************** */
/* 锟斤拷式锟斤拷锟斤拷址 + 锟斤拷锟捷革�?? + 锟斤拷锟捷�?�拷 + 校锟斤拷锟� + 0x0D + 0x0A */

const uint8_t CMD_PUMP_ON[6] = {0x02, 0x00, 0x01, 0x03, 0x0D, 0x0A};           // 锟斤拷燃锟酵憋拷
const uint8_t CMD_PUMP_OFF[6] = {0x02, 0x00, 0x00, 0x02, 0x0D, 0x0A};          // 锟截憋拷燃锟酵憋�??

const uint8_t CMD_IGNITION1_ON[6] = {0x01, 0x00, 0x01, 0x02, 0x0D, 0x0A};      // 锟津开碉拷锟斤拷锟�??1
const uint8_t CMD_IGNITION1_OFF[6] = {0x01, 0x00, 0x00, 0x01, 0x0D, 0x0A};     // 锟截�??碉拷锟斤拷锟�??1

const uint8_t CMD_IGNITION2_ON[6] = {0x05, 0x00, 0x01, 0x06, 0x0D, 0x0A};      // 锟津开碉拷锟斤拷锟�??2
const uint8_t CMD_IGNITION2_OFF[6] = {0x05, 0x00, 0x00, 0x05, 0x0D, 0x0A};     // 锟截�??碉拷锟斤拷锟�??2

const uint8_t CMD_CHOKE_ON[6] = {0x04, 0x00, 0x01, 0x05, 0x0D, 0x0A};          // 锟斤拷强锟斤拷锟斤拷锟斤�??
const uint8_t CMD_CHOKE_OFF[6] = {0x04, 0x00, 0x00, 0x04, 0x0D, 0x0A};         // 锟截憋拷强锟斤拷锟斤拷锟斤拷

const uint8_t CMD_RPM_MODE_OFF[6] = {0x03, 0x00, 0x00, 0x03, 0x0D, 0x0A};      // 锟截�??讹拷锟斤拷模式锟斤拷锟�?��?�拷模式锟斤�??
const uint8_t CMD_RPM_MODE_ON[6] = {0x03, 0x00, 0x01, 0x04, 0x0D, 0x0A};       // 锟斤拷锟斤拷锟斤拷锟斤拷模式

const uint8_t CMD_STOP_ENGINE[6] = {0x41, 0x00, 0x01, 0x42, 0x0D, 0x0A};       // 停锟斤拷指锟斤拷 指锟斤拷锟�65

const uint8_t CMD_CLOSE_THROTTLE[6] = {0x23, 0x00, 0x01, 0x24, 0x0D, 0x0A};    //锟截�??凤拷锟斤�?? 指锟斤拷锟�35
const uint8_t CMD_OPEN_THROTTLE[6] = {0x23, 0x00, 0x00, 0x23, 0x0D, 0x0A};     //锟津开凤拷锟斤�??

/************************************** 锟斤拷锟斤拷锟斤拷锟斤拷 ******************************************* */
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
extern unsigned char EngineRunningFlag;

/************************************** 私锟�??猴拷锟斤拷锟斤拷锟斤�?? ******************************************* */

static void prv_engine_state_machine(struct ecu_cmd *pcmd);
static void prv_engine_warmup_state_machine(void);
static void prv_engine_shutdown_state_machine();
static void prv_Generate_Throttle_Cmd(float percent, uint8_t *buffer);
static bool prv_check_sum(uint8_t *pbuf);
static void prv_analyse_data(uint8_t *pbuf);
static int32_t prv_analyse(uint8_t *pbuf, int32_t len);
static void prv_engine_task(ULONG thread_input);
static void prv_Set_Throttle_Percent(float percent);

/************************************** 私锟�??猴拷锟斤�?? ******************************************* */
static TX_THREAD engine_task_tcb;
static UCHAR engine_task_stack[TASK_STACK_SIZE];

static void prv_engine_task(ULONG thread_input)
{
    // 锟斤拷锟斤拷锟绞癸拷貌锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷锟�??警锟斤拷
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
        // prv_engine_state_machine(fd, pcmd);
        prv_engine_state_machine(pcmd);

        // rxlen = fcs_uart_recv(fd, &msg_buf[wt_idx], sizeof(msg_buf) - wt_idx);
        rxlen = fcs_uart_recv(RT_ENGINE, &msg_buf[wt_idx], sizeof(msg_buf) - wt_idx);

        if(rxlen > 0)
        {
           // g_DeviceState.ecuCountDown = 200;
            // 锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷 (rxlen + wt_idx) 锟斤拷锟斤拷
            wt_idx = prv_analyse(msg_buf, (rxlen + wt_idx));
        }
    }
}

/** 锟斤拷锟斤拷锟斤拷状态锟斤拷
 * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷停锟斤拷锟斤拷锟酵憋拷锟斤拷锟斤拷锟斤拷锟酵憋拷停�?�锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷指锟斤拷
 * */
static void prv_engine_state_machine(struct ecu_cmd *pcmd)
{
    /*********** 锟斤拷锟斤拷锟斤拷状态锟斤拷 ***************** */
    switch(s_engineStatus.CntState)
    {
        case ENGINE_STOPED:
            {
                EngineRunningFlag=ENGINE_STOPED;
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_START))
                {
                    s_engineStatus.CntState = ENGINE_WARMUP;
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
                EngineRunningFlag=ENGINE_WARMUP;
            prv_engine_warmup_state_machine();
            break;
        case ENGINE_RUNNING:
            {
                EngineRunningFlag=ENGINE_RUNNING;
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_STOP))
                {
                    s_engineStatus.CntState = ENGINE_SHUTTING_DOWN;
                }
                if((pcmd != NULL) && (pcmd->cmd == ECU_CMD_ENGIN_THO))
                {
                    prv_Set_Throttle_Percent(pcmd->cmd_para.tho_val);
                }
            }
            break;
        case ENGINE_SHUTTING_DOWN:
            EngineRunningFlag=ENGINE_SHUTTING_DOWN;
            prv_engine_shutdown_state_machine();
            break;
    }
}


/*********** 简化启动状态机（无油压/�?速�?�查，无重试，非阻塞） ***************** */
static void prv_engine_warmup_state_machine(void)
{
    static uint32_t s_startTime = 0;
    static uint32_t s_sleepTime = 0;

    /** 简化启动状态机状态枚�? */
    typedef enum {
        SIMPLE_INIT_PWM = 0,
        SIMPLE_OUTPUT_PWM_1MS,
        SIMPLE_SEND_CMD_PUMP_ON,
        SIMPLE_SLEEP,
        SIMPLE_SEND_CMD_IGNITION1_ON,
        SIMPLE_SEND_CMD_IGNITION2_ON,
        SIMPLE_SEND_CMD_CHOKE_ON,
        SIMPLE_OUTPUT_PWM_2MS,
        SIMPLE_START_DONE,
    } SM_SimpleStartEngine_t;
    static SM_SimpleStartEngine_t s_current_state = SIMPLE_INIT_PWM;
    static SM_SimpleStartEngine_t s_next_state;

    switch (s_current_state)
    {
        case SIMPLE_INIT_PWM:
            s_engineStatus.ecuError = NO_ERROR;
            PulseServo_Init(ECU_PWM8, 1);
            // fall through
        case SIMPLE_OUTPUT_PWM_1MS:
            PulseServo_SetPulseWidth(ECU_PWM8, 1);
            s_startTime = tx_time_get();
            s_sleepTime = PWM_1MS_DURATION;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_SEND_CMD_PUMP_ON;
            break;
        case SIMPLE_SEND_CMD_PUMP_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_PUMP_ON, sizeof(CMD_PUMP_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_SEND_CMD_IGNITION1_ON;
            break;
        case SIMPLE_SEND_CMD_IGNITION1_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_IGNITION1_ON, sizeof(CMD_IGNITION1_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_SEND_CMD_IGNITION2_ON;
            break;
        case SIMPLE_SEND_CMD_IGNITION2_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_IGNITION2_ON, sizeof(CMD_IGNITION2_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_SEND_CMD_CHOKE_ON;
            break;
        case SIMPLE_SEND_CMD_CHOKE_ON:
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_CHOKE_ON, sizeof(CMD_CHOKE_ON));
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_OUTPUT_PWM_2MS;
            break;
        case SIMPLE_OUTPUT_PWM_2MS:
            PulseServo_SetPulseWidth(ECU_PWM8, 2);
            s_startTime = tx_time_get();
            s_sleepTime = WAIT_50MS;
            s_current_state = SIMPLE_SLEEP;
            s_next_state = SIMPLE_START_DONE;
            break;
        case SIMPLE_SLEEP:
            if((tx_time_get() - s_startTime) > s_sleepTime) {
                s_current_state = s_next_state;
            }
            break;
        case SIMPLE_START_DONE:
            PulseServo_Deinit(ECU_PWM8);
            s_current_state = SIMPLE_INIT_PWM;
            s_engineStatus.CntState = ENGINE_RUNNING;
            prv_Set_Throttle_Percent(0.0f);
            break;
    }
}

/*********** 停锟斤拷状态锟斤拷 ***************** */
static void prv_engine_shutdown_state_machine()
{
    static uint8_t s_StopEngineRetryCnt = 0;
    static uint32_t s_startTime = 0;
    static uint32_t s_sleepTime = 0;
    static bool isfirstFail = true;
    static uint32_t s_firstFail = 0;

    /** 停锟斤拷状态锟斤拷锟斤拷状态锟斤拷锟斤�?? */
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
            // fcs_uart_send(fd, (const uint8_t *)&CMD_STOP_ENGINE, sizeof(CMD_STOP_ENGINE));
            fcs_uart_send(RT_ENGINE, (const uint8_t *)&CMD_STOP_ENGINE, sizeof(CMD_STOP_ENGINE));

            s_startTime = tx_time_get();
            s_current_state = CHECK_THROTTLE;
            break;
        case CHECK_THROTTLE:
					  s_engineStatus.throttle_state == 1;
            if(s_engineStatus.throttle_state == 1)// 锟斤拷锟斤拷锟斤拷锟角凤拷乇锟� 35锟斤拷指锟筋�??1为锟�??�??ｏ拷0�??锟截憋拷
            {
                s_current_state = STOP_SUCCESS;
                isfirstFail = true;
            }
            else    // 锟截憋拷锟届�??
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
                        // 锟斤拷锟斤拷失锟杰达拷锟斤拷�??锟洁�??
                        s_current_state = STOP_FAILED;
                    }
                    isfirstFail = true;
                }
            }
            break;
        case STOP_SUCCESS:
            s_current_state = SEND_CMD_STOP_ENGINE;//为锟铰达拷锟斤拷准锟斤拷
            s_StopEngineRetryCnt = 0;
            s_engineStatus.CntState = ENGINE_STOPED;
            break;
        case STOP_FAILED:
            s_current_state = SEND_CMD_STOP_ENGINE;//为锟铰达拷锟斤拷准锟斤拷
            s_StopEngineRetryCnt = 0;
            s_engineStatus.CntState = ENGINE_RUNNING;
            break;
    }
}


/** @brief 锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟筋（锟斤拷锟斤拷俜直龋锟�
 * @param percent 锟斤拷锟脚百分比ｏ拷0-100.0锟斤拷支锟斤拷一位小锟斤拷锟斤拷
 * @param buffer 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�??6锟�?�节ｏ拷
 */
static void prv_Generate_Throttle_Cmd(float percent, uint8_t *buffer) {
    /* 锟斤拷锟�??分憋拷转锟斤拷为0-1000锟斤拷�? */
    uint16_t value = (uint16_t)(percent * 10.0f);  // 25.5% -> 255
    uint8_t data_h = (value >> 8) & 0xFF;         // 锟斤拷锟街斤�??
    uint8_t data_l = value & 0xFF;                // 锟斤拷锟街斤�??
    
    /* 锟斤拷锟斤拷锟斤拷锟斤拷 */
    buffer[0] = 0x40;                   // 锟斤拷址64
    buffer[1] = data_h;                 // 锟斤拷锟捷革拷位
    buffer[2] = data_l;                 // 锟斤拷锟捷�?�拷�??
    buffer[3] = 0x40 + data_h + data_l; // 校锟斤拷锟�
    buffer[4] = 0x0D;                   // 锟斤拷锟斤拷锟斤�??1
    buffer[5] = 0x0A;                   // 锟斤拷锟斤拷锟斤�??2
}

/** 锟斤拷锟捷凤拷锟斤拷锟斤拷协锟介，锟斤拷锟叫ｏ拷锟斤�?? */
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

/** 锟斤拷锟捷凤拷锟斤拷锟斤拷协锟介，锟斤拷锟捷斤拷锟斤拷锟斤拷锟斤拷锟�??发锟斤拷锟斤拷状态锟斤拷锟斤拷锟结构锟斤�?? */
static void prv_analyse_data(uint8_t *pbuf)
{
    switch (pbuf[0])
    {
    case 6:
        s_engineStatus.ambient_temp     = (pbuf[1] << 8) + pbuf[2];
				//SETDATA(pDataPoolSrv,"Sr1A",s_engineStatus.ambient_temp,short);
        break;
    case 8:
        s_engineStatus.air_pressure     = (pbuf[1] << 8) + pbuf[2];
				//SETDATA(pDataPoolSrv,"Sr2A",s_engineStatus.air_pressure,unsigned short);
        break;
    case 9:
        s_engineStatus.fuel_pressure    = (pbuf[1] << 8) + pbuf[2];
        break;
    case 19:
        s_engineStatus.jet1_duty        = (pbuf[1] << 8) + pbuf[2];
				//SETDATA(pDataPoolSrv,"Sr3A", s_engineStatus.jet1_duty,unsigned short);
        break;
    case 35:
        s_engineStatus.throttle_state   = (pbuf[1] << 8) + pbuf[2];
        break;
    case 39:
        s_engineStatus.jet2_duty        = (pbuf[1] << 8) + pbuf[2];
				//SETDATA(pDataPoolSrv,"Sr4A", s_engineStatus.jet2_duty,unsigned short);
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

/** 锟斤拷锟斤拷锟斤拷锟斤拷 
 * @param 锟斤拷锟秸�?�拷锟斤拷锟斤拷锟斤�??
 * @param 锟斤拷锟介长锟斤�??
 * @return 锟斤拷锟�??讹拷锟劫革拷�??锟斤拷锟斤拷锟斤拷锟斤拷锟斤�??
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

/** @brief 锟斤拷queue锟斤拷取锟斤�??3锟斤拷锟斤拷锟斤拷时锟斤拷去锟斤拷锟斤拷锟斤拷锟斤拷
 * @param percent 锟斤拷锟脚百分比ｏ拷0-100.0锟斤拷支锟斤拷一位小锟斤拷锟斤拷
 */
static void prv_Set_Throttle_Percent(float percent)
{
    uint8_t throttle_cmd[6];
    
    /* 锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?? */
    prv_Generate_Throttle_Cmd(percent, throttle_cmd);
    
    /* 锟斤拷锟斤拷锟斤拷锟斤拷 */
    // fcs_uart_send(fd, (const uint8_t *)&throttle_cmd, sizeof(throttle_cmd));
    fcs_uart_send(RT_ENGINE, (const uint8_t *)&throttle_cmd, sizeof(throttle_cmd));
}

/************************************* 锟斤拷锟斤拷涌诓锟斤拷锟�?? ******************************************** */
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

/** @brief 锟斤拷锟斤拷锟斤拷锟斤拷锟矫碉拷锟脚猴拷
 * @param percent 锟斤拷锟脚百分比ｏ拷0-100.0锟斤拷支锟斤拷一位小锟斤拷锟斤拷
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
        &engine_task_tcb,                    // 锟�??程匡拷锟狡匡�??
        "Engine Task",                  // 锟�??筹拷锟斤拷锟斤拷
        prv_engine_task,                    // 锟�??筹拷锟斤拷�?�锟斤拷锟�
        0,                                // 锟�??筹拷锟斤拷锟斤拷锟斤拷锟�??
        engine_task_stack,                  // 锟斤拷栈锟斤拷�?�锟斤拷址
        TASK_STACK_SIZE,        // 锟斤拷栈锟斤拷小
        TASK_PRIORITY,            // 锟�??筹拷锟斤拷锟饺硷拷锟斤拷锟较革拷锟斤拷锟饺硷拷锟斤拷
        TASK_PRIORITY,            // 锟斤拷占锟斤拷�?
        TX_NO_TIME_SLICE,                 // 时锟斤拷�??
        TX_AUTO_START                     // 锟皆讹拷锟斤拷锟斤拷
    );
}
