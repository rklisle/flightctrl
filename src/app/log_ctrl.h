#ifndef __LOG_CTRL_H__
#define __LOG_CTRL_H__

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "./AppSource/core/BusInteract.h"

#define LOG_ENABLE

extern int32_t fcs_uart_send(int32_t fd, const uint8_t* pdata, uint32_t len);

#ifdef LOG_ENABLE
    // 第一种：固定字符串
    #define LOG_STR(msg) \
        do { \
            const char* msg_log = (msg); \
            fcs_uart_send(RT_LOG, (const uint8_t*)msg_log, strlen(msg_log)); \
        } while(0)
    
    // 第二种：变量值（格式化输出）
    #define LOG_VAL(fmt, ...) \
        do { \
            char info[128] = {0}; \
            sprintf(info, fmt, ##__VA_ARGS__); \
            fcs_uart_send(RT_LOG, (const uint8_t*)info, strlen(info)); \
        } while(0)

    #define LOG_HEX16(data, prefix) \
        do { \
            LOG_STR(prefix); \
            for (int _j = 0; _j < 16; _j++) { \
                LOG_VAL("%02X ", ((uint8_t*)(data))[_j]); \
            } \
            LOG_STR("\n"); \
        } while(0)
#else
    #define LOG_STR(msg) ((void)0)
    #define LOG_VAL(fmt, ...) ((void)0)
    #define LOG_HEX16(data, prefix) ((void)0)
#endif

#endif