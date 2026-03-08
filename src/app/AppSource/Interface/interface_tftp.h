#ifndef __INTERFACE_TFTP_H
#define __INTERFACE_TFTP_H

#include "lwip/opt.h"
#include "lwip/apps/tftp_server.h"  // 需要 tftp_context 的定义

// TFTP 句柄结构体
typedef struct {
    uint8_t is_write;     // 1=写模式(客户端上传), 0=读模式(客户端下载)
    uint8_t is_open;      // 1=文件已打开, 0=文件未打开
    uint8_t reserved[2];  // 保留字节，用于对齐（可添加其他状态）
} TFTP_Handler;

// 函数声明
void tftp_server_init(void);

#endif /* __INTERFACE_TFTP_H */