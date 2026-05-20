#include "interface_tftp.h"

// ==================== includes ====================
#include "lwip/apps/tftp_server.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>  // 用于 printf, snprintf

// ==================== 变量定义 ====================
static TFTP_Handler tftp_handler = {0};      // 初始化
static FIL tftp_file;                        // 单个文件句柄
static char current_filename[64] = {0};      // 当前文件名
extern TX_MUTEX    mutex_rwSD;   

// ==================== TFTP 文件操作函数 ====================
/**
 * @brief 打开文件
 * @param fname 文件名
 * @param mode 模式字符串("octet", "netascii"等)
 * @param write 1=写(客户端上传), 0=读(客户端下载)
 * @return 返回句柄指针，失败返回NULL
 */
void* TFTP_Open(const char* fname, const char* mode, u8_t write)
{
    FRESULT res;
    char full_path[128];

    // 安全检查
    if (fname == NULL || strlen(fname) == 0) {
        // printf("[TFTP] Error: Empty filename\r\n");
        return NULL;
    }
    
    // 检查是否已有传输在进行（单客户端限制）
    if (tftp_handler.is_open) {
        // printf("[TFTP] Error: Another transfer in progress\r\n");
        return NULL;
    }
    
    // 保存文件名（用于调试）
    strncpy(current_filename, fname, sizeof(current_filename)-1);
    current_filename[sizeof(current_filename)-1] = '\0';
    
    // 构建完整路径（SD卡根目录）
    // 注意：根据实际文件系统修改"0:/"
    snprintf(full_path, sizeof(full_path), "0:/%s", fname); 
    
    // printf("[TFTP] Opening file: %s (mode: %s)\r\n", 
    //        full_path, write ? "WRITE" : "READ");

    // 根据模式打开文件
    if (write) {
        // 客户端上传文件到STM32
        res = f_open(&tftp_file, full_path, FA_CREATE_ALWAYS | FA_WRITE);
    } else {
        // 客户端从STM32下载文件
        res = f_open(&tftp_file, full_path, FA_OPEN_EXISTING | FA_READ);
    }
    
    // 检查打开结果
    if (res != FR_OK) {
        // printf("[TFTP] Failed to open file, error: %d\r\n", res);
        return NULL;
    }
    
    // 初始化句柄
    tftp_handler.is_write = write;
    tftp_handler.is_open = 1;
    
    // printf("[TFTP] File opened successfully\r\n");
    return &tftp_handler;
}

/**
 * @brief 关闭文件
 * @param handle TFTP句柄
 */
void TFTP_Close(void* handle)
{
    // 验证句柄（简单检查）
    if (handle != &tftp_handler) {
        return;
    }
    
    // 只有打开的文件才需要关闭
    if (tftp_handler.is_open) {
        f_close(&tftp_file);
    }
    
    // 重置状态
    tftp_handler.is_open = 0;
    current_filename[0] = '\0';
}

/**
 * @brief 读取文件数据
 * @param handle TFTP句柄
 * @param buf 数据缓冲区
 * @param bytes 请求的字节数
 * @return 实际读取的字节数，错误返回-1
 */
int TFTP_Read(void* handle, void* buf, int bytes)
{
    UINT bytes_read = 0;
    FRESULT res;

    // 参数检查
    if (handle != &tftp_handler || buf == NULL || bytes <= 0) {
        return -1;
    }
    
    if (!tftp_handler.is_open || tftp_handler.is_write) {
        // printf("[TFTP] Error: File not open for reading\r\n");
        return -1;
    }
    
    // 读取文件数据
    tx_mutex_get(&mutex_rwSD, TX_WAIT_FOREVER);
    res = f_read(&tftp_file, buf, bytes, &bytes_read);
    if (res != FR_OK) {
        // printf("[TFTP] Read error: %d\r\n", res);
        return -1;
    }
    tx_mutex_put(&mutex_rwSD);

    // 调试信息（可选，频繁传输时建议关闭）
    // printf("[TFTP] Read %u bytes (requested %d)\r\n", bytes_read, bytes);
    
    return (int)bytes_read;
}

/**
 * @brief 写入文件数据
 * @param handle TFTP句柄
 * @param p lwIP数据包缓冲区
 * @return 成功返回0，失败返回-1
 */
int TFTP_Write(void* handle, struct pbuf* p)
{
    UINT bytes_written = 0;
    FRESULT res;

    // 参数检查
    if (handle != &tftp_handler || p == NULL) {
        return -1;
    }
    
    if (!tftp_handler.is_open || !tftp_handler.is_write) {
        // printf("[TFTP] Error: File not open for writing\r\n");
        return -1;
    }
    
    tx_mutex_get(&mutex_rwSD, TX_WAIT_FOREVER);
    // 写入数据
    res = f_write(&tftp_file, p->payload, p->len, &bytes_written);
    if (res != FR_OK) {
        // printf("[TFTP] Write error: %d\r\n", res);
        return -1;
    }
    
    // 重要：立即同步到存储设备，防止数据丢失
    f_sync(&tftp_file);
    tx_mutex_put(&mutex_rwSD);

    // 调试信息
    // printf("[TFTP] Wrote %u bytes\r\n", bytes_written);
    
    return 0;
}

// 设置 TFTP 上下文
const struct tftp_context ctx = {
    .open = TFTP_Open,
    .close = TFTP_Close,
    .read = TFTP_Read,
    .write = TFTP_Write
};

// ==================== 对上层接口函数 ====================
void tftp_server_init(void)
{
    // 初始化静态变量
    memset(&tftp_handler, 0, sizeof(tftp_handler));
    memset(&tftp_file, 0, sizeof(tftp_file));
    current_filename[0] = '\0';

    // 初始化 TFTP 服务器
    tftp_init(&ctx);
}

