#include <cstdlib>
#include <new>
#include "tx_api.h"

// 使用 extern "C" 声明 C 文件中定义的变量
extern "C" {
    extern TX_BYTE_POOL byte_pool_0;
}

// 全局 operator new - 使用 byte_pool_0
void* operator new(std::size_t size) {
    void* ptr = nullptr;
    
    // 使用已有的 byte_pool_0 分配内存
    UINT status = tx_byte_allocate(&byte_pool_0, &ptr, size, TX_NO_WAIT);
    
    if (status != TX_SUCCESS) {
        // 内存分配失败，返回 nullptr
        return nullptr;
    }
    
    return ptr;
}

// operator new[] - 数组版本
void* operator new[](std::size_t size) {
    return operator new(size);
}

// 全局 operator delete
void operator delete(void* ptr) noexcept {
    if (ptr != nullptr) {
        tx_byte_release(ptr);
    }
}

void operator delete[](void* ptr) noexcept {
    operator delete(ptr);
}
