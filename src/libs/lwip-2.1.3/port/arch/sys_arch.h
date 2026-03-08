
#ifndef __SYS_ARCH_THREADX_H__
#define __SYS_ARCH_THREADX_H__

#include "lwip/opt.h"

#ifdef LWIP_PROVIDE_ERRNO
#include <lwip/errno.h>
#endif

#if NO_SYS
#error ("porting error: no os is not supported !!")
#endif

#include "tx_api.h"

typedef UINT sys_prot_t;

#define SYS_WAIT_FOREVER TX_WAIT_FOREVER

typedef TX_SEMAPHORE*   sys_sem_t;
typedef TX_MUTEX*       sys_mutex_t;
typedef TX_QUEUE*       sys_mbox_t;
typedef TX_THREAD*      sys_thread_t;

sys_prot_t sys_arch_protect(void);
void sys_arch_unprotect(sys_prot_t pval);

#endif /* __SYS_ARCH_THREADX_H__ */

