

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#include "sys_arch.h"

/* This is the number of threads that can be started with sys_thread_new() */
#define SYS_THREAD_MAX 6

void sys_assert( const char *msg );
	
static u16_t s_nextthread = 0;

#if !NO_SYS
extern TX_BYTE_POOL byte_pool_0;
static void* prv_malloc(uint32_t size)
{
    void* ptr = NULL;
    tx_byte_allocate(&byte_pool_0,&ptr, size, TX_NO_WAIT);
    return ptr;
}

#define  malloc(sz) prv_malloc(sz)
/*-----------------------------------------------------------------------------------*/
//  Creates an empty mailbox.
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	(void ) size;

    TX_QUEUE * pqueue;
    void*      queue_buffer;
    // init as null
    *mbox = NULL;
    // allocate the queue
    pqueue = malloc(sizeof(TX_QUEUE) + size * sizeof(void *));
    if (pqueue == NULL) 
    {
        return ERR_MEM;
    }
    queue_buffer = (void *)(pqueue + 1);
    // init the queue
    if (TX_SUCCESS != tx_queue_create(pqueue, "lwip queue", 1, queue_buffer,  size * sizeof(void *)))
    {
        free(pqueue);
        return ERR_MEM;
    }

#if SYS_STATS
      ++lwip_stats.sys.mbox.used;
      if (lwip_stats.sys.mbox.max < lwip_stats.sys.mbox.used) {
         lwip_stats.sys.mbox.max = lwip_stats.sys.mbox.used;
	  }
#endif /* SYS_STATS */

      *mbox = (sys_mbox_t)pqueue; // Assign the created queue to the mailbox pointer
 return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
    TX_QUEUE * pqueue = (TX_QUEUE *)(*mbox);
    if(TX_SUCCESS != tx_queue_delete(pqueue))
    {
        sys_assert("sys_mbox_free: tx_queue_delete failed");
    }
    free(pqueue);

#if SYS_STATS
     --lwip_stats.sys.mbox.used;
#endif /* SYS_STATS */
}

/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
    tx_queue_send(*mbox, &data, TX_WAIT_FOREVER);
}


/*-----------------------------------------------------------------------------------*/
//   Try to post the "msg" to the mailbox.
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    err_t result = ERR_MEM;

    if(TX_SUCCESS != tx_queue_send(*mbox, &msg, TX_NO_WAIT))
    {   // could not post, queue must be full
        result = ERR_MEM;
#if SYS_STATS
        lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */
    }
    else
    {
        result = ERR_OK;
    }
   return result;
}

/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread until a message arrives in the mailbox, but does
  not block the thread longer than "timeout" milliseconds (similar to
  the sys_arch_sem_wait() function). The "msg" argument is a result
  parameter that is set by the function (i.e., by doing "*msg =
  ptr"). The "msg" parameter maybe NULL to indicate that the message
  should be dropped.

  The return values are the same as for the sys_arch_sem_wait() function:
  Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
  timeout.

  Note that a function with a similar name, sys_mbox_fetch(), is
  implemented by lwIP.
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    void *dummyptr;
    ULONG start_time, end_time, elapsed;
    UINT status;
    ULONG wait_option;

    start_time = tx_time_get();

    if (msg == NULL) {
        msg = &dummyptr;
    }

    if (timeout != 0) 
    {   // Convert timeout from ms to ticks (assuming 1 tick = 1 ms, adjust if needed)
        wait_option = timeout;
    } else 
    {
        wait_option = TX_WAIT_FOREVER;
    }
    // Receive message from the mailbox
    status = tx_queue_receive(*mbox, msg, wait_option);

    if (status == TX_SUCCESS) 
    {
        end_time = tx_time_get();
        elapsed = end_time - start_time;
        return elapsed;
    } 
    else
    {
        *msg = NULL;
        return SYS_ARCH_TIMEOUT;
    }
}

/*-----------------------------------------------------------------------------------*/
/*
  Similar to sys_arch_mbox_fetch, but if message is not ready immediately, we'll
  return with SYS_MBOX_EMPTY.  On success, 0 is returned.
*/
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *dummyptr;

    if (msg == NULL) {
        msg = &dummyptr;
    }

    if(tx_queue_receive(*mbox, msg, TX_NO_WAIT) == TX_SUCCESS)
    {
        return ERR_OK; // Message received successfully
    }
    else
    {
        return SYS_MBOX_EMPTY; // Indicate that the mailbox is empty
    }
}
/*----------------------------------------------------------------------------------*/
int sys_mbox_valid(sys_mbox_t *mbox)
{
  if (*mbox == NULL)
    return 0;
  else
    return 1;
}
/*-----------------------------------------------------------------------------------*/
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  *mbox = 0;
}

/*-----------------------------------------------------------------------------------*/
//  Creates a new semaphore. The "count" argument specifies
//  the initial state of the semaphore.
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    TX_SEMAPHORE* psem;

    psem = malloc(sizeof(TX_SEMAPHORE));
    if (psem == NULL) 
    {   
#if SYS_STATS
        ++lwip_stats.sys.sem.err;
#endif /* SYS_STATS */
        return ERR_MEM; // Memory allocation failed
    }

    // Initialize the semaphore
    if (TX_SUCCESS != tx_semaphore_create(psem, "lwip semaphore", count)) 
    {
        free(psem);
#if SYS_STATS
        ++lwip_stats.sys.sem.err;        
#endif /* SYS_STATS */
        return ERR_MEM; // Semaphore creation failed   
    }
    
#if SYS_STATS
	++lwip_stats.sys.sem.used;
 	if (lwip_stats.sys.sem.max < lwip_stats.sys.sem.used) 
    {
		lwip_stats.sys.sem.max = lwip_stats.sys.sem.used;
	}
#endif /* SYS_STATS */
    *sem = psem; // Assign the created semaphore to the pointer
	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/**
 * @ingroup sys_sem
 *  Blocks the thread while waiting for the semaphore to be signaled. If the
 * "timeout" argument is non-zero, the thread should only be blocked for the
 * specified time (measured in milliseconds). If the "timeout" argument is zero,
 * the thread should be blocked until the semaphore is signalled.
 *
 * The return value is SYS_ARCH_TIMEOUT if the semaphore wasn't signaled within
 * the specified time or any other value if it was signaled (with or without
 * waiting).
 * Notice that lwIP implements a function with a similar name,
 * sys_sem_wait(), that uses the sys_arch_sem_wait() function.
 *
 * @param sem the semaphore to wait for
 * @param timeout timeout in milliseconds to wait (0 = wait forever)
 * @return SYS_ARCH_TIMEOUT on timeout, any other value on success
 */
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    ULONG start_time, end_time, elapsed;

    start_time = tx_time_get();

    if(timeout != 0)
    {
        // Convert timeout from ms to ticks (assuming 1 tick = 1 ms, adjust if needed)
        if(tx_semaphore_get(*sem, timeout) == TX_SUCCESS)
        {
            end_time = tx_time_get();
            elapsed = end_time - start_time;
            return elapsed; // return time blocked
        }
        else
        {
            return SYS_ARCH_TIMEOUT; // Timeout occurred
        }
    }
    else // must block without a timeout
    {
        while(tx_semaphore_get(*sem, TX_WAIT_FOREVER) != TX_SUCCESS){}
        // Semaphore was signaled, calculate elapsed time
        end_time = tx_time_get();
        elapsed = end_time - start_time;
        return elapsed; // return time blocked
    }
}

/*-----------------------------------------------------------------------------------*/
// Signals a semaphore

void sys_sem_signal(sys_sem_t *sem)
{
    tx_semaphore_put(*sem);
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void sys_sem_free(sys_sem_t *sem)
{
#if SYS_STATS
      --lwip_stats.sys.sem.used;
#endif /* SYS_STATS */

    tx_semaphore_delete(*sem);
    free(*sem);
    *sem = 0; // Set to NULL after deletion
}
/*-----------------------------------------------------------------------------------*/
int sys_sem_valid(sys_sem_t *sem)
{
    if (*sem == NULL)
        return 0;
    else
        return 1;
}

/*-----------------------------------------------------------------------------------*/
void sys_sem_set_invalid(sys_sem_t *sem)
{
    *sem = NULL;
}

/*-----------------------------------------------------------------------------------*/
                                      /* Mutexes*/
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
#if LWIP_COMPAT_MUTEX == 0
/* Create a new mutex*/
err_t sys_mutex_new(sys_mutex_t *mutex) 
{
    TX_MUTEX *pMutex;
    pMutex = malloc(sizeof(TX_MUTEX));
    if (pMutex == NULL) 
    {   
#if SYS_STATS
        ++lwip_stats.sys.mutex.err;
#endif /* SYS_STATS */
        return ERR_MEM; // Memory allocation failed
    }

    // Initialize the mutex
    if (TX_SUCCESS != tx_mutex_create(pMutex, "lwip mutex", TX_INHERIT))
    {
        free(pMutex);
#if SYS_STATS
        ++lwip_stats.sys.mutex.err;
#endif /* SYS_STATS */
        return ERR_MEM; // Mutex creation failed
    }   
    *mutex = pMutex; // Assign the created mutex to the pointer
#if SYS_STATS
	++lwip_stats.sys.mutex.used;
 	if (lwip_stats.sys.mutex.max < lwip_stats.sys.mutex.used) 
    {
		lwip_stats.sys.mutex.max = lwip_stats.sys.mutex.used;
	}
#endif /* SYS_STATS */
    return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* Deallocate a mutex*/
void sys_mutex_free(sys_mutex_t *mutex)
{
#if SYS_STATS
    --lwip_stats.sys.mutex.used;
#endif /* SYS_STATS */
    tx_mutex_delete(*mutex);
    free(*mutex);
    *mutex = NULL; // Set to NULL after deletion
}
/*-----------------------------------------------------------------------------------*/
/* Lock a mutex*/
void sys_mutex_lock(sys_mutex_t *mutex)
{
    if(TX_SUCCESS != tx_mutex_get(*mutex, TX_WAIT_FOREVER))
    {
        sys_assert("sys_mutex_lock: tx_mutex_get failed");
    }
}

/*-----------------------------------------------------------------------------------*/
/* Unlock a mutex*/
void sys_mutex_unlock(sys_mutex_t *mutex)
{
    if(TX_SUCCESS != tx_mutex_put(*mutex))
    {
        sys_assert("sys_mutex_unlock: tx_mutex_put failed");
    }
}
#endif /*LWIP_COMPAT_MUTEX*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread , void *arg, int stacksize, int prio)
{
    TX_THREAD* pthread;

    pthread = malloc(sizeof(TX_THREAD) + (stacksize * sizeof(uint32_t)));
    if (pthread == NULL) 
    {
        return NULL; // Memory allocation failed    
    }   
    // Initialize the thread
    if (TX_SUCCESS != tx_thread_create(pthread,(char*) name, (void(*)(ULONG))thread, (ULONG)arg, 
                                       (void *)(pthread + 1), stacksize * sizeof(uint32_t), 
                                       prio, prio, TX_NO_TIME_SLICE, TX_AUTO_START)) 
    {
        free(pthread);
        return NULL; // Thread creation failed
    }
    return pthread;
}

void sys_thread_delete(sys_thread_t thread)
{
    tx_thread_delete(thread);
    free(thread);
}

#endif /*#if !NO_SYS*/

/*
  This optional function does a "fast" critical region protection and returns
  the previous protection level. This function is only called during very short
  critical regions. An embedded system which supports ISR-based drivers might
  want to implement this function by disabling interrupts. Task-based systems
  might want to implement this by using a mutex or disabling tasking. This
  function should support recursive calls from the same task or interrupt. In
  other words, sys_arch_protect() could be called while already protected. In
  that case the return value indicates that it is already protected.

  sys_arch_protect() is only required if your port is supporting an operating
  system.
*/
static uint32_t s_protect_nest = 0;
sys_prot_t sys_arch_protect(void)
{
    sys_prot_t ret;
    ret = tx_interrupt_control(TX_INT_DISABLE);
    s_protect_nest++;
    return ret;
}

/*
  This optional function does a "fast" set of critical region protection to the
  value specified by pval. See the documentation for sys_arch_protect() for
  more information. This function is only required if your port is supporting
  an operating system.
*/
void sys_arch_unprotect(sys_prot_t pval)
{
    s_protect_nest--;
    tx_interrupt_control((UINT)pval);
}

/*
 * Prints an assertion messages and aborts execution.
 */
void sys_assert( const char *msg )
{
    ( void ) msg;
    tx_interrupt_control(TX_INT_DISABLE);
    for(;;)
    ;
}

/*-----------------------------------------------------------------------------------*/
// Initialize sys arch
void sys_init(void)
{
	// keep track of how many threads have been created
	s_nextthread = 0;
}

u32_t sys_now(void)
{
    return (u32_t)tx_time_get();
}

uint32_t sys_rand(void)
{
    return (u32_t)tx_time_get();
}