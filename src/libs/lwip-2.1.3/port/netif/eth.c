
#include <string.h>
#include "arch/sys_arch.h"
#include "lwip/opt.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"
#include "eth.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#ifndef LWIP_NETIF_ETH_INST_CNT
#define LWIP_NETIF_ETH_INST_CNT  (1) // default 1 ethernet interface
#endif

/* Stack size of the interface thread */
#ifndef LWIP_NETIF_THREAD_STACK_SIZE
#define LWIP_NETIF_THREAD_STACK_SIZE (512)
#endif

#ifndef LWIP_NETIF_THREAD_PRIORITY  
// higher priority than tcpip thread
#define LWIP_NETIF_THREAD_PRIORITY      (TCPIP_THREAD_PRIO - 1)
#endif

#ifndef LWIP_ETHIF_MAX_FRAME_SIZE
#define LWIP_ETHIF_MAX_FRAME_SIZE 1526  // for double tagged ENET frames
#endif

#define LWIP_ETH_DEFAULT_MTU_SIZE 1500 // default mtu size

#define ETHIF_CONFIG_TX_EVENT_NODE_CNT   8
/* Private macro -------------------------------------------------------------*/
#define LWIP_ETHIF_MAX_FRAME_SIZE_ALIGN(len, align) \
    (((len + (align-1)) / align) * align) // align to 4 bytes

static netif_eth_ctx_t*  s_eth_mac_ctx = NULL;

struct netif_prv_ctx
{
    // ethernet interface context : optional
    int                         if_idx;
    uint32_t                    rx_buf_size;
    uint8_t*                    rx_buf;

    sys_sem_t                   rx_sem;
    sys_mutex_t                 tx_mutex;
    
    sys_thread_t                rx_task;
};

/* Private function prototypes -----------------------------------------------*/
static void  eth_mac_rx_task(void *argument);

extern TX_BYTE_POOL byte_pool_0;
static void* prv_malloc(uint32_t size)
{
    void* ptr = NULL;
    tx_byte_allocate(&byte_pool_0,&ptr, size, TX_NO_WAIT);
    return ptr;
}

#define  malloc(sz) prv_malloc(sz)
/**
 * @brief This function is called by the CMSIS-Driver when an event occurs.
 * It should be used to signal the lwIP stack that a packet has been received
 * or that the link state has changed.
 *
 * @param event the event that occurred
 */
static uint32_t rx_cnt = 0;
static uint32_t tx_cnt = 0;
static void eth_mac_callback(uint32_t event)
{
    struct netif_prv_ctx* prv_ctx = (struct netif_prv_ctx*)(s_eth_mac_ctx->prv);

    if (0 != (event & ARM_ETH_MAC_EVENT_RX_FRAME))
    {
        rx_cnt++;
        sys_sem_signal(&(prv_ctx->rx_sem));
    }
    if(0 != (event & ARM_ETH_MAC_EVENT_TX_FRAME))
    {   //TODO: should never to here
        tx_cnt++;
    }
}


static err_t low_level_output(struct netif *netif, struct pbuf *p);
/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t netif_eth_init(struct netif *netif)
{
    netif_eth_ctx_t *     eth_ctx;
    struct netif_prv_ctx *prv_ctx;
    ARM_ETH_LINK_INFO    info;
    ARM_DRIVER_ETH_MAC * eth_mac_handle;
    uint32_t             mtu_size;
    uint32_t             alloc_size;

    if ((NULL == netif) || (NULL == netif->state))
    {
        return ERR_ARG;
    }
    // load configuration
    eth_ctx = (netif_eth_ctx_t *)netif->state;

    // get mtu setting
    mtu_size = (0 == eth_ctx->mtu) ? LWIP_ETH_DEFAULT_MTU_SIZE : eth_ctx->mtu;

    // checking basic input setting
    if ((NULL == eth_ctx->pdrv_mac) || (NULL == eth_ctx->cb_phy_init) || (mtu_size > LWIP_ETHIF_MAX_FRAME_SIZE))
    {
        return ERR_ARG;
    }

    // check interface
    if(s_eth_mac_ctx != NULL)
    {   // already in use
        /* no more free interface */
        return ERR_IF;
    }
    s_eth_mac_ctx = eth_ctx;
#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

//    if (NULL != eth_ctx->name)
//    {
        netif->name[0] = eth_ctx->name[0];
        netif->name[1] = eth_ctx->name[1];
//    }
    /* common output */
    netif->output     = etharp_output;
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    eth_mac_handle = eth_ctx->pdrv_mac;
    eth_mac_handle->Initialize(eth_mac_callback);    
    eth_mac_handle->PowerControl(ARM_POWER_FULL);
    eth_mac_handle->SetMacAddress(&eth_ctx->macaddr);

    if (0 != eth_ctx->cb_phy_init(eth_ctx->pdrv_mac, eth_ctx->pdrv_phy, &info, eth_ctx->cb_ctx))
    {
        eth_mac_handle->PowerControl(ARM_POWER_OFF);
        eth_mac_handle->Uninitialize();
        s_eth_mac_ctx = NULL;
        return ERR_IF;
    }
    // set link callback
    //netif_set_link_callback();
    // default link off
    netif->flags &= (~NETIF_FLAG_LINK_UP);

    /* set netif MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set netif MAC hardware address */
    netif->hwaddr[0] = eth_ctx->macaddr.b[0];
    netif->hwaddr[1] = eth_ctx->macaddr.b[1];
    netif->hwaddr[2] = eth_ctx->macaddr.b[2];
    netif->hwaddr[3] = eth_ctx->macaddr.b[3];
    netif->hwaddr[4] = eth_ctx->macaddr.b[4];
    netif->hwaddr[5] = eth_ctx->macaddr.b[5];
    /* set netif maximum transfer unit */
    // TODO:confirm check RFC for VLAN Frame
    netif->mtu = (u16_t)mtu_size;

    /* Accept broadcast address and ARP traffic */
    netif->flags |= (NETIF_FLAG_BROADCAST | NETIF_FLAG_IGMP| NETIF_FLAG_ETHARP);

    // alloc private context  
    alloc_size = LWIP_ETHIF_MAX_FRAME_SIZE_ALIGN(LWIP_ETHIF_MAX_FRAME_SIZE, sizeof(uint32_t));
    prv_ctx = malloc(sizeof(struct netif_prv_ctx) + alloc_size);
    prv_ctx->rx_buf_size = alloc_size;
    
    if (NULL == prv_ctx)
    {   // reset the ctx table
        eth_mac_handle->PowerControl(ARM_POWER_OFF);
        eth_mac_handle->Uninitialize();
        s_eth_mac_ctx = NULL;
        return ERR_MEM;
    }
    /* create a binary semaphore used for informing ethernetif of frame reception */
    if(sys_sem_new(&prv_ctx->rx_sem, 0) != ERR_OK)
    {
        s_eth_mac_ctx = NULL;
        eth_mac_handle->Uninitialize();
        free(prv_ctx);
        return ERR_MEM;
    }

    if(sys_mutex_new(&prv_ctx->tx_mutex) != ERR_OK)
    {
        s_eth_mac_ctx = NULL;
        sys_sem_free(&prv_ctx->rx_sem);
        free(prv_ctx);
        return ERR_MEM;
    }
    /* alloc buffer */
    // eth_ctx->tx_buf_size = LWIP_ETHIF_MAX_FRAME_SIZE_ALIGN(LWIP_ETHIF_MAX_FRAME_SIZE, sizeof(uint32_t));
    // eth_ctx->tx_buf = malloc(eth_ctx->tx_buf_size);
    prv_ctx->rx_buf = ((uint8_t*)prv_ctx ) + sizeof(struct netif_prv_ctx);

    /* create the task that handles the ETH_MAC */

    prv_ctx->rx_task = sys_thread_new("eth_if",eth_mac_rx_task, netif, LWIP_NETIF_THREAD_STACK_SIZE, LWIP_NETIF_THREAD_PRIORITY);
    if (NULL == (void*)(prv_ctx->rx_task))
    {  // we failed when create task
        eth_mac_handle->PowerControl(ARM_POWER_OFF);
        eth_mac_handle->Uninitialize();

        sys_sem_free(&prv_ctx->rx_sem);
        sys_mutex_free(&prv_ctx->tx_mutex);
        s_eth_mac_ctx = NULL;
        free(prv_ctx);
        // free(eth_ctx->tx_buf);
        return ERR_IF;
    }
    // save private context
    eth_ctx->prv = prv_ctx;

    return  ERR_OK;
}

/**
 * @brief This function should do the actual transmission of the packet. The
 * packet is contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and
 * type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    ARM_DRIVER_ETH_MAC * pmac;
    netif_eth_ctx_t *ethif_ctx = (netif_eth_ctx_t*)netif->state;
    struct netif_prv_ctx* prv_ctx = (struct netif_prv_ctx*)ethif_ctx->prv;
    int32_t ret;
    struct pbuf *q;

    if ((NULL == netif) || (NULL == netif->state) || (NULL == ((netif_eth_ctx_t *)netif->state)->pdrv_mac))
    {
        return ERR_ARG;
    }

    pmac        = ((netif_eth_ctx_t *)netif->state)->pdrv_mac;

    sys_mutex_lock(&prv_ctx->tx_mutex);

    /* copy frame from pbufs to driver buffers */
    for (q = p; q != NULL; q = q->next)
    {
        // segment copy
        ret = pmac->SendFrame(  q->payload, 
                                q->len, 
                                // if not last segment, set fragment flag
                                (q->next != NULL) ? ARM_ETH_MAC_TX_FRAME_FRAGMENT : ARM_ETH_MAC_TX_FRAME_EVENT);
        if(ret != ARM_DRIVER_OK)
        {   // TODO: send error handling
            break;
        }
    }

    sys_mutex_unlock(&prv_ctx->tx_mutex);


    return (ret != ARM_DRIVER_OK) ? ERR_IF : ERR_OK;
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif)
{
    ARM_DRIVER_ETH_MAC * eth_mac_handle;
    netif_eth_ctx_t *ethif_ctx = (netif_eth_ctx_t*)(netif->state);
    struct netif_prv_ctx* prv_ctx = (struct netif_prv_ctx*)ethif_ctx->prv;

    struct pbuf *p = NULL;
    struct pbuf *q = NULL;

    uint32_t len    = 0;
    uint8_t *buffer;
    uint32_t length = 0;

    if ((NULL == netif) || (NULL == netif->state) || ((netif_eth_ctx_t *)netif->state)->pdrv_mac == NULL)
    {
        return NULL;
    }
    ethif_ctx = (netif_eth_ctx_t *)netif->state;

    eth_mac_handle = ethif_ctx->pdrv_mac;

    len = eth_mac_handle->GetRxFrameSize();
    if (len == 0) 
    {
        /* no frame received or frame too large */
        return NULL;
    }

    buffer = prv_ctx->rx_buf;

    /* get received frame */
    if (ARM_DRIVER_ERROR == eth_mac_handle->ReadFrame(buffer, (prv_ctx->rx_buf_size > len) ? len : prv_ctx->rx_buf_size))
    {
        /* error reading frame */
        return NULL;
    }

    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    if (NULL == (p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL)))
    {
        return NULL;
    }
    /* We iterate over the pbuf chain until we have read the entire packet
     * into the pbuf. */
    for (q = p; q != NULL; q = q->next)
    {
        /* Copy data in pbuf */
        length = q->len;
        memcpy((uint8_t *)q->payload, buffer, length);
        buffer += length;
    }

    return p;
}

/**
 * @brief This function is the eth_mac_rx_task task, it is processed when a
 * packet is ready to be read from the interface. It uses the function
 * low_level_input() that should handle the actual reception of bytes from the
 * network interface. Then the type of the received packet is determined and the
 * appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */

static void eth_mac_rx_task(void *argument)
{
    struct pbuf * p;
    struct netif *netif = (struct netif *)argument;
    netif_eth_ctx_t *eth_ctx = (netif_eth_ctx_t *)netif->state;
    struct netif_prv_ctx* prv_ctx = (struct netif_prv_ctx*)eth_ctx->prv;

    for (;;)
    {
        if (sys_arch_sem_wait(&(prv_ctx->rx_sem), SYS_WAIT_FOREVER) == SYS_ARCH_TIMEOUT)
        {
            continue;
        }
        do
        {
            LOCK_TCPIP_CORE();
            p = low_level_input(netif);
            if (p != NULL)
            {
                if (netif->input(p, netif) != ERR_OK)
                {
                    pbuf_free(p);
                }
            }
            UNLOCK_TCPIP_CORE();
        } while (p != NULL);
    }
}
