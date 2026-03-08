#ifndef _NETIF_ETH_H_
#define _NETIF_ETH_H_

#include "Driver_ETH.h"
#include "Driver_ETH_MAC.h"
#include "Driver_ETH_PHY.h"
#include "lwip/netif.h"
#include "arch/sys_arch.h"//MML20251120
/* Exported types ------------------------------------------------------------*/
/**
 * @brief  PHY initialization function.
 */
typedef int32_t (*eth_phy_init_cb_t)(ARM_DRIVER_ETH_MAC *mac_drv,
                                     ARM_DRIVER_ETH_PHY *phy_drv,
                                     ARM_ETH_LINK_INFO * info,
                                     void *              cb_ctx);

typedef struct
{
    // eth interface name : optional
    const char          name[16];

    // cmsis mac driver : mandantory
    ARM_DRIVER_ETH_MAC *pdrv_mac;

    // cmsis PHY driver : optional
    ARM_DRIVER_ETH_PHY *pdrv_phy;

    // PHY init callback (PHY initialized by App) : mandantory
    eth_phy_init_cb_t   cb_phy_init;

    // mac address for ethernet if : mandantory
    ARM_ETH_MAC_ADDR    macaddr;
    // if setting value is 0, the the mtu will use the default value 1500 : mandantory
    uint32_t            mtu;

    /* provide a private data for netif private data,
     please init as NULL, will be assign value and used by netif when send and receive
     */
    void *              prv;

    // application layer eth ctx: optional
    void *              cb_ctx;
} netif_eth_ctx_t;

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 */
err_t netif_eth_init(struct netif *netif);

#endif
