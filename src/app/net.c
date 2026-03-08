
//-------------------------------------------------------------------------------------------------
// System include
//-------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>

#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/ip.h"
#include "lwip/netifapi.h"
#include "lwip/ip_addr.h"
#include "lwip/init.h"

#include "netif/eth.h"

#include "EMAC_STM32H7xx.h"
#include "interface_tftp.h"

/*******************************************************************************
                       PHY 8742 Driver
*******************************************************************************/

/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here
  */

  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
    Driver_ETH_MAC0.PHY_Read((uint8_t)DevAddr, (uint8_t)RegAddr, (uint16_t *)pRegVal);
    return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
    Driver_ETH_MAC0.PHY_Write((uint8_t)DevAddr, (uint8_t)RegAddr, (uint16_t)RegVal);
    return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return (int32_t)(tx_time_get() & 0x7FFFFFFF); // Get current time in milliseconds
}
// phy
#include "lan8742.h"
lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx =
{
    ETH_PHY_IO_Init,
    ETH_PHY_IO_DeInit,
    ETH_PHY_IO_WriteReg,
    ETH_PHY_IO_ReadReg,
    ETH_PHY_IO_GetTick
};


// ethernet phy init callback
int32_t eth_phy_init(   ARM_DRIVER_ETH_MAC *mac_drv,
                        ARM_DRIVER_ETH_PHY *phy_drv,
                        ARM_ETH_LINK_INFO * info,
                        void *              cb_ctx)
{
    (void)cb_ctx;
    (void)phy_drv;
    (void)mac_drv;
    /* Set PHY IO functions */
    LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);

    /* Initialize the LAN8742 ETH PHY */
    if(LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
    {
        return -1; // Initialization failed
    }

    LAN8742_GetLinkState(&LAN8742);
    
    // as default, set link down
    info->speed = ARM_ETH_MAC_SPEED_100M;
    info->duplex = (uint8_t)ARM_ETH_MAC_DUPLEX_FULL;

    return 0; // Initialization successful
}


/**
  * @brief  Check the ETH link state and update netif accordingly.
  * @param  argument: netif
  * @retval None
  */

#define DHCP_LINK_DOWN      0x01
#define DHCP_LINK_UP        0x02
static TX_EVENT_FLAGS_GROUP s_dhcp_event_flags;

static TX_THREAD s_eth_link_tcb;
static uint32_t s_eth_link_stack[1024];

void eth_link_demo( void const * argument )
{
    struct netif *netif = (struct netif *) argument;
    ARM_DRIVER_ETH_MAC* pmac = ((netif_eth_ctx_t*)(netif->state))->pdrv_mac;
    int32_t link_state;
    uint32_t ctrl_arg;
    static uint8_t tftp_initialized = 0;  // add a flag

    while(1)
    {
        link_state = LAN8742_GetLinkState(&LAN8742);

        if(netif_is_link_up(netif) && (link_state <= LAN8742_STATUS_LINK_DOWN))
        {
            netif_set_down(netif);
            netif_set_link_down(netif);
            // disable mac tx and rx
            pmac->Control(ARM_ETH_MAC_CONTROL_RX, 0);
            pmac->Control(ARM_ETH_MAC_CONTROL_TX, 0);

            //printf("Network interface is down\n");
            //tx_event_flags_set(&s_dhcp_event_flags, DHCP_LINK_DOWN, TX_OR); // notify DHCP thread

            tftp_initialized = 0;
        }
        else if((!netif_is_link_up(netif)) && (link_state > LAN8742_STATUS_LINK_DOWN))
        {   //connected again
            if(link_state == LAN8742_STATUS_100MBITS_FULLDUPLEX)
            {
               ctrl_arg = ARM_ETH_MAC_SPEED_100M | ARM_ETH_MAC_DUPLEX_FULL | ARM_ETH_MAC_ADDRESS_ALL;
            }
            else if(link_state == LAN8742_STATUS_100MBITS_HALFDUPLEX)
            {
                ctrl_arg = ARM_ETH_MAC_SPEED_100M | ARM_ETH_MAC_DUPLEX_HALF | ARM_ETH_MAC_ADDRESS_ALL;
            }
            else if(link_state == LAN8742_STATUS_10MBITS_FULLDUPLEX)
            {
                ctrl_arg = ARM_ETH_MAC_SPEED_10M | ARM_ETH_MAC_DUPLEX_FULL | ARM_ETH_MAC_ADDRESS_ALL;
            }
            else if(link_state == LAN8742_STATUS_10MBITS_HALFDUPLEX)
            {
                ctrl_arg = ARM_ETH_MAC_SPEED_10M | ARM_ETH_MAC_DUPLEX_HALF | ARM_ETH_MAC_ADDRESS_ALL;
            }
            else
            {   // default to 100M full duplex
                ctrl_arg = ARM_ETH_MAC_SPEED_100M | ARM_ETH_MAC_DUPLEX_FULL | ARM_ETH_MAC_ADDRESS_ALL;
            }
            pmac->Control(ARM_ETH_MAC_CONFIGURE, ctrl_arg);
            pmac->Control(ARM_ETH_MAC_CONTROL_RX, 1);
            pmac->Control(ARM_ETH_MAC_CONTROL_TX, 1);
            netif_set_link_up(netif);
            netif_set_up(netif);

            //printf("Network interface is up\n");

            //tx_event_flags_set(&s_dhcp_event_flags, DHCP_LINK_UP, TX_OR); // notify DHCP thread

            // 链接建立之后（lwip初始化之后），进行tftp初始化（只初始化一次）
            if (!tftp_initialized) {
                tftp_server_init();
                tftp_initialized = 1;
            }
        }
        tx_thread_sleep(200);
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// DHCP

// static TX_THREAD s_dhcp_tcb;
// static uint32_t s_dhcp_stack[1024];

// static void dhcp_demo(void *argument)
// {
//     struct netif *netif = (struct netif *) argument;
//     ip_addr_t ipaddr;
//     ip_addr_t netmask;
//     ip_addr_t gw;
//     struct dhcp *dhcp;
//     uint32_t dhcp_flags = 0;
//     bool dhcp_status = false;
//     int32_t dhcp_timeout;

//     while(1)
//     {
//         tx_event_flags_get(&s_dhcp_event_flags, DHCP_LINK_UP | DHCP_LINK_DOWN, TX_OR_CLEAR, &dhcp_flags, TX_WAIT_FOREVER);
//         if(dhcp_flags & DHCP_LINK_DOWN)
//         {
//             /* Link is down, stop DHCP */
//             //dhcp_release_and_stop(netif);
//             //dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
//         }
//         else if(dhcp_flags & DHCP_LINK_UP)
//         {   // start dhcp if link is up
//             if (dhcp_status == false)
//             {   // dhcp is not start yet                
//                 ip_addr_set_zero_ip4(&netif->ip_addr);
//                 ip_addr_set_zero_ip4(&netif->netmask);
//                 ip_addr_set_zero_ip4(&netif->gw);
//                 dhcp_start(netif);
//             }
//             else
//             {   // try to reboot
//                 dhcp_network_changed_link_up(netif);
//             }
//             // wait dhpc done
//             dhcp_timeout = 60000; // 60 seconds timeout as default
//             while((!dhcp_supplied_address(netif)) && (dhcp_timeout > 0) )
//             {
//                 dhcp_timeout -= 100; // decrement timeout
//                 tx_thread_sleep(100);
//             }
//             // check if dhcp is done
//             if (dhcp_supplied_address(netif))
//             {
//                 dhcp_status = true; // dhcp is done
//                 /* DHCP address used */
//                 printf("DHCP assigned IP: %s\n", ipaddr_ntoa(&netif->ip_addr));
//                 // add mdns
//                 //start_mdns(netif);
//                 continue; // continue to next loop
//             }
//             // default 
//             {   // TODO: dhcp failed when hot unlink and link again 
//                 /* Static address used */
//                 printf("DHCP failed, assign static IP: %s\n", ipaddr_ntoa(&netif->ip_addr));
//                 IP_ADDR4(&ipaddr, 192 ,168 , 1 , 123 );
//                 IP_ADDR4(&netmask, 255, 255, 255, 255);
//                 IP_ADDR4(&gw, 192, 168, 1, 1);
//                 netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
//             }
//         }
//     }
// }


static TX_THREAD s_udp_tcb;
static uint32_t s_udp_stack[1024];
static void udp_echo_demo(void *argument)
{
    (void)argument; // unused parameter
    int32_t            sfd;
    struct sockaddr_in saddr, caddr;
    socklen_t          caddr_len = 0;
    ssize_t            rlen      = 0;
    //uint32_t            cnt = 0;
    uint8_t             rbuf[128] = { 0 };

    /* create a TCP socket */
    if (-1 == (sfd = socket(AF_INET, SOCK_DGRAM, 0)))
    {
        //printf("UDP server socket fail\r\n");
    }
    else
    {
        //printf("UDP server socket success\r\n");
    }
    /* bind to port 80 at any interface */
    saddr.sin_family      = AF_INET;
    saddr.sin_port        = htons(7);
    saddr.sin_addr.s_addr = INADDR_ANY;

    if (-1 == bind(sfd, (struct sockaddr *)&saddr, sizeof(saddr)))
    {
        //printf("UDP bind fail\r\n");
    }
    else
    {
        //printf("UDP server bind port 7\r\n");
    }

    caddr_len = sizeof(struct sockaddr_in);
    memset(&caddr, 0, sizeof(struct sockaddr_in));

    for (;;)
    {
        /* Read in the request */
        memset(rbuf, 0, sizeof(rbuf));
        if (-1 != (rlen = recvfrom(sfd, rbuf, sizeof(rbuf), 0, (struct sockaddr *)&caddr, (socklen_t *)&caddr_len)))
        {
            //cnt++;
            sendto(sfd, rbuf, rlen, 0, (struct sockaddr *)&caddr, caddr_len);
        }
        /* Close connection socket */
    }
}

//-------------------------------------------------------------------------------------------------

static struct netif         s_netif;
static netif_ext_callback_t s_netif_cb;

static netif_eth_ctx_t s_netif_eth_ctx = 
{
    .name       = "eth0",
    .pdrv_mac   = &Driver_ETH_MAC0,
    .pdrv_phy   = NULL,
    .cb_phy_init= eth_phy_init,
    .macaddr    = { 0x00, 0x05, 0x19, 0x00, 0x40, 0x01 },
    .mtu        = 1500,
    .prv        = NULL,
    .cb_ctx     = NULL
};



void netif_ext_callback(struct netif* netif, netif_nsc_reason_t reason, const netif_ext_callback_args_t* args)
{
    //printf("netif_ext_callback: netif=0x%08X, reason=0x%08x\n", netif->name, reason);
	;
}

void net_init(void)
{
	ip_addr_t ipaddr;
	ip_addr_t netmask;
	ip_addr_t gw;

    // Initialize the lwIP stack
    tcpip_init(NULL, NULL); // 1、初始化lwip协议栈

    // add the network interface to the list of network interfaces
    netif_add_ext_callback(&s_netif_cb, netif_ext_callback);

    // netif initialization
    netif_add(&s_netif, NULL, NULL, NULL, &s_netif_eth_ctx, netif_eth_init, tcpip_input);   // 2、添加网络接口
	
	
    /* 设置静态IP地址 */
    IP4_ADDR(&ipaddr, 192, 168, 2, 100);    // 电路板IP
    IP4_ADDR(&netmask, 255, 255, 255, 0);   // 子网掩码
    IP4_ADDR(&gw, 192, 168, 2, 1);          // 网关
    
    netif_set_addr(&s_netif, &ipaddr, &netmask, &gw);   // 3、设置IP
	
    /*  Registers the default network interface. */
    netif_set_default(&s_netif);    // 4、设置为默认接口

    // Set the network interface up
    netif_set_up(&s_netif); // 5、启用接口

    // create link check demo
    // tx_thread_create(&s_eth_link_tcb, "eth_link_demo", (void *)eth_link_demo,
    //                  (ULONG)&s_netif, s_eth_link_stack, sizeof(s_eth_link_stack),
    //                  TX_MAX_PRIORITIES - 4 , TX_MAX_PRIORITIES - 4 , TX_NO_TIME_SLICE, TX_AUTO_START);
     tx_thread_create(&s_eth_link_tcb, "eth_link_demo", (void *)eth_link_demo,
                      (ULONG)&s_netif, s_eth_link_stack, sizeof(s_eth_link_stack),
                      15,15, TX_NO_TIME_SLICE, TX_AUTO_START);
    // // create dhcp demo
    // tx_event_flags_create(&s_dhcp_event_flags, "dhcp_event_flags");
    // tx_thread_create(&s_dhcp_tcb, "dhcp_demo", dhcp_demo,
    //                   &s_netif, s_dhcp_stack, sizeof(s_dhcp_stack),
    //                   TX_MAX_PRIORITIES - 1 , TX_MAX_PRIORITIES - 1 , TX_NO_TIME_SLICE, TX_AUTO_START);
                      // create tcp echo demo
    // tx_thread_create(&s_tcp_tcb, "tcp_echo_demo", tcp_echo_demo,
    //                   &s_netif, s_tcp_stack, sizeof(s_tcp_stack),
    //                   TX_MAX_PRIORITIES - 2 , TX_MAX_PRIORITIES - 2 , TX_NO_TIME_SLICE, TX_AUTO_START);
                      // create udp echo demo
    //  tx_thread_create(&s_udp_tcb, "udp_echo_demo", (void *)udp_echo_demo,
    //                   (ULONG)&s_netif, s_udp_stack, sizeof(s_udp_stack), 
    //                   16, 16, TX_NO_TIME_SLICE, TX_AUTO_START);
}

