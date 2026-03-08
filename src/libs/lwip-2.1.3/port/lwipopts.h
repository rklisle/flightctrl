
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/**
 * NO_SYS==1: Use lwIP without OS-awareness (no thread, semaphores, mutexes or
 * mboxes). This means threaded APIs cannot be used (socket, netconn,
 * i.e. everything in the 'api' folder), only the callback-style raw API is
 * available (and you have to watch out for yourself that you don't access
 * lwIP functions/structures from more than one context at a time!)
 */
#define NO_SYS                             0

/**
* Number of IP interface the system can support
*/
#define SYS_NETIF_NUM                      1

/** LWIP_PROVIDE_ERRNO==1: Let lwIP provide ERRNO values and the 'errno' variable.
 * If this is disabled, cc.h must either define 'errno', include <errno.h>,
 * define LWIP_ERRNO_STDINCLUDE to get <errno.h> included or
 * define LWIP_ERRNO_INCLUDE to <errno.h> or equivalent.
 */
#define LWIP_PROVIDE_ERRNO                 1

/*
   ------------------------------------
   ----------- Core locking -----------
   ------------------------------------
*/
/**
 * @defgroup lwip_opts_lock Core locking and MPU
 * @ingroup lwip_opts_infrastructure
 * @{
 */
/**
 * LWIP_MPU_COMPATIBLE: enables special memory management mechanism
 * which makes lwip able to work on MPU (Memory Protection Unit) system
 * by not passing stack-pointers to other threads
 * (this decreases performance as memory is allocated from pools instead
 * of keeping it on the stack)
 */
#define LWIP_MPU_COMPATIBLE                0  //1 zsh

/**
 * LWIP_TCPIP_CORE_LOCKING
 * Creates a global mutex that is held during TCPIP thread operations.
 * Can be locked by client code to perform lwIP operations without changing
 * into TCPIP thread using callbacks. See LOCK_TCPIP_CORE() and
 * UNLOCK_TCPIP_CORE().
 * Your system should provide mutexes supporting priority inversion to use this.
 */
#define LWIP_TCPIP_CORE_LOCKING            1

/**
 * SYS_LIGHTWEIGHT_PROT==1: enable inter-task protection (and task-vs-interrupt
 * protection) for certain critical regions during buffer allocation, deallocation
 * and memory allocation and deallocation.
 * ATTENTION: This is required when using lwIP from more than one context! If
 * you disable this, you must be sure what you are doing!
 */
#define SYS_LIGHTWEIGHT_PROT               1

/*
   ------------------------------------
   ---------- Memory options ----------
   ------------------------------------
*/

/**
 * MEM_ALIGNMENT: should be set to the alignment of the CPU
 *    4 byte alignment -> \#define MEM_ALIGNMENT 4
 *    2 byte alignment -> \#define MEM_ALIGNMENT 2
 */
#define MEM_ALIGNMENT                      8

/**
 * MEM_SIZE: the size of the heap memory. If the application will send
 * a lot of data that needs to be copied, this should be set high.
 */
#define MEM_SIZE                           (16 * 1024)
/**
 * MEMP_OVERFLOW_CHECK: memp overflow protection reserves a configurable
 * amount of bytes before and after each memp element in every pool and fills
 * it with a prominent default value.
 *    MEMP_OVERFLOW_CHECK == 0 no checking
 *    MEMP_OVERFLOW_CHECK == 1 checks each element when it is freed
 *    MEMP_OVERFLOW_CHECK >= 2 checks each element in every pool every time
 *      memp_malloc() or memp_free() is called (useful but slow!)
 */
#define MEMP_OVERFLOW_CHECK                0

/**
 * MEMP_SANITY_CHECK==1: run a sanity check after each memp_free() to make
 * sure that there are no cycles in the linked lists.
 */
#define MEMP_SANITY_CHECK                  0

/**
 * MEM_OVERFLOW_CHECK: mem overflow protection reserves a configurable
 * amount of bytes before and after each heap allocation chunk and fills
 * it with a prominent default value.
 *    MEM_OVERFLOW_CHECK == 0 no checking
 *    MEM_OVERFLOW_CHECK == 1 checks each element when it is freed
 *    MEM_OVERFLOW_CHECK >= 2 checks all heap elements every time
 *      mem_malloc() or mem_free() is called (useful but slow!)
 */
#define MEM_OVERFLOW_CHECK                 0

/**
 * MEM_SANITY_CHECK==1: run a sanity check after each mem_free() to make
 * sure that the linked list of heap elements is not corrupted.
 */
#define MEM_SANITY_CHECK                   0

/**
 * MEMP_NUM_PBUF: the number of memp struct pbufs (used for PBUF_ROM and PBUF_REF).
 * If the application sends a lot of data out of ROM (or other static memory),
 * this should be set high.
 */
#define MEMP_NUM_PBUF                      60

/**
 * MEMP_NUM_RAW_PCB: Number of raw connection PCBs
 * (requires the LWIP_RAW option)
 */
#define MEMP_NUM_RAW_PCB                   4

/**
 * MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
 * per active UDP "connection".
 * (requires the LWIP_UDP option)
 */
#define MEMP_NUM_UDP_PCB                   8

/**
 * MEMP_NUM_TCP_PCB: the number of simultaneously active TCP connections.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_PCB                   20

/**
 * MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP connections.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_PCB_LISTEN            15

/**
 * MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_SEG                   20

/**
 * MEMP_NUM_REASSDATA: the number of IP packets simultaneously queued for
 * reassembly (whole packets, not fragments!)
 */
#define MEMP_NUM_REASSDATA                 5

/**
 * MEMP_NUM_FRAG_PBUF: the number of IP fragments simultaneously sent
 * (fragments, not whole packets!).
 * This is only used with LWIP_NETIF_TX_SINGLE_PBUF==0 and only has to be > 1
 * with DMA-enabled MACs where the packet is not yet sent when netif->output
 * returns.
 */
#define MEMP_NUM_FRAG_PBUF                 15

/**
 * MEMP_NUM_ARP_QUEUE: the number of simultaneously queued outgoing
 * packets (pbufs) that are waiting for an ARP request (to resolve
 * their destination address) to finish.
 * (requires the ARP_QUEUEING option)
 */
#define MEMP_NUM_ARP_QUEUE                 30

/**
 * MEMP_NUM_SYS_TIMEOUT: the number of simultaneously active timeouts.
 * The default number of timeouts is calculated here for all enabled modules.
 * The formula expects settings to be either '0' or '1'.
 */
#define MEMP_NUM_SYS_TIMEOUT               100

/**
 * MEMP_NUM_NETBUF: the number of struct netbufs.
 * (only needed if you use the sequential API, like api_lib.c)
 */
#define MEMP_NUM_NETBUF                    30

/**
 * MEMP_NUM_NETCONN: the number of struct netconns.
 * (only needed if you use the sequential API, like api_lib.c)
 */
#define MEMP_NUM_NETCONN                   20

/**
 * MEMP_NUM_SELECT_CB: the number of struct lwip_select_cb.
 * (Only needed if you have LWIP_MPU_COMPATIBLE==1 and use the socket API.
 * In that case, you need one per thread calling lwip_select.)
 */
#define MEMP_NUM_SELECT_CB                 4

/**
 * MEMP_NUM_TCPIP_MSG_API: the number of struct tcpip_msg, which are used
 * for callback/timeout API communication.
 * (only needed if you use tcpip.c)
 */
#define MEMP_NUM_TCPIP_MSG_API             8

/**
 * MEMP_NUM_TCPIP_MSG_INPKT: the number of struct tcpip_msg, which are used
 * for incoming packets.
 * (only needed if you use tcpip.c)
 */
#define MEMP_NUM_TCPIP_MSG_INPKT           8

/**
 * PBUF_POOL_SIZE: the number of buffers in the pbuf pool.
 */
#define PBUF_POOL_SIZE                     32

/*
   ---------------------------------
   ---------- ARP options ----------
   ---------------------------------
*/
/**
 * @defgroup lwip_opts_arp ARP
 * @ingroup lwip_opts_ipv4
 * @{
 */
/**
 * LWIP_ARP==1: Enable ARP functionality.
 */
#define LWIP_ARP                           1

/**
 * ARP_TABLE_SIZE: Number of active MAC-IP address pairs cached.
 */
#define ARP_TABLE_SIZE                     10

/** the time an ARP entry stays valid after its last update,
 *  for ARP_TMR_INTERVAL = 1000, this is
 *  (60 * 5) seconds = 5 minutes.
 */
#define ARP_MAXAGE                         300

/**
 * ARP_QUEUEING==1: Multiple outgoing packets are queued during hardware address
 * resolution. By default, only the most recent packet is queued per IP address.
 * This is sufficient for most protocols and mainly reduces TCP connection
 * startup time. Set this to 1 if you know your application sends more than one
 * packet in a row to an IP address that is not in the ARP cache.
 */
#define ARP_QUEUEING                       0

/** The maximum number of packets which may be queued for each
 *  unresolved address by other network layers. Defaults to 3, 0 means disabled.
 *  Old packets are dropped, new packets are queued.
 */
#define ARP_QUEUE_LEN                      3

/**
 * ETHARP_SUPPORT_VLAN==1: support receiving and sending ethernet packets with
 * VLAN header. See the description of LWIP_HOOK_VLAN_CHECK and
 * LWIP_HOOK_VLAN_SET hooks to check/set VLAN headers.
 * Additionally, you can define ETHARP_VLAN_CHECK to an u16_t VLAN ID to check.
 * If ETHARP_VLAN_CHECK is defined, only VLAN-traffic for this VLAN is accepted.
 * If ETHARP_VLAN_CHECK is not defined, all traffic is accepted.
 * Alternatively, define a function/define ETHARP_VLAN_CHECK_FN(eth_hdr, vlan)
 * that returns 1 to accept a packet or 0 to drop a packet.
 */
#define ETHARP_SUPPORT_VLAN                0

/** LWIP_ETHERNET==1: enable ethernet support even though ARP might be disabled
 */
#define LWIP_ETHERNET                      LWIP_ARP

/** ETH_PAD_SIZE: number of bytes added before the ethernet header to ensure
 * alignment of payload after that header. Since the header is 14 bytes long,
 * without this padding e.g. addresses in the IP header will not be aligned
 * on a 32-bit boundary, so setting this to 2 can speed up 32-bit-platforms.
 */
#define ETH_PAD_SIZE                       0

/** ETHARP_SUPPORT_STATIC_ENTRIES==1: enable code to support static ARP table
 * entries (using etharp_add_static_entry/etharp_remove_static_entry).
 */
#define ETHARP_SUPPORT_STATIC_ENTRIES      0

/**
 * @}
 */

/* enable lwip mdns */
#define LWIP_MDNS_RESPONDER               1

/*
   --------------------------------
   ---------- IP options ----------
   --------------------------------
*/
/**
 * @defgroup lwip_opts_ipv4 IPv4
 * @ingroup lwip_opts
 * @{
 */
/**
 * LWIP_IPV4==1: Enable IPv4
 */
#define LWIP_IPV4                          1
/**
 * IP_FORWARD==1: Enables the ability to forward IP packets across network
 * interfaces. If you are going to run lwIP on a device with only one network
 * interface, define this to 0.
 */
#define IP_FORWARD                         (SYS_NETIF_NUM > 1)

/**
 * IP_REASSEMBLY==1: Reassemble incoming fragmented IP packets. Note that
 * this option does not affect outgoing packet sizes, which can be controlled
 * via IP_FRAG.
 */
#define IP_REASSEMBLY                      1

/**
 * IP_FRAG==1: Fragment outgoing IP packets if their size exceeds MTU. Note
 * that this option does not affect incoming packet sizes, which can be
 * controlled via IP_REASSEMBLY.
 */
#define IP_FRAG                            1

/**
 * IP_OPTIONS_ALLOWED: Defines the behavior for IP options.
 *      IP_OPTIONS_ALLOWED==0: All packets with IP options are dropped.
 *      IP_OPTIONS_ALLOWED==1: IP options are allowed (but not parsed).
 */
#define IP_OPTIONS_ALLOWED                 1

/**
 * IP_REASS_MAXAGE: Maximum time (in multiples of IP_TMR_INTERVAL - so seconds, normally)
 * a fragmented IP packet waits for all fragments to arrive. If not all fragments arrived
 * in this time, the whole packet is discarded.
 */
#define IP_REASS_MAXAGE                    3

/**
 * IP_REASS_MAX_PBUFS: Total maximum amount of pbufs waiting to be reassembled.
 * Since the received pbufs are enqueued, be sure to configure
 * PBUF_POOL_SIZE > IP_REASS_MAX_PBUFS so that the stack is still able to receive
 * packets even if the maximum amount of fragments is enqueued for reassembly!
 * When IPv4 *and* IPv6 are enabled, this even changes to
 * (PBUF_POOL_SIZE > 2 * IP_REASS_MAX_PBUFS)!
 */
#define IP_REASS_MAX_PBUFS                 10

/**
 * IP_DEFAULT_TTL: Default value for Time-To-Live used by transport layers.
 */
#define IP_DEFAULT_TTL                     255

/**
 * IP_SOF_BROADCAST=1: Use the SOF_BROADCAST field to enable broadcast
 * filter per pcb on udp and raw send operations. To enable broadcast filter
 * on recv operations, you also have to set IP_SOF_BROADCAST_RECV=1.
 */
#define IP_SOF_BROADCAST                   0

/**
 * IP_SOF_BROADCAST_RECV (requires IP_SOF_BROADCAST=1) enable the broadcast
 * filter on recv operations.
 */
#define IP_SOF_BROADCAST_RECV              0

/**
 * @}
 */

/*
   ----------------------------------
   ---------- ICMP options ----------
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_icmp ICMP
 * @ingroup lwip_opts_ipv4
 * @{
 */
/**
 * LWIP_ICMP==1: Enable ICMP module inside the IP stack.
 * Be careful, disable that make your product non-compliant to RFC1122
 */
#define LWIP_ICMP                          1

/**
 * ICMP_TTL: Default value for Time-To-Live used by ICMP packets.
 */
#define ICMP_TTL                           IP_DEFAULT_TTL

/**
 * LWIP_BROADCAST_PING==1: respond to broadcast pings (default is unicast only)
 */
#define LWIP_BROADCAST_PING                1

/**
 * LWIP_MULTICAST_PING==1: respond to multicast pings (default is unicast only)
 */
#define LWIP_MULTICAST_PING                1  //0
/**
 * @}
 */

/*
   ----------------------------------
   ---------- DHCP options ----------
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_dhcp DHCP
 * @ingroup lwip_opts_ipv4
 * @{
 */
/**
 * LWIP_DHCP==1: Enable DHCP module.
 */
#define LWIP_DHCP                          1

/**
 * The maximum of NTP servers requested
 */
#define LWIP_DHCP_MAX_NTP_SERVERS          1

/**
 * LWIP_DHCP_MAX_DNS_SERVERS > 0: Request DNS servers with discover/select.
 * DNS servers received in the response are passed to DNS via @ref dns_setserver()
 * (up to the maximum limit defined here).
 */
#define LWIP_DHCP_MAX_DNS_SERVERS          DNS_MAX_SERVERS
/**
 * @}
 */

/*
   ------------------------------------
   ---------- AUTOIP options ----------
   ------------------------------------
*/
/**
 * @defgroup lwip_opts_autoip AUTOIP
 * @ingroup lwip_opts_ipv4
 * @{
 */
/**
 * LWIP_AUTOIP==1: Enable AUTOIP module.
 */
#define LWIP_AUTOIP                        1

/**
 * LWIP_DHCP_AUTOIP_COOP==1: Allow DHCP and AUTOIP to be both enabled on
 * the same interface at the same time.
 */
#define LWIP_DHCP_AUTOIP_COOP              ((LWIP_DHCP) && (LWIP_AUTOIP))

/**
 * LWIP_DHCP_AUTOIP_COOP_TRIES: Set to the number of DHCP DISCOVER probes
 * that should be sent before falling back on AUTOIP (the DHCP client keeps
 * running in this case). This can be set as low as 1 to get an AutoIP address
 * very  quickly, but you should be prepared to handle a changing IP address
 * when DHCP overrides AutoIP.
 */
#define LWIP_DHCP_AUTOIP_COOP_TRIES        5

/**
 * @}
 */

/*
   ----------------------------------
   ----- SNMP MIB2 support      -----
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_mib2 SNMP MIB2 callbacks
 * @ingroup lwip_opts_infrastructure
 * @{
 */
/**
 * LWIP_MIB2_CALLBACKS==1: Turn on SNMP MIB2 callbacks.
 * Turn this on to get callbacks needed to implement MIB2.
 * Usually MIB2_STATS should be enabled, too.
 */
#define LWIP_MIB2_CALLBACKS                0
/**
 * @}
 */

/*
   ----------------------------------
   ---------- IGMP options ----------
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_igmp IGMP
 * @ingroup lwip_opts_ipv4
 * @{
 */
/**
 * LWIP_IGMP==1: Turn on IGMP module.
 */
#define LWIP_IGMP                          1

/**
 * @}
 */

/*
   ----------------------------------
   ---------- DNS options -----------
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_dns DNS
 * @ingroup lwip_opts_callback
 * @{
 */
/**
 * LWIP_DNS==1: Turn on DNS module. UDP must be available for DNS
 * transport.
 */
#define LWIP_DNS                           1

/** DNS maximum number of entries to maintain locally. */
#define DNS_TABLE_SIZE                     4

/** DNS maximum host name length supported in the name table. */
#define DNS_MAX_NAME_LENGTH                256

/** The maximum of DNS servers
 * The first server can be initialized automatically by defining
 * DNS_SERVER_ADDRESS(ipaddr), where 'ipaddr' is an 'ip_addr_t*'
 */
#define DNS_MAX_SERVERS                    2

/** DNS maximum number of retries when asking for a name, before "timeout". */
#define DNS_MAX_RETRIES                    4

/** DNS do a name checking between the query and the response. */
#define DNS_DOES_NAME_CHECK                1

/**
 * @}
 */

/*
   ---------------------------------
   ---------- UDP options ----------
   ---------------------------------
*/
/**
 * @defgroup lwip_opts_udp UDP
 * @ingroup lwip_opts_callback
 * @{
 */
/**
 * LWIP_UDP==1: Turn on UDP.
 */
#define LWIP_UDP                           1

/**
 * UDP_TTL: Default Time-To-Live value.
 */
#define UDP_TTL                            IP_DEFAULT_TTL

/**
 * @}
 */

/*
   ---------------------------------
   ---------- TCP options ----------
   ---------------------------------
*/
/**
 * @defgroup lwip_opts_tcp TCP
 * @ingroup lwip_opts_callback
 * @{
 */
/**
 * LWIP_TCP==1: Turn on TCP.
 */
#define LWIP_TCP                           1

/**
 * TCP_TTL: Default Time-To-Live value.
 */
#define TCP_TTL                            IP_DEFAULT_TTL

/**
 * TCP_WND: The size of a TCP window.  This must be at least
 * (2 * TCP_MSS) for things to work well.
 * ATTENTION: when using TCP_RCV_SCALE, TCP_WND is the total size
 * with scaling applied. Maximum window value in the TCP header
 * will be TCP_WND >> TCP_RCV_SCALE
 */
#define TCP_WND                            (4 * TCP_MSS)

/**
 * TCP_MAXRTX: Maximum number of retransmissions of data segments.
 */
#define TCP_MAXRTX                         6

/**
 * TCP_SYNMAXRTX: Maximum number of retransmissions of SYN segments.
 */
#define TCP_SYNMAXRTX                      3

/**
 * TCP_QUEUE_OOSEQ==1: TCP will queue segments that arrive out of order.
 * Define to 0 if your device is low on memory.
 */
#define TCP_QUEUE_OOSEQ                    LWIP_TCP

/**
 * LWIP_TCP_SACK_OUT==1: TCP will support sending selective acknowledgements (SACKs).
 */
#define LWIP_TCP_SACK_OUT                  0

/**
 * LWIP_TCP_MAX_SACK_NUM: The maximum number of SACK values to include in TCP segments.
 * Must be at least 1, but is only used if LWIP_TCP_SACK_OUT is enabled.
 * NOTE: Even though we never send more than 3 or 4 SACK ranges in a single segment
 * (depending on other options), setting this option to values greater than 4 is not pointless.
 * This is basically the max number of SACK ranges we want to keep track of.
 * As new data is delivered, some of the SACK ranges may be removed or merged.
 * In that case some of those older SACK ranges may be used again.
 * The amount of memory used to store SACK ranges is LWIP_TCP_MAX_SACK_NUM * 8 bytes for each TCP PCB.
 */
#define LWIP_TCP_MAX_SACK_NUM              4

/**
 * TCP_MSS: TCP Maximum segment size. (default is 536, a conservative default,
 * you might want to increase this.)
 * For the receive side, this MSS is advertised to the remote side
 * when opening a connection. For the transmit size, this MSS sets
 * an upper limit on the MSS advertised by the remote host.
 */
#define TCP_MSS                            (1500 - 40)

/**
 * TCP_CALCULATE_EFF_SEND_MSS: "The maximum size of a segment that TCP really
 * sends, the 'effective send MSS,' MUST be the smaller of the send MSS (which
 * reflects the available reassembly buffer size at the remote host) and the
 * largest size permitted by the IP layer" (RFC 1122)
 * Setting this to 1 enables code that checks TCP_MSS against the MTU of the
 * netif used for a connection and limits the MSS if it would be too big otherwise.
 */
#define TCP_CALCULATE_EFF_SEND_MSS         1

/**
 * TCP_SND_BUF: TCP sender buffer space (bytes).
 * To achieve good performance, this should be at least 2 * TCP_MSS.
 */
#define TCP_SND_BUF                        (4 * TCP_MSS)

/**
 * TCP_SND_QUEUELEN: TCP sender buffer space (pbufs). This must be at least
 * as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work.
 */
#define TCP_SND_QUEUELEN                   ((2 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

/**
 * TCP_LISTEN_BACKLOG: Enable the backlog option for tcp listen pcb.
 */
#define TCP_LISTEN_BACKLOG                 0

/**
 * LWIP_TCP_TIMESTAMPS==1: support the TCP timestamp option.
 * The timestamp option is currently only used to help remote hosts, it is not
 * really used locally. Therefore, it is only enabled when a TS option is
 * received in the initial SYN packet from a remote host.
 */
#define LWIP_TCP_TIMESTAMPS                0

/**
 * TCP_WND_UPDATE_THRESHOLD: difference in window to trigger an
 * explicit window update
 */
#define TCP_WND_UPDATE_THRESHOLD           LWIP_MIN((TCP_WND / 4), (TCP_MSS * 4))

/**
 * @}
 */

/*
   ----------------------------------
   ---------- Pbuf options ----------
   ----------------------------------
*/
/**
 * @defgroup lwip_opts_pbuf PBUF
 * @ingroup lwip_opts
 * @{
 */

/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. The default is
 * designed to accommodate single full size TCP frame in one pbuf, including
 * TCP_MSS, IP header, and link header.
 */
#define PBUF_POOL_BUFSIZE                  1524

/**
 * @}
 */

/*
   ------------------------------------------------
   ---------- Network Interfaces options ----------
   ------------------------------------------------
*/
/**
 * @defgroup lwip_opts_netif NETIF
 * @ingroup lwip_opts
 * @{
 */
/**
 * LWIP_SINGLE_NETIF==1: use a single netif only. This is the common case for
 * small real-life targets. Some code like routing etc. can be left out.
 */
#define LWIP_SINGLE_NETIF                  1

/**
 * LWIP_NETIF_HOSTNAME==1: use DHCP_OPTION_HOSTNAME with netif's hostname
 * field.
 */
#define LWIP_NETIF_HOSTNAME                1

/**
 * LWIP_NETIF_API==1: Support netif api (in netifapi.c)
 */
#define LWIP_NETIF_API                     1

/**
 * LWIP_NETIF_STATUS_CALLBACK==1: Support a callback function whenever an interface
 * changes its up/down status (i.e., due to DHCP IP acquisition)
 */
#define LWIP_NETIF_STATUS_CALLBACK         1

/**
 * LWIP_NETIF_EXT_STATUS_CALLBACK==1: Support an extended callback function
 * for several netif related event that supports multiple subscribers.
 * @see netif_ext_status_callback
 */
#define LWIP_NETIF_EXT_STATUS_CALLBACK     1

/**
 * LWIP_NETIF_LINK_CALLBACK==1: Support a callback function from an interface
 * whenever the link changes (i.e., link down)
 */
#define LWIP_NETIF_LINK_CALLBACK           1

/**
 * LWIP_NETIF_REMOVE_CALLBACK==1: Support a callback function that is called
 * when a netif has been removed
 */
#define LWIP_NETIF_REMOVE_CALLBACK         1

/**
 * LWIP_NETIF_HWADDRHINT==1: Cache link-layer-address hints (e.g. table
 * indices) in struct netif. TCP and UDP can make use of this to prevent
 * scanning the ARP table for every sent packet. While this is faster for big
 * ARP tables or many concurrent connections, it might be counterproductive
 * if you have a tiny ARP table or if there never are concurrent connections.
 */
#define LWIP_NETIF_HWADDRHINT              0

/**
 * LWIP_NETIF_TX_SINGLE_PBUF: if this is set to 1, lwIP *tries* to put all data
 * to be sent into one single pbuf. This is for compatibility with DMA-enabled
 * MACs that do not support scatter-gather.
 * Beware that this might involve CPU-memcpy before transmitting that would not
 * be needed without this flag! Use this only if you need to!
 *
 * ATTENTION: a driver should *NOT* rely on getting single pbufs but check TX
 * pbufs for being in one piece. If not, @ref pbuf_clone can be used to get
 * a single pbuf:
 *   if (p->next != NULL) {
 *     struct pbuf *q = pbuf_clone(PBUF_RAW, PBUF_RAM, p);
 *     if (q == NULL) {
 *       return ERR_MEM;
 *     }
 *     p = q; ATTENTION: do NOT free the old 'p' as the ref belongs to the caller!
 *   }
 */
#define LWIP_NETIF_TX_SINGLE_PBUF          0

/**
 * LWIP_NUM_NETIF_CLIENT_DATA: Number of clients that may store
 * data in client_data member array of struct netif (max. 256).
 */
#define LWIP_NUM_NETIF_CLIENT_DATA         0
/**
 * @}
 */

/*
   ------------------------------------
   ---------- LOOPIF options ----------
   ------------------------------------
*/
/**
 * @defgroup lwip_opts_loop Loopback interface
 * @ingroup lwip_opts_netif
 * @{
 */
/**
 * LWIP_HAVE_LOOPIF==1: Support loop interface (127.0.0.1).
 * This is only needed when no real netifs are available. If at least one other
 * netif is available, loopback traffic uses this netif.
 */
#define LWIP_HAVE_LOOPIF                   (LWIP_NETIF_LOOPBACK && !LWIP_SINGLE_NETIF)

/**
 * LWIP_LOOPIF_MULTICAST==1: Support multicast/IGMP on loop interface (127.0.0.1).
 */
#define LWIP_LOOPIF_MULTICAST              0

/**
 * LWIP_NETIF_LOOPBACK==1: Support sending packets with a destination IP
 * address equal to the netif IP address, looping them back up the stack.
 */
#define LWIP_NETIF_LOOPBACK                0

/**
 * LWIP_LOOPBACK_MAX_PBUFS: Maximum number of pbufs on queue for loopback
 * sending for each netif (0 = disabled)
 */
#define LWIP_LOOPBACK_MAX_PBUFS            0

/**
 * LWIP_NETIF_LOOPBACK_MULTITHREADING: Indicates whether threading is enabled in
 * the system, as netifs must change how they behave depending on this setting
 * for the LWIP_NETIF_LOOPBACK option to work.
 * Setting this is needed to avoid reentering non-reentrant functions like
 * tcp_input().
 *    LWIP_NETIF_LOOPBACK_MULTITHREADING==1: Indicates that the user is using a
 *       multithreaded environment like tcpip.c. In this case, netif->input()
 *       is called directly.
 *    LWIP_NETIF_LOOPBACK_MULTITHREADING==0: Indicates a polling (or NO_SYS) setup.
 *       The packets are put on a list and netif_poll() must be called in
 *       the main application loop.
 */
#define LWIP_NETIF_LOOPBACK_MULTITHREADING (!NO_SYS)
/**
 * @}
 */

/*
   ------------------------------------
   ---------- Thread options ----------
   ------------------------------------
*/
/**
 * @defgroup lwip_opts_thread Threading
 * @ingroup lwip_opts_infrastructure
 * @{
 */
/**
 * TCPIP_THREAD_NAME: The name assigned to the main tcpip thread.
 */
#define TCPIP_THREAD_NAME                  "tcpip_thread"

/**
 * TCPIP_THREAD_STACKSIZE: The stack size used by the main tcpip thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_STACKSIZE             2048

/**
 * TCPIP_THREAD_PRIO: The priority assigned to the main tcpip thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_PRIO                  14//(TX_MAX_PRIORITIES - 4)

/**
 * TCPIP_MBOX_SIZE: The mailbox size for the tcpip thread messages
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when tcpip_init is called.
 */
#define TCPIP_MBOX_SIZE                    32

/**
 * Define this to something that triggers a watchdog. This is called from
 * tcpip_thread after processing a message.
 */
#define LWIP_TCPIP_THREAD_ALIVE()

/**
 * DEFAULT_THREAD_NAME: The name assigned to any other lwIP thread.
 */
#define DEFAULT_THREAD_NAME               "lwIP"

/**
 * DEFAULT_THREAD_STACKSIZE: The stack size used by any other lwIP thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define DEFAULT_THREAD_STACKSIZE          512

/**
 * DEFAULT_THREAD_PRIO: The priority assigned to any other lwIP thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define DEFAULT_THREAD_PRIO               1

/**
 * DEFAULT_RAW_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_RAW. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_RAW_RECVMBOX_SIZE         16

/**
 * DEFAULT_UDP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_UDP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_UDP_RECVMBOX_SIZE         128

/**
 * DEFAULT_TCP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_TCP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_TCP_RECVMBOX_SIZE         128

/**
 * DEFAULT_ACCEPTMBOX_SIZE: The mailbox size for the incoming connections.
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when the acceptmbox is created.
 */
#define DEFAULT_ACCEPTMBOX_SIZE           128
/**
 * @}
 */

/*
   ----------------------------------------------
   ---------- Sequential layer options ----------
   ----------------------------------------------
*/
/**
 * @defgroup lwip_opts_netconn Netconn
 * @ingroup lwip_opts_threadsafe_apis
 * @{
 */
/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN                      1

/** LWIP_TCPIP_TIMEOUT==1: Enable tcpip_timeout/tcpip_untimeout to create
 * timers running in tcpip_thread from another thread.
 */
#define LWIP_TCPIP_TIMEOUT                1

/** LWIP_NETCONN_SEM_PER_THREAD==1: Use one (thread-local) semaphore per
 * thread calling socket/netconn functions instead of allocating one
 * semaphore per netconn (and per select etc.)
 * ATTENTION: a thread-local semaphore for API calls is needed:
 * - LWIP_NETCONN_THREAD_SEM_GET() returning a sys_sem_t*
 * - LWIP_NETCONN_THREAD_SEM_ALLOC() creating the semaphore
 * - LWIP_NETCONN_THREAD_SEM_FREE() freeing the semaphore
 * The latter 2 can be invoked up by calling netconn_thread_init()/netconn_thread_cleanup().
 * Ports may call these for threads created with sys_thread_new().
 */
#define LWIP_NETCONN_SEM_PER_THREAD       0

/** LWIP_NETCONN_FULLDUPLEX==1: Enable code that allows reading from one thread,
 * writing from a 2nd thread and closing from a 3rd thread at the same time.
 * ATTENTION: This is currently really alpha! Some requirements:
 * - LWIP_NETCONN_SEM_PER_THREAD==1 is required to use one socket/netconn from
 *   multiple threads at once
 * - sys_mbox_free() has to unblock receive tasks waiting on recvmbox/acceptmbox
 *   and prevent a task pending on this during/after deletion
 */
#define LWIP_NETCONN_FULLDUPLEX           0
/**
 * @}
 */

/*
   ------------------------------------
   ---------- Socket options ----------
   ------------------------------------
*/
/**
 * @defgroup lwip_opts_socket Sockets
 * @ingroup lwip_opts_threadsafe_apis
 * @{
 */
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET                       1

/**
 * LWIP_COMPAT_SOCKETS==1: Enable BSD-style sockets functions names through defines.
 * LWIP_COMPAT_SOCKETS==2: Same as ==1 but correctly named functions are created.
 * While this helps code completion, it might conflict with existing libraries.
 * (only used if you use sockets.c)
 */
#define LWIP_COMPAT_SOCKETS               1

/**
 * LWIP_POSIX_SOCKETS_IO_NAMES==1: Enable POSIX-style sockets functions names.
 * Disable this option if you use a POSIX operating system that uses the same
 * names (read, write & close). (only used if you use sockets.c)
 */
#define LWIP_POSIX_SOCKETS_IO_NAMES       1

/**
 * LWIP_SOCKET_OFFSET==n: Increases the file descriptor number created by LwIP with n.
 * This can be useful when there are multiple APIs which create file descriptors.
 * When they all start with a different offset and you won't make them overlap you can
 * re implement read/write/close/ioctl/fnctl to send the requested action to the right
 * library (sharing select will need more work though).
 */
#define LWIP_SOCKET_OFFSET                1

/**
 * LWIP_TCP_KEEPALIVE==1: Enable TCP_KEEPIDLE, TCP_KEEPINTVL and TCP_KEEPCNT
 * options processing. Note that TCP_KEEPIDLE and TCP_KEEPINTVL have to be set
 * in seconds. (does not require sockets.c, and will affect tcp.c)
 */
#define LWIP_TCP_KEEPALIVE                0

/**
 * LWIP_SO_SNDTIMEO==1: Enable send timeout for sockets/netconns and
 * SO_SNDTIMEO processing.
 */
#define LWIP_SO_SNDTIMEO                  1

/**
 * LWIP_SO_RCVTIMEO==1: Enable receive timeout for sockets/netconns and
 * SO_RCVTIMEO processing.
 */
#define LWIP_SO_RCVTIMEO                  1

/**
 * LWIP_SO_SNDRCVTIMEO_NONSTANDARD==1: SO_RCVTIMEO/SO_SNDTIMEO take an int
 * (milliseconds, much like winsock does) instead of a struct timeval (default).
 */
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD   0

/**
 * LWIP_SO_RCVBUF==1: Enable SO_RCVBUF processing.
 */
#define LWIP_SO_RCVBUF                    1

/**
 * LWIP_SO_LINGER==1: Enable SO_LINGER processing.
 */
#define LWIP_SO_LINGER                    0

/**
 * If LWIP_SO_RCVBUF is used, this is the default value for recv_bufsize.
 */
#define RECV_BUFSIZE_DEFAULT              INT_MAX

/**
 * By default, TCP socket/netconn close waits 20 seconds max to send the FIN
 */
#define LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT 20000

/**
 * SO_REUSE==1: Enable SO_REUSEADDR option.
 */
#define SO_REUSE                          1

/**
 * SO_REUSE_RXTOALL==1: Pass a copy of incoming broadcast/multicast packets
 * to all local matches if SO_REUSEADDR is turned on.
 * WARNING: Adds a memcpy for every packet if passing to more than one pcb!
 */
#define SO_REUSE_RXTOALL                  0

/**
 * LWIP_FIONREAD_LINUXMODE==0 (default): ioctl/FIONREAD returns the amount of
 * pending data in the network buffer. This is the way windows does it. It's
 * the default for lwIP since it is smaller.
 * LWIP_FIONREAD_LINUXMODE==1: ioctl/FIONREAD returns the size of the next
 * pending datagram in bytes. This is the way linux does it. This code is only
 * here for compatibility.
 */
#define LWIP_FIONREAD_LINUXMODE           0

/**
 * LWIP_SOCKET_SELECT==1 (default): enable select() for sockets (uses a netconn
 * callback to keep track of events).
 * This saves RAM (counters per socket) and code (netconn event callback), which
 * should improve performance a bit).
 */
#define LWIP_SOCKET_SELECT                1

/**
 * LWIP_SOCKET_POLL==1 (default): enable poll() for sockets (including
 * struct pollfd, nfds_t, and constants)
 */
#define LWIP_SOCKET_POLL                  1
/**
 * @}
 */

/*
   ----------------------------------------
   ---------- Statistics options ----------
   ----------------------------------------
*/
/**
 * @defgroup lwip_opts_stats Statistics
 * @ingroup lwip_opts_debug
 * @{
 */
/**
 * LWIP_STATS==1: Enable statistics collection in lwip_stats.
 */
#define LWIP_STATS                        1

#if LWIP_STATS

/**
 * LWIP_STATS_DISPLAY==1: Compile in the statistics output functions.
 */
#define LWIP_STATS_DISPLAY 0

/**
 * LINK_STATS==1: Enable link stats.
 */
#define LINK_STATS         1

/**
 * ETHARP_STATS==1: Enable etharp stats.
 */
#define ETHARP_STATS       (LWIP_ARP)

/**
 * IP_STATS==1: Enable IP stats.
 */
#define IP_STATS           1

/**
 * IPFRAG_STATS==1: Enable IP fragmentation stats. Default is
 * on if using either frag or reass.
 */
#define IPFRAG_STATS       (IP_REASSEMBLY || IP_FRAG)

/**
 * ICMP_STATS==1: Enable ICMP stats.
 */
#define ICMP_STATS         1

/**
 * IGMP_STATS==1: Enable IGMP stats.
 */
#define IGMP_STATS         (LWIP_IGMP)

/**
 * UDP_STATS==1: Enable UDP stats. Default is on if
 * UDP enabled, otherwise off.
 */
#define UDP_STATS          (LWIP_UDP)

/**
 * TCP_STATS==1: Enable TCP stats. Default is on if TCP
 * enabled, otherwise off.
 */
#define TCP_STATS          (LWIP_TCP)

/**
 * MEM_STATS==1: Enable mem.c stats.
 */
#define MEM_STATS          ((MEM_LIBC_MALLOC == 0) && (MEM_USE_POOLS == 0))

/**
 * MEMP_STATS==1: Enable memp.c pool stats.
 */
#define MEMP_STATS         (MEMP_MEM_MALLOC == 0)

/**
 * SYS_STATS==1: Enable system stats (sem and mbox counts, etc).
 */
#define SYS_STATS          (NO_SYS == 0)

/**
 * IP6_STATS==1: Enable IPv6 stats.
 */
#define IP6_STATS          (LWIP_IPV6)

/**
 * ICMP6_STATS==1: Enable ICMP for IPv6 stats.
 */
#define ICMP6_STATS        (LWIP_IPV6 && LWIP_ICMP6)

/**
 * IP6_FRAG_STATS==1: Enable IPv6 fragmentation stats.
 */
#define IP6_FRAG_STATS     (LWIP_IPV6 && (LWIP_IPV6_FRAG || LWIP_IPV6_REASS))

/**
 * MLD6_STATS==1: Enable MLD for IPv6 stats.
 */
#define MLD6_STATS         (LWIP_IPV6 && LWIP_IPV6_MLD)

/**
 * ND6_STATS==1: Enable Neighbor discovery for IPv6 stats.
 */
#define ND6_STATS          (LWIP_IPV6)

/**
 * MIB2_STATS==1: Stats for SNMP MIB2.
 */
#define MIB2_STATS         0

#else

#define LINK_STATS         0
#define ETHARP_STATS       0
#define IP_STATS           0
#define IPFRAG_STATS       0
#define ICMP_STATS         0
#define IGMP_STATS         0
#define UDP_STATS          0
#define TCP_STATS          0
#define MEM_STATS          0
#define MEMP_STATS         0
#define SYS_STATS          0
#define LWIP_STATS_DISPLAY 0
#define IP6_STATS          0
#define ICMP6_STATS        0
#define IP6_FRAG_STATS     0
#define MLD6_STATS         0
#define ND6_STATS          0
#define MIB2_STATS         0

#endif /* LWIP_STATS */
/**
 * @}
 */

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/
/**
 * @defgroup lwip_opts_checksum Checksum
 * @ingroup lwip_opts_infrastructure
 * @{
 */
/**
 * LWIP_CHECKSUM_CTRL_PER_NETIF==1: Checksum generation/check can be enabled/disabled
 * per netif.
 * ATTENTION: if enabled, the CHECKSUM_GEN_* and CHECKSUM_CHECK_* defines must be enabled!
 */
#define LWIP_CHECKSUM_CTRL_PER_NETIF        1

/*
The STM32F4x7 allows computing and verifying the IP, UDP, TCP and ICMP checksums by hardware:
 - To use this feature let the following define uncommented.
 - To disable it and process by CPU comment the  the checksum.
*/
#define CHECKSUM_BY_HARDWARE                0

/* CHECKSUM_GEN_IP==0: Generate checksums by hardware for outgoing IP packets.*/
#define CHECKSUM_GEN_IP                     !CHECKSUM_BY_HARDWARE
/* CHECKSUM_GEN_UDP==0: Generate checksums by hardware for outgoing UDP packets.*/
#define CHECKSUM_GEN_UDP                    !CHECKSUM_BY_HARDWARE
/* CHECKSUM_GEN_TCP==0: Generate checksums by hardware for outgoing TCP packets.*/
#define CHECKSUM_GEN_TCP                    !CHECKSUM_BY_HARDWARE
/* CHECKSUM_CHECK_IP==0: Check checksums by hardware for incoming IP packets.*/
#define CHECKSUM_CHECK_IP                   !CHECKSUM_BY_HARDWARE
/* CHECKSUM_CHECK_UDP==0: Check checksums by hardware for incoming UDP packets.*/
#define CHECKSUM_CHECK_UDP                  !CHECKSUM_BY_HARDWARE
/* CHECKSUM_CHECK_TCP==0: Check checksums by hardware for incoming TCP packets.*/
#define CHECKSUM_CHECK_TCP                  !CHECKSUM_BY_HARDWARE
/* CHECKSUM_CHECK_ICMP==0: Check checksums by hardware for incoming ICMP packets.*/
#define CHECKSUM_GEN_ICMP                   !CHECKSUM_BY_HARDWARE

/**
 * @}
 */

/*
   ---------------------------------------
   ---------- IPv6 options ---------------
   ---------------------------------------
*/
/**
 * @defgroup lwip_opts_ipv6 IPv6
 * @ingroup lwip_opts
 * @{
 */
/**
 * LWIP_IPV6==1: Enable IPv6
 */
#define LWIP_IPV6                           0

/**
 * IPV6_REASS_MAXAGE: Maximum time (in multiples of IP6_REASS_TMR_INTERVAL - so seconds, normally)
 * a fragmented IP packet waits for all fragments to arrive. If not all fragments arrived
 * in this time, the whole packet is discarded.
 */
#define IPV6_REASS_MAXAGE                   60

/**
 * LWIP_IPV6_SCOPES==1: Enable support for IPv6 address scopes, ensuring that
 * e.g. link-local addresses are really treated as link-local. Disable this
 * setting only for single-interface configurations.
 * All addresses that have a scope according to the default policy (link-local
 * unicast addresses, interface-local and link-local multicast addresses) should
 * now have a zone set on them before being passed to the core API, although
 * lwIP will currently attempt to select a zone on the caller's behalf when
 * necessary. Applications that directly assign IPv6 addresses to interfaces
 * (which is NOT recommended) must now ensure that link-local addresses carry
 * the netif's zone. See the new ip6_zone.h header file for more information and
 * relevant macros. For now it is still possible to turn off scopes support
 * through the new LWIP_IPV6_SCOPES option. When upgrading an implementation that
 * uses the core API directly, it is highly recommended to enable
 * LWIP_IPV6_SCOPES_DEBUG at least for a while, to ensure e.g. proper address
 * initialization.
 */
#define LWIP_IPV6_SCOPES                    (LWIP_IPV6 && !LWIP_SINGLE_NETIF)

/**
 * LWIP_IPV6_SCOPES_DEBUG==1: Perform run-time checks to verify that addresses
 * are properly zoned (see ip6_zone.h on what that means) where it matters.
 * Enabling this setting is highly recommended when upgrading from an existing
 * installation that is not yet scope-aware; otherwise it may be too expensive.
 */
#define LWIP_IPV6_SCOPES_DEBUG              0

/**
 * LWIP_IPV6_NUM_ADDRESSES: Number of IPv6 addresses per netif.
 */
#define LWIP_IPV6_NUM_ADDRESSES             3

/**
 * LWIP_IPV6_FORWARD==1: Forward IPv6 packets across netifs
 */
#define LWIP_IPV6_FORWARD                   0

/**
 * LWIP_IPV6_FRAG==1: Fragment outgoing IPv6 packets that are too big.
 */
#define LWIP_IPV6_FRAG                      1

/**
 * LWIP_IPV6_REASS==1: reassemble incoming IPv6 packets that fragmented
 */
#define LWIP_IPV6_REASS                     LWIP_IPV6

/**
 * LWIP_IPV6_SEND_ROUTER_SOLICIT==1: Send router solicitation messages during
 * network startup.
 */
#define LWIP_IPV6_SEND_ROUTER_SOLICIT       1

/**
 * LWIP_IPV6_AUTOCONFIG==1: Enable stateless address autoconfiguration as per RFC 4862.
 */
#define LWIP_IPV6_AUTOCONFIG                LWIP_IPV6

/**
 * LWIP_IPV6_ADDRESS_LIFETIMES==1: Keep valid and preferred lifetimes for each
 * IPv6 address. Required for LWIP_IPV6_AUTOCONFIG. May still be enabled
 * otherwise, in which case the application may assign address lifetimes with
 * the appropriate macros. Addresses with no lifetime are assumed to be static.
 * If this option is disabled, all addresses are assumed to be static.
 */
#define LWIP_IPV6_ADDRESS_LIFETIMES         LWIP_IPV6_AUTOCONFIG

/**
 * LWIP_IPV6_DUP_DETECT_ATTEMPTS=[0..7]: Number of duplicate address detection attempts.
 */
#define LWIP_IPV6_DUP_DETECT_ATTEMPTS       1
/**
 * @}
 */

/**
 * @defgroup lwip_opts_icmp6 ICMP6
 * @ingroup lwip_opts_ipv6
 * @{
 */
/**
 * LWIP_ICMP6==1: Enable ICMPv6 (mandatory per RFC)
 */
#define LWIP_ICMP6                          LWIP_IPV6

/**
 * LWIP_ICMP6_DATASIZE: bytes from original packet to send back in
 * ICMPv6 error messages.
 */
#define LWIP_ICMP6_DATASIZE                 8

/**
 * LWIP_ICMP6_HL: default hop limit for ICMPv6 messages
 */
#define LWIP_ICMP6_HL                       255
/**
 * @}
 */

/**
 * @defgroup lwip_opts_mld6 Multicast listener discovery
 * @ingroup lwip_opts_ipv6
 * @{
 */
/**
 * LWIP_IPV6_MLD==1: Enable multicast listener discovery protocol.
 * If LWIP_IPV6 is enabled but this setting is disabled, the MAC layer must
 * indiscriminately pass all inbound IPv6 multicast traffic to lwIP.
 */
#define LWIP_IPV6_MLD                       LWIP_IPV6  //0 works zsh

/**
 * MEMP_NUM_MLD6_GROUP: Max number of IPv6 multicast groups that can be joined.
 * There must be enough groups so that each netif can join the solicited-node
 * multicast group for each of its local addresses, plus one for MDNS if
 * applicable, plus any number of groups to be joined on UDP sockets.
 */
#define MEMP_NUM_MLD6_GROUP                 4
/**
 * @}
 */

/**
 * @defgroup lwip_opts_nd6 Neighbor discovery
 * @ingroup lwip_opts_ipv6
 * @{
 */
/**
 * LWIP_ND6_QUEUEING==1: queue outgoing IPv6 packets while MAC address
 * is being resolved.
 */
#define LWIP_ND6_QUEUEING                   LWIP_IPV6

/**
 * MEMP_NUM_ND6_QUEUE: Max number of IPv6 packets to queue during MAC resolution.
 */
#define MEMP_NUM_ND6_QUEUE                  20

/**
 * LWIP_ND6_NUM_NEIGHBORS: Number of entries in IPv6 neighbor cache
 */
#define LWIP_ND6_NUM_NEIGHBORS              10

/**
 * LWIP_ND6_NUM_DESTINATIONS: number of entries in IPv6 destination cache
 */
#define LWIP_ND6_NUM_DESTINATIONS           10

/**
 * LWIP_ND6_NUM_PREFIXES: number of entries in IPv6 on-link prefixes cache
 */
#define LWIP_ND6_NUM_PREFIXES               5

/**
 * LWIP_ND6_NUM_ROUTERS: number of entries in IPv6 default router cache
 */
#define LWIP_ND6_NUM_ROUTERS                3

/**
 * LWIP_ND6_MAX_MULTICAST_SOLICIT: max number of multicast solicit messages to send
 * (neighbor solicit and router solicit)
 */
#define LWIP_ND6_MAX_MULTICAST_SOLICIT      3

/**
 * LWIP_ND6_MAX_UNICAST_SOLICIT: max number of unicast neighbor solicitation messages
 * to send during neighbor reachability detection.
 */
#define LWIP_ND6_MAX_UNICAST_SOLICIT        3

/**
 * Unused: See ND RFC (time in milliseconds).
 */
#define LWIP_ND6_MAX_ANYCAST_DELAY_TIME     1000

/**
 * Unused: See ND RFC
 */
#define LWIP_ND6_MAX_NEIGHBOR_ADVERTISEMENT 3

/**
 * LWIP_ND6_REACHABLE_TIME: default neighbor reachable time (in milliseconds).
 * May be updated by router advertisement messages.
 */
#define LWIP_ND6_REACHABLE_TIME             30000

/**
 * LWIP_ND6_RETRANS_TIMER: default retransmission timer for solicitation messages
 */
#define LWIP_ND6_RETRANS_TIMER              1000

/**
 * LWIP_ND6_DELAY_FIRST_PROBE_TIME: Delay before first unicast neighbor solicitation
 * message is sent, during neighbor reachability detection.
 */
#define LWIP_ND6_DELAY_FIRST_PROBE_TIME     5000

/**
 * LWIP_ND6_ALLOW_RA_UPDATES==1: Allow Router Advertisement messages to update
 * Reachable time and retransmission timers, and netif MTU.
 */
#define LWIP_ND6_ALLOW_RA_UPDATES           1

/**
 * LWIP_ND6_TCP_REACHABILITY_HINTS==1: Allow TCP to provide Neighbor Discovery
 * with reachability hints for connected destinations. This helps avoid sending
 * unicast neighbor solicitation messages.
 */
#define LWIP_ND6_TCP_REACHABILITY_HINTS     1

/**
 * LWIP_ND6_RDNSS_MAX_DNS_SERVERS > 0: Use IPv6 Router Advertisement Recursive
 * DNS Server Option (as per RFC 6106) to copy a defined maximum number of DNS
 * servers to the DNS module.
 */
#define LWIP_ND6_RDNSS_MAX_DNS_SERVERS      0
/**
 * @}
 */

/**
 * @defgroup lwip_opts_dhcpv6 DHCPv6
 * @ingroup lwip_opts_ipv6
 * @{
 */
/**
 * LWIP_IPV6_DHCP6==1: enable DHCPv6 stateful/stateless address autoconfiguration.
 */
#define LWIP_IPV6_DHCP6                     0

/**
 * LWIP_IPV6_DHCP6_STATEFUL==1: enable DHCPv6 stateful address autoconfiguration.
 * (not supported, yet!)
 */
#define LWIP_IPV6_DHCP6_STATEFUL            0

/**
 * LWIP_IPV6_DHCP6_STATELESS==1: enable DHCPv6 stateless address autoconfiguration.
 */
#define LWIP_IPV6_DHCP6_STATELESS           LWIP_IPV6_DHCP6

/**
 * LWIP_DHCP6_GETS_NTP==1: Request NTP servers via DHCPv6. For each
 * response packet, a callback is called, which has to be provided by the port:
 * void dhcp6_set_ntp_servers(u8_t num_ntp_servers, ip_addr_t* ntp_server_addrs);
*/
#define LWIP_DHCP6_GET_NTP_SRV              0

/**
 * The maximum of NTP servers requested
 */
#define LWIP_DHCP6_MAX_NTP_SERVERS          1

/**
 * LWIP_DHCP6_MAX_DNS_SERVERS > 0: Request DNS servers via DHCPv6.
 * DNS servers received in the response are passed to DNS via @ref dns_setserver()
 * (up to the maximum limit defined here).
 */
#define LWIP_DHCP6_MAX_DNS_SERVERS          DNS_MAX_SERVERS
/**
 * @}
 */

/*
   ---------------------------------------
   ---------- Debugging options ----------
   ---------------------------------------
*/
/**
 * @defgroup lwip_opts_debugmsg Debug messages
 * @ingroup lwip_opts_debug
 * @{
 */
/**
 * LWIP_DBG_MIN_LEVEL: After masking, the value of the debug is
 * compared against this value. If it is smaller, then debugging
 * messages are written.
 * @see debugging_levels
 */
#define LWIP_DBG_MIN_LEVEL                  LWIP_DBG_LEVEL_ALL

/**
 * LWIP_DBG_TYPES_ON: A mask that can be used to globally enable/disable
 * debug messages of certain types.
 * @see debugging_levels
 */
#define LWIP_DBG_TYPES_ON                   LWIP_DBG_ON

/**
 * ETHARP_DEBUG: Enable debugging in etharp.c.
 */
#define ETHARP_DEBUG                        LWIP_DBG_OFF

/**
 * NETIF_DEBUG: Enable debugging in netif.c.
 */
#define NETIF_DEBUG                         LWIP_DBG_OFF

/**
 * PBUF_DEBUG: Enable debugging in pbuf.c.
 */
#define PBUF_DEBUG                          LWIP_DBG_OFF

/**
 * API_LIB_DEBUG: Enable debugging in api_lib.c.
 */
#define API_LIB_DEBUG                       LWIP_DBG_OFF

/**
 * API_MSG_DEBUG: Enable debugging in api_msg.c.
 */
#define API_MSG_DEBUG                       LWIP_DBG_OFF

/**
 * SOCKETS_DEBUG: Enable debugging in sockets.c.
 */
#define SOCKETS_DEBUG                       LWIP_DBG_OFF

/**
 * ICMP_DEBUG: Enable debugging in icmp.c.
 */
#define ICMP_DEBUG                          LWIP_DBG_OFF

/**
 * IGMP_DEBUG: Enable debugging in igmp.c.
 */
#define IGMP_DEBUG                          LWIP_DBG_OFF

/**
 * INET_DEBUG: Enable debugging in inet.c.
 */
#define INET_DEBUG                          LWIP_DBG_OFF

/**
 * IP_DEBUG: Enable debugging for IP.
 */
#define IP_DEBUG                            LWIP_DBG_OFF

/**
 * IP_REASS_DEBUG: Enable debugging in ip_frag.c for both frag & reass.
 */
#define IP_REASS_DEBUG                      LWIP_DBG_OFF

/**
 * RAW_DEBUG: Enable debugging in raw.c.
 */
#define RAW_DEBUG                           LWIP_DBG_OFF

/**
 * MEM_DEBUG: Enable debugging in mem.c.
 */
#define MEM_DEBUG                           LWIP_DBG_OFF

/**
 * MEMP_DEBUG: Enable debugging in memp.c.
 */
#define MEMP_DEBUG                          LWIP_DBG_OFF

/**
 * SYS_DEBUG: Enable debugging in sys.c.
 */
#define SYS_DEBUG                           LWIP_DBG_OFF

/**
 * TIMERS_DEBUG: Enable debugging in timers.c.
 */
#define TIMERS_DEBUG                        LWIP_DBG_OFF

/**
 * TCP_DEBUG: Enable debugging for TCP.
 */
#define TCP_DEBUG                           LWIP_DBG_OFF

/**
 * TCP_INPUT_DEBUG: Enable debugging in tcp_in.c for incoming debug.
 */
#define TCP_INPUT_DEBUG                     LWIP_DBG_OFF

/**
 * TCP_FR_DEBUG: Enable debugging in tcp_in.c for fast retransmit.
 */
#define TCP_FR_DEBUG                        LWIP_DBG_OFF

/**
 * TCP_RTO_DEBUG: Enable debugging in TCP for retransmit
 * timeout.
 */
#define TCP_RTO_DEBUG                       LWIP_DBG_OFF

/**
 * TCP_CWND_DEBUG: Enable debugging for TCP congestion window.
 */
#define TCP_CWND_DEBUG                      LWIP_DBG_OFF

/**
 * TCP_WND_DEBUG: Enable debugging in tcp_in.c for window updating.
 */
#define TCP_WND_DEBUG                       LWIP_DBG_OFF

/**
 * TCP_OUTPUT_DEBUG: Enable debugging in tcp_out.c output functions.
 */
#define TCP_OUTPUT_DEBUG                    LWIP_DBG_OFF

/**
 * TCP_RST_DEBUG: Enable debugging for TCP with the RST message.
 */
#define TCP_RST_DEBUG                       LWIP_DBG_OFF

/**
 * TCP_QLEN_DEBUG: Enable debugging for TCP queue lengths.
 */
#define TCP_QLEN_DEBUG                      LWIP_DBG_OFF

/**
 * UDP_DEBUG: Enable debugging in UDP.
 */
#define UDP_DEBUG                           LWIP_DBG_OFF

/**
 * TCPIP_DEBUG: Enable debugging in tcpip.c.
 */
#define TCPIP_DEBUG                         LWIP_DBG_OFF

/**
 * SLIP_DEBUG: Enable debugging in slipif.c.
 */
#define SLIP_DEBUG                          LWIP_DBG_OFF

/**
 * DHCP_DEBUG: Enable debugging in dhcp.c.
 */
#define DHCP_DEBUG                          LWIP_DBG_OFF

/**
 * AUTOIP_DEBUG: Enable debugging in autoip.c.
 */
#define AUTOIP_DEBUG                        LWIP_DBG_OFF

/**
 * DNS_DEBUG: Enable debugging for DNS.
 */
#define DNS_DEBUG                           LWIP_DBG_OFF

/**
 * IP6_DEBUG: Enable debugging for IPv6.
 */
#define IP6_DEBUG                           LWIP_DBG_OFF

/**
 * DHCP6_DEBUG: Enable debugging in dhcp6.c.
 */
#define DHCP6_DEBUG                         LWIP_DBG_OFF
/**
 * @}
 */

/**
 * LWIP_TESTMODE: Changes to make unit test possible
 */
#define LWIP_TESTMODE                       0

/*
   --------------------------------------------------
   ---------- Performance tracking options ----------
   --------------------------------------------------
*/
/**
 * @defgroup lwip_opts_perf Performance
 * @ingroup lwip_opts_debug
 * @{
 */
/**
 * LWIP_PERF: Enable performance testing for lwIP
 * (if enabled, arch/perf.h is included)
 */
#define LWIP_PERF                           0
/**
 * @}
 */

// #define HTTPD_ENABLE_HTTPS     1
// #define LWIP_ALTCP             1
// #define LWIP_ALTCP_TLS         1
// #define LWIP_ALTCP_TLS_MBEDTLS 1
#define LWIP_DEBUG             1
//#define ALTCP_MBEDTLS_DEBUG				LWIP_DBG_ON


#endif /* __LWIPOPTS_H__ */
