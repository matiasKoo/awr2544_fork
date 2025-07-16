
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <networking/enet/core/include/per/cpsw.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include <networking/enet/core/include/enet.h>
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_lwipif.h"
#include "app_cpswconfighandler.h"


#define HOST_SERVER_IP6  ("FE80::12:34FF:FE56:78AB")

#define HOST_SERVER_PORT  (8888)

#define APP_MAX_RX_DATA_LEN (1024U)

#define APP_NUM_ITERATIONS (2U)

#define APP_SEND_DATA_NUM_ITERATIONS (5U)

#define MAX_IPV4_STRING_LEN (20U)

struct App_hostInfo_t
{
    ip_addr_t ipAddr;
    uint16_t port;
};

char snd_buf[APP_MAX_RX_DATA_LEN];
LwipifEnetApp_Handle hlwipIfApp = NULL;
struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void AppTcp_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo);

static struct App_hostInfo_t gHostInfo;
static char   gHostServerIp4[MAX_IPV4_STRING_LEN] = "";


static void AppTcp_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo)
{
    EnetAppUtils_print(" IP eneterd is: %s \r\n", gHostServerIp4);
    int32_t addr_ok;
    pHostInfo->port = HOST_SERVER_PORT;
    memset(&pHostInfo->ipAddr, 0, sizeof(pHostInfo->ipAddr));
    ip_addr_t*  pAddr = &pHostInfo->ipAddr;
    IP_SET_TYPE_VAL(*pAddr, IPADDR_TYPE_V4);
    addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(pAddr));
    EnetAppUtils_assert(addr_ok);

    return;
}


static inline int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}


static void App_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("Enet IF UP Event. Local interface IP:%s\r\n",
                    ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("Enet IF DOWN Event\r\n");
    }
    return;
}

static void App_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("Network Link UP Event\r\n");
    }
    else
    {
        DebugP_log("Network Link DOWN Event\r\n");
    }
    return;
}


static void App_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;
 

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    // 192.168.1.10
    ipaddr.addr = (192U) | (168U << 8) | (1U << 16) | (10U << 24);

    DebugP_log("Starting lwIP, local interface IP is not dhcp-enabled\r\n");
    hlwipIfApp = LwipifEnetApp_getHandle();
    sys_lock_tcpip_core();
    for (uint32_t i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        /* Open the netif and get it populated*/
        g_pNetif[i] = LwipifEnetApp_netifOpen(hlwipIfApp, NETIF_INST_ID0 + i, &ipaddr, &netmask, &gw);
        netif_set_status_callback(g_pNetif[i], App_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[i], App_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + i]);
    }
    sys_unlock_tcpip_core();
    LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);

}


static void App_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    App_setupNetif();



    sys_sem_signal(pSem);
}

static void App_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(App_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

void AppTcp_showMenu(void)
{
    ip_addr_t ipAddr;
    int32_t addr_ok = 0;
    EnetAppUtils_print(" TCP Client Menu: \r\n");

    do
    {
        EnetAppUtils_print(" Enter TCP server IPv4 address:(example: 192.168.101.100)\r\n");
        DebugP_scanf("%s", gHostServerIp4);
        addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(&ipAddr));
        TaskP_yield();
    } while (addr_ok != 1);
}

static void AppTcp_simpleclient(void *pArg)
{
    struct netconn *pConn = NULL;
    err_t err = ERR_OK, connectError = ERR_OK;
    struct App_hostInfo_t* pHostInfo = (struct App_hostInfo_t*) pArg;
    uint32_t buf_len = 0;
    const enum netconn_type connType = NETCONN_TCP;

    /* Create a new connection identifier. */
    for (uint32_t pktIdx = 0; pktIdx < APP_NUM_ITERATIONS; pktIdx++)
    {
        struct netbuf *rxBbuf = NULL;
        pConn = netconn_new(connType);
        if (pConn != NULL)
        {
            /* Connect to the TCP Server */
            EnetAppUtils_print("<<<< ITERATION %d >>>>\r\n", (pktIdx + 1));
            EnetAppUtils_print(" Connecting to: %s:%d \r\n", gHostServerIp4, HOST_SERVER_PORT);
            connectError = netconn_connect(pConn, &pHostInfo->ipAddr, pHostInfo->port);
            if (connectError != ERR_OK)
            {
                netconn_close(pConn);
                DebugP_log("Connection with the server isn't established error %s\r\n",lwip_strerr(connectError));
                continue;
            }

            DebugP_log("Connection with the server is established\r\n");
            // send the data to the server
            for ( uint32_t i = 0; i < APP_SEND_DATA_NUM_ITERATIONS; i++)
            {
                memset(&snd_buf, 0, sizeof(snd_buf));
                buf_len = snprintf(snd_buf, sizeof(snd_buf), "Hello over TCP %d", i+1);
                err = netconn_write(pConn, snd_buf, buf_len, NETCONN_COPY);
                if (err == ERR_OK)
                {
                    printf("\"%s\" was sent to the Server\r\n", snd_buf);
                }
                else
                {
                    DebugP_log("couldn't send packet to server\r\n");
                    continue;
                }

                /* wait until the data is sent by the server */
                if (netconn_recv(pConn, &rxBbuf) == ERR_OK)
                {
                    DebugP_log("Successfully received the packet %d\r\n", i+1);
                    netbuf_delete(rxBbuf);
                }
                else
                {
                    DebugP_log("No response from server\r\n");
                }
            }
            netconn_close(pConn);
            netconn_delete(pConn);
            DebugP_log("Connection closed\r\n");
            ClockP_sleep(1);
        }
    }
}


void enet_test(void *args){
    int32_t ret = 0;

    EnetAppUtils_enableClocks(ENET_CPSW_2G, 0);

    DebugP_log("Initializing enet\r\n");
    EnetApp_driverInit();

    DebugP_log("Opening enet\r\n");
    ret = EnetApp_driverOpen(ENET_CPSW_2G, 0);
    DebugP_assert(ret == 0);

    DebugP_log("Getting handle...\r\n");
    Enet_Handle handle = Enet_getHandle(ENET_CPSW_2G, 0);
    if(handle == NULL){
        DebugP_logError("Got NULL for handle\r\n");
    }else{
        DebugP_log("Handle is %p\r\n", handle);
    }


    EnetApp_addMCastEntry(ENET_CPSW_2G,
                        0,
                        EnetSoc_getCoreId(),
                        BROADCAST_MAC_ADDRESS,
                        CPSW_ALE_ALL_PORTS_MASK);

    DebugP_log("Setting up network stack\r\n");
    App_setupNetworkStack();


    while (false == App_isNetworkUp(netif_default))
    {
        DebugP_log("Waiting for network UP ...\r\n");
        ClockP_sleep(2);
    }

    DebugP_log("network is UP!\r\n");
    AppTcp_showMenu();
    AppTcp_fillHostSocketInfo(&gHostInfo);
    sys_thread_t ret2 = sys_thread_new("tcpinit_thread", AppTcp_simpleclient, &gHostInfo, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    DebugP_log("ret from thread %d\r\n",ret2);

    while(1){ ClockP_sleep(2);}
    //vTaskDelete(NULL);
}