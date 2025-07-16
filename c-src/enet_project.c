
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


void EnetApp_addMCastEntry(Enet_Type enetType,
                          uint32_t instId,
                          uint32_t coreId,
                          const uint8_t *testMCastAddr,
                          uint32_t portMask)
{
    Enet_IoctlPrms prms;
    int32_t status;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;

    if (Enet_isCpswFamily(enetType))
    {
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);

        EnetAppUtils_assert(hEnet != NULL);
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
               sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                               status);
        }
    }
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

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}


static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}


void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    int32_t status;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = enetType;
    ethPort.instId   = instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = EnetBoard_getId();

    /* Get the Mii config for Ethernet port */
    EnetBoard_getMiiConfig(&ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    if (Enet_isCpswFamily(ethPort.enetType))
    {
        CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)linkArgs->macCfg;
        CpswMacPort_initCfg(macCfg);
        if (EnetMacPort_isSgmii(mii) || EnetMacPort_isQsgmii(mii))
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_SGMII_WITH_PHY;
        }
        else
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_INVALID;
        }
    }

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL)
    {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr     = boardPhyCfg->phyAddr;
        phyCfg->isStrapped  = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn  = false;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);

    }
    else
    {
        DebugP_log("No PHY configuration found for MAC port %u\r\n",
                           ENET_MACPORT_ID(ethPort.macPort));
        EnetAppUtils_assert(false);
    }

    mii->layerType     = ethPort.mii.layerType;
    mii->sublayerType  = ethPort.mii.sublayerType;
    mii->variantType   = ENET_MAC_VARIANT_FORCED;
    linkCfg->speed     = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    if (Enet_isCpswFamily(ENET_CPSW_3G))
    {
        CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)linkArgs->macCfg;

        if (EnetMacPort_isSgmii(mii) || EnetMacPort_isQsgmii(mii))
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_SGMII_WITH_PHY;
        }
        else
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_INVALID;
        }
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetCpdma_Cfg * dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);
    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
#endif


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb    = &EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif
    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
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