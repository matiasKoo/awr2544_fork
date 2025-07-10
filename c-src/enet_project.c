
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/ClockP.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "app_cpswconfighandler.h"
#include "ti_enet_lwipif.h"

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct EnetApp_AppEnetInfo
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports List to use for the above EnetType & InstId*/
    uint8_t     numMacPort;

    /* Num MAC ports to use for the above EnetType & InstId*/
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];
} EnetApp_AppEnetInfo;

static struct dhcp g_netifDhcp[ENET_SYSCFG_NETIF_COUNT];
static struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];
static EnetApp_AppEnetInfo gEnetAppParams[ENET_SYSCFG_MAX_ENET_INSTANCES];

/* Handle to the Application interface for the LwIPIf Layer
 */


void init_cb(){

}

static inline int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}


void enet_test(void *args){
    int32_t status = ENET_SOK;
    // Most of this is from the TI example TCP server
    /* Read MAC Port details and enable clock for each ENET instance */
    for (uint32_t enetInstIdx = 0; enetInstIdx < ENET_SYSCFG_MAX_ENET_INSTANCES; enetInstIdx++)
    {
        EnetApp_AppEnetInfo* pEnetInstInfo = &gEnetAppParams[enetInstIdx];
        EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0 + enetInstIdx, &pEnetInstInfo->enetType, &pEnetInstInfo->instId);
        EnetApp_getEnetInstMacInfo(pEnetInstInfo->enetType,
                                   pEnetInstInfo->instId,
                                   &pEnetInstInfo->macPortList[0],
                                   &pEnetInstInfo->numMacPort);
        EnetAppUtils_enableClocks(pEnetInstInfo->enetType, pEnetInstInfo->instId);
    }


    EnetApp_driverInit();
    for(uint32_t enetInstIdx = 0; enetInstIdx < ENET_SYSCFG_MAX_ENET_INSTANCES; enetInstIdx++)
    {
        status = EnetApp_driverOpen(gEnetAppParams[enetInstIdx].enetType, gEnetAppParams[enetInstIdx].instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET[%d]: %d\r\n", enetInstIdx, status);
            EnetAppUtils_assert(status == ENET_SOK);
        }
        
        EnetApp_addMCastEntry(gEnetAppParams[enetInstIdx].enetType,
                              gEnetAppParams[enetInstIdx].instId,
                              EnetSoc_getCoreId(),
                              BROADCAST_MAC_ADDRESS,
                              CPSW_ALE_ALL_PORTS_MASK);
    }

    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(init_cb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);


    uint32_t netupMask = 0;
    /* wait for atleast one Network Interface to get IP */
    while (netupMask == 0)
    {
        for(uint32_t netifIdx = 0; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
        {
            if (App_isNetworkUp(g_pNetif[netifIdx]))
            {
                netupMask |= (1 << netifIdx);
            }
            else
            {
                DebugP_log("[%d]Waiting for network UP ...\r\n",g_pNetif[netifIdx]->num);
            }
            ClockP_sleep(2);
        }
    }

    DebugP_log("Network is UP ...\r\n");
    ClockP_sleep(2);


    while(1) __asm__ volatile("wfi");
}
