#include <stdio.h>
#include <stdlib.h>

#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/uart.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>

#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <edma.h>


static Edma_IntrObject gIntrObj;
static uint32_t gBaseAddr;
static uint32_t gRegion;
static uint32_t gCh;
static void *gSrcBuff;
static void *gDstBuff;
static size_t gSize;


static void edma_cb(Edma_IntrHandle intrHandle, void *args){
    CacheP_inv(gSrcBuff, gSize, CacheP_TYPE_ALL);
    CacheP_inv(gDstBuff, gSize, CacheP_TYPE_ALL);

}


void edma_write(){
    DebugP_log("Transferring\r\n");
    CacheP_wb(gSrcBuff , gSize, CacheP_TYPE_ALL);
    CacheP_wb(gDstBuff, gSize, CacheP_TYPE_ALL);

    volatile uint32_t *addr = (uint32_t*)(EDMA_getBaseAddr(gEdmaHandle[0])+0x1010);
    *addr = 0b1;
    DebugP_log("Done\r\n");
}


void edma_configure(void *dst, void *src, size_t n){
    uint32_t base = 0;
    uint32_t region = 0;
    uint32_t ch = 0;
    uint32_t tcc = 0;
    uint32_t param = 0;
    int32_t ret = 0;
    uint8_t *srcp = (uint8_t*)src;
    uint8_t *dstp = (uint8_t*)dst;
    gSrcBuff = src;
    gDstBuff = dst;
    gSize = n;

    EDMACCPaRAMEntry edmaparam;

    base = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(base != 0);

    region = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(region < SOC_EDMA_NUM_REGIONS);

    ch = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocDmaChannel(gEdmaHandle[0], &ch);
    DebugP_assert(ret == 0);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(ret == 0);

    param = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocParam(gEdmaHandle[0], &param);

    EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch, tcc , param, 0);
    EDMA_ccPaRAMEntry_init(&edmaparam);
    edmaparam.srcAddr       = (uint32_t) SOC_virtToPhy(srcp);
    edmaparam.destAddr      = (uint32_t) SOC_virtToPhy(dstp);
    edmaparam.aCnt          = (uint16_t) n;     // (assuming this works,) do everything in one dimension
    edmaparam.bCnt          = (uint16_t) 1U;    
    edmaparam.cCnt          = (uint16_t) 1U;    // in one block
    edmaparam.bCntReload    = 0U;
    edmaparam.srcBIdx       = 0U;
    edmaparam.destBIdx      = 0U;
    edmaparam.srcCIdx       = 0U;
    edmaparam.destBIdx      = 0U;
    edmaparam.linkAddr      = 0x4000;           // PaRAM set 0 so we reuse the configuration
    edmaparam.srcBIdxExt    = 0U;
    edmaparam.destBIdxExt   = 0U;
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param, &edmaparam);

    gBaseAddr = base;
    gCh = ch;
    gRegion = 0;
    gIntrObj.tccNum = tcc;
    gIntrObj.cbFxn = &edma_cb;
    gIntrObj.appData = (void*)0;
    ret = EDMA_registerIntr(gEdmaHandle[0], &gIntrObj);
        DebugP_assert(ret == 0);

    EDMA_enableTransferRegion(gBaseAddr, gRegion, gCh, EDMA_TRIG_MODE_EVENT);
    DebugP_log("Edma initialized\r\n");
}

