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

#define SIZE 128

static uint8_t testsrc[SIZE];
static uint8_t testdst[SIZE];

struct test{
    uint32_t base;
    uint32_t region;
    uint32_t ch;
};

void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args){
    SemaphoreP_Object *semobj = (SemaphoreP_Object*)args;
    DebugP_assert(semobj != NULL);
    SemaphoreP_post(semobj);
}

static inline __attribute__((always_inline)) void edma_write(uint32_t base, uint32_t region, uint32_t ch){
    DebugP_log("Transferring\r\n");
    CacheP_wb(testsrc, SIZE, CacheP_TYPE_ALL);
    CacheP_wb(testdst, SIZE, CacheP_TYPE_ALL);

    EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_MANUAL);

    CacheP_inv(testdst, SIZE, CacheP_TYPE_ALL);
    CacheP_inv(testsrc, SIZE, CacheP_TYPE_ALL);
    DebugP_log("Done\r\n");
}


struct test edma_configure(void *cb, void *dst, void *src, size_t n){
    uint32_t base = 0;
    uint32_t region = 0;
    uint32_t ch = 0;
    uint32_t tcc = 0;
    uint32_t param = 0;
    int32_t ret = 0;
    uint8_t *srcp = (uint8_t*)src;
    uint8_t *dstp = (uint8_t*)dst;
        struct test t;

    
    EDMACCPaRAMEntry edmaparam;
    Edma_IntrObject intrObj;



    
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
    edmaparam.bCnt          = (uint16_t) 1U;    // With just one array
    edmaparam.cCnt          = (uint16_t) 1U;    // in one block
    edmaparam.bCntReload    = 0U;
    edmaparam.srcBIdx       = 0U;
    edmaparam.destBIdx      = 0U;
    edmaparam.srcCIdx       = 0U;
    edmaparam.destBIdx      = 0U;
    edmaparam.linkAddr      = 0xFFFF;
    edmaparam.srcBIdxExt    = 0U;
    edmaparam.destBIdxExt   = 0U;
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param, &edmaparam);

    intrObj.tccNum = tcc;
    intrObj.cbFxn = cb;
    intrObj.appData = (void*)0;
    ret = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(ret == 0);
    DebugP_log("Edma initialized\r\n");

    t.base = base;
    t.region = region;
    t.ch = ch;
    DebugP_log("Inside configure\r\nt.base:%u\r\nt.region:%u\r\nt.ch:1\r\n",t.base,t.region,t.ch);
        edma_write(t.base, t.region, t.ch);

    return t;

}



void edma_cb(){
}


void edma_test(void *args){
    for(int i = 0; i < SIZE; ++i){
        testsrc[i] = 0xFE;
        testdst[i] = 0x00;
    }
    struct test t;
    t = edma_configure(&edma_cb, &testdst, &testsrc, 128);
    DebugP_log("t.base:%u\r\nt.region:%u\r\nt.ch:%u\r\n", t.base, t.region, t.ch);

    while(1)__asm__("wfi");
}