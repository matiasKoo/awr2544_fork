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

#define A_COUNT (16U)
#define B_COUNT (8U)
#define C_COUNT (1U)

#define BUFF_SIZE (A_COUNT * C_COUNT * B_COUNT)



static SemaphoreP_Object gEdmaTestDoneSem;




static volatile uint8_t srcBuff[BUFF_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
static volatile uint8_t dstBuff[BUFF_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args){
    SemaphoreP_Object *semobj = (SemaphoreP_Object*)args;
    DebugP_assert(semobj != NULL);
    SemaphoreP_post(semobj);
}

void test_transfer(){
    uint32_t base, region;
    uint8_t *src, *dst;
    int32_t ret = 0;
    EDMACCPaRAMEntry edmaparam;
    Edma_IntrObject     intrObj;

    uint32_t ch, tcc, param;

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
    DebugP_assert(ret == 0);

    src = (uint8_t*)srcBuff;
    dst = (uint8_t*)dstBuff;

    EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch, tcc , param, 0);
    EDMA_ccPaRAMEntry_init(&edmaparam);
    edmaparam.srcAddr       = (uint32_t) SOC_virtToPhy(src);
    edmaparam.destAddr      = (uint32_t)SOC_virtToPhy(dst);
    edmaparam.aCnt          = (uint16_t) A_COUNT;
    edmaparam.bCnt          = (uint16_t) B_COUNT;
    edmaparam.cCnt          = (uint16_t) C_COUNT;
    edmaparam.bCntReload    = (uint16_t) B_COUNT;
    edmaparam.srcBIdx       = (int16_t)EDMA_PARAM_BIDX(A_COUNT);
    edmaparam.destBIdx      = (int16_t)EDMA_PARAM_BIDX(A_COUNT);
    edmaparam.srcCIdx       = (int16_t) A_COUNT;
    edmaparam.destBIdx      = (int16_t) A_COUNT;
    edmaparam.linkAddr      = 0xFFFF;
    edmaparam.srcBIdxExt    = (int8_t)EDMA_PARAM_BIDX_EXT(A_COUNT);
    edmaparam.destBIdxExt   = (int8_t)EDMA_PARAM_BIDX_EXT(A_COUNT);
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param, &edmaparam);

    ret = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(ret == 0);
    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    ret = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(ret == SystemP_SUCCESS);



    for(int i = 0; i < (C_COUNT * B_COUNT); ++i){
        EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_MANUAL);
        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    CacheP_inv((void *)dstBuff, BUFF_SIZE, CacheP_TYPE_ALL);

    
    for(int i = 0; i < BUFF_SIZE; ++i){
        if(srcBuff[i] != dstBuff[i]){
            printf("Mismatch at: %p src=%hd dst=%hd\r\n",&dstBuff[i],srcBuff[i], dstBuff[i]);
        }
    }
}


void edma_hwa_main(void *args){
    Drivers_open();
    Board_driversOpen();
    srand(10);

    for(int i = 0; i < BUFF_SIZE; ++i){
        srcBuff[i] = rand() % UINT8_MAX;
        dstBuff[i] = 0;
    }
    CacheP_wb((void *)srcBuff, BUFF_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuff, BUFF_SIZE, CacheP_TYPE_ALL);

    test_transfer();


    while(1)__asm__("wfi");
}