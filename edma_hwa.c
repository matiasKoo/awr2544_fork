#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/uart.h>
#include <stdio.h>
#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define BUFF_SIZE 128
#define A_COUNT (BUFF_SIZE * sizeof(uint16_t))






static uint16_t srcBuff[BUFF_SIZE] = {0};
static uint16_t dstBuff[BUFF_SIZE] = {0};


void test_transfer(){
    uint32_t base, region;
    uint8_t *src, *dst;
    int32_t ret = 0;
    EDMACCPaRAMEntry edmaparam;
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
    edmaparam.srcAddr = (uint32_t)src;
    edmaparam.destAddr = (uint32_t)dst;
    edmaparam.aCnt = A_COUNT;
    edmaparam.bCnt = 1;
    edmaparam.cCnt = 1;
    edmaparam.bCntReload = 0;
    edmaparam.srcBIdx = (int16_t)EDMA_PARAM_BIDX(A_COUNT);
    edmaparam.destBIdx = (int16_t)EDMA_PARAM_BIDX(A_COUNT);
    edmaparam.srcCIdx = A_COUNT;
    edmaparam.destBIdx = A_COUNT;
    edmaparam.linkAddr = 0xFFFFU;
    edmaparam.srcBIdxExt = (int8_t)EDMA_PARAM_BIDX_EXT(A_COUNT);
    edmaparam.destBIdxExt = (int8_t)EDMA_PARAM_BIDX_EXT(A_COUNT);
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));

   ret= EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_MANUAL);

}

int edma_hwa_main(void *args){
    Drivers_open();
    Board_driversOpen();
    srand(10);

    for(int i = 0; i < BUFF_SIZE; ++i){
        srcBuff[i] = rand() % UINT16_MAX;
    }
    test_transfer();

    for(int i = 0; i < BUFF_SIZE; ++i){
        printf("%#x ",dstBuff[i]);
    }
    while(1)__asm__("wfi");
}