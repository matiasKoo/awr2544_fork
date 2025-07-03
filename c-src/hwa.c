#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"

static HWA_CommonConfig HwaCommonConfig[1] =
{
    {
        .configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG,
		.numLoops = 1,
		.paramStartIdx = 0,
		.paramStopIdx = 0,
    },
};


static HWA_ParamConfig HwaParamConfig[1] =
{
    {
		.triggerMode = HWA_TRIG_MODE_SOFTWARE,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = 255,
            .srcAIdx = 2,
            .srcBcnt = 0,
            .srcBIdx = 0,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_UNSIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 0,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 0x4000,
            .dstAcnt = 255,
            .dstAIdx = 4,
            .dstBIdx = 0,
            .dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_SIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 0,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .fftMode =
            {
                .mode2X = HWA_FEATURE_BIT_DISABLE,
                .fftEn = HWA_FEATURE_BIT_ENABLE,
                .fftSize = 8,   // size is 2^fftSize
                .butterflyScaling = 0,
                .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                .windowEn = HWA_FEATURE_BIT_DISABLE,
                .dcEstProfileSelect = HWA_DCEST_PROFILE_SELECT_PROFILE0,
                .preProcCfg =
                {
                    .dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                    .interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .interfLocalize =
                    {
                        .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .interfMitigation =
                    {
                        .enable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                },
            },
        },
    },
};
/* HWA RAM atrributes */
HWA_RAMAttrs HwaRamCfg[HWA_NUM_RAMS] =
{
    {CSL_DSS_HWA_WINDOW_RAM_U_BASE, CSL_DSS_HWA_WINDOW_RAM_U_SIZE}
};

void hwa_test(){
    int32_t err;
    HWA_Handle handle = NULL;
    handle = HWA_open(0, NULL, &err);
    DebugP_assert(handle != NULL);
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &HwaParamConfig[0], NULL);
    HWA_reset(handle);
    HWA_enable(handle, 1U);
    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
    //DSSHWACCRegs *pctrl = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    //pctrl->FW2HWA_TRIG_0 |= 1; // software trigger 1
}

uint32_t hwa_getaddr(HWA_Handle handle){
    HWA_MemInfo meminfo;
    HWA_getHWAMemInfo(handle, &meminfo);

    return meminfo.baseAddress;
}


void hwa_run(HWA_Handle handle){
    DSSHWACCRegs *pctrl = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    pctrl->FW2HWA_TRIG_0 |= 1; // software trigger 1
    //HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}


void hwa_manual(HWA_Handle handle){
    // powerUpHWA from oob demo mmw_dpc.c
    CSL_dss_rcmRegs *p = (CSL_dss_rcmRegs*)CSL_DSS_RCM_U_BASE;
    p->DSS_HWA_PD_CTRL  = 0x07007U; // PONIN
    p->DSS_HWA_PD_CTRL  = 0x77007U; // PGOODIN
    p->DSS_HWA_PD_CTRL  = 0x77077U; // AONIN
    p->DSS_HWA_PD_CTRL  = 0x77777U; // AGOODIN
    p->DSS_HWA_CLK_GATE = 0x0U;
    p->DSS_HWA_CLK_GATE = 0x7U;
    p->HW_SPARE_RW0     = 0x0U;
    p->DSS_HWA_PD_CTRL  = 0x77770U; // Deassert ISO
    p->DSS_HWA_CLK_GATE = 0x0U;


    // reconfigHWA from mmw_dpc.c
    DSSHWACCRegs *pctrl = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;

    // unlock hwa registers
    pctrl->LOCK0_KICK0 = 0x01234567U;
    pctrl->LOCK0_KICK1 = 0xFEDCBA8U;

    // disable accelerator and clock
    pctrl->HWA_ENABLE = 0x0;

    // enable clock
    CSL_FINSR(pctrl->HWA_ENABLE, HWA_ENABLE_HWA_CLK_EN_END, HWA_ENABLE_HWA_CLK_EN_START, 0x7U);

    // clear PARAM_DONE_SET_STATUS and TRIGGER_SET_STATUS
    pctrl->PARAM_DONE_CLR[0] = 0xFFFFFFFFU;
    pctrl->TRIGGER_SET_IN_CLR[0] = 0xFFFFFFFFU;

    // clear FFTCLIP
    pctrl->CLR_FFTCLIP = 0x1U;

    // clear other clip ones
    pctrl->CLR_CLIP_MISC = 0x1U;


    memset(pparam, 0, sizeof(DSSHWACCPARAMRegs));

    pparam->HEADER =  1; // TRIG_MODE software

    
    // SRCADDR and DSTADDR
    pparam->SRC = 0x0;      // bank 0 at 0
    pparam->DST = 0x8000;   // bank 2 at 32768

    // just set this, leave rest as 0 for
    // 16 bit input
    // unsigned
    // i/q swap = 0 (doesn't matter for real)
    // don't conjugate
    pparam->SRC |= (1U << SRC_SRCREAL_START);   // SRCREAL
    pparam->SRC |= (3U << SRC_SRCSCAL_START);   // SCALE

    // leave rest as 0 for
    // 16 bit output
    // i/q swap = 0
    // complex
    // don't conjugate
    pparam->DST |= (1U << DST_DSTSIGNED_START);
    pparam->DST |= (3U << DST_DSTSCAL_START);    // scale


    // SRCAIDX and DSTAIDX
    pparam->SRCA |= (2U << SRCA_SRCAINDX_START); // 16 bit input means 2 bytes separate each sample
    pparam->DSTA |= (4U << DSTA_DSTAINDX_START); // 16 bit complex output so 4 bytes for each
    // SRCACNT and DSTACNT
    pparam->SRCA |= (255U << SRCA_SRCACNT_START);
    pparam->DSTA |= (255U << DSTA_DSTACNT_START);

    // SRCBIDX and DSTBIDX
    pparam->SRCB = 0;
    pparam->DSTB = 0;

    // SRCBCNT 
    pparam->SRCB |= (1U << 20);

    // FFTEN 1
    pparam->accelModeParam.FFTPATH.BFLYFFT = 0x1;

    // Not using FFTSIZE_3X_EN so FFT size will be 2^FFTSIZE 
    pparam->accelModeParam.FFTPATH.BFLYFFT |= (8U << BFLYFFT_FFTSIZE_START); // 256pt fft

    // WINEN 0
    pparam->accelModeParam.FFTPATH.POSTPROCWIN = 0x0;
 

    pctrl->HWA_ENABLE |= (0b111U << HWA_ENABLE_HWA_EN_START); // enable the accelerator

}

// Print out n samples from addr
// addr is relative to HWA base memory address
void hwa_print_samples(HWA_Handle handle, uint16_t addr, size_t n, bool sign){
    const uint32_t hwa_base = (uint32_t)SOC_virtToPhy((void*)hwa_getaddr(handle));

    for(size_t i = 0; i < n; ++i){
        if(sign){
            printf("%hd,", *((int16_t*)(hwa_base+addr)+i));
        }else{
            printf("%hu,", *((uint16_t*)(hwa_base+addr)+i));
        }
        putchar('\n');
    }
}