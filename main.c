/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "FreeRTOS.h"
#include "task.h"


#include <stdlib.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/uart.h>
#include <drivers/adcbuf.h>
#include <drivers/hwa.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>


#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_sensor.h>

#define EXEC_TASK_PRI   (configMAX_PRIORITIES-1)     //priority of this should be set to highest availabe
#define MAIN_TASK_PRI   (configMAX_PRIORITIES-3)
#define INIT_TASK_PRI   (configMAX_PRIORITIES-2)
#define EXEC_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define MAIN_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define DPC_TASK_SIZE   (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define INIT_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))



StackType_t gInitTaskStack[INIT_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gExecTaskStack[EXEC_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gDpcTaskStack[DPC_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gInitTaskObj;
StaticTask_t gMainTaskObj;
StaticTask_t gExecTaskObj;
StaticTask_t gDpcTaskObj;

TaskHandle_t gInitTask;
TaskHandle_t gMainTask;
TaskHandle_t gExecTask;
TaskHandle_t gDpcTask;

void btn_isr(void*);
static void init_task(void*);
static void exec_task(void*);
static void main_task(void*);
static inline void fail(void);
void init_butt(void);


/* Tracks the current (intended) state of the RSS */
volatile bool gState = 0;

uint32_t gGpioBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;

MMWave_Handle gMmwHandle;
ADCBuf_Handle gADCHandle;
MMWave_ProfileHandle gMmwProfiles[MMWAVE_MAX_PROFILE];

static inline void fail(void){
    DebugP_log("Failed\r\n");
    DebugP_assertNoLog(0);
    while(1) __asm__ volatile("wfi");
}


int32_t Mmw_callback(uint8_t devIdx, uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload){
    DebugP_log("MMWave callback called\r\n");
    DebugP_log("devidx: %d, msgId: %u, sbId: %u, sbLen: %u\r\n", devIdx, msgId, sbId, sbLen);
    return 0;
}


void Mmw_printErr(const char *s, int32_t err){
    int16_t mmwErr;
    int16_t subErr;
    MMWave_ErrorLevel errLvl;

    MMWave_decodeError(err, &errLvl, &mmwErr, &subErr);
    DebugP_log("ERROR: %s: ", s);
    DebugP_log("Error level %s, mmWaveErr: %hd, subSysErr: %hd\r\n", (errLvl == MMWave_ErrorLevel_ERROR ? "Error" : "Warning"), mmwErr, subErr);
}


void init_butt(){
    int32_t ret = 0;
    gGpioBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(gGpioBaseAddr);
    const uint32_t pinnum = GPIO_PUSH_BUTTON_PIN;
    const uint32_t intrnum = GPIO_PUSH_BUTTON_INTR_LEVEL == GPIO_INTR_LEVEL_HIGH ? 
                        GPIO_PUSH_BUTTON_INTR_HIGH : GPIO_PUSH_BUTTON_INTR_LOW;

    const uint32_t ledaddr = (uint32_t)AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    const uint32_t ledpin = GPIO_LED_PIN;
    GPIO_setDirMode(ledaddr, ledpin, GPIO_LED_DIR);

    GPIO_setDirMode(gGpioBaseAddr, pinnum, GPIO_PUSH_BUTTON_DIR);

    ret = GPIO_ignoreOrHonorPolarity(gGpioBaseAddr, pinnum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    if(ret != 0){ DebugP_log("Failed to ignore or honor polarity\r\n");}

    ret = GPIO_setTrigType(gGpioBaseAddr, pinnum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    if(ret != 0){ DebugP_log("Failed to set trig type\r\n");}

    ret = GPIO_markHighLowLevelInterrupt(gGpioBaseAddr, pinnum, GPIO_PUSH_BUTTON_INTR_LEVEL);
    if(ret != 0){ DebugP_log("Failed to mark\r\n");}

    ret = GPIO_clearInterrupt(gGpioBaseAddr, pinnum);
    if(ret != 0){ DebugP_log("Failed to clear\r\n");}

    ret = GPIO_enableInterrupt(gGpioBaseAddr, pinnum);
    if(ret != 0){ DebugP_log("Failed to enable\r\n");}

    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = intrnum;
    params.args = (void*)pinnum;
    params.callback = &btn_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}
    HwiP_enable();

}


void init_mmw(int32_t *err){
    MMWave_InitCfg initCfg;
    memset(&initCfg, 0, sizeof(initCfg));

    initCfg.domain = MMWave_Domain_MSS;
    initCfg.eventFxn = Mmw_callback;
    initCfg.linkCRCCfg.crcBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1;
    initCfg.linkCRCCfg.crcChannel = CRC_CHANNEL_1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;

    gMmwHandle = MMWave_init(&initCfg, err);

    if(gMmwHandle == NULL){
        Mmw_printErr("Failed to get handle", *err);
        fail();
    }

}


void init_adc(){
    int32_t ret = 0;
    ADCBuf_Params ADCBufParams;
    DebugP_log("Initializing ADC...\r\n");

    ADCBuf_Params_init(&ADCBufParams);
    ADCBufParams.chirpThresholdPing = 1;
    ADCBufParams.chirpThresholdPong = 1;
    ADCBufParams.continousMode = 0;
    ADCBufParams.source = ADCBUF_SOURCE_DFE;

    gADCHandle = ADCBuf_open(0, &ADCBufParams);

    if (gADCHandle == NULL){
        DebugP_logError("Failed to get ADC Handle\r\n");
    }

    DebugP_log("Got ADC handle\r\n");

    // These should apparently be 4 byte aligned
    ADCBuf_dataFormat datafmt __attribute__((aligned(4))) = {0};
    ADCBuf_RxChanConf chanconf __attribute__((aligned(4))) = {0};
    datafmt.adcOutFormat = 1;       // real
    datafmt.sampleInterleave = 0;
    datafmt.channelInterleave = 1;  // apparently non-interleaved might be the only valid option for AWR2544
    chanconf.channel = 0;
    chanconf.offset = 0;

    ret = ADCBuf_control(gADCHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, &datafmt);
    if(ret != 0){ DebugP_logError("Failed to conf data fmt\r\n"); return;}

    ret = ADCBuf_control(gADCHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, &chanconf);
    if(ret != 0){ DebugP_logError("Failed to conf channels\r\n"); return;}

    DebugP_log("ADC configured!\r\n");
}


int32_t open_device(int32_t *err){
    int32_t ret = 0;
    MMWave_OpenCfg openCfg;
    memset(&openCfg, 0, sizeof(MMWave_OpenCfg));

    // these are from the oob demo
    // probably corresponds to 76-81 GHz
    openCfg.freqLimitLow = 760U;
    openCfg.freqLimitHigh = 810U;

    // disable these because nothing to handle them
    openCfg.disableFrameStartAsyncEvent = true;
    openCfg.disableFrameStopAsyncEvent = true;

    // don't run in lower power mode
    openCfg.lowPowerMode.lpAdcMode = 0;

    // ADC output configuration
    // 16 bit output
    openCfg.adcOutCfg.fmt.b2AdcBits |= 0b10;

    // this is either real or complex, docs can't really seem to agree
    openCfg.adcOutCfg.fmt.b2AdcOutFmt |= 0b00;

    // how many bits to reduce adc output by
    // has to be 0 for 16 bits
    openCfg.adcOutCfg.fmt.b8FullScaleReducFctr |= 0x0;

    // select LVDS as the interface
    openCfg.dataPathCfg.intfSel = 1;

    // only get out ADC data and suppress pkt1
    openCfg.dataPathCfg.transferFmtPkt0 = 1;
    openCfg.dataPathCfg.transferFmtPkt1 = 0;

    // size of cq samples just set to 16 bit I guess
    openCfg.dataPathCfg.cqConfig = 0b10;

    // cq0 can't be disabled so set it to 32 halfwords and others to 0
    openCfg.dataPathCfg.cq0TransSize = 32;
    openCfg.dataPathCfg.cq1TransSize = 0;
    openCfg.dataPathCfg.cq2TransSize = 0;

    // SDR clock, this has to be DDR for CSI2
    openCfg.dataPathClkCfg.laneClkCfg = 0;

    // 300Mbps data rate
    openCfg.dataPathClkCfg.dataRate = 0b100;

    // clock speed for that
    openCfg.hsiClkCfg.hsiClk = 0xB;

    // only enable 1 lane
    openCfg.laneEnCfg.laneEn = 0b1;
    // and 1 channel
    openCfg.chCfg.rxChannelEn = 0b1;
    openCfg.chCfg.txChannelEn = 0b1;

    openCfg.chCfg.cascading = 0;
    openCfg.chCfg.cascadingPinoutCfg = 0;

    // hopefully not getting complex output but this gives I first
    openCfg.iqSwapSel = 0;
    openCfg.chInterleave = 1; // counterintuitively this means we don't interleave

    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;

    openCfg.useCustomCalibration = 0;
    openCfg.customCalibrationEnableMask = 0;
    openCfg.calibMonTimeUnit = 1;

    ret = MMWave_open(gMmwHandle, &openCfg, NULL, err);

    return ret;
}

static void create_profiles(int32_t *err){
    rlProfileCfg_t profileCfg;
    memset(&profileCfg, 0, sizeof(profileCfg));
    profileCfg.profileId = 0;
    profileCfg.pfCalLutUpdate |= 0b00;
    profileCfg.startFreqConst = 0x558E4BBC; // approx. 77GHz 
    profileCfg.idleTimeConst = 700;         // 7 usec
    profileCfg.adcStartTimeConst = 700;     // 7 usec
    profileCfg.rampEndTime = 2081;	    // 20,81 usec
    profileCfg.txStartTime = 0;
    profileCfg.numAdcSamples = 128;     // to match TRM example for HWA
    profileCfg.digOutSampleRate = 30000;
    profileCfg.rxGain = 164;

    gMmwProfiles[0] = MMWave_addProfile(gMmwHandle, &profileCfg, err);
}


MMWave_ChirpHandle add_chirp(MMWave_ProfileHandle profile, int32_t *err){
    rlChirpCfg_t chirpCfg;
    memset(&chirpCfg, 0, sizeof(chirpCfg));
    chirpCfg.chirpEndIdx = 5;
    chirpCfg.chirpStartIdx = 0;
    chirpCfg.profileId = 0;
    chirpCfg.startFreqVar = 0;
    chirpCfg.freqSlopeVar = 0;
    chirpCfg.idleTimeVar = 0;
    chirpCfg.adcStartTimeVar = 0;
    chirpCfg.txEnable |= 0b0001;

    return MMWave_addChirp(profile, &chirpCfg, err);
}

int32_t configure_mmw(int32_t *err){
    int32_t ret = 0;
    MMWave_CtrlCfg ctrlCfg;

    memset(&ctrlCfg, 0, sizeof(ctrlCfg));
    ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    ctrlCfg.numOfPhaseShiftChirps[0] = 768U;
    ctrlCfg.u.frameCfg[0].profileHandle[0] = gMmwProfiles[0];
    ctrlCfg.u.frameCfg[0].profileHandle[1] = NULL;
    ctrlCfg.u.frameCfg[0].profileHandle[2] = NULL;
    ctrlCfg.u.frameCfg[0].profileHandle[3] = NULL;

    ctrlCfg.u.frameCfg[0].frameCfg.chirpStartIdx = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.chirpEndIdx = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.framePeriodicity = 20000000;
    ctrlCfg.u.frameCfg[0].frameCfg.numFrames = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.triggerSelect = 1;
    ctrlCfg.u.frameCfg[0].frameCfg.numLoops = 1;

    ret = MMWave_config(gMmwHandle, &ctrlCfg, err);

    return ret;
}


int32_t start_mmw(int32_t *err){
    int32_t ret = 0;
    MMWave_CalibrationCfg calibCfg;
    memset(&calibCfg, 0, sizeof(calibCfg));
    calibCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    calibCfg.u.chirpCalibrationCfg.enableCalibration = 0;
    calibCfg.u.chirpCalibrationCfg.enablePeriodicity = 0;
    calibCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 0;
    calibCfg.u.chirpCalibrationCfg.reportEn = 1;

    ret = MMWave_start(gMmwHandle, &calibCfg, err);

    return ret;
}

void read_adc(){
    int32_t err;
    volatile uint32_t *addr = (volatile uint32_t*)ADCBuf_getChanBufAddr(gADCHandle, 0, &err);
    if(addr == NULL){ 
        DebugP_logError("Failed to get adc address");
        return;
    }
    
    printf("%u\n", *addr);
    printf("%u\n", *(addr+1));
    printf("%u\n", *(addr+2));
}


static void dpc_task(void *args){
    int32_t err = 0;
    
}


static void exec_task(void *args){
    int32_t err;
    while(1){
        MMWave_execute(gMmwHandle, &err);
    }
}


/* init process goes as follows:
 * 
 *  - initialize both the ADCBuf and MMW peripherals
 *  - synchronize mmwavelink
 *  - create the MMWave_execute task
 *  - open the device
 *  - create profile(s) 
 *  - add chirp configuration
 *  - configure the device to use profile(s)
 *  - create the main_task
 *  - and finally terminate itself
 *  
 */
static void init_task(void *args){
    int32_t err = 0;
    int32_t ret = 0;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Init task launched\r\n");

    /* init adc and mmwave */
    init_adc();
    init_mmw(&err);

    DebugP_log("Synchronizing...\r\n");

    /* NOTE: According to the documentation a return value of 1
     * is supposed to mean synchronized, however MMWave_sync()
     * will under no circumstances return a value of 1.
     * So unless this is changed/fixed in a later version
     * treat 0 as a success and != 0 (<0) as a failure */
    while(MMWave_sync(gMmwHandle, &err) != 0);

    DebugP_log("Synced!\r\n");

    /* We must create a task that calls MMWave_exec at this point
     * otherwise the device will get stuck in an internal sync loop */
    gExecTask = xTaskCreateStatic(
        exec_task,      /*  Pointer to the function that implements the task. */
        "exec task",    /*  Text name for the task.  This is to facilitate debugging
                            only. */
        EXEC_TASK_SIZE, /*  Stack depth in units of StackType_t typically uint32_t
                            on 32b CPUs */
        NULL,           /*  We are not using the task parameter. */
        EXEC_TASK_PRI,  /*  task priority, 0 is lowest priority,
                            configMAX_PRIORITIES-1 is highest */
        gExecTaskStack, /*  pointer to stack base */
        &gExecTaskObj); /*  pointer to statically allocated task object memory */
    configASSERT(gExecTask != NULL);

    ret = open_device(&err);
    if(ret != 0){
        Mmw_printErr("Failed to open device", err);
        fail();
    }

    create_profiles(&err);
    MMWave_ChirpHandle chirp = add_chirp(gMmwProfiles[0], &err);
    if(chirp == NULL){
        Mmw_printErr("Failed to add chirp", err);
        fail();
    }


    ret = configure_mmw(&err);
    if (ret != 0){
        Mmw_printErr("Failed to configure", err);
        fail();
    }



    DebugP_log("Configured!\r\n");

    DebugP_log("Creating main task...\r\n");

    gMainTask = xTaskCreateStatic(
        main_task,      /*  Pointer to the function that implements the task. */
        "main task",    /*  Text name for the task.  This is to facilitate debugging
                            only. */
        MAIN_TASK_SIZE, /*  Stack depth in units of StackType_t typically uint32_t
                            on 32b CPUs */
        NULL,           /*  We are not using the task parameter. */
        MAIN_TASK_PRI,  /*  task priority, 0 is lowest priority,
                            configMAX_PRIORITIES-1 is highest */
        gMainTaskStack, /*  pointer to stack base */
        &gMainTaskObj); /*  pointer to statically allocated task object memory */
    configASSERT(gMainTask != NULL);

    DebugP_log("Done. バイバイ\r\n");

    vTaskDelete(NULL);
}


void btn_isr(void *arg){
    uint32_t pending;
    uint32_t pin = (uint32_t)arg;
    const uint32_t ledaddr = (uint32_t)AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);

    pending = GPIO_getHighLowLevelPendingInterrupt(gGpioBaseAddr, pin);
    GPIO_clearInterrupt(GPIO_PUSH_BUTTON_BASE_ADDR, pin);
    if(pending){
        gState = gState ? 0 : 1;
        gState ? GPIO_pinWriteHigh(ledaddr, GPIO_LED_PIN) : GPIO_pinWriteLow(ledaddr,GPIO_LED_PIN);
        
    }

}


void chirp_isr(void *arg){
    int32_t err;
    volatile uint64_t const *adcAddr = (volatile uint64_t*)ADCBuf_getChanBufAddr(gADCHandle, 0, &err);
    printf("%llu\n", *adcAddr);
    return;
}


static void main_task(void *args){
    int32_t err = 0;
    int32_t ret = 0;
    static bool started = 0;

    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = 155; // DFE_CHIRP_CYCLE_START
    params.args = NULL;
    params.callback = &chirp_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}
    
    init_butt();
    DebugP_log("Press down on SW2 to toggle the radar on/off\r\n");

    while(1){
        if(gState == 1 && started == 0){
            ret = start_mmw(&err);
            if(ret != 0){
                Mmw_printErr("Failed to start", err);
                fail();
            }

            DebugP_log("Started\r\n");
            started = 1;
        }else if(gState == 0 && started == 1){
            ret = MMWave_stop(gMmwHandle, &err);
            if(ret != 0){
                Mmw_printErr("Failed to stop", err);
                fail();
            }
            DebugP_log("Stopped\r\n");
            started = 0;
        }

        ClockP_usleep(50000);

    }
}


int main(void) {
    /* init SOC specific modules */
    System_init();
    Board_init();

    /* Create this at 2nd highest priority to initialize everything
     * the MMWave_execute task must have a higher priority than this */
    gInitTask = xTaskCreateStatic(
            init_task,   /* Pointer to the function that implements the task. */
            "init task", /* Text name for the task.  This is to facilitate debugging
                            only. */
            INIT_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t
                               on 32b CPUs */
            NULL,           /* We are not using the task parameter. */
            INIT_TASK_PRI,  /* task priority, 0 is lowest priority,
                               configMAX_PRIORITIES-1 is highest */
            gInitTaskStack, /* pointer to stack base */
            &gInitTaskObj); /* pointer to statically allocated task object memory */
    configASSERT(gInitTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
       will only return if there was not enough FreeRTOS heap memory available to
       create the Idle and (if configured) Timer tasks.  Heap management, and
       techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);

    return 0;
}
