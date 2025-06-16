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
#include <kernel/dpl/AddrTranslateP.h>


#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_sensor.h>
#define MAIN_TASK_PRI  (configMAX_PRIORITIES-2)
#define EXEC_TASK_PRI  (configMAX_PRIORITIES-1)

#define EXEC_TASK_SIZE (1024U/sizeof(configSTACK_DEPTH_TYPE))
#define MAIN_TASK_SIZE (16384U/sizeof(configSTACK_DEPTH_TYPE)-EXEC_TASK_SIZE)

StackType_t gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gExecTaskStack[EXEC_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gMainTaskObj;
StaticTask_t gExecTaskObj;
TaskHandle_t gMainTask;
TaskHandle_t gExecTask;


MMWave_Handle gMmwHandle;
ADCBuf_Handle gADCHandle;


void Mmw_printErr(const char *s, int32_t err){
    int16_t mmwErr;
    int16_t subErr;
    MMWave_ErrorLevel errLvl;

    MMWave_decodeError(err, &errLvl, &mmwErr, &subErr);
    DebugP_log("ERROR: %s: ", s);
    DebugP_log("Error level %s, mmWaveErr: %hd, subSysErr: %hd\r\n", (errLvl == MMWave_ErrorLevel_ERROR ? "Error" : "Warning"), mmwErr, subErr);
}


int32_t Mmw_callback(uint8_t devIdx, uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload){
    DebugP_log("MMWave callback called\r\n");
    DebugP_log("devidx: %d, msgId: %u, sbId: %u, sbLen: %u\r\n", devIdx, msgId, sbId, sbLen);
    return 0;
}

static void exec_task(void *args){
    int32_t err;
    while(1){
        MMWave_execute(gMmwHandle, &err);
    }
}

static void init_adc(){
    int32_t ret = 0;
    ADCBuf_Params ADCBufParams;
    DebugP_log("Initializing ADC...\r\n");

    ADCBuf_Params_init(&ADCBufParams);
    ADCBufParams.chirpThresholdPing = 1;
    ADCBufParams.chirpThresholdPong = 1;
    ADCBufParams.continousMode = 0;

    gADCHandle = ADCBuf_open(0, &ADCBufParams);

    if (gADCHandle == NULL){
        DebugP_logError("Failed to get ADC Handle\r\n");
    }

    DebugP_log("Got ADC handle\r\n");

    __aligned(4) ADCBuf_dataFormat datafmt = {0};
    __aligned(4) ADCBuf_RxChanConf chanconf = {0};
    datafmt.adcOutFormat = 1;
    datafmt.sampleInterleave = 0;
    datafmt.channelInterleave = 0;
    chanconf.channel = 0;

    ret = ADCBuf_control(gADCHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, &datafmt);
    if(ret != 0){ DebugP_logError("Failed to conf data fmt\r\n");}

    ret = ADCBuf_control(gADCHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, &chanconf);
    if(ret != 0){ DebugP_logError("Failed to conf channels\r\n");}

    DebugP_log("ADC configured!\r\n");

}


static void init_task(void *args){
    int32_t err;
    int32_t ret;

    MMWave_InitCfg initCfg;

    Drivers_open();
    Board_driversOpen();

    init_adc();
    DebugP_log("Init task launched\r\n");
    
    memset(&initCfg, 0, sizeof(initCfg));


    initCfg.domain = MMWave_Domain_MSS;
    initCfg.eventFxn = Mmw_callback;
    initCfg.linkCRCCfg.crcBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1;
    initCfg.linkCRCCfg.crcChannel = CRC_CHANNEL_1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;

    gMmwHandle = MMWave_init(&initCfg, &err);

    if(gMmwHandle == NULL){
        Mmw_printErr("Failed to get handle", err);
        goto end;
    }

    DebugP_log("Synchronizing...\r\n");

    while(MMWave_sync(gMmwHandle, &err) != 0);

    DebugP_log("Synced!\r\n");

    MMWave_OpenCfg openCfg;
    memset(&openCfg, 0, sizeof(MMWave_OpenCfg));

    /* We must create a task that calls MMWave_exec at this point
     * otherwise the device will get stuck in an internal sync loop */
    gExecTask = xTaskCreateStatic(
        exec_task,   /* Pointer to the function that implements the task. */
        "exec task", /* Text name for the task.  This is to facilitate debugging
                      only. */
        EXEC_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t
                         on 32b CPUs */
        NULL,           /* We are not using the task parameter. */
        EXEC_TASK_PRI,  /* task priority, 0 is lowest priority,
                         configMAX_PRIORITIES-1 is highest */
        gExecTaskStack, /* pointer to stack base */
        &gExecTaskObj); /* pointer to statically allocated task object memory */
    configASSERT(gExecTask != NULL);

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
    openCfg.chInterleave = 1; // counterintuivitely this means we don't interleave

    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;

    openCfg.useCustomCalibration = 0;
    openCfg.customCalibrationEnableMask = 0;
    openCfg.calibMonTimeUnit = 1;

    ret = MMWave_open(gMmwHandle, &openCfg, NULL, &err);

    if (ret != 0) {
      Mmw_printErr("Failed to open", err);
    }

    MMWave_CtrlCfg ctrlCfg;
    memset(&ctrlCfg, 0, sizeof(ctrlCfg));
    ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    ctrlCfg.enableProgFilter = 0;
    ctrlCfg.numOfPhaseShiftChirps[0] = 768U;
    ctrlCfg.u.frameCfg[0].profileHandle[0] = gMmwHandle;
    ctrlCfg.u.frameCfg[0].frameCfg.chirpStartIdx = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.chirpEndIdx = 10;
    ctrlCfg.u.frameCfg[0].frameCfg.framePeriodicity = 10000;
    ctrlCfg.u.frameCfg[0].frameCfg.numFrames = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.triggerSelect = 1;
    ctrlCfg.u.frameCfg[0].frameCfg.numLoops = 1;

    ret = MMWave_config(gMmwHandle, &ctrlCfg, &err);
    if (ret != 0){
        Mmw_printErr("Failed to configure", err);
    }

    // MMwave_start
    // ... 
    // MMWave_stop

end:
  // sit here
  while (1)
    __asm__ volatile("wfi");
}

int main(void) {
  /* init SOC specific modules */
  System_init();
  Board_init();

  /* This task is created at highest priority, it should create more tasks and
   * then delete itself */
  gMainTask = xTaskCreateStatic(
      init_task,   /* Pointer to the function that implements the task. */
      "init task", /* Text name for the task.  This is to facilitate debugging
                      only. */
      MAIN_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t
                         on 32b CPUs */
      NULL,           /* We are not using the task parameter. */
      MAIN_TASK_PRI,  /* task priority, 0 is lowest priority,
                         configMAX_PRIORITIES-1 is highest */
      gMainTaskStack, /* pointer to stack base */
      &gMainTaskObj); /* pointer to statically allocated task object memory */
  configASSERT(gMainTask != NULL);

  /* Start the scheduler to start the tasks executing. */
  vTaskStartScheduler();

  /* The following line should never be reached because vTaskStartScheduler()
  will only return if there was not enough FreeRTOS heap memory available to
  create the Idle and (if configured) Timer tasks.  Heap management, and
  techniques for trapping heap exhaustion, are described in the book text. */
  DebugP_assertNoLog(0);

  return 0;
}
