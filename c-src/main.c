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

/* TI Header files */
#include <stdlib.h>


#include "FreeRTOS.h"
#include "task.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"

#include <drivers/uart.h>
#include <drivers/adcbuf.h>
#include <drivers/hwa.h>

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>

#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_sensor.h>

/* Project header files */
#include <mmw.h>
#include <adcbuf.h>
#include <edma.h>
#include <cfg.h>
#include <gpio.h>
#include <hwa.h>

/* Task related macros */
#define EXEC_TASK_PRI   (configMAX_PRIORITIES-1)     // must be higher than INIT_TASK_PRI
#define MAIN_TASK_PRI   (configMAX_PRIORITIES-3)
#define INIT_TASK_PRI   (configMAX_PRIORITIES-2)
#define EXEC_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define MAIN_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define DPC_TASK_SIZE   (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define INIT_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))

/* Project related macros */
#define NUM_CHIRPS  1
#define SAMPLE_SIZE (sizeof(uint16_t))
#define SAMPLE_BUFF_SIZE (NUM_CHIRPS * CFG_PROFILE_NUMADCSAMPLES * SAMPLE_SIZE)

#define SKIP_MMW



/* Task related global variables */
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


/* == Function Declarations == */
/* ISRs */
void btn_isr(void*);
void chirp_isr(void*);

/* Tasks */
static void init_task(void*);
static void exec_task(void*);
static void main_task(void*);

/* Other functions */
static inline void fail(void);


/* == Global Variables == */
/* Handles */
MMWave_Handle gMmwHandle = NULL;
ADCBuf_Handle gADCBufHandle = NULL;
MMWave_ProfileHandle gMmwProfiles[MMWAVE_MAX_PROFILE];

SemaphoreP_Object gAdcSampledSem;

/* Rest of them */
volatile bool gState = 0; /* Tracks the current (intended) state of the RSS */
static uint32_t gPushButtonBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;

// Known values from a previous measurement to see if the HWA is doing things right
static const uint16_t gTestBuff[] = {816,617,442,237,55,65414,65328,65238,65228,
65287,65235,65226,65174,65102,65064,64975,64924,64896,
64879,64919,65003,65063,65187,65300,65422,6,63,
147,184,201,166,171,243,268,268,322,
357,376,505,596,715,800,717,607,451,
177,65470,65207,64957,64781,64691,64583,64506,64483,
64417,64485,64579,64716,64849,64976,65210,65398,52,
254,358,511,640,739,816,835,870,914,
924,908,893,827,732,516,373,150,65453,
65259,65043,64903,64776,64684,64594,64617,64730,64779,
64891,65018,65075,65168,65300,65417,65511,38,115,
150,184,296,378,460,586,737,794,797,
814,810,774,652,550,416,277,153,47,
65479,65406,65345,65313,65243,65232,65237,65207,65212,
65213,65194,65083,64987,64848,64820,64799,64805,64884,
65002,65215,65458,161,404,616,781,855,898,
932,886,814,711,627,522,449,387,299,
178,118,65534,65400,65250,65095,64967,64882,64737,
64630,64662,64708,64785,64909,65044,65154,65279,65394,
65441,9,68,100,124,213,292,390,554,
673,836,910,986,987,922,815,657,477,
215,7,65361,65160,65002,64921,64874,64886,64955,
65049,65125,65153,65185,65141,65090,64972,64917,64829,
64855,64883,65047,65265,65493,236,456,624,719,796,801,
792,739,686,622,589,601,640,706,711,766,692,559,352,34,
65268,64943,64627,64462,64306,64167,64187,64391,64535,64765,
65038,65205,65331,65404,65445,65470,65436,65505,
58,159,312,459,592,756
};


static inline void fail(void){
    DebugP_log("Failed\r\n");
    DebugP_assertNoLog(0);
    while(1) __asm__ volatile("wfi");
}


void edma_callback(Edma_IntrHandle handle, void *args){
    DebugP_log("Edma callback called\r\n");
}


static void exec_task(void *args){
    int32_t err;
    while(1){
        MMWave_execute(gMmwHandle, &err);
    }
}


static void main_task(void *args){
    int32_t err = 0;
    int32_t ret = 0;
    static bool started = 0;

#ifndef SKIP_MMW
    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE;
    params.args = NULL;
    params.callback = &chirp_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}

    ret = SemaphoreP_constructBinary(&gAdcSampledSem, 0);
    
    gPushButtonBaseAddr = gpio_init(&btn_isr);
  //  DebugP_log("Press SW2 to toggle the radar on/off\r\n");
    
    mmw_start(gMmwHandle, &err);
    SemaphoreP_pend(&gAdcSampledSem, SystemP_WAIT_FOREVER);
    MMWave_stop(gMmwHandle, &err);

    DebugP_log("done\r\n");
#endif /* SKIP_MMW */
#ifdef SKIP_MMW
    edma_write();
#endif 
    uint32_t paramregs = (uint32_t)SOC_virtToPhy(HWA_getParamSetAddr(gHwaHandle[0], 0));
    DebugP_log("Params at address %#x\r\n",paramregs);
    ClockP_sleep(1);
    DebugP_log("Launching HWA\r\n");
    hwa_test();
   // hwa_run(gHwaHandle);
/*    ClockP_sleep(1);
    DebugP_log("HWA data:\r\n");
    hwa_print_samples(gHwaHandle[0], 0x8000, 256, true);*/

    while(1) __asm__("wfi");

    while(1){
        if(gState == 1 && started == 0){
            ret = mmw_start(gMmwHandle, &err);
            if(ret != 0){
                mmw_printerr("Failed to start", err);
                fail();
            }

            DebugP_log("Started\r\n");
            started = 1;
        }else if(gState == 0 && started == 1){
            ret = MMWave_stop(gMmwHandle, &err);
            if(ret != 0){
                mmw_printerr("Failed to stop", err);
                fail();
            }
            DebugP_log("Stopped\r\n");
            started = 0;
        }

        ClockP_usleep(50000);

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

    uint32_t hwaaddr = (uint32_t)SOC_virtToPhy((void*)hwa_getaddr(gHwaHandle[0]));
 #ifdef SKIP_MMW
    edma_configure((void*)hwaaddr, (void*)&gTestBuff, SAMPLE_BUFF_SIZE);
 //   hwa_manual(gHwaHandle[0]);
    DebugP_log("Skipping MMW...\r\n");

    goto end;
 #else
    /* init adc and mmwave */
    gADCBufHandle = adcbuf_init();
    DebugP_assert(gADCBufHandle != NULL);
    gMmwHandle = mmw_init(&err);
    DebugP_assert(gMmwHandle != NULL);
    DebugP_log("Configuring HWA\r\n");
    uint32_t adcaddr = (uint32_t)ADCBuf_getChanBufAddr(gADCBufHandle, 0, &err);
    hwa_manual(gHwaHandle[0]);
    DebugP_log("Done\r\n");
    edma_configure((void*)hwaaddr, (void*)adcaddr, SAMPLE_BUFF_SIZE);
 

    DebugP_log("HWA address is %#x\r\n",hwaaddr);

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

    ret = mmw_open(gMmwHandle, &err);
    if(ret != 0){
        mmw_printerr("Failed to open device", err);
        fail();
    }

    gMmwProfiles[0] = mmw_create_profile(gMmwHandle, &err);
    MMWave_ChirpHandle chirp = mmw_add_chirp(gMmwProfiles[0], &err);
    if(chirp == NULL){
        mmw_printerr("Failed to add chirp", err);
        fail();
    }

    ret = mmw_config(gMmwHandle, gMmwProfiles, &err);
    if (ret != 0){
        mmw_printerr("Failed to configure", err);
        fail();
    }

    DebugP_log("Configured!\r\n");
#endif /* SKIP_MMW */
end:
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

    pending = GPIO_getHighLowLevelPendingInterrupt(gPushButtonBaseAddr, pin);
    GPIO_clearInterrupt(GPIO_PUSH_BUTTON_BASE_ADDR, pin);
    if(pending){
        gState = led_state(!gState);
    }    
}


void chirp_isr(void *arg){
    SemaphoreP_post(&gAdcSampledSem);
    edma_write();

    return;
}


int main(void) {
    /* init SOC specific modules */
    System_init();
    Board_init();

/* Define to not run mmw related stuff */
#ifdef EDMA_TEST
    Drivers_open();
    Board_driversOpen(); 
    //NOTE: remove this or edma dev test will take over
    gMainTask = xTaskCreateStatic(
        edma_test,   /* Pointer to the function that implements the task. */
        "edma task", /* Text name for the task.  This is to facilitate debugging
                            only. */
        MAIN_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t
                               on 32b CPUs */
        NULL,           /* We are not using the task parameter. */
        configMAX_PRIORITIES-3,  /* task priority, 0 is lowest priority,
                               configMAX_PRIORITIES-1 is highest */
        gMainTaskStack, /* pointer to stack base */
        &gMainTaskObj); /* pointer to statically allocated task object memory */
    configASSERT(gMainTask != NULL);
    vTaskStartScheduler();
    DebugP_assertNoLog(0);
#else
    /* Create this at 2nd highest priority to initialize everything
     * the MMWave_execute task must have a higher priority than this */
   gInitTask = xTaskCreateStatic(
            init_task,   
            "init task", 
            INIT_TASK_SIZE,
            NULL,           
            INIT_TASK_PRI,  
            gInitTaskStack, 
            &gInitTaskObj); 
    configASSERT(gInitTask != NULL);

    vTaskStartScheduler();

    DebugP_assertNoLog(0);
#endif
}
