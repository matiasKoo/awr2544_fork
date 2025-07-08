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
// TODO: might need to rename these as to not conflict with SDK includes
// but for now enet is the only one that clashes
#include <mmw.h>
#include <adcbuf.h>
#include <edma.h>
#include <cfg.h>
#include <gpio.h>
#include <hwa.h>
#include <enet_project.h>

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

//#define ENET_TEST



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

extern void uart_dump_samples(void *buff, size_t n/*, bool real, bool sign, bool bits*/);


/* == Global Variables == */
/* Handles */
MMWave_Handle gMmwHandle = NULL;
ADCBuf_Handle gADCBufHandle = NULL;
MMWave_ProfileHandle gMmwProfiles[MMWAVE_MAX_PROFILE];

SemaphoreP_Object gAdcSampledSem;
SemaphoreP_Object gBtnPressedSem;
SemaphoreP_Object gEdmaDoneSem;

/* Rest of them */
volatile bool gState = 0; /* Tracks the current (intended) state of the RSS */
static uint32_t gPushButtonBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;


static inline void fail(void){
    DebugP_log("Failed\r\n");
    DebugP_assertNoLog(0);
    while(1) __asm__ volatile("wfi");
}


void edma_callback(Edma_IntrHandle handle, void *args){
    SemaphoreP_post(&gEdmaDoneSem);
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

    // TODO: grab this from sysconfig somehow but for now assume bank 2 will be output
    void *hwaout = (void*)(hwa_getaddr(gHwaHandle[0])+0x4000);

    // Enable interrupts
    HwiP_enable();

    DebugP_log("Ready to roll\r\n");

    while(1){
        ClockP_usleep(5000);
        if(gState == 0){led_state(0); continue;}
        
        led_state(gState);

        // to give the python script time 
        ClockP_usleep(1000*1000);

        // got one, do a measurement
        mmw_start(gMmwHandle, &err);

        // sampling done, stop measuring
        SemaphoreP_pend(&gAdcSampledSem, SystemP_WAIT_FOREVER);
        MMWave_stop(gMmwHandle, &err);


        // move the samples to HWA input
        edma_write();
        SemaphoreP_pend(&gEdmaDoneSem, SystemP_WAIT_FOREVER);

        hwa_run(gHwaHandle[0]);

        // TODO: same thing here, the HWA should have be able to generate an interrupt when done
        // but again just sleep a bit to make sure it's done its thing
        ClockP_usleep(50 * 1000);

        // write out hwa output to UART
        uart_dump_samples(hwaout, 256);
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

    // assume for now that input memory will be at HWA base
    uint32_t hwaaddr = (uint32_t)SOC_virtToPhy((void*)hwa_getaddr(gHwaHandle[0]));

    // init adc
    DebugP_log("Init adc...\r\n");
    gADCBufHandle = adcbuf_init();
    DebugP_assert(gADCBufHandle != NULL);

    // and mmw
    DebugP_log("Init mmw...\r\n");
    gMmwHandle = mmw_init(&err);
    DebugP_assert(gMmwHandle != NULL);

    // and EDMA
    DebugP_log("Init edma...\r\n");
    uint32_t adcaddr = (uint32_t)ADCBuf_getChanBufAddr(gADCBufHandle, 0, &err);
    edma_configure(&edma_callback, (void*)hwaaddr, (void*)adcaddr, SAMPLE_BUFF_SIZE);

    // Not that any should happen but disable interrupts here 
    HwiP_disable();

    // ADC sampling done interrupt
    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE;
    params.args = NULL;
    params.callback = &chirp_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}

    // Push button interrupt
    gPushButtonBaseAddr = gpio_init(&btn_isr);

    // ADC sampling done semaphore
    ret = SemaphoreP_constructBinary(&gAdcSampledSem, 0);
    DebugP_assert(ret == 0);

    // Button pressed semaphore
    ret = SemaphoreP_constructBinary(&gBtnPressedSem, 0);
    DebugP_assert(ret == 0);

    // Edma done semaphore
    ret = SemaphoreP_constructBinary(&gEdmaDoneSem, 0);
    DebugP_assert(ret == 0);
    

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
      gState = !gState;
    }    
}


void chirp_isr(void *arg){
    SemaphoreP_post(&gAdcSampledSem);
}


int main(void) {
    /* init SOC specific modules */
    System_init();
    Board_init();


#ifdef ENET_TEST
    Drivers_open();
    Board_driversOpen(); 
    
   gInitTask = xTaskCreateStatic(
            enet_test,   
            "enet task", 
            INIT_TASK_SIZE,
            NULL,           
            INIT_TASK_PRI,  
            gInitTaskStack, 
            &gInitTaskObj); 
    configASSERT(gInitTask != NULL);
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
#endif
    vTaskStartScheduler();

    DebugP_assertNoLog(0);
}
