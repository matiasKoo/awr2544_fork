#include <drivers/adcbuf.h>
#include <kernel/dpl/DebugP.h>


int32_t adcbuf_init(ADCBuf_Handle handle){
    int32_t ret = 0;
    int32_t err = 0;
    ADCBuf_Params ADCBufParams;
    DebugP_log("Initializing ADC...\r\n");

    ADCBuf_Params_init(&ADCBufParams);
    ADCBufParams.chirpThresholdPing = 1;
    ADCBufParams.chirpThresholdPong = 1;
    ADCBufParams.continousMode = 0;
    ADCBufParams.source = ADCBUF_SOURCE_DFE;

    handle = ADCBuf_open(0, &ADCBufParams);

    DebugP_assert(handle != NULL);

    DebugP_log("Got ADC handle\r\n");

    // These should apparently be 4 byte aligned
    ADCBuf_dataFormat datafmt __attribute__((aligned(4))) = {0};
    ADCBuf_RxChanConf chanconf __attribute__((aligned(4))) = {0};
    datafmt.adcOutFormat = 1;       // real
    datafmt.sampleInterleave = 0;
    datafmt.channelInterleave = 1;  // apparently non-interleaved might be the only valid option for AWR2544
    chanconf.channel = 0;
    chanconf.offset = 0;

    ret = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, &datafmt);
    if(ret != 0){ DebugP_logError("Failed to conf data fmt\r\n"); return ret; }

    ret = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, &chanconf);
    if(ret != 0){ DebugP_logError("Failed to conf channels\r\n"); return ret; }

    DebugP_log("ADC configured!\r\n");
    return 0;
}