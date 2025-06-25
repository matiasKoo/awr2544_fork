#ifndef ADCBUF_H
#define ADCBUF_H

/* Initialize the ADC Buffer 
 * sets some default settings that work with the MMWave DFE
 * handle will be set to the ADCBuf handle 
 * returns 0 on success, <0 otherwise */
int32_t adcbuf_init(ADCBuf_Handle handle);

#endif /* ADCBUF_H */
