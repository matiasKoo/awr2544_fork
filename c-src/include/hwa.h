#ifndef HWA_H
#define HWA_H
#include <stdint.h>

uint32_t hwa_getaddr(HWA_Handle handle);
void hwa_run(HWA_Handle handle);
void hwa_manual(HWA_Handle handle);

#endif /* HWA_H */