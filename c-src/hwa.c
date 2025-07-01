
#include <drivers/hwa.h>
    
uint32_t hwa_getaddr(HWA_Handle handle){
    HWA_MemInfo meminfo;
    HWA_getHWAMemInfo(handle, &meminfo);

    return meminfo.baseAddress;
}