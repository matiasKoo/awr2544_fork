#ifndef EDMA_H
#define EDMA_H
#include <stdint.h>

void edma_test(void *args);
void edma_configure(EDMA_Handle handle, void *cb, void *dst, void *src, size_t n);
void edma_write();

#endif /* EDMA_H */
