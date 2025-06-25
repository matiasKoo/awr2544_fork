#ifndef MMW_H
#define MMW_H

#include <stdint.h>
#include <ti/control/mmwave/mmwave.h>

void mmw_printerr(const char *s, int32_t err);
int32_t mmw_open(MMWave_Handle, int32_t *err);
int32_t mmw_config(MMWave_Handle handle, MMWave_ProfileHandle profiles, int32_t *err);
int32_t mmw_start(MMWave_Handle handle, int32_t *err);
MMWave_Handle mmw_init(int32_t *err);
MMWave_ProfileHandle mmw_create_profile(MMWave_Handle handle, int32_t *err);
MMWave_ChirpHandle mmw_add_chirp(MMWave_ProfileHandle *profiles, int32_t *err);


#endif /* MMW_H */
