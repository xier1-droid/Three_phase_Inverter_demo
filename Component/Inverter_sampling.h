#ifndef INVERTER_SAMPLING_H
#define INVERTER_SAMPLING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float u_uv;
    float u_vw;
    float u_u;
    float u_v;
    float u_w;
    float iu;
    float iv;
    float iw;
    float vdc;
    uint16_t vdc_raw;
} InverterMeasurements;

bool InverterSampling_Init(void);
void InverterSampling_Update(InverterMeasurements *measurements);
uint16_t InverterSampling_GetVdcRaw(void);
float InverterSampling_GetVdc(void);
float InverterSampling_GetVdcZeroOffset(void);

#endif
