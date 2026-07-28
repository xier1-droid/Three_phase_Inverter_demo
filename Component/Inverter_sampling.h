#ifndef INVERTER_SAMPLING_H
#define INVERTER_SAMPLING_H

#include <stdbool.h>

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
} InverterMeasurements;

bool InverterSampling_Init(void);
void InverterSampling_Update(InverterMeasurements *measurements);

#endif
