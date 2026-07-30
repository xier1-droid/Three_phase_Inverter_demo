#include "Inverter_sampling.h"

#include "adc.h"
#include "dma.h"
#include "Filter.h"
#include "tim.h"

#include <stddef.h>
#include <stdint.h>

#define ADC_FULL_SCALE_COUNTS          4096.0f
#define ADC_REFERENCE_V                3.3f
#define ADC_OFFSET_FILTER_ALPHA        0.9999f
#define ADC_OFFSET_CALIBRATION_SAMPLES 256U
#define ADC_OFFSET_TRACKING_DELAY_SAMPLES 10000U
#define VDC_FILTER_ALPHA               0.995f
#define VDC_SENSING_GAIN               0.0103982f
#define VDC_ZERO_OFFSET_COUNTS         2099.7f

static volatile uint32_t adc1_buffer[2];
static volatile uint32_t adc2_buffer[3];

static LowPassFilter_t uvw_offset_filter;
static LowPassFilter_t iw_offset_filter;
static LowPassFilter_t uuv_offset_filter;
static LowPassFilter_t iu_offset_filter;
static LowPassFilter_t vdc_filter;
static uint32_t offset_tracking_delay_count;

static float InverterSampling_ConvertVdc(float raw)
{
    float vdc = ((raw - VDC_ZERO_OFFSET_COUNTS)
                 * ADC_REFERENCE_V / ADC_FULL_SCALE_COUNTS)
                / VDC_SENSING_GAIN;

    return (vdc > 0.0f) ? vdc : 0.0f;
}

static void InverterSampling_Calibrate(void)
{
    uint32_t uvw_sum = 0U;
    uint32_t iw_sum = 0U;
    uint32_t uuv_sum = 0U;
    uint32_t iu_sum = 0U;
    uint32_t vdc_sum = 0U;
    uint32_t primask;
    uint16_t index;

    for (index = 0U; index < ADC_OFFSET_CALIBRATION_SAMPLES; index++)
    {
        iw_sum += adc1_buffer[0];
        uvw_sum += adc1_buffer[1];
        iu_sum += adc2_buffer[0];
        uuv_sum += adc2_buffer[1];
        vdc_sum += adc2_buffer[2];
        HAL_Delay(1U);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    LowPass_Init(&uvw_offset_filter,
                 ADC_OFFSET_FILTER_ALPHA,
                 (float)uvw_sum / ADC_OFFSET_CALIBRATION_SAMPLES);
    LowPass_Init(&iw_offset_filter,
                 ADC_OFFSET_FILTER_ALPHA,
                 (float)iw_sum / ADC_OFFSET_CALIBRATION_SAMPLES);
    LowPass_Init(&uuv_offset_filter,
                 ADC_OFFSET_FILTER_ALPHA,
                 (float)uuv_sum / ADC_OFFSET_CALIBRATION_SAMPLES);
    LowPass_Init(&iu_offset_filter,
                 ADC_OFFSET_FILTER_ALPHA,
                 (float)iu_sum / ADC_OFFSET_CALIBRATION_SAMPLES);
    LowPass_Init(&vdc_filter,
                 VDC_FILTER_ALPHA,
                 InverterSampling_ConvertVdc(
                     (float)vdc_sum / ADC_OFFSET_CALIBRATION_SAMPLES));
    offset_tracking_delay_count = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

bool InverterSampling_Init(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 2U) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buffer, 3U) != HAL_OK)
    {
        return false;
    }

    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT | DMA_IT_TC);

    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
    {
        return false;
    }
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    if (HAL_TIM_Base_Start_IT(&htim8) != HAL_OK)
    {
        return false;
    }

    HAL_Delay(2U);
    InverterSampling_Calibrate();
    return true;
}

uint16_t InverterSampling_GetVdcRaw(void)
{
    return (uint16_t)adc2_buffer[2];
}

float InverterSampling_GetVdc(void)
{
    return vdc_filter.filtered;
}

float InverterSampling_GetVdcZeroOffset(void)
{
    return VDC_ZERO_OFFSET_COUNTS;
}

void InverterSampling_Update(InverterMeasurements *measurements)
{
    uint32_t iw_raw;
    uint32_t uvw_raw;
    uint32_t iu_raw;
    uint32_t uuv_raw;
    uint32_t vdc_raw;
    float iw_adc_v;
    float uvw_adc_v;
    float iu_adc_v;
    float uuv_adc_v;
    float iw_offset;
    float uvw_offset;
    float iu_offset;
    float uuv_offset;
    bool track_offsets;

    if (measurements == NULL)
    {
        return;
    }

    iw_raw = adc1_buffer[0];
    uvw_raw = adc1_buffer[1];
    iu_raw = adc2_buffer[0];
    uuv_raw = adc2_buffer[1];
    vdc_raw = adc2_buffer[2];

    track_offsets = false;
    if ((TIM8->BDTR & TIM_BDTR_MOE) != 0U)
    {
        offset_tracking_delay_count = 0U;
    }
    else if (offset_tracking_delay_count < ADC_OFFSET_TRACKING_DELAY_SAMPLES)
    {
        offset_tracking_delay_count++;
    }
    else
    {
        track_offsets = true;
    }

    if (track_offsets)
    {
        iw_offset = LowPass_Update(&iw_offset_filter, (float)iw_raw);
        uvw_offset = LowPass_Update(&uvw_offset_filter, (float)uvw_raw);
        iu_offset = LowPass_Update(&iu_offset_filter, (float)iu_raw);
        uuv_offset = LowPass_Update(&uuv_offset_filter, (float)uuv_raw);
    }
    else
    {
        iw_offset = iw_offset_filter.filtered;
        uvw_offset = uvw_offset_filter.filtered;
        iu_offset = iu_offset_filter.filtered;
        uuv_offset = uuv_offset_filter.filtered;
    }

    iw_adc_v = ((float)iw_raw - iw_offset)
               * ADC_REFERENCE_V / ADC_FULL_SCALE_COUNTS;
    uvw_adc_v = ((float)uvw_raw - uvw_offset)
                * ADC_REFERENCE_V / ADC_FULL_SCALE_COUNTS;
    iu_adc_v = ((float)iu_raw - iu_offset)
               * ADC_REFERENCE_V / ADC_FULL_SCALE_COUNTS;
    uuv_adc_v = ((float)uuv_raw - uuv_offset)
                * ADC_REFERENCE_V / ADC_FULL_SCALE_COUNTS;

    measurements->iw = iw_adc_v * 4.990f;
    measurements->u_vw =((uvw_adc_v * ((39.0f / 2.0f) * 1000.0f)) / (3.922f * 150.0f)) * 0.981517f;
    measurements->iu = iu_adc_v * 4.8823f;
    measurements->u_uv =((uuv_adc_v * ((39.0f / 2.0f) * 1000.0f)) / (4.0f * 150.0f)) * 0.9829186f;
    measurements->vdc = LowPass_Update(
        &vdc_filter, InverterSampling_ConvertVdc((float)vdc_raw));
    measurements->vdc_raw = (uint16_t)vdc_raw;

    measurements->iv = -(measurements->iu + measurements->iw);
    measurements->u_u =(2.0f * measurements->u_uv + measurements->u_vw) / 3.0f;
    measurements->u_v =(measurements->u_vw - measurements->u_uv) / 3.0f;
    measurements->u_w =-(measurements->u_uv + 2.0f * measurements->u_vw) / 3.0f;
}
