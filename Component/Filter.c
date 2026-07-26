#include "filter.h"

// ��ʼ������
void MovingAverage_Init(MovingAverageFilter_t *filter, float *buf, uint16_t length)
{
    filter->buffer = buf;
    filter->len = length;
    filter->idx = 0;
    filter->sum = 0.0f;
    
    // �ѻ�����ȫ������
    for(uint16_t i = 0; i < length; i++) {
        filter->buffer[i] = 0.0f;
    }
}

// �����˲������ĺ�����ʹ������ʽ���㣬����Ч��
float MovingAverage_Update(MovingAverageFilter_t *filter, float new_val)
{
    // �ȼ������ϵ�ֵ�������µ�ֵ
    filter->sum = filter->sum - filter->buffer[filter->idx] + new_val;
    
    // ����ֵ���뻺����
    filter->buffer[filter->idx] = new_val;
    
    // ָ��ѭ���ƶ�
    filter->idx = (filter->idx + 1) % filter->len;
    
    // ����ƽ��ֵ
    return (filter->sum / filter->len);
}
//-------------------------------------------------------------------------------
void LowPass_Init(LowPassFilter_t *filter, float alpha, float initial_value)
{
    filter->alpha = alpha;
    filter->filtered = initial_value;
}

float LowPass_Update(LowPassFilter_t *filter, float new_val)
{
    filter->filtered = filter->filtered * filter->alpha 
                     + new_val * (1.0f - filter->alpha);
    return filter->filtered;
}
//-------------------------------------------------------------------------------

