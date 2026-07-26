#ifndef __FILTER_H
#define __FILTER_H

#include "mydefine.h"

// ================== 滑动平均滤波器结构体 ==================
typedef struct {
    float *buffer;        // 指向缓冲区数组的指针
    uint16_t len;         // 滤波窗口长度（比如32）
    uint16_t idx;         // 当前写入位置
    float sum;            // 当前所有数据的总和（增量式计算用，提高效率）
} MovingAverageFilter_t;

// ================== 函数声明 ==================
/**
 * @brief  初始化滑动平均滤波器
 * @param  filter   滤波器结构体指针
 * @param  buf      你提前定义好的缓冲区数组
 * @param  length   缓冲区长度
 */
void MovingAverage_Init(MovingAverageFilter_t *filter, float *buf, uint16_t length);

/**
 * @brief  执行一次滑动平均滤波
 * @param  filter   滤波器结构体指针
 * @param  new_val  新输入的原始值（比如Voltage_Rms）
 * @return 滤波后的平滑值
 */
float MovingAverage_Update(MovingAverageFilter_t *filter, float new_val);
//*****************************************************************************************
// 一阶低通滤波器结构体（简单版）
typedef struct {
    float alpha;          // 滤波系数
    float filtered;       // 当前滤波值
} LowPassFilter_t;

// 函数声明
void LowPass_Init(LowPassFilter_t *filter, float alpha, float initial_value);
float LowPass_Update(LowPassFilter_t *filter, float new_val);

#endif
