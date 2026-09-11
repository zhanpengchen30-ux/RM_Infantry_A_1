//#ifndef __RAMP_H
//#define __RAMP_H

//#include "stdint.h"

///**
//  * @brief 斜坡函数结构体
//  */
//typedef struct {
//    float target;      // 目标值
//    float current;     // 当前斜坡输出值
//    float step;        // 步长（决定了变化的快慢/加速度）
//} Ramp_TypeDef;

///**
//  * @brief  斜坡函数初始化
//  * @param  ramp: 指向结构体的指针
//  * @param  step: 步长 (建议 2ms 频率下设为 5.0f~15.0f)
//  */
//void Ramp_Init(Ramp_TypeDef *ramp, float step);

///**
//  * @brief  斜坡计算函数
//  * @param  ramp: 指向结构体的指针
//  * @param  target: 当前瞬时目标值
//  * @return 计算后的平滑值
//  */
//float Ramp_Calc(Ramp_TypeDef *ramp, float target);

//#endif
