//#include "ramp.h"

///**
//  * @brief  斜坡函数初始化
//  */
//void Ramp_Init(Ramp_TypeDef *ramp, float step) {
//    ramp->target = 0.0f;
//    ramp->current = 0.0f;
//    ramp->step = step;
//}

///**
//  * @brief  斜坡计算
//  */
//float Ramp_Calc(Ramp_TypeDef *ramp, float target) {
//    ramp->target = target;

//    // 如果当前值小于目标值，则按步长增加
//    if (ramp->target > ramp->current) {
//        ramp->current += ramp->step;
//        if (ramp->current > ramp->target) {
//            ramp->current = ramp->target;
//        }
//    } 
//    // 如果当前值大于目标值，则按步长减小
//    else if (ramp->target < ramp->current) {
//        ramp->current -= ramp->step;
//        if (ramp->current < ramp->target) {
//            ramp->current = ramp->target;
//        }
//    }

//    return ramp->current;
//}
