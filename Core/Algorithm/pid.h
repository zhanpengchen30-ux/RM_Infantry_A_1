#ifndef __PID_H
#define __PID_H
#include "stdint.h"

typedef struct {
    float Kp, Ki, Kd;
    float max_out;      // 最大输出限制 (M3508最大电流为16384)
    float max_iout;     // 积分限幅 (防止积分饱和Windup)
    
    float set;          // 目标值
    float fdb;          // 反馈值
    
    float out;          // 最终输出
    float Pout, Iout, Dout;
    float error[3];     // 误差项: 0:当前误差 1:上次误差 2:上上次误差
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout);
float PID_Calc(PID_TypeDef *pid, float ref, float set);
float circle_error(float target, float current, float period) ;

#endif
