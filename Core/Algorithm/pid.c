#include "pid.h"

// PID 参数初始化
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Iout = 0.0f;
}

// 位置式 PID 计算
float PID_Calc(PID_TypeDef *pid, float fdb, float set) {
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    pid->error[0] = set - fdb;

    // 比例项
    pid->Pout = pid->Kp * pid->error[0];
    // 积分项 (带限幅)
    pid->Iout += pid->Ki * pid->error[0];
    if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;
    // 微分项
    pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);

    // 总输出计算与限幅
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}

/**
 * @brief  循环误差计算函数（专门解决 0~8191 编码器过零问题）
 * @param  target:  目标编码器值 (0~8191)
 * @param  current: 当前电机反馈编码器值 (0~8191)
 * @param  period:  计数周期 (对于大疆电机编码器，通常是 8192)
 * @return 得到的最短距离误差值 (-4096 ~ 4096)
 */
float circle_error(float target, float current, float period) {
    float error = target - current;
    
    // 如果误差大于半圈，说明反向走更近
    if (error > period / 2.0f) {
        error -= period;
    }
    // 如果误差小于负半圈，说明正向走更近
    else if (error < -period / 2.0f) {
        error += period;
    }
    
    return error;
}
