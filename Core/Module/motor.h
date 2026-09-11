#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h"

// 大疆 3508 / 2006 电机反馈结构体
typedef struct {
    uint16_t ecd;          // 0~8191
    int16_t  speed_rpm;
    int16_t  given_current;
    uint8_t  temperate;
} motor_measure_t;

// 大疆 6020 电机反馈结构体
typedef struct {
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  given_current;
    uint8_t  temperate;
    uint16_t last_ecd;     // 用于计算多圈角度
} motor_6020_t;

// 声明全局变量，供其他文件使用
extern motor_measure_t motor_chassis[4];
extern motor_measure_t motor_feeder;
extern motor_6020_t    motor_yaw, motor_pitch;

// 解析函数声明
void decode_motor_measure(motor_measure_t *motor, uint8_t *data);
void decode_6020_measure(motor_6020_t *motor, uint8_t *data);

#endif
