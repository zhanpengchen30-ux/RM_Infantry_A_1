#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "stdint.h"

/**
  * @brief 云台控制模式枚举 (为未来多模式控制做准备)
  */
typedef enum {
    GIMBAL_RELAX,      // 云台无力模式（安全模式）
    GIMBAL_MANUAL,     // 遥控器手动控制模式
    GIMBAL_AUTO,       // 视觉自动跟随模式
    GIMBAL_GYRO        // 陀螺仪闭环控制模式
} Gimbal_Mode_e;

/**
  * @brief          云台任务入口函数
  * @param[in]      argument: FreeRTOS 传参
  * @retval         none
  */
extern void StartGimbalTask(void *argument);

/* 
   未来可以在这里定义云台的限制常量，防止撞坏机械结构 
   例如：
   #define PITCH_MAX_ANGLE  2000.0f
   #define PITCH_MIN_ANGLE -1000.0f
*/

#endif
