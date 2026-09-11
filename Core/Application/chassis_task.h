#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"

/**
  * @brief          底盘任务入口
  * @param[in]      argument: FreeRTOS 传参，目前为 NULL
  * @retval         none
  */
extern void StartChassisTask(void *argument);

/* 
   
   例如底盘模式枚举：
   typedef enum {
       CHASSIS_STOP,      // 停止
       CHASSIS_MANUAL,    // 遥控器手动控制
       CHASSIS_AUTO       // 视觉/自动驾驶控制
   } Chassis_Mode_e;
*/

#endif
