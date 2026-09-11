#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h" 

// 初始化所有 CAN 过滤器
void CAN_Filter_Init(void);

// CAN2 发送：底盘电流
void CAN_Send_Chassis_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

// CAN1 发送：云台电压 (Yaw, Pitch)
void CAN_Send_Gimbal_Voltage(int16_t yaw_v, int16_t pitch_v);

// CAN1 发送：拨盘电流 (M2006)
void CAN_Send_Feeder_Current(int16_t iq);

#endif
