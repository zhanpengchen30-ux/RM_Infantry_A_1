#ifndef __BOARD_COM_H
#define __BOARD_COM_H

#include <stdint.h> 
#include "can.h"  
#include "main.h"  
#include "remote.h"

#pragma pack(1) // 强制 1 字节对齐

// A板 -> C板：打包遥控器通道、开关以及模式（共 8 字节）
typedef struct {
    int16_t ch_yaw;      // Yaw 控制通道 (对应 rc_ctrl.rc.ch[2])
    int16_t ch_pitch;    // Pitch 控制通道 (对应 rc_ctrl.rc.ch[3])
    uint8_t s0;          // 右拨杆状态 (对应 rc_ctrl.rc.s[0])
    uint8_t s1;          // 左拨杆状态 (对应 rc_ctrl.rc.s[1])
    uint16_t robot_mode; //模式 
} BoardA_to_BoardC_t;

// C板 -> A板：同步云台电机反馈数据至底盘（共 8 字节）

typedef struct {
    int16_t yaw_relative_ecd; 
    int16_t vx; // 前后速度
    int16_t vy; // 左右速度
    int16_t wz; // 旋转速度 
} BoardC_to_BoardA_t;

#pragma pack()

// 全局变量声明
extern BoardA_to_BoardC_t board_a_tx_data;
extern BoardC_to_BoardA_t board_c_tx_data;

// 函数声明
void Board_Com_Init(void);
HAL_StatusTypeDef CAN_Send_A_to_C(void);
HAL_StatusTypeDef CAN_Send_C_to_A(void);

// A板设置发送数据
void BoardA_Set_TxData(int16_t yaw, int16_t pitch, uint8_t s0, uint8_t s1, uint16_t mode);
// A板执行发送
HAL_StatusTypeDef CAN_Send_A_to_C(void);
// A板获取反馈 
BoardC_to_BoardA_t BoardA_Get_RxData(void);

#endif /* __BOARD_COM_H */
