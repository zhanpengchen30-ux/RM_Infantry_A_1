#ifndef __REMOTE_H
#define __REMOTE_H

#include "stdint.h"

typedef enum {
    MODE_RELAX = 0,       // 全车失能
    MODE_GIMBAL_ONLY,     // 只动云台
    MODE_SPIN,            // 小陀螺模式
    MODE_FOLLOW,          // 双中-跟随模式
    MODE_CHASSIS_ONLY     // 只动底盘 
} Robot_Mode_e;

// 遥控器数据结构
typedef struct {
    struct {
        int16_t ch[5]; // 摇杆通道 (ch0:右横, ch1:右纵, ch2:左横, ch3:左纵, ch4:左上角拨轮)

        uint8_t s0 : 2;  // 右拨杆状态（因为只有 1, 2, 3 三种状态，2个bit就够了）
        uint8_t s1 : 2;  // 左拨杆状态
        uint8_t    : 4;  
    } rc;
} RC_ctrl_t;

// 全局变量声明
extern RC_ctrl_t rc_ctrl;
extern Robot_Mode_e robot_mode; 

// 函数声明
void Remote_Mode_Update(void); 
void RC_Decode(volatile const uint8_t *sbus_buf);
void BoardA_Sync_And_Send(void); 

#endif /* __REMOTE_H */

