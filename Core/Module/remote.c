#include "remote.h"
#include "main.h"
#include "board_com.h"
#include "bsp_can.h"
#include <string.h>

RC_ctrl_t rc_ctrl;  // 全局遥控器结构体

static uint32_t last_rc_tick; // 看门狗时间戳

// 大疆 DBUS 协议解析
void RC_Decode(volatile const uint8_t *sbus_buf) {
    // 摇杆通道解析 
    rc_ctrl.rc.ch[0] = (sbus_buf[0] | sbus_buf[1] << 8) & 0x07FF;
    rc_ctrl.rc.ch[0] -= 1024;
    
    rc_ctrl.rc.ch[1] = (sbus_buf[1] >> 3 | sbus_buf[2] << 5) & 0x07FF;
    rc_ctrl.rc.ch[1] -= 1024;
    
    rc_ctrl.rc.ch[2] = (sbus_buf[2] >> 6 | sbus_buf[3] << 2 | sbus_buf[4] << 10) & 0x07FF;
    rc_ctrl.rc.ch[2] -= 1024;
    
    rc_ctrl.rc.ch[3] = (sbus_buf[4] >> 1 | sbus_buf[5] << 7) & 0x07FF;
    rc_ctrl.rc.ch[3] -= 1024;

    rc_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);
    rc_ctrl.rc.ch[4] -= 1024; // 减去 1024 归一化
	
    // 左右拨杆开关解析
   rc_ctrl.rc.s0 = ((sbus_buf[5] >> 4) & 0x0003);      // 右拨杆
   rc_ctrl.rc.s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2; // 左拨杆
    
    // 每次串口 DMA 成功调用此解析函数，就更新当前时间戳
    last_rc_tick = HAL_GetTick(); 
}

Robot_Mode_e robot_mode = MODE_RELAX;



void Remote_Mode_Update(void) {
    
    // 看门狗掉线保护 (100ms 没收到遥控器数据，强制失能)
    if (HAL_GetTick() - last_rc_tick > 100) {
        robot_mode = MODE_RELAX; 
        memset(&rc_ctrl, 0, sizeof(RC_ctrl_t)); 
        return; 
    }


    // 最高优先级：双下急停 (左下2 + 右下2)
    if (rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s0 == 2) {
        robot_mode = MODE_RELAX;
    }
    // 只动底盘模式：左下(2) + 右中(3)
    else if (rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s0 == 3) {
        robot_mode = MODE_CHASSIS_ONLY;
    }
    // 只动云台：左中(3) + 右下(2)
    else if (rc_ctrl.rc.s1 == 3 && rc_ctrl.rc.s0 == 2) {
        robot_mode = MODE_GIMBAL_ONLY;
    }
    // 双中跟随模式：左中(3) + 右中(3)
    else if (rc_ctrl.rc.s1 == 3 && rc_ctrl.rc.s0 == 3) {
        robot_mode = MODE_FOLLOW;
    }
    // 小陀螺模式：左中(3) + 右上(1) 
    else if (rc_ctrl.rc.s1 == 3 && rc_ctrl.rc.s0 == 1) {
        robot_mode = MODE_SPIN;
    }
}

// 供 Default_Task 调用发送数据给 C 板的函数
void BoardA_Sync_And_Send(void) {
    board_a_tx_data.ch_yaw   = rc_ctrl.rc.ch[0]; 
    board_a_tx_data.ch_pitch = rc_ctrl.rc.ch[1]; 
    board_a_tx_data.s0       = rc_ctrl.rc.s0;  
    board_a_tx_data.s1       = rc_ctrl.rc.s1;  
    board_a_tx_data.robot_mode = robot_mode;     // 将 A 板计算出的工作模式直接同步过去
    
    CAN_Send_A_to_C();
}
