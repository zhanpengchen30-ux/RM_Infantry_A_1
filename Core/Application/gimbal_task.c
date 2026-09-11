#include "gimbal_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "motor.h"
#include "bsp_can.h"
#include "remote.h"
#include "board_com.h"

PID_TypeDef yaw_pos_pid; 
PID_TypeDef yaw_spd_pid; 

volatile float YAW_P_KP = 0.5f;     
volatile float YAW_S_KP = 80.0f;   
volatile float YAW_S_KI = 1.0f;     

// 在 gimbal_task.c 顶部定义
float debug_pos_out;   // 位置环输出的目标速度
float debug_final_v;   // 速度环输出的最终电压

float yaw_target_ecd = 0;   

void StartGimbalTask(void *argument) {
    PID_Init(&yaw_pos_pid, YAW_P_KP, 0.0f, 0.0f, 3000.0f, 0.0f);
    PID_Init(&yaw_spd_pid, YAW_S_KP, YAW_S_KI, 0.0f, 16384.0f, 5000.0f);

    osDelay(200); 
    yaw_target_ecd = (float)motor_yaw.ecd;

    for(;;) {
        // --- 核心修改：从双板通信获取指令 ---
        BoardA_to_BoardC_t cmd = BoardC_Get_RxData(); 

        yaw_pos_pid.Kp = YAW_P_KP;
        yaw_spd_pid.Kp = YAW_S_KP;
        yaw_spd_pid.Ki = YAW_S_KI;

        // 判断模式：使用双板通信收到的 robot_mode
        if (cmd.robot_mode == MODE_GIMBAL_ONLY || cmd.robot_mode == MODE_FOLLOW || cmd.robot_mode == MODE_FRICTION) {
            
            // 使用 A 板传过来的 cmd.ch_yaw 而不是本地的 rc_ctrl
            yaw_target_ecd += (float)cmd.ch_yaw * 0.015f; 
            
            if (yaw_target_ecd > 8191) yaw_target_ecd -= 8192;
            if (yaw_target_ecd < 0)    yaw_target_ecd += 8192;

            float pos_out = yaw_pos_pid.Kp * circle_error(yaw_target_ecd, motor_yaw.ecd, 8192.0f);
            float final_v = PID_Calc(&yaw_spd_pid, motor_yaw.speed_rpm, pos_out);
            
            CAN_Send_Gimbal_Voltage((int16_t)final_v, 0);
        } 
        else {
            CAN_Send_Gimbal_Voltage(0, 0);
            yaw_target_ecd = (float)motor_yaw.ecd; 
        }

        // 反馈给 A 板
        BoardC_Set_TxData(motor_yaw.ecd);
        BoardC_Send_To_A();

        osDelay(1); 
    }
}
