#include "chassis_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "motor.h"
#include "bsp_can.h"
#include "remote.h"
#include "math.h"
#include <stdlib.h> 

//#define SPIN_SPEED  2500.0f  // 小陀螺自旋转速 
#define GIMBAL_CENTER_ECD 4427.0f // 物理零点

// 定义拨盘速度环 PID 变量
PID_TypeDef motor_pid_feeder;
PID_TypeDef motor_pos_pid;   
PID_TypeDef chassis_follow_pid; //跟随 PID 
PID_TypeDef motor_pid[4];    // 4 个轮子的速度环 PID
int16_t target_speed[4];     // 轮子目标速度

float debug_kp = 14.0f;
float debug_ki = 0.3f;
float debug_kd = 2.5f;

// 声明外部云台电机变量，用于获取相对角度
extern motor_6020_t motor_yaw;
// 声明队列句柄
extern osMessageQueueId_t remoteQueueHandle;

int32_t feeder_total_ecd = 0;    // 累加后的绝对总编码器值
uint16_t feeder_last_ecd = 0;
uint8_t feeder_init_flag = 0;
float feeder_target_ecd = 0;     // 拨盘目标位置

    // 遥控器右拨杆在“左拨杆朝上”
    uint8_t fire_single = 0;  // 单发指令
    uint8_t fire_burst = 0;   // 连发指令

/**
 * @brief 逆运动学解算
 */
static void omni_kinematics(int16_t vx, int16_t vy, int16_t vw) {
    target_speed[0] = -vx + vy + vw;  // 左前
    target_speed[1] =  vx + vy + vw;  // 右前
    target_speed[2] =  vx - vy + vw;  // 左后
    target_speed[3] = -vx - vy + vw;  // 右后
}

// 浮点数限幅函数
static float float_constrain(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    else if (val > max_val) return max_val;
    else return val;
}

void StartChassisTask(void *argument) {

     // 1. 位置环 PID 初始化（外环）：输入脉冲差，输出目标转速
    // 参数建议：Kp = 0.5f, Ki = 0.01f（加一点积分消静差）, Kd = 0.0f
    PID_Init(&motor_pos_pid, 1.0f, 0.0f, 0.0f, 2000.0f, 200.0f); 

    // 2. 速度环 PID 初始化（内环）：输入转速差，输出电流
    PID_Init(&motor_pid_feeder, 1.0f, 0.0f, 5.0f, 16384.0f, 5000.0f);
    int16_t raw_vx, raw_vy, raw_vw;        
    int16_t v_forward, v_left, v_rotate;  

    int16_t out_current[4];
    int i;

    /* 初始化底盘跟随 PID */
    PID_Init(&chassis_follow_pid, 5.0f, 4.0f, 2.0f, 4000.0f, 0.0f); 

    /* 初始化 4 个轮子的速度环 PID */
    for (i = 0; i < 4; i++) {
        PID_Init(&motor_pid[i], debug_kp, debug_ki, debug_kd, 
                 16384.0f, 5000.0f);  // max_out=16384, max_iout=5000
    }

		// 结构体接收队列里的遥控器数据
		RC_ctrl_t rcv_rc = {0};
		
    osDelay(100);

    for (;;) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);  // 心跳灯

        Remote_Mode_Update();

        /* 实时更新 Debug 参数 */
        for (i = 0; i < 4; i++) {
            motor_pid[i].Kp = debug_kp;
            motor_pid[i].Ki = debug_ki;
            motor_pid[i].Kd = debug_kd;
        }
				
   /* 如果当前是失能状态 (MODE_RELAX)，直接切断输出，不进行 PID 计算！ */
        if (robot_mode == MODE_RELAX) {
            for (i = 0; i < 4; i++) {
                motor_pid[i].Iout = 0.0f;  // 清空积分
                out_current[i] = 0;        // 输出电流直接清零
            }
            // 直接发送 4 个零电流，电机彻底失电放松
            CAN_Send_Chassis_Current(0, 0, 0, 0);
        }
				
        else {
					
					 /* 从队列中获取遥控器数据（超时时间设为 10ms） */
            if (osMessageQueueGet(remoteQueueHandle, &rcv_rc, NULL, pdMS_TO_TICKS(10)) == osOK) {
                // 成功拿到数据
            }
					 if (rcv_rc.rc.s1 == 1) // 左拨杆在上方（左上）
            {
                if (rcv_rc.rc.s0 == 1)      // 右拨杆在上（右上）：单发
                {
                    fire_single = 1;
                    fire_burst = 0;
                }
                else if (rcv_rc.rc.s0 == 3) // 右拨杆在中（右中）：不动
                {
                    fire_single = 0;
                    fire_burst = 0;
                }
                else if (rcv_rc.rc.s0 == 2) // 右拨杆在下（右下）：连发
                {
                    fire_single = 0;
                    fire_burst = 1;
                }
            }
            else 
            {
                // 左拨杆不在上方时，安全锁死，不准发射
                fire_single = 0;
                fire_burst = 0;
            }
						
// 定义常量：单发一格的精确脉冲数 (589824 / 9 = 65536)
    const float FEEDER_STEP = 65536.0f; 

    // 计算多圈绝对位置（实时追踪 M2006 转了多少度）
    if (!feeder_init_flag) {
        feeder_last_ecd = motor_feeder.ecd;
        feeder_total_ecd = motor_feeder.ecd;
        feeder_init_flag = 1;
    } else {
        int16_t diff = (int16_t)motor_feeder.ecd - (int16_t)feeder_last_ecd;
        // 处理 0 和 8192 跨圈衔接的跳变
        if (diff > 4096)  feeder_total_ecd += diff - 8192;
        else if (diff < -4096) feeder_total_ecd += diff + 8192;
        else                  feeder_total_ecd += diff;
        
        feeder_last_ecd = motor_feeder.ecd;
    }

    // 静态变量声明区（把所有状态变量集中在这里，确保全部定义）
    static float base_zero_ecd = 0;         // 开机对齐的绝对基准零位
    static int32_t target_bullet_index = 0;   // 发射子弹的“绝对索引计数器”
    static uint8_t absolute_init_done = 0;    // 基准初始化标志位
    static uint8_t wheel_triggered = 0;       // 拨轮防连击锁
    static uint8_t last_burst_state = 0;      // 连发状态记录（解决未定义报错）

    // 开机或刚上电时，把当前位置“就近对齐”作为基准零位
    if (!absolute_init_done) {
        base_zero_ecd = roundf((float)feeder_total_ecd / FEEDER_STEP) * FEEDER_STEP;
        target_bullet_index = 0;
        feeder_target_ecd = base_zero_ecd;
        absolute_init_done = 1;
    }

    // 读取遥控器拨轮值 ch[4]
    int16_t wheel_val = rcv_rc.rc.ch[4]; 
    if (wheel_val > -15 && wheel_val < 15) {
        wheel_val = 0; 
    }

   // ----------------- 键位逻辑解析 -----------------
    uint8_t is_burst_wheel_mode = 0;

		// ----------------- 手动校准零位功能 -----------------
    // 设定组合键：比如左右拨杆同时打到最下方（s1 == 2 且 s0 == 2），持续按住约 0.5 秒 (250个循环 * 2ms = 500ms)
    static uint16_t calib_hold_counter = 0;
    
    if (rcv_rc.rc.s1 == 2 && rcv_rc.rc.s0 == 2) 
    {
        calib_hold_counter++;
        if (calib_hold_counter > 250) // 按住超过 0.5 秒
        {
            // ★ 核心：以当前手动的实际位置为准，强制重新对齐绝对零位！
            base_zero_ecd = (float)feeder_total_ecd; 
            target_bullet_index = 0;
            feeder_target_ecd = base_zero_ecd;
            
            calib_hold_counter = 250; // 锁住计数器，防止一直触发
            
        }
    } 
    else 
    {
        calib_hold_counter = 0; // 松开组合键后重置计时
    }
		
    if (rcv_rc.rc.s1 == 1) // 左拨杆向上
    {
        if (rcv_rc.rc.s0 == 1)      // 右拨杆向上 -> 【双上】：单发模式（拨轮点动触发）
        {
            if (wheel_val > 300 && !wheel_triggered) {
                target_bullet_index++; 
                wheel_triggered = 1;   
            }
            else if (wheel_val < -300 && !wheel_triggered) {
                target_bullet_index--; 
                wheel_triggered = 1;   
            }
            
            if (wheel_val > -100 && wheel_val < 100) {
                wheel_triggered = 0;
            }

            feeder_target_ecd = base_zero_ecd + (float)target_bullet_index * FEEDER_STEP;
        }
        else if (rcv_rc.rc.s0 == 2) // 右拨杆在下（连发档位）
        {
            // ★ 核心改动：只有当拨杆在连发档，且“拨轮确实被推了（!= 0）”时，才进入连发！
            // 如果你松手让拨轮归零（== 0），is_burst_wheel_mode 就会变成 0
            if (wheel_val != 0) {
                is_burst_wheel_mode = 1;
            }
        }
    }
    else 
    {
        wheel_triggered = 0;
    }

    // ----------------- 拨盘电机执行逻辑 -----------------
    if (is_burst_wheel_mode) 
    {
        // 【连发中】：只要你推着拨轮，目标位置就连续向前/向后推
        feeder_target_ecd += (float)wheel_val * 2.0f;
        
        // 标记当前处于连发状态
        last_burst_state = 1; 
    }
    else 
    {
        // 【单发模式 / 连发拨轮一松手瞬间】
        if (last_burst_state == 1) {
            // ★ 只要拨轮一松手（wheel_val == 0），立刻“就近吸附”到最近的正规弹孔网格上！
            feeder_target_ecd = roundf(feeder_target_ecd / FEEDER_STEP) * FEEDER_STEP;
            target_bullet_index = (int32_t)((feeder_target_ecd - base_zero_ecd) / FEEDER_STEP);
            last_burst_state = 0;
        }
    }

    // ★ 全过程走串级双环位置 PID 闭环
    float feeder_target_spd = PID_Calc(&motor_pos_pid, (float)feeder_total_ecd, feeder_target_ecd);
    feeder_target_spd = float_constrain(feeder_target_spd, -3000.0f, 3000.0f); 

    int16_t feeder_current = (int16_t)PID_Calc(&motor_pid_feeder, (float)motor_feeder.speed_rpm, feeder_target_spd);
    CAN_Send_Feeder_Current(feeder_current);

						
        /* 读取遥控器值 */
            raw_vx = rcv_rc.rc.ch[3];  // 左摇杆上下（前后）
            raw_vy = rcv_rc.rc.ch[2];  // 左摇杆左右（平移）
            raw_vw = rcv_rc.rc.ch[0];  // 用右摇杆左右控制自旋

        /* 死区保护 */
        if (raw_vx > -20 && raw_vx < 20) raw_vx = 0;
        if (raw_vy > -20 && raw_vy < 20) raw_vy = 0;
        if (raw_vw > -20 && raw_vw < 20) raw_vw = 0; 

        v_forward = raw_vx * 12;
        v_left    = raw_vy * 12;
        v_rotate  = raw_vw * 10;     

        /* 根据模式控制底盘 */
        switch (robot_mode) {

            // 底盘模式 
            case MODE_CHASSIS_ONLY: {
                // 摇杆直接控制底盘平移，右横摇杆（v_rotate）控制底盘自转
                omni_kinematics(v_forward, v_left, v_rotate);
                break;
            }

            // 双中跟随模式
            case MODE_FOLLOW: {
                //计算云台跟底盘的角度偏差
                float theta_rad = ((float)motor_yaw.ecd - GIMBAL_CENTER_ECD) / 22.755556f * 3.14159265f / 180.0f;
                
                //如果云台在右偏90°，那我想直走，我就要按原来的右平移
                int16_t rotated_vx = v_forward * cosf(theta_rad) - v_left * sinf(theta_rad);
                int16_t rotated_vy = v_forward * sinf(theta_rad) + v_left * cosf(theta_rad);
                
                //计算自旋扭头追云台的角速度
                float angle_error = circle_error(GIMBAL_CENTER_ECD, motor_yaw.ecd, 8192.0f);
                int16_t follow_vw = (int16_t)PID_Calc(&chassis_follow_pid, 0, angle_error);
                
                // 限制自旋跟随转速
                follow_vw = (follow_vw > 3000) ? 3000 : (follow_vw < -3000 ? -3000 : follow_vw);

                // 逆解输出：云台指哪打哪，同时底盘自动转头对准云台
                omni_kinematics(rotated_vx, rotated_vy, follow_vw);
                break;
            }

            case MODE_SPIN: {        // 智能小陀螺模式（拨轮动则转，拨轮不回中则自动跟随）
                
                // 读取遥控器左上角拨轮 ch[4]
                int16_t wheel_val = rcv_rc.rc.ch[4]; 
                
                // 拨轮死区保护
                if (wheel_val > -15 && wheel_val < 15) {
                    wheel_val = 0;
                }
                
                // ★ 核心逻辑：如果拨轮没有在动（== 0），直接执行“跟随云台”的逻辑！
                if (wheel_val == 0) 
                {
                    // 计算云台跟底盘的角度偏差（借用 MODE_FOLLOW 的跟随算法）
                    float theta_rad = ((float)motor_yaw.ecd - GIMBAL_CENTER_ECD) / 22.755556f * 3.14159265f / 180.0f;
                    
                    int16_t rotated_vx = v_forward * cosf(theta_rad) - v_left * sinf(theta_rad);
                    int16_t rotated_vy = v_forward * sinf(theta_rad) + v_left * cosf(theta_rad);
                    
                    float angle_error = circle_error(GIMBAL_CENTER_ECD, motor_yaw.ecd, 8192.0f);
                    int16_t follow_vw = (int16_t)PID_Calc(&chassis_follow_pid, 0, angle_error);
                    
                    // 限制自旋跟随转速
                    follow_vw = (follow_vw > 3000) ? 3000 : (follow_vw < -3000 ? -3000 : follow_vw);

                    omni_kinematics(rotated_vx, rotated_vy, follow_vw);
                }
                else // 如果拨轮在动，执行小陀螺自旋
                {
                    // 负号 - ，改变投影的旋转方向极性
                    float theta_rad = -((float)motor_yaw.ecd - 4427.0f) / 22.755556f * 3.14159265f / 180.0f;
                    
                    int16_t rotated_vx = v_forward * cosf(theta_rad) - v_left * sinf(theta_rad);
                    int16_t rotated_vy = v_forward * sinf(theta_rad) + v_left * cosf(theta_rad);
                    
                    // 拨轮值比例放大映射为自旋角速度 wz
                    int16_t spin_wz = - wheel_val * 4; // 比例系数  
                    
                    omni_kinematics(rotated_vx, rotated_vy, spin_wz); 
                }
                break;
            }

            case MODE_GIMBAL_ONLY: {
                // 只动云台：底盘停止 
                omni_kinematics(0, 0, 0);
                break;
            }

            case MODE_RELAX:
            default: {
                // 失能 
                omni_kinematics(0, 0, 0);
                
                break;
            }
        }

        /* PID 计算并发送 CAN */
        for (i = 0; i < 4; i++) {
            out_current[i] = (int16_t)PID_Calc(&motor_pid[i],
                                                (float)motor_chassis[i].speed_rpm,
                                                (float)target_speed[i]);
        }
        CAN_Send_Chassis_Current(out_current[0], out_current[1],
                                 out_current[2], out_current[3]);

        osDelay(2);  // 2ms 周期（500Hz）
    }
	}
}
