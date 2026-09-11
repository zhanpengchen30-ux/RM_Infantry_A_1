#include "motor.h"

// 实例化底盘电机 (ID: 0x201-0x204)
motor_measure_t motor_chassis[4]; 

// 实例化拨盘电机 (ID: 0x203 在 CAN1)
motor_measure_t motor_feeder;

//  实例化云台电机 (ID: 0x205, 0x206)
motor_6020_t motor_yaw;
motor_6020_t motor_pitch;

// 3508 / 2006 解析函数
void decode_motor_measure(motor_measure_t *motor, uint8_t *data) {
    motor->ecd           = (uint16_t)(data[0] << 8 | data[1]);
    motor->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    motor->given_current = (int16_t)(data[4] << 8 | data[5]);
    motor->temperate     = data[6];
}

// 6020 解析函数
void decode_6020_measure(motor_6020_t *motor, uint8_t *data) {
    motor->last_ecd = motor->ecd;
    motor->ecd       = (uint16_t)(data[0] << 8 | data[1]);
    // GM6020 的速度在第 2,3 字节
    motor->speed_rpm = (int16_t)(data[2] << 8 | data[3]); 
    motor->given_current = (int16_t)(data[4] << 8 | data[5]);
    motor->temperate = data[6];
}
