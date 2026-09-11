#include "bsp_can.h"
#include "motor.h"
#include "board_com.h" 
#include "string.h"

// 必须声明外部句柄
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN_Filter_Init(void) {
    CAN_FilterTypeDef can_filter_st;

    //  配置 CAN1 过滤器
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    can_filter_st.FilterBank = 0;          
    can_filter_st.SlaveStartFilterBank = 14; 
    
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    
    // 配置 CAN2 过滤器
    can_filter_st.FilterBank = 14;         
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st); 

    // 启动两个 CAN 并开启中断 
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CAN接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // CAN2 (底盘) 
    if (hcan->Instance == CAN2) {
        // ID 0x201 ~ 0x204 是 3508 反馈
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204) {
            uint8_t index = rx_header.StdId - 0x201;
            decode_motor_measure(&motor_chassis[index], rx_data);
        }
    }
    // 处理 CAN1 (云台、拨盘、双板通信)
    else if (hcan->Instance == CAN1) {
        if (rx_header.StdId == 0x205) { // Yaw 电机反馈
            decode_6020_measure(&motor_yaw, rx_data);
        } else if (rx_header.StdId == 0x206) { // Pitch 电机反馈
            decode_6020_measure(&motor_pitch, rx_data);
        } else if (rx_header.StdId == 0x203) { // 拨盘 M2006
            decode_motor_measure(&motor_feeder, rx_data);
        } 
        // 双板接收逻辑：memcpy 
        else if (rx_header.StdId == 0x301) { // 接收 A->C 遥控器与模式数据
            memcpy(&board_a_tx_data, rx_data, sizeof(BoardA_to_BoardC_t));
        } else if (rx_header.StdId == 0x302) { // 接收 C->A 编码器反馈
            memcpy(&board_c_tx_data, rx_data, sizeof(BoardC_to_BoardA_t));
        }
    }
}

// 【CAN2 发送】向底盘 4 个 M3508 发送电流指令 (ID: 0x200)
void CAN_Send_Chassis_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    tx_header.StdId = 0x200; 
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = iq1 >> 8; tx_data[1] = iq1;
    tx_data[2] = iq2 >> 8; tx_data[3] = iq2;
    tx_data[4] = iq3 >> 8; tx_data[5] = iq3;
    tx_data[6] = iq4 >> 8; tx_data[7] = iq4;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

// 【CAN1 发送】向云台 M6020 发送电流指令 (ID: 0x1FE 控制 0x205, 0x206)
void CAN_Send_Gimbal_Voltage(int16_t yaw_v, int16_t pitch_v) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;

    tx_header.StdId = 0x1FE; 
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = yaw_v >> 8;   tx_data[1] = yaw_v;
    tx_data[2] = pitch_v >> 8; tx_data[3] = pitch_v;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}

void CAN_Send_Feeder_Current(int16_t iq) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;

    tx_header.StdId = 0x200; 
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[4] = iq >> 8; 
    tx_data[5] = iq;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}
