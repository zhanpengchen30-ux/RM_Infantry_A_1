#include "board_com.h"
#include "string.h"

// 声明外部 CAN 句柄
extern CAN_HandleTypeDef hcan1;

// 1. 定义全局双板通信数据结构体变量
BoardA_to_BoardC_t board_a_tx_data; // A板发送给C板的数据
BoardC_to_BoardA_t board_c_tx_data; // C板发送给A板的数据

/**
  * @brief          双板数据初始化，设定安全默认值
  * @param[in]      none
  * @retval         none
  */
	
	// 在 board_com.c 中，确保函数名是 BoardA_Get_RxData
BoardC_to_BoardA_t BoardA_Get_RxData(void)
{
    return board_c_tx_data;
}

void Board_Com_Init(void)
{
    // 将所有数据清零
    memset(&board_a_tx_data, 0, sizeof(BoardA_to_BoardC_t));
    memset(&board_c_tx_data, 0, sizeof(BoardC_to_BoardA_t));
    
	
	
    // 遥控器通道中位值为 0
    board_a_tx_data.ch_yaw = 0;
    board_a_tx_data.ch_pitch = 0;
    
    // 大疆 DR16 拨码开关默认安全值设置为 2 (下档)
    board_a_tx_data.s0 = 2; 
    board_a_tx_data.s1 = 2;
    board_a_tx_data.robot_mode = 0; // 对应遥控器的 MODE_RELAX
    
    // C板反馈数据初始化
    board_c_tx_data.yaw_relative_ecd = 0;
    board_c_tx_data.vx = 0; 
    board_c_tx_data.vy = 0;
    board_c_tx_data.wz = 0;
}

/**
  * @brief          A板发送数据至C板（标准CAN1发送，ID: 0x301）
  * @param[in]      none
  * @retval         HAL_StatusTypeDef 发送状态
  */
HAL_StatusTypeDef CAN_Send_A_to_C(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    
    // 检查 CAN 外设是否正常启用
    if (hcan1.State != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }
    
    // 将结构体数据安全拷贝至待发送数组
    memcpy(tx_data, &board_a_tx_data, sizeof(BoardA_to_BoardC_t));
    
    // 配置 CAN 发送报头
    tx_header.StdId = 0x301;             
    tx_header.IDE = CAN_ID_STD;          
    tx_header.RTR = CAN_RTR_DATA;        
    tx_header.DLC = sizeof(BoardA_to_BoardC_t); 
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 压入邮箱发送
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

/**
  * @brief          C板发送数据至A板（标准CAN1发送，ID: 0x302）
  * @param[in]      none
  * @retval         HAL_StatusTypeDef 发送状态
  */
HAL_StatusTypeDef CAN_Send_C_to_A(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    
    // 检查 CAN 外设是否正常启用
    if (hcan1.State != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }
    
    // 将结构体数据安全拷贝至待发送数组
    memcpy(tx_data, &board_c_tx_data, sizeof(BoardC_to_BoardA_t));
    
    // 配置 CAN 发送报头
    tx_header.StdId = 0x302;             
    tx_header.IDE = CAN_ID_STD;          
    tx_header.RTR = CAN_RTR_DATA;        
    tx_header.DLC = sizeof(BoardC_to_BoardA_t); 
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 压入邮箱发送
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}
