#include "bsp_usart.h"
#include "remote.h"
#include "cmsis_os.h"

uint8_t sbus_rx_buf[18]; 

// 声明外部的队列句柄
extern osMessageQueueId_t remoteQueueHandle;

void RC_Init(void) {

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_rx_buf, 18);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) { 
        RC_Decode(sbus_rx_buf);
			osMessageQueuePut(remoteQueueHandle, &rc_ctrl, 0, 0);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_rx_buf, 18); 
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { 
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_rx_buf, 18); 
    }
}
