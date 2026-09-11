#include "detect_task.h"
#include "cmsis_os.h"

// 离线检测任务 (以后用来判断遥控器、电机是否掉线)
void StartDetectTask(void *argument) {
    
    for(;;) {
        // 这里以后写状态监控逻辑
        osDelay(10); // 10ms 检查一次 (100Hz)
    }
}
