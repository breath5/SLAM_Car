#include "uart_class.h"
#include "uart_app.h"
#include "usart.h"
#include "cJSON.h"
#include <stdio.h>

// 全局UART实例
static UART* uart2 = nullptr;
// 添加任务句柄声明
static TaskHandle_t xUARTProcessTaskHandle = NULL;

void UART2_TxCpltCallback(void) {
    if (uart2 != nullptr) {
        uart2->TxCpltCallback();
    }
}

void UART2_RxCpltCallback(void) {
    if (uart2 != nullptr) {
        uart2->RxCpltCallback();
    }
}
    
// 在C++代码中声明外部函数
extern "C" {
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART2) {
        // printf("USART2 revice data\r\n");
        // UART2_RxCpltCallback();
        // 发送任务通知（带中断安全版本）
        vTaskNotifyGiveFromISR(xUARTProcessTaskHandle, &xHigherPriorityTaskWoken);
        // 如果有更高优先级任务需要唤醒，执行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // UART2_TxCpltCallback();
    }
}


// 全局回调函数，用于处理接收到的JSON数据
static void JsonDataCallback(const char* jsonStr, uint16_t len) {

    printf("Received complete JSON (%d bytes): %s\r\n", len, jsonStr);
    
    // 解析JSON
    cJSON* root = cJSON_Parse(jsonStr);
    if (root == NULL) {
        printf("Error: Failed to parse JSON\r\n");
        return;
    }
    
    // 获取ID
    cJSON* idItem = cJSON_GetObjectItem(root, "ID");
    if (idItem && cJSON_IsNumber(idItem)) {
        int id = idItem->valueint;
        printf("Parsed ID: %d\r\n", id);
        
        // 获取PID参数
        cJSON* pItem = cJSON_GetObjectItem(root, "P");
        cJSON* iItem = cJSON_GetObjectItem(root, "I");
        cJSON* dItem = cJSON_GetObjectItem(root, "D");
        
        if (pItem && iItem && dItem && 
            cJSON_IsNumber(pItem) && 
            cJSON_IsNumber(iItem) && 
            cJSON_IsNumber(dItem)) {
            
            float p = (float)pItem->valuedouble;
            float i = (float)iItem->valuedouble;
            float d = (float)dItem->valuedouble;
            int32_t temp_pid = p * 1000;
            
            printf("Parsed PID: P=%d, I=%d, D=%.2f\r\n", temp_pid, i, d);
            
            // 这里可以根据ID设置不同电机的PID参数
            // 例如：
            // if (id == 1) {
            //     left_front.SetPID(p, i, d);
            // } else if (id == 2) {
            //     right_front.SetPID(p, i, d);
            // } ...
            if(id == 1){
                left_front.SetPIDParams(p, i, d);
            }else if(id == 2){
                right_front.SetPIDParams(p, i, d);
            }else if(id == 3){
                left_rear.SetPIDParams(p, i, d);
            }else if(id == 4){
                right_rear.SetPIDParams(p, i, d);
            }

        } else {
            printf("Error: Missing or invalid PID parameters\r\n");
        }
    } else {
        printf("Error: Missing or invalid ID\r\n");
    }
    
    // 释放JSON对象
    cJSON_Delete(root);
}

// 处理UART接收到的数据的任务
static void UARTProcessTask(void* pvParameters) {
    while (1) {
        // 任务主循环，可以添加其他处理逻辑
        // 等待任务通知（永久阻塞）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // 在任务上下文中处理回调
        if (uart2 != nullptr) {
// printf("UARTProcessTask Received data\r\n");
            uart2->RxCpltCallback();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// C接口实现，直接在cpp文件中实现
#ifdef __cplusplus
extern "C" {
#endif

void UARTApp_Init(void) {
    // 创建UART实例
    uart2 = new UART(&huart2);
    
    // 初始化UART
    uart2->Init();
    
    // 设置回调函数
    uart2->SetRxCallback(JsonDataCallback);
    
    // 启动接收
    uart2->StartReceive();

    printf("UART Application initialized\r\n");
    // uart2->PrintString("Hello, UART!\r\n");
    // 创建UART处理任务
    xTaskCreate(UARTProcessTask, "UARTProcess", 256, NULL, 15, &xUARTProcessTaskHandle);

}


#ifdef __cplusplus
}
#endif