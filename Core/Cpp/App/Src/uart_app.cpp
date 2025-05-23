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
        // 对于中断模式，直接在中断中处理
        if (uart2 != nullptr) {
            if (uart2->GetReceiveMode() == UART_RECEIVE_MODE_IT) {
                uart2->RxITCallback();
            } else {
                // DMA模式下，继续使用任务通知
                vTaskNotifyGiveFromISR(xUARTProcessTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // UART2_TxCpltCallback();
    }
}


// 全局回调函数，用于处理接收到的JSON数据
static void JsonDataCallback(const char* jsonStr, uint16_t len) {

    // printf("Received complete JSON (%d bytes): %s\r\n", len, jsonStr);
    
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
        int32_t temp_pid = 0;
        printf("Parsed ID: %d\r\n", id);

        // 分别检查每个参数
        cJSON* pItem = cJSON_GetObjectItem(root, "P");
        cJSON* iItem = cJSON_GetObjectItem(root, "I");
        cJSON* dItem = cJSON_GetObjectItem(root, "D");
        cJSON* TargetSpeedItem = cJSON_GetObjectItem(root, "TargetSpeed");

        //运动控制
        if(id == 5){
            if (pItem && cJSON_IsNumber(pItem)) {
                uint8_t p = (uint8_t)pItem->valuedouble;
                switch(id) {
                    //小车移动控制
                    case 1: chassis.MoveForward(0.2f);; break;  // 前进
                    case 2: chassis.MoveBackward(0.2f); break; // 后退
                    case 3: chassis.MoveLeft(0.2f); break;    // 左转
                    case 4: chassis.MoveRight(0.2f); break;   // 右转
                    case 5: chassis.Stop(); break;         // 停止
                }
            }
        }

        // 单独处理每个参数
        if (pItem && cJSON_IsNumber(pItem)) {
            float p = (float)pItem->valuedouble;
            temp_pid = p * 1000;
            printf("Set P: %d\r\n", temp_pid);
            // 根据ID设置对应电机
            switch(id) {
                case 1: left_front.SetKp(p); break;
                case 2: right_front.SetKp(p); break;
                case 3: left_rear.SetKp(p); break;
                case 4: right_rear.SetKp(p); break;
                 
            }
        }

        if (iItem && cJSON_IsNumber(iItem)) {
            float i = (float)iItem->valuedouble;
            temp_pid = i * 1000;
            printf("Set I: %d\r\n", temp_pid);
            switch(id) {
                case 1: left_front.SetKi(i); break;
                case 2: right_front.SetKi(i); break;
                case 3: left_rear.SetKi(i); break;
                case 4: right_rear.SetKi(i); break;
            }
        }

        if (dItem && cJSON_IsNumber(dItem)) {
            float d = (float)dItem->valuedouble;
            temp_pid = d * 1000;
            printf("Set D: %d\r\n", temp_pid);
            switch(id) {
                case 1: left_front.SetKd(d); break;
                case 2: right_front.SetKd(d); break;
                case 3: left_rear.SetKd(d); break;
                case 4: right_rear.SetKd(d); break;
            }
        }

        if(TargetSpeedItem && cJSON_IsNumber(TargetSpeedItem)) {
            float TargetSpeed = (float)TargetSpeedItem->valuedouble;
            switch(id) {
                case 1: left_front.SetTargetSpeed(TargetSpeed); break;
                case 2: right_front.SetTargetSpeed(TargetSpeed); break;
                case 3: left_rear.SetTargetSpeed(TargetSpeed); break;
                case 4: right_rear.SetTargetSpeed(TargetSpeed); break;
            }
        }

    } else {
        printf("Error: Missing or invalid ID\r\n");
    }
    // 释放JSON对象
    cJSON_Delete(root);
}

// 处理UART接收到的数据的任务
static void UARTProcessTask(void* pvParameters) {
    printf("xTaskCreate:UARTProcessTask created;  uxPriority:15; usStackDepth:256\r\n");
    while (1) {
        // 任务主循环，可以添加其他处理逻辑
        // 等待任务通知（永久阻塞）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // 在任务上下文中处理回调
        if (uart2 != nullptr) {
            // printf("UARTProcessTask Received data\r\n");
            uart2->RxCpltCallback();
            // uart2->RxITCallback();
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
    
    // 设置为中断接收模式
    uart2->SetReceiveMode(UART_RECEIVE_MODE_IT);

    printf("UART2: Application initialized. RxCallback:JsonDataCallback. Mode:IT\r\n");
    // uart2->PrintString("Hello, UART!\r\n");
    // 创建UART处理任务
    xTaskCreate(UARTProcessTask, "UARTProcess", 256, NULL, 20, &xUARTProcessTaskHandle);

}

#ifdef __cplusplus
}
#endif