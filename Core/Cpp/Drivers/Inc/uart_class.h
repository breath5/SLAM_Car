#ifndef UART_CLASS_H
#define UART_CLASS_H
#pragma once

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// 定义缓冲区大小
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 512  // 发送缓冲区大小
#define JSON_BUFFER_SIZE 512

// 回调函数类型定义
typedef void (*UARTRxCallback)(const char* jsonStr, uint16_t len);

class UART {
private:
    UART_HandleTypeDef* huart;                  // UART句柄
    uint8_t rxBuffer1[UART_RX_BUFFER_SIZE];     // 接收缓冲区1
    uint8_t rxBuffer2[UART_RX_BUFFER_SIZE];     // 接收缓冲区2
    uint8_t* activeBuffer;                      // 当前活动缓冲区
    uint8_t* processBuffer;                     // 当前处理缓冲区
    
    char jsonBuffer[JSON_BUFFER_SIZE];          // JSON数据缓冲区
    uint16_t jsonBufferIndex;                   // JSON缓冲区索引
    
    uint8_t txBuffer[UART_TX_BUFFER_SIZE];      // 发送缓冲区
    SemaphoreHandle_t txSemaphore;              // 发送信号量
    volatile bool isTxBusy;                     // 发送忙标志
    
    UARTRxCallback rxCallback;                  // 接收回调函数
    SemaphoreHandle_t bufferSwitchSemaphore;    // 缓冲区切换信号量
    
    int bracketCount;                           // 大括号计数器，用于JSON帧检测
    bool inJsonFrame;                           // 是否在JSON帧内

public:
    UART(UART_HandleTypeDef* huart);
    ~UART();
    
    // 初始化UART和DMA
    void Init();
    
    // 启动接收
    void StartReceive();
    
    // 设置接收回调函数
    void SetRxCallback(UARTRxCallback callback);
    
    // DMA接收完成回调
    void RxCpltCallback();
    
    // 处理接收到的数据
    void ProcessData(uint8_t* buffer, uint16_t size);
    
    // 检测并处理JSON帧
    void ProcessJsonFrame(uint8_t byte);
    
    // 重置JSON缓冲区
    void ResetJsonBuffer();
    
    // 发送数据（阻塞模式）
    HAL_StatusTypeDef SendData(const uint8_t* data, uint16_t size, uint32_t timeout = 100);
    
    // 发送数据（DMA模式）
    HAL_StatusTypeDef SendDataDMA(const uint8_t* data, uint16_t size);
    
    // 发送完成回调
    void TxCpltCallback();
    
    // 格式化打印函数
    int Printf(const char* format, ...);
    
    // 打印字符串
    HAL_StatusTypeDef PrintString(const char* str);
};

#endif // UART_CLASS_H