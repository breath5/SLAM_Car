#include "uart_class.h"
#include <stdio.h>

UART::UART(UART_HandleTypeDef* huart) : 
    huart(huart),
    activeBuffer(rxBuffer1),
    processBuffer(rxBuffer2),
    jsonBufferIndex(0),
    rxCallback(nullptr),
    bracketCount(0),
    inJsonFrame(false),
    isTxBusy(false) {
    
    // 创建信号量
    bufferSwitchSemaphore = xSemaphoreCreateBinary();
    txSemaphore = xSemaphoreCreateMutex();
    
    // 初始释放信号量
    xSemaphoreGive(bufferSwitchSemaphore);
    xSemaphoreGive(txSemaphore);
    
    // 清空缓冲区
    memset(rxBuffer1, 0, UART_RX_BUFFER_SIZE);
    memset(rxBuffer2, 0, UART_RX_BUFFER_SIZE);
    memset(jsonBuffer, 0, JSON_BUFFER_SIZE);
    memset(txBuffer, 0, UART_TX_BUFFER_SIZE);
}

UART::~UART() {
    if (bufferSwitchSemaphore != nullptr) {
        vSemaphoreDelete(bufferSwitchSemaphore);
    }
    if (txSemaphore != nullptr) {
        vSemaphoreDelete(txSemaphore);
    }
}

void UART::Init() {
    // 确保UART已经初始化
    if (huart == nullptr) {
        return;
    }
    
    // 禁用UART接收中断
    HAL_UART_AbortReceive(huart);
}

void UART::StartReceive() {
    // 开始DMA接收
    HAL_UART_Receive_DMA(huart, activeBuffer, UART_RX_BUFFER_SIZE);
}

void UART::SetRxCallback(UARTRxCallback callback) {
    rxCallback = callback;
}

void UART::RxCpltCallback() {
    // 获取信号量，确保缓冲区切换的原子性
    if (xSemaphoreTake(bufferSwitchSemaphore, (TickType_t)10) == pdTRUE) {
        // 交换缓冲区
        uint8_t* temp = activeBuffer;
        activeBuffer = processBuffer;
        processBuffer = temp;
        
        // 释放信号量
        xSemaphoreGive(bufferSwitchSemaphore);
        
        // 重新启动DMA接收
        HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_DMA(huart, activeBuffer, UART_RX_BUFFER_SIZE);
        
        // 处理接收到的数据
        // printf("UART RxCpltCallback\r\n");
        ProcessData(processBuffer, UART_RX_BUFFER_SIZE);
    }
}

// 以下是新增的函数

HAL_StatusTypeDef UART::SendData(const uint8_t* data, uint16_t size, uint32_t timeout) {
    if (huart == nullptr || data == nullptr || size == 0) {
        return HAL_ERROR;
    }
    
    // 使用阻塞模式发送数据
    return HAL_UART_Transmit(huart, (uint8_t*)data, size, timeout);
}

HAL_StatusTypeDef UART::SendDataDMA(const uint8_t* data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    
    if (huart == nullptr || data == nullptr || size == 0 || size > UART_TX_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    
    // 获取发送信号量
    if (xSemaphoreTake(txSemaphore, (TickType_t)100) == pdTRUE) {
        // 检查是否正在发送
        if (!isTxBusy) {
            // 复制数据到发送缓冲区
            memcpy(txBuffer, data, size);
            
            // 标记为忙
            isTxBusy = true;
            
            // 使用DMA发送数据
            status = HAL_UART_Transmit_DMA(huart, txBuffer, size);
            
            // 如果发送失败，清除忙标志
            if (status != HAL_OK) {
                isTxBusy = false;
            }
        }
        
        // 释放信号量
        xSemaphoreGive(txSemaphore);
    }
    
    return status;
}

void UART::TxCpltCallback() {
    // 发送完成，清除忙标志
    isTxBusy = false;
}

int UART::Printf(const char* format, ...) {
    char buffer[UART_TX_BUFFER_SIZE];
    va_list args;
    int length;
    
    // 格式化字符串
    va_start(args, format);
    length = vsnprintf(buffer, UART_TX_BUFFER_SIZE, format, args);
    va_end(args);
    
    if (length > 0) {
        // 发送格式化后的字符串
        SendDataDMA((uint8_t*)buffer, length);
    }
    
    return length;
}

HAL_StatusTypeDef UART::PrintString(const char* str) {
    if (str == nullptr) {
        return HAL_ERROR;
    }
    
    uint16_t length = strlen(str);
    if (length == 0 || length >= UART_TX_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    
    // 发送字符串
    return SendDataDMA((uint8_t*)str, length);
}

void UART::ProcessData(uint8_t* buffer, uint16_t size) {
    // 逐字节处理数据，检测JSON帧
    for (uint16_t i = 0; i < size; i++) {
        ProcessJsonFrame(buffer[i]);
    }
}

void UART::ProcessJsonFrame(uint8_t byte) {
    // printf("%c", byte);
    // 检测JSON帧开始
    if (byte == '{') {
        if (bracketCount == 0) {
            inJsonFrame = true;
            // printf("JSON frame start\r\n");
        }
        bracketCount++;
    }
    // 检测JSON帧结束
    else if (byte == '}') {
        // printf("cJSON frame end\r\n");
        bracketCount--;
        if (bracketCount == 0 && inJsonFrame) {
            // printf("JSON ready output\r\n");
            // 添加当前字节到JSON缓冲区
            if (jsonBufferIndex < JSON_BUFFER_SIZE - 1) {
                jsonBuffer[jsonBufferIndex++] = byte;
            }
            
            // 添加字符串结束符
            jsonBuffer[jsonBufferIndex] = '\0';
            
            // 如果设置了回调函数，则调用
            if (rxCallback != nullptr) {
                printf("rxCallback: %s\n", jsonBuffer);
                rxCallback(jsonBuffer, jsonBufferIndex);
            }
            
            // 重置JSON缓冲区
            ResetJsonBuffer();
            return;
        }
    }
    
    // 如果在JSON帧内，则添加字节到缓冲区
    if (inJsonFrame && jsonBufferIndex < JSON_BUFFER_SIZE - 1) {
        jsonBuffer[jsonBufferIndex++] = byte;
    }
    
    // 如果JSON缓冲区已满，重置缓冲区
    if (jsonBufferIndex >= JSON_BUFFER_SIZE - 1) {
        // Printf("JSON buffer overflow, resetting\r\n");
        printf("JSON buffer overflow, resetting\r\n");
        ResetJsonBuffer();

    }
}

void UART::ResetJsonBuffer() {
    jsonBufferIndex = 0;
    bracketCount = 0;
    inJsonFrame = false;
    memset(jsonBuffer, 0, JSON_BUFFER_SIZE);
}