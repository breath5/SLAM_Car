#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32f4xx_hal.h"
#include <string>
#include <vector>
#include <functional>

class UartComm {
public:
    // 传输模式枚举
    enum TransferMode {
        MODE_INTERRUPT,  // 中断模式
        MODE_DMA,        // DMA模式
        MODE_POLLING     // 轮询模式
    };
    
    // 单例模式获取实例
    static UartComm& getInstance();
    
    // 初始化串口
    void init(UART_HandleTypeDef* huart = nullptr, TransferMode mode = MODE_DMA);
    
    // 发送数据方法
    bool sendData(const uint8_t* data, uint16_t size, uint32_t timeout = 100);
    bool sendString(const std::string& str, uint32_t timeout = 100);
    bool sendLine(const std::string& str, uint32_t timeout = 100); // 发送字符串并添加换行符
    
    // 接收数据方法
    void startReceive(); // 开始接收数据
    void stopReceive();  // 停止接收数据
    
    // 设置接收回调函数
    void setRxCallback(std::function<void(uint8_t*, uint16_t)> callback);
    
    // 中断处理函数
    void handleRxComplete(UART_HandleTypeDef* huart);
    void handleRxHalfComplete(UART_HandleTypeDef* huart); // DMA半满中断处理
    void handleTxComplete(UART_HandleTypeDef* huart);     // 发送完成中断处理
    void handleError(UART_HandleTypeDef* huart);          // 错误处理
    
    // 获取接收缓冲区数据
    const std::vector<uint8_t>& getRxBuffer() const;
    void clearRxBuffer();
    
    // 检查是否支持DMA
    bool isDmaSupported() const;
    
    // 设置传输模式
    void setTransferMode(TransferMode mode);
    TransferMode getTransferMode() const;
    
private:
    // 单例模式私有构造函数
    UartComm();
    ~UartComm();
    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;
    
    UART_HandleTypeDef* huart_; // UART句柄
    TransferMode transfer_mode_; // 传输模式
    
    // 接收相关
    static constexpr uint16_t RX_BUFFER_SIZE = 512;
    uint8_t rx_temp_buffer_[1];                // 单字节接收缓冲区（中断模式）
    uint8_t rx_dma_buffer_[RX_BUFFER_SIZE];    // DMA接收缓冲区
    std::vector<uint8_t> rx_buffer_;           // 完整接收缓冲区
    bool is_receiving_;
    
    // 发送相关
    bool is_sending_;
    
    // 回调函数
    std::function<void(uint8_t*, uint16_t)> rx_callback_;
    
    // 私有辅助方法
    bool startDmaReceive();
    bool startInterruptReceive();
};

#endif // UART_COMM_H