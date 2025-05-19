#include "uart_comm.h"
#include "usart.h" // 包含STM32CubeMX生成的UART配置
#include <cstring>
#include <algorithm>

// 单例实现,全局唯一访问点,确保整个系统中只存在一个UartComm实例
UartComm& UartComm::getInstance() {
    static UartComm instance;
    return instance;
}

UartComm::UartComm()
    : huart_(nullptr), transfer_mode_(MODE_DMA), 
      is_receiving_(false), is_sending_(false), rx_callback_(nullptr) {
    rx_buffer_.reserve(RX_BUFFER_SIZE);
    std::fill(rx_dma_buffer_, rx_dma_buffer_ + RX_BUFFER_SIZE, 0);
}

UartComm::~UartComm() {
    stopReceive();
}

void UartComm::init(UART_HandleTypeDef* huart, TransferMode mode) {
    // 默认使用UART2
    huart_ = (huart != nullptr) ? huart : &huart2;
    transfer_mode_ = mode;
    
    // 确保UART已经初始化
    if (huart_->gState == HAL_UART_STATE_RESET) {
        // 如果UART未初始化，可以在这里初始化
        // 但通常应该由CubeMX生成的代码初始化
        // MX_USART2_UART_Init();
    }
    
    // 清空接收缓冲区
    clearRxBuffer();
    
    // 检查是否支持DMA，如果不支持则切换到中断模式
    if (mode == MODE_DMA && !isDmaSupported()) {
        transfer_mode_ = MODE_INTERRUPT;
    }
}

bool UartComm::isDmaSupported() const {
    if (huart_ == nullptr) {
        return false;
    }
    
    // 检查是否有DMA句柄
    return (huart_->hdmatx != nullptr && huart_->hdmarx != nullptr);
}

void UartComm::setTransferMode(TransferMode mode) {
    // 如果正在接收或发送，先停止
    if (is_receiving_) {
        stopReceive();
    }
    
    // 设置新模式
    transfer_mode_ = mode;
    
    // 如果设置为DMA但不支持，则回退到中断模式
    if (mode == MODE_DMA && !isDmaSupported()) {
        transfer_mode_ = MODE_INTERRUPT;
    }
}

UartComm::TransferMode UartComm::getTransferMode() const {
    return transfer_mode_;
}

bool UartComm::sendData(const uint8_t* data, uint16_t size, uint32_t timeout) {
    if (huart_ == nullptr || data == nullptr || size == 0) {
        return false;
    }
    
    HAL_StatusTypeDef status;
    
    // 根据传输模式选择发送方式
    switch (transfer_mode_) {
        case MODE_DMA:
            if (isDmaSupported()) {
                is_sending_ = true;
                status = HAL_UART_Transmit_DMA(huart_, (uint8_t*)data, size);
                // 注意：DMA发送是非阻塞的，这里不等待完成
                return (status == HAL_OK);
            }
            // 如果DMA不可用，回退到中断模式
            // 故意不加break，继续执行中断模式代码
            
        case MODE_INTERRUPT:
            is_sending_ = true;
            status = HAL_UART_Transmit_IT(huart_, (uint8_t*)data, size);
            return (status == HAL_OK);
            
        case MODE_POLLING:
        default:
            is_sending_ = true;
            status = HAL_UART_Transmit(huart_, (uint8_t*)data, size, timeout);
            is_sending_ = false;
            return (status == HAL_OK);
    }
}

bool UartComm::sendString(const std::string& str, uint32_t timeout) {
    return sendData((uint8_t*)str.c_str(), str.length(), timeout);
}

bool UartComm::sendLine(const std::string& str, uint32_t timeout) {
    std::string line = str + "\r\n";
    return sendString(line, timeout);
}

void UartComm::startReceive() {
    if (huart_ == nullptr || is_receiving_) {
        return;
    }
    
    is_receiving_ = true;
    
    // 根据传输模式选择接收方式
    switch (transfer_mode_) {
        case MODE_DMA:
            if (isDmaSupported()) {
                startDmaReceive();
                return;
            }
            // 如果DMA不可用，回退到中断模式
            // 故意不加break，继续执行中断模式代码
            
        case MODE_INTERRUPT:
        default:
            startInterruptReceive();
            break;
    }
}

bool UartComm::startDmaReceive() {
    if (!isDmaSupported()) {
        return false;
    }
    
    // 使用DMA循环模式接收
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart_, rx_dma_buffer_, RX_BUFFER_SIZE);
    return (status == HAL_OK);
}

bool UartComm::startInterruptReceive() {
    // 启动中断接收，每次接收一个字节
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart_, rx_temp_buffer_, 1);
    return (status == HAL_OK);
}

void UartComm::stopReceive() {
    if (huart_ == nullptr || !is_receiving_) {
        return;
    }
    
    is_receiving_ = false;
    
    // 根据传输模式选择停止方式
    switch (transfer_mode_) {
        case MODE_DMA:
            if (isDmaSupported()) {
                HAL_UART_AbortReceive(huart_);
                return;
            }
            // 如果DMA不可用，回退到中断模式
            // 故意不加break，继续执行中断模式代码
            
        case MODE_INTERRUPT:
        default:
            HAL_UART_AbortReceive_IT(huart_);
            break;
    }
}

void UartComm::setRxCallback(std::function<void(uint8_t*, uint16_t)> callback) {
    rx_callback_ = callback;
}

void UartComm::handleRxComplete(UART_HandleTypeDef* huart) {
    if (huart != huart_ || !is_receiving_) {
        return;
    }
    
    if (transfer_mode_ == MODE_DMA) {
        // DMA模式下，这个回调表示一个完整的DMA循环已经完成
        // 处理DMA缓冲区后半部分的数据
        uint16_t half_size = RX_BUFFER_SIZE / 2;
        uint16_t start_idx = half_size;
        
        // 将DMA缓冲区后半部分的数据添加到接收缓冲区
        for (uint16_t i = 0; i < half_size; i++) {
            rx_buffer_.push_back(rx_dma_buffer_[start_idx + i]);
            if (rx_buffer_.size() > RX_BUFFER_SIZE) {
                rx_buffer_.erase(rx_buffer_.begin());
            }
        }
        
        // 如果设置了回调函数，调用回调
        if (rx_callback_) {
            rx_callback_(&rx_dma_buffer_[start_idx], half_size);
        }
    } else {
        // 中断模式下，每次只接收一个字节
        // 将接收到的字节添加到缓冲区
        rx_buffer_.push_back(rx_temp_buffer_[0]);
        
        // 如果缓冲区超过最大大小，移除最早的数据
        if (rx_buffer_.size() > RX_BUFFER_SIZE) {
            rx_buffer_.erase(rx_buffer_.begin());
        }
        
        // 如果设置了回调函数，调用回调
        if (rx_callback_) {
            rx_callback_(rx_temp_buffer_, 1);
        }
        
        // 继续接收下一个字节
        HAL_UART_Receive_IT(huart_, rx_temp_buffer_, 1);
    }
}

void UartComm::handleRxHalfComplete(UART_HandleTypeDef* huart) {
    if (huart != huart_ || !is_receiving_ || transfer_mode_ != MODE_DMA) {
        return;
    }
    
    // DMA半满中断，处理DMA缓冲区前半部分的数据
    uint16_t half_size = RX_BUFFER_SIZE / 2;
    
    // 将DMA缓冲区前半部分的数据添加到接收缓冲区
    for (uint16_t i = 0; i < half_size; i++) {
        rx_buffer_.push_back(rx_dma_buffer_[i]);
        if (rx_buffer_.size() > RX_BUFFER_SIZE) {
            rx_buffer_.erase(rx_buffer_.begin());
        }
    }
    
    // 如果设置了回调函数，调用回调
    if (rx_callback_) {
        rx_callback_(rx_dma_buffer_, half_size);
    }
}

void UartComm::handleTxComplete(UART_HandleTypeDef* huart) {
    if (huart != huart_) {
        return;
    }
    
    // 发送完成，清除发送标志
    is_sending_ = false;
}

void UartComm::handleError(UART_HandleTypeDef* huart) {
    if (huart != huart_) {
        return;
    }
    
    // 发生错误，重新启动接收
    if (is_receiving_) {
        stopReceive();
        startReceive();
    }
}

const std::vector<uint8_t>& UartComm::getRxBuffer() const {
    return rx_buffer_;
}

void UartComm::clearRxBuffer() {
    rx_buffer_.clear();
}