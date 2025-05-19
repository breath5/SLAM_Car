#include "uart_comm.h"
#include <string>



// 在适当的位置添加头文件引用
extern "C" {
#include "stm32f4xx_it.h"
}

// 在C++代码中声明外部函数
extern "C" {
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
}

// 在您的C++文件中实现这些函数
#include "uart_comm.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UartComm::getInstance().handleRxComplete(huart);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    UartComm::getInstance().handleRxHalfComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    UartComm::getInstance().handleTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    UartComm::getInstance().handleError(huart);
}





// 接收回调函数示例
void onDataReceived(uint8_t* data, uint16_t size) {
    // 处理接收到的数据
    
    if (size > 0) {
        // 例如，回显接收到的数据
        UartComm::getInstance().sendData(data, size);
    }
}

// 初始化串口通信
void initUartComm() {
    // 获取单例并初始化
    UartComm& uart = UartComm::getInstance();
    uart.init(); // 使用默认的UART2
    
    // 设置接收回调
    uart.setRxCallback(onDataReceived);
    
    // 开始接收数据
    uart.startReceive();
    
    // 发送欢迎消息
    uart.sendLine("UART通信初始化完成!");
}