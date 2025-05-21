#ifndef UART_APP_H
#define UART_APP_H

#include "wheel_motor_app.h"

extern WheelMotor left_front;
extern WheelMotor right_front;
extern WheelMotor left_rear;
extern WheelMotor right_rear;

#ifdef __cplusplus
extern "C" {
#endif

// 应用函数声明
void UARTApp_Init(void);
// void UART2_RxCpltCallback(void);

#ifdef __cplusplus
}
#endif



#endif // UART_APP_H