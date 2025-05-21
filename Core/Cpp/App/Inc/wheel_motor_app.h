#ifndef WHEEL_MOTOR_APP_H
#define WHEEL_MOTOR_APP_H
#pragma once
#include "stdio.h"
#include "wheel_motor.h"
#include "chassis_controller.h"
#include "FreeRTOS.h"
#include "task.h"
extern WheelMotor left_front;
extern WheelMotor right_front;
extern WheelMotor left_rear;
extern WheelMotor right_rear;
extern ChassisController chassis;

void FourWheelMotorApp();
void FourWheelInterruptCountReset();


#endif // WHEEL_MOTOR_APP_H