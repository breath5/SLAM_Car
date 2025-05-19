#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include "wheel_motor.h"

class ChassisController {
public:
    // 构造函数，传入四个轮子的引用
    ChassisController(WheelMotor& left_front, WheelMotor& right_front, 
                      WheelMotor& left_rear, WheelMotor& right_rear);
    
    // 设置整车线速度和角速度
    void SetVelocity(float vx, float vy, float omega);
    
    // 前进（Y轴正方向）
    void MoveForward(float speed);
    
    // 后退（Y轴负方向）
    void MoveBackward(float speed);
    
    // 左移（X轴负方向）
    void MoveLeft(float speed);
    
    // 右移（X轴正方向）
    void MoveRight(float speed);
    
    // 原地旋转（正值为顺时针，负值为逆时针）
    void Rotate(float angular_speed);
    
    // 停止所有轮子
    void Stop();
    
    // 更新所有轮子的速度控制
    void Update(float dt);
    
private:
    // 四个轮子的引用
    WheelMotor& left_front_;
    WheelMotor& right_front_;
    WheelMotor& left_rear_;
    WheelMotor& right_rear_;
    
    // 当前目标速度
    float vx_; // X方向速度（左右）
    float vy_; // Y方向速度（前后）
    float omega_; // 角速度（旋转）
    
    // 轮子到中心的距离
    float wheel_base_x_; // 轮距的一半（左右方向）
    float wheel_base_y_; // 轴距的一半（前后方向）
};

#endif // CHASSIS_CONTROLLER_H