#ifndef WHEEL_MOTOR_H
#define WHEEL_MOTOR_H
#pragma once
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "pid_controller.h"
#include <iostream>

class WheelMotor {
public:
    static constexpr float WHEEL_RADIUS = 0.0485f; // 97mm/2
    static constexpr int REDUCTION_RATIO = 30;  // 减速比
    static constexpr int PULSES_PER_ROUND = 11;  // 编码器脉冲数
    static constexpr float GEAR_OUTPUT_MAX_RPM = 180.0f;  // 额定空载转速
    
    // 电机方向枚举
    enum Direction {
        FORWARD = 1,
        REVERSE = -1,
        STOP = 0
    };
    
    WheelMotor(int id, TIM_HandleTypeDef* pwm_htim, uint32_t pwm_channel, 
               TIM_HandleTypeDef* enc_htim, 
               GPIO_TypeDef* dir_port1 = nullptr, uint16_t dir_pin1 = 0,
               GPIO_TypeDef* dir_port2 = nullptr, uint16_t dir_pin2 = 0);
    
    // 基本控制函数
    void SetDutyCycle(int pwm_value);
    int GetDutyCycle() const;
    void UpdateEncoder();
    void SetSampleTime(float sample_time);
    float GetSampleTime() const;
    float CalculateLinearSpeed();
    float GetLinearSpeed() const;
    int GetID() const;
    
    // 方向控制
    void SetDirection(Direction dir);
    Direction GetDirection() const;
    
    // 速度控制
    void SetLinearSpeed(float target_speed);
    void SetTargetSpeed(float target_speed); // 设置目标速度，用于PID控制
    void UpdateSpeedControl(float dt);       // 更新速度控制，应在定时器中调用
    
    // 初始化和重置
    void ResetEncoder();
    void Init();
    
    // PID控制器访问
    void SetPIDParams(float kp, float ki, float kd);
    PIDController& GetPIDController() { return pid_controller_; }
    
    // 设置最大RPM
    void SetMaxRPM(float max_rpm) { max_rpm_ = max_rpm; }
    float GetMaxRPM() const { return max_rpm_; }

    // 新增：读取 PID 参数的接口
    float GetKp() const;  // 读取比例系数 Kp
    float GetKi() const;  // 读取积分系数 Ki
    float GetKd() const;  // 读取微分系数 Kd
    
private:
    int id_;
    TIM_HandleTypeDef* pwm_htim_;
    uint32_t pwm_channel_;
    TIM_HandleTypeDef* enc_htim_;
    
    // 方向控制GPIO - 使用两个引脚
    GPIO_TypeDef* dir_port1_;
    uint16_t dir_pin1_;
    GPIO_TypeDef* dir_port2_;
    uint16_t dir_pin2_;
    Direction direction_;
    
    int32_t last_count_;
    int32_t current_count_;
    int pwm_value_;
    float linear_speed_;
    float target_speed_;
    float sample_time_ = 0.1f; // 采样周期，单位秒，默认100ms
    float max_rpm_ = 320.0f;   // 最大转速，单位RPM
    
    // PID控制器
    PIDController pid_controller_;
    
    // 平均滤波器
    static const int FILTER_SIZE = 4;
    int32_t filter_buffer[FILTER_SIZE];
    int filter_index;

    const float deadzone_threshold_ = 0.1f; // 死区阈值
    
    int32_t ApplyFilter(int32_t new_value);
};

#endif