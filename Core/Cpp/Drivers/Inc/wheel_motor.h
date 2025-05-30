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
    static constexpr float WHEEL_DN = 0.097f;  // 轮子直径
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

    // 新增单独设置函数
    void SetKp(float kp);
    void SetKi(float ki);
    void SetKd(float kd);
    
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
    float sample_time_ = 0.01f; // 采样周期，单位秒，10ms 采样周期
    float max_rpm_ = 320.0f;   // 最大转速，单位RPM
    
    // PID控制器
    PIDController pid_controller_;
    
    // 平均滤波器
    static const int FILTER_SIZE = 4;
    int32_t filter_buffer[FILTER_SIZE];
    int filter_index;

    const float deadzone_threshold_ = 0.04f; // 将死区阈值从0.1f降低到0.04f,提高低速响应阈值
    
    int32_t ApplyFilter(int32_t new_value);
    static const int SPEED_FILTER_SIZE = 10;  // 速度滤波窗口大小
    float speed_buffer_[SPEED_FILTER_SIZE];  // 速度缓存数组
    int speed_buffer_index_;                 // 速度缓存索引
    float filtered_speed_;                   // 滤波后的速度
    
    // 卡尔曼滤波参数
    float kalman_q_;    // 过程噪声协方差  
    float kalman_r_;    // 测量噪声协方差
    float kalman_p_;    // 估计误差协方差
    float kalman_k_;    // 卡尔曼增益
    float kalman_x_;    // 状态估计值
    
    // 新增滤波方法
    float ApplySpeedFilter(float new_speed);
    float ApplyKalmanFilter(float measurement);
    float ApplyMedianFilter(float new_speed);
    
    // PID输出滤波相关
    static const int PID_OUTPUT_FILTER_SIZE = 4;  // PID输出滤波窗口大小
    float pid_output_buffer_[PID_OUTPUT_FILTER_SIZE];  // PID输出缓存数组
    int pid_output_buffer_index_;                 // PID输出缓存索引
    float filtered_pid_output_;                   // 滤波后的PID输出
    
    // 低通滤波参数
    float pid_output_alpha_ = 0.5f;  // 低通滤波系数 (0-1)，越小滤波效果越强
    float prev_filtered_pid_output_ = 0.0f;  // 上一次滤波后的PID输出
    
    // PID输出滤波方法
    float ApplyPIDOutputFilter(float new_output);  // 滑动平均滤波
    float ApplyPIDOutputLowPassFilter(float new_output);  // 低通滤波
    float ApplyPIDOutputMedianFilter(float new_output);   // 中值滤波
    float ApplyPIDOutputRampFilter(float new_output, float dt);  // 斜坡滤波(限制变化率)

    // 获取卡尔曼滤波参数
    float GetKalmanQ() const;
    float GetKalmanR() const;
    float GetKalmanP() const;
    float GetKalmanK() const;
    float GetKalmanX() const;

    // 设置卡尔曼滤波参数
    void SetKalmanQ(float q);
    void SetKalmanR(float r);
    void SetKalmanP(float p);
    void SetKalmanK(float k);
    void SetKalmanX(float x);
    
    // 高通滤波参数
    float highpass_alpha_ = 0.1f;  // 高通滤波系数 (0-1)，越大高频通过越多
    float prev_raw_speed_ = 0.0f;  // 上一次原始速度值
    float prev_filtered_speed_ = 0.0f;  // 上一次高通滤波后的速度值
    
    // 高通滤波方法
    float ApplyHighPassFilter(float new_speed);
};

#endif