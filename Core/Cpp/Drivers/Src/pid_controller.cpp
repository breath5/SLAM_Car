#include "pid_controller.h"
#include <algorithm> // for std::clamp

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd),
      prev_error_(0.0f), integral_(0.0f),
      min_output_(-1.0f), max_output_(1.0f),
      min_integral_(-1.0f), max_integral_(1.0f) {
}

void PIDController::SetParams(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::SetOutputLimits(float min_output, float max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::SetIntegralLimits(float min_integral, float max_integral) {
    min_integral_ = min_integral;
    max_integral_ = max_integral;
}

float PIDController::Compute(float setpoint, float measurement, float dt) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = kp_ * error;
    
    // 积分项
    integral_ += error * dt;
    // 积分限幅，防止积分饱和
    if (integral_ > max_integral_) integral_ = max_integral_;
    if (integral_ < min_integral_) integral_ = min_integral_;
    float i_term = ki_ * integral_;
    
    // 微分项 (使用误差的变化率)
    float derivative = (error - prev_error_) / dt;
    float d_term = kd_ * derivative;
    
    // 保存当前误差用于下次计算
    prev_error_ = error;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > max_output_) output = max_output_;
    if (output < min_output_) output = min_output_;
    
    return output;
}

void PIDController::Reset() {
    prev_error_ = 0.0f;
    integral_ = 0.0f;
}


// 新增：读取比例系数 Kp
float PIDController::GetKp() const {
    return kp_;
}

// 新增：读取积分系数 Ki
float PIDController::GetKi() const {
    return ki_;
}

// 新增：读取微分系数 Kd
float PIDController::GetKd() const {
    return kd_;
}