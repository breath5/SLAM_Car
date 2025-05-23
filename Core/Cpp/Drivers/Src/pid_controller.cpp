#include "pid_controller.h"
#include <algorithm> // for std::clamp

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd),
      prev_error_(0.0f), prev_prev_error_(0.0f), prev_output_(0.0f),
      min_output_(-0.5f), max_output_(0.5f) {
}

float PIDController::Compute(float setpoint, float measurement, float dt) {
    // 计算当前误差
    float error = setpoint - measurement;
    
    // 计算增量式PID的增量
    float dp = kp_ * (error - prev_error_);                    // 比例项增量
    float di = ki_ * error * dt;                              // 积分项增量
    float dd = kd_ * (error - 2 * prev_error_ + prev_prev_error_) / dt;  // 微分项增量
    
    // 计算输出增量
    float delta_output = dp + di + dd;
    
    // 更新误差
    prev_prev_error_ = prev_error_;
    prev_error_ = error;
    
    // 计算新的输出值
    float output = prev_output_ + delta_output;
    
    // 输出限幅
    output = std::clamp(output, min_output_, max_output_);
    
    // 保存当前输出
    prev_output_ = output;
    
    return output;
}

void PIDController::Reset() {
    prev_error_ = 0.0f;
    prev_prev_error_ = 0.0f;
    prev_output_ = 0.0f;
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

void PIDController::SetKp(float kp) {
    kp_ = kp;
}
void PIDController::SetKi(float ki) {
    ki_ = ki;
}
void PIDController::SetKd(float kd) {
    kd_ = kd;
}

void PIDController::SetOutputLimits(float min_output, float max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::SetParams(float kp, float ki, float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}