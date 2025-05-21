#include "wheel_motor.h"
#include <cmath>
#include <string>
#include <algorithm>  // 添加此头文件

WheelMotor::WheelMotor(int id, TIM_HandleTypeDef* pwm_htim, uint32_t pwm_channel, 
                       TIM_HandleTypeDef* enc_htim, 
                       GPIO_TypeDef* dir_port1, uint16_t dir_pin1,
                       GPIO_TypeDef* dir_port2, uint16_t dir_pin2)
    : id_(id), pwm_htim_(pwm_htim), pwm_channel_(pwm_channel), 
      enc_htim_(enc_htim), 
      dir_port1_(dir_port1), dir_pin1_(dir_pin1),
      dir_port2_(dir_port2), dir_pin2_(dir_pin2),
      direction_(STOP), last_count_(0), current_count_(0),
      pwm_value_(0), linear_speed_(0.0f), target_speed_(0.0f),
      sample_time_(0.1f), pid_controller_(0.5f, 0.1f, 0.01f),
      filter_index(0) {
    std::fill(std::begin(filter_buffer), std::end(filter_buffer), 0);
    
    // 设置PID限制
    pid_controller_.SetOutputLimits(-1.0f, 1.0f);
    pid_controller_.SetIntegralLimits(-0.5f, 0.5f);
}

void WheelMotor::SetDutyCycle(int pwm_value) {
    pwm_value_ = pwm_value;
    if(pwm_htim_ != nullptr) {
        __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value_);
    }
}

int WheelMotor::GetDutyCycle() const { return pwm_value_; }

void WheelMotor::UpdateEncoder() {
    current_count_ = __HAL_TIM_GET_COUNTER(enc_htim_);
}

void WheelMotor::SetSampleTime(float sample_time) {
    sample_time_ = sample_time;
}

float WheelMotor::GetSampleTime() const {
    return sample_time_;
}

float WheelMotor::CalculateLinearSpeed() {
    int32_t count_diff = current_count_ - last_count_;
    last_count_ = current_count_;
    // 使用滤波器
    int32_t filtered_count_diff = ApplyFilter(count_diff);
    // printf("count_diff=%d\n", filtered_count_diff);
    // 考虑方向
    filtered_count_diff *= static_cast<int>(direction_);
    float motor_shaft_revolutions = (float)filtered_count_diff / (PULSES_PER_ROUND * 4);
    float wheel_revolutions = motor_shaft_revolutions / REDUCTION_RATIO;
    linear_speed_ = wheel_revolutions * 2.0f * 3.1415926f * WHEEL_RADIUS / sample_time_;
    int32_t temp = linear_speed_ * 1000;
    // printf("linear_speed=%d\n", temp);
    return linear_speed_;
}

float WheelMotor::GetLinearSpeed() const { return linear_speed_; }

int WheelMotor::GetID() const { return id_; }

void WheelMotor::SetDirection(Direction dir) {
    direction_ = dir;
    
    // 如果有方向控制引脚
    if (dir_port1_ != nullptr && dir_port2_ != nullptr) {
        if (dir == FORWARD) {
            // 正转：引脚1高，引脚2低
            HAL_GPIO_WritePin(dir_port1_, dir_pin1_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(dir_port2_, dir_pin2_, GPIO_PIN_RESET);
        } else if (dir == REVERSE) {
            // 反转：引脚1低，引脚2高
            HAL_GPIO_WritePin(dir_port1_, dir_pin1_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(dir_port2_, dir_pin2_, GPIO_PIN_SET);
        } else { // STOP
            // 停止：两个引脚都低
            HAL_GPIO_WritePin(dir_port1_, dir_pin1_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(dir_port2_, dir_pin2_, GPIO_PIN_RESET);
        }
    }
}

WheelMotor::Direction WheelMotor::GetDirection() const {
    return direction_;
}

void WheelMotor::SetLinearSpeed(float target_speed) {
    // 设置目标速度和方向
    if (target_speed > 0) {
        SetDirection(FORWARD);
    } else if (target_speed < 0) {
        SetDirection(REVERSE);
        target_speed = -target_speed; // 取绝对值用于计算
    } else {
        SetDirection(STOP);
    }
    
    float target_rps = target_speed / (2.0f * 3.1415926f * WHEEL_RADIUS);
    // float motor_rps = target_rps * REDUCTION_RATIO;
    float motor_rpm = target_rps * 60.0f;
    // 使用类成员变量max_rpm_替代局部变量
    float duty = motor_rpm / max_rpm_;
    
    if(duty > 1.0f) duty = 1.0f;
    if(duty < 0.0f) duty = 0.0f; // 只用正值，方向由SetDirection控制
    
    uint32_t pwm_period = 0;
    if(pwm_htim_ != nullptr) {
        pwm_period = __HAL_TIM_GET_AUTORELOAD(pwm_htim_);
    }
    
    int pwm_value = (int)(duty * pwm_period);
    pwm_value_ = pwm_value;
    
    if(pwm_htim_ != nullptr) {
        __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value_);
    }
}

void WheelMotor::SetTargetSpeed(float target_speed) {
    target_speed_ = target_speed;
}

void WheelMotor::UpdateSpeedControl(float dt) {
    // 计算当前速度
    float current_speed = CalculateLinearSpeed();
    
    // 使用PID控制器计算输出
    float pid_output = pid_controller_.Compute(target_speed_, current_speed, dt);
    
    // 死区补偿处理
    if (std::abs(pid_output) < deadzone_threshold_) {
        pid_output = 0;
    } else {
        pid_output = (pid_output - std::copysign(deadzone_threshold_, pid_output)) 
                   / (1.0f - deadzone_threshold_);
    }

    // 非线性映射，增强小控制量时的响应灵敏度，同时平滑大控制量的变化率，实际效果：改善电机低速线性度，防止高速区突变。
    float non_linear_output = std::copysign(
        std::pow(std::abs(pid_output), 0.7f), 
        pid_output
    );

    // 方向控制
    if (non_linear_output > 0) {
        SetDirection(FORWARD);
    } else if (non_linear_output < 0) {
        SetDirection(REVERSE);
        non_linear_output = -non_linear_output;
    } else {
        SetDirection(STOP);
    }

    // 获取PWM周期并设置限幅
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(pwm_htim_);
    constexpr float max_duty_ratio = 0.8f;
    // 最终确保PWM占空比在[0%, max_duty_ratio%]的安全范围内
    float clamped_output = std::clamp(non_linear_output, 0.0f, max_duty_ratio);
    
    // 设置PWM占空比
    SetDutyCycle(static_cast<int>(clamped_output * pwm_period));
    //原始PID输出 → 死区补偿 → 非线性映射 → 输出限幅 → 最终PWM生成
}

void WheelMotor::SetPIDParams(float kp, float ki, float kd) {
    pid_controller_.SetParams(kp, ki, kd);
}

void WheelMotor::ResetEncoder() {
    int32_t current = __HAL_TIM_GET_COUNTER(enc_htim_); // 先获取当前值
    last_count_ = current_count_; // 保存最后一次有效计数
    __HAL_TIM_SET_COUNTER(enc_htim_, 0);
    current_count_ = 0; // 重置后当前计数归零
}

void WheelMotor::Init() {
    HAL_TIM_Encoder_Start_IT(enc_htim_, TIM_CHANNEL_ALL);
    current_count_ = 0;
    last_count_ = 0;
    
    // 初始化方向控制引脚
    if (dir_port1_ != nullptr && dir_port2_ != nullptr) {
        // 假设GPIO已经在其他地方初始化
        SetDirection(STOP);
    }
}

// 去极值平均滤波
int32_t WheelMotor::ApplyFilter(int32_t new_value) {
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    int32_t max_value = filter_buffer[0];
    int32_t min_value = filter_buffer[0];
    int32_t sum = 0;

    for (int i = 0; i < FILTER_SIZE; i++) {
        if (filter_buffer[i] > max_value) {
            max_value = filter_buffer[i];
        }
        if (filter_buffer[i] < min_value) {
            min_value = filter_buffer[i];
        }
        sum += filter_buffer[i];
    }

    // 去掉最大值和最小值后求平均
    sum -= (max_value + min_value);
    return sum / (FILTER_SIZE - 2);
}

// 新增：读取 PID 参数的实现
float WheelMotor::GetKp() const {
    return pid_controller_.GetKp();
}

float WheelMotor::GetKi() const {
    return pid_controller_.GetKi();
}

float WheelMotor::GetKd() const {
    return pid_controller_.GetKd();
}