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
      sample_time_(0.01f), pid_controller_(0.5f, 0.1f, 0.01f),
      filter_index(0), speed_buffer_index_(0), filtered_speed_(0.0f),
      kalman_q_(0.05f), kalman_r_(0.3f), kalman_p_(0.5f), kalman_k_(0.0f), kalman_x_(0.0f),
      pid_output_buffer_index_(0), filtered_pid_output_(0.0f), prev_filtered_pid_output_(0.0f) {
    std::fill(std::begin(filter_buffer), std::end(filter_buffer), 0);
    std::fill(speed_buffer_, speed_buffer_ + SPEED_FILTER_SIZE, 0.0f);
    std::fill(pid_output_buffer_, pid_output_buffer_ + PID_OUTPUT_FILTER_SIZE, 0.0f);
    
    // 设置PID限制
    pid_controller_.SetOutputLimits(-1.0f, 1.0f);
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
    int32_t count_diff = std::abs(current_count_ - last_count_);
    last_count_ = current_count_;
    
    // 使用去极值平均滤波处理编码器计数
    int32_t filtered_count_diff = ApplyFilter(count_diff);
    filtered_count_diff *= static_cast<int>(direction_);
  
    // 计算原始速度
    float motor_shaft_revolutions = (float)filtered_count_diff / (PULSES_PER_ROUND * 4);
    float wheel_revolutions = motor_shaft_revolutions / REDUCTION_RATIO;
    float raw_speed = wheel_revolutions * 3.1415926f * WHEEL_DN / sample_time_;
    // 多级滤波处理
    // float speed1 = ApplyMedianFilter(raw_speed);     // 中值滤波
    // float speed2 = ApplyKalmanFilter(speed1);        // 卡尔曼滤波
    // linear_speed_ = ApplySpeedFilter(speed1);         // 滑动平均滤波 
    linear_speed_ = raw_speed;
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
   
    // 对PID输出进行滤波处理
    // 可以选择以下一种或多种滤波方式
    // pid_output = ApplyPIDOutputLowPassFilter(pid_output);     // 低通滤波 
    // pid_output = ApplyPIDOutputFilter(pid_output);  // 滑动平均滤波
    // pid_output = ApplyPIDOutputMedianFilter(pid_output); // 中值滤波
    // pid_output = ApplyPIDOutputRampFilter(pid_output, dt); // 斜坡滤波(限制变化率)
    
    // 非线性映射，增强小控制量时的响应灵敏度，同时平滑大控制量的变化率
    float non_linear_output = pid_output;
    
    // 方向控制
    if (non_linear_output > 0) {
        SetDirection(FORWARD);
    } else if (non_linear_output < 0) {
        SetDirection(REVERSE);
        non_linear_output = -non_linear_output;
    } else {
        SetDirection(STOP);
    }
    float min_output = 0.07f; // 最小输出值，确保能启动电机(Duty=7%)
    if (non_linear_output > 0 && non_linear_output < min_output) {
        non_linear_output = min_output;
    }

    // 获取PWM周期并设置限幅
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(pwm_htim_);
    constexpr float max_duty_ratio = 0.4f;
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
    HAL_TIM_PWM_Start(pwm_htim_, pwm_channel_);
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

void WheelMotor::SetKp(float kp) { pid_controller_.SetKp(kp); }
void WheelMotor::SetKi(float ki) { pid_controller_.SetKi(ki); }
void WheelMotor::SetKd(float kd) { pid_controller_.SetKd(kd); }

// 滑动平均滤波
float WheelMotor::ApplySpeedFilter(float new_speed) {
    speed_buffer_[speed_buffer_index_] = new_speed;
    speed_buffer_index_ = (speed_buffer_index_ + 1) % SPEED_FILTER_SIZE;
    
    float sum = 0.0f;
    float max_value = speed_buffer_[0];
    float min_value = speed_buffer_[0];
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        if(speed_buffer_[i] > max_value) {
            max_value = speed_buffer_[i];
        }
        if(speed_buffer_[i] < min_value) {
            min_value = speed_buffer_[i];
        }
        sum += speed_buffer_[i];
    }


      // 去掉最大值和最小值后求平均
      sum -= (max_value + min_value);
    return sum / (SPEED_FILTER_SIZE - 2);
}

// 卡尔曼滤波
float WheelMotor::ApplyKalmanFilter(float measurement) {
    // 预测
    kalman_p_ = kalman_p_ + kalman_q_;
    
    // 更新
    kalman_k_ = kalman_p_ / (kalman_p_ + kalman_r_);
    kalman_x_ = kalman_x_ + kalman_k_ * (measurement - kalman_x_);
    kalman_p_ = (1 - kalman_k_) * kalman_p_;
    
    return kalman_x_;
}

// 中值滤波
float WheelMotor::ApplyMedianFilter(float new_speed) {
    speed_buffer_[speed_buffer_index_] = new_speed;
    speed_buffer_index_ = (speed_buffer_index_ + 1) % SPEED_FILTER_SIZE;
    
    float temp_buffer[SPEED_FILTER_SIZE];
    std::copy(speed_buffer_, speed_buffer_ + SPEED_FILTER_SIZE, temp_buffer);
    std::sort(temp_buffer, temp_buffer + SPEED_FILTER_SIZE);
    
    return temp_buffer[SPEED_FILTER_SIZE / 2];
}

// PID输出滑动平均滤波
float WheelMotor::ApplyPIDOutputFilter(float new_output) {
    pid_output_buffer_[pid_output_buffer_index_] = new_output;
    pid_output_buffer_index_ = (pid_output_buffer_index_ + 1) % PID_OUTPUT_FILTER_SIZE;
    
    int32_t max_value = pid_output_buffer_[0];
    int32_t min_value = pid_output_buffer_[0];
    float sum = 0.0f;
    for (int i = 0; i < PID_OUTPUT_FILTER_SIZE; i++) {
        if(pid_output_buffer_[i] > max_value) {
            max_value = pid_output_buffer_[i];
        }
        if(pid_output_buffer_[i] < min_value) {
            min_value = pid_output_buffer_[i];
        }
        sum += pid_output_buffer_[i];
    }
      // 去掉最大值和最小值后求平均
      sum -= (max_value + min_value);
    return sum / (PID_OUTPUT_FILTER_SIZE - 2);
}

// PID输出低通滤波
float WheelMotor::ApplyPIDOutputLowPassFilter(float new_output) {
    // 一阶低通滤波: y(n) = α·x(n) + (1-α)·y(n-1)
    // α越小，滤波效果越强，但响应越慢
    filtered_pid_output_ = pid_output_alpha_ * new_output + (1.0f - pid_output_alpha_) * prev_filtered_pid_output_;
    prev_filtered_pid_output_ = filtered_pid_output_;
    
    return filtered_pid_output_;
}

// PID输出中值滤波
float WheelMotor::ApplyPIDOutputMedianFilter(float new_output) {
    pid_output_buffer_[pid_output_buffer_index_] = new_output;
    pid_output_buffer_index_ = (pid_output_buffer_index_ + 1) % PID_OUTPUT_FILTER_SIZE;
    
    float temp_buffer[PID_OUTPUT_FILTER_SIZE];
    std::copy(pid_output_buffer_, pid_output_buffer_ + PID_OUTPUT_FILTER_SIZE, temp_buffer);
    std::sort(temp_buffer, temp_buffer + PID_OUTPUT_FILTER_SIZE);
    
    return temp_buffer[PID_OUTPUT_FILTER_SIZE / 2];
}

// PID输出斜坡滤波(限制变化率)
float WheelMotor::ApplyPIDOutputRampFilter(float new_output, float dt) {
    // 限制每次输出变化的最大幅度
    const float max_change_rate = 0.5f;  // 每秒最大变化率
    float max_change = max_change_rate * dt;  // 当前周期最大允许变化量
    
    float output_change = new_output - prev_filtered_pid_output_;
    if (std::abs(output_change) > max_change) {
        // 如果变化太大，限制变化量
        output_change = std::copysign(max_change, output_change);
    }
    
    filtered_pid_output_ = prev_filtered_pid_output_ + output_change;
    prev_filtered_pid_output_ = filtered_pid_output_;
    
    return filtered_pid_output_;
}

// 获取卡尔曼滤波参数
float WheelMotor::GetKalmanQ() const { return kalman_q_; }
float WheelMotor::GetKalmanR() const { return kalman_r_; }
float WheelMotor::GetKalmanP() const { return kalman_p_; }
float WheelMotor::GetKalmanK() const { return kalman_k_; }
float WheelMotor::GetKalmanX() const { return kalman_x_; }

// 设置卡尔曼滤波参数
void WheelMotor::SetKalmanQ(float q) { kalman_q_ = q; }
void WheelMotor::SetKalmanR(float r) { kalman_r_ = r; }
void WheelMotor::SetKalmanP(float p) { kalman_p_ = p; }
void WheelMotor::SetKalmanK(float k) { kalman_k_ = k; }
void WheelMotor::SetKalmanX(float x) { kalman_x_ = x; }