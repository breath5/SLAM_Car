#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    
    // 设置PID参数
    void SetParams(float kp, float ki, float kd);
    
    // 设置输出限幅
    void SetOutputLimits(float min_output, float max_output);
    
    // 设置积分限幅，防止积分饱和
    void SetIntegralLimits(float min_integral, float max_integral);
    
    // 计算PID输出
    float Compute(float setpoint, float measurement, float dt);
    
    // 重置PID状态
    void Reset();

    // 新增：读取PID参数的声明
    float GetKp() const;  // 读取比例系数 Kp
    float GetKi() const;  // 读取积分系数 Ki
    float GetKd() const;  // 读取微分系数 Kd
    
private:
    float kp_;        // 比例系数
    float ki_;        // 积分系数
    float kd_;        // 微分系数
    
    float prev_error_; // 上一次误差
    float integral_;   // 积分项
    
    float min_output_; // 输出下限
    float max_output_; // 输出上限
    
    float min_integral_; // 积分下限
    float max_integral_; // 积分上限
};

#endif // PID_CONTROLLER_H