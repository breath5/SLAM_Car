#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    
    // 设置PID参数
    void SetParams(float kp, float ki, float kd);
    
    // 设置输出限幅
    void SetOutputLimits(float min_output, float max_output);
    
    // 计算PID输出（增量式）
    float Compute(float setpoint, float measurement, float dt);
    
    // 重置PID状态
    void Reset();

    // 读取PID参数
    float GetKp() const;
    float GetKi() const;
    float GetKd() const;

    // 单独设置PID参数
    void SetKp(float kp);
    void SetKi(float ki);
    void SetKd(float kd);

    
private:
    float kp_;        // 比例系数
    float ki_;        // 积分系数
    float kd_;        // 微分系数
    
    float prev_error_;    // 上一次误差
    float prev_prev_error_; // 上上次误差
    float prev_output_;    // 上一次输出值
    
    float min_output_; // 输出下限
    float max_output_; // 输出上限

};

#endif // PID_CONTROLLER_H