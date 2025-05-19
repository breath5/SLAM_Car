#include "wheel_motor_app.h"
#include "tim.h"
#include <sstream>
#include <iomanip>

// 定义GPIO引脚用于方向控制（根据实际硬件连接修改）
#define MOTOR_DIR_PORT GPIOE

#define MOTOR_LF_DIR_PORT GPIOB
#define MOTOR_LF_DIR_PIN1 GPIO_PIN_10
#define MOTOR_LF_DIR_PIN2 GPIO_PIN_11

#define MOTOR_RF_DIR_PORT GPIOB
#define MOTOR_RF_DIR_PIN1 GPIO_PIN_0
#define MOTOR_RF_DIR_PIN2 GPIO_PIN_1

#define MOTOR_LR_DIR_PORT GPIOD
#define MOTOR_LR_DIR_PIN1 GPIO_PIN_14
#define MOTOR_LR_DIR_PIN2 GPIO_PIN_15

#define MOTOR_RR_DIR_PORT GPIOE
#define MOTOR_RR_DIR_PIN1 GPIO_PIN_2
#define MOTOR_RR_DIR_PIN2 GPIO_PIN_3

// 创建四个轮子对象
WheelMotor left_front(1, &htim1, TIM_CHANNEL_1, &htim2, 
                     MOTOR_LF_DIR_PORT, MOTOR_LF_DIR_PIN1, 
                     MOTOR_LF_DIR_PORT, MOTOR_LF_DIR_PIN2);
WheelMotor right_front(2, &htim1, TIM_CHANNEL_2, &htim3, 
    MOTOR_RF_DIR_PORT, MOTOR_RF_DIR_PIN1, 
    MOTOR_RF_DIR_PORT, MOTOR_RF_DIR_PIN2);
WheelMotor left_rear(3, &htim1, TIM_CHANNEL_3, &htim4, 
    MOTOR_LR_DIR_PORT, MOTOR_LR_DIR_PIN1, 
    MOTOR_LR_DIR_PORT, MOTOR_LR_DIR_PIN2);
WheelMotor right_rear(4, &htim1, TIM_CHANNEL_4, &htim5, 
    MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN1, 
    MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN2);

// 创建底盘控制器
ChassisController chassis(left_front, right_front, left_rear, right_rear);

// 定时器回调，用于更新编码器计数
void FourWheelInterruptCountReset() {
    left_front.UpdateEncoder();
    right_front.UpdateEncoder();
    left_rear.UpdateEncoder();
    right_rear.UpdateEncoder();
}

// 定时器回调，用于更新底盘控制  在HAL_TIM_PeriodElapsedCallback中调用
/*
* @brief 动态计算 dt,适应系统延迟,但时间波动影响控制稳定性
* @param None
* @retval None
* @note 底盘控制更新
*    
*/
void ChassisControlUpdate() {
    static constexpr float MAX_DT = 0.015f; // 允许最大15ms波动
    static uint32_t prev_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    
    // 计算时间差（毫秒转秒）
    float dt = (current_tick - prev_tick) / 1000.0f;
    prev_tick = current_tick;
    
    if (dt > MAX_DT) {
        // 触发异常处理（如重置控制器）
        return;
    }
    chassis.Update(dt);  // 底盘控制更新, 传入dt, 单位m/s, 再次函数中会更新速度（使用PID）
}

void FourWheelMotorApp() {
    // 初始化四个轮子
    left_front.Init();
    right_front.Init();
    left_rear.Init();
    right_rear.Init();
    
    // 设置PID参数（根据实际调试调整）
    left_front.SetPIDParams(0.5f, 0.1f, 0.01f);
    right_front.SetPIDParams(0.5f, 0.1f, 0.01f);
    left_rear.SetPIDParams(0.5f, 0.1f, 0.01f);
    right_rear.SetPIDParams(0.5f, 0.1f, 0.01f);
    
    // m/s :0.3f 每秒前进30厘米
    // left_front.SetTargetSpeed(0.1f);
    left_front.SetLinearSpeed(0.3f); 
    int ID = left_front.GetID();
    printf("left_front:ID: %d\n", ID);

    
    // 主循环
    while (1) {
        // 可以在这里添加其他控制逻辑
        // 读取编码器计数并计算速度
        left_front.CalculateLinearSpeed();
        
        HAL_Delay(100);
    }
}

/*
 * @brief 底盘控制示例
 * @param None
 * @retval None
 * @note 底盘控制示例
 *     // 示例：前进一段时间，然后停止
    chassis.MoveForward(0.1f); // 以0.1m/s的速度前进
    HAL_Delay(2000);           // 前进2秒
    
    chassis.MoveRight(0.1f);   // 以0.1m/s的速度右移
    HAL_Delay(2000);           // 右移2秒
    
    chassis.Rotate(0.5f);      // 以0.5rad/s的速度顺时针旋转
    HAL_Delay(2000);           // 旋转2秒
    
    chassis.Stop();            // 停止
 */