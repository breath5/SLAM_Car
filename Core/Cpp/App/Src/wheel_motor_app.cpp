#include "wheel_motor_app.h"
#include "tim.h"
#include <sstream>
#include <iomanip>
#include <climits>

extern TIM_HandleTypeDef htim1;
void Speed_observation();
// 定义GPIO引脚用于方向控制
#define MOTOR_LF_DIR_PORT GPIOB
#define MOTOR_LF_DIR_PIN1 GPIO_PIN_10
#define MOTOR_LF_DIR_PIN2 GPIO_PIN_11

#define MOTOR_RF_DIR_PORT GPIOB
#define MOTOR_RF_DIR_PIN1 GPIO_PIN_0
#define MOTOR_RF_DIR_PIN2 GPIO_PIN_1

#define MOTOR_LR_DIR_PORT GPIOE
#define MOTOR_LR_DIR_PIN1 GPIO_PIN_1
#define MOTOR_LR_DIR_PIN2 GPIO_PIN_2

#define MOTOR_RR_DIR_PORT GPIOE
#define MOTOR_RR_DIR_PIN1 GPIO_PIN_3
#define MOTOR_RR_DIR_PIN2 GPIO_PIN_4


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


// 修改为全局可访问的句柄
TaskHandle_t chassis_task_handle = NULL; // 移除static修饰符

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
    // static constexpr float MAX_DT = 0.013f; // 允许最大13ms波动
    // static uint32_t prev_tick = 0;
    // uint32_t current_tick = HAL_GetTick();
    
    // // 计算时间差（毫秒转秒）
    // // printf("delta: %d\n", current_tick - prev_tick);
    // float dt = (current_tick - prev_tick) / 1000.0f;   //现在PID的积分时间为0.01s(10ms)
    // prev_tick = current_tick;
    
    //if (dt > MAX_DT) {return;}// 触发异常处理（如重置控制器）
    float dt = 0.01f;//严格周期控制。
    chassis.Update(dt);  // 底盘控制更新, 传入dt, 单位m/s, 再次函数中会更新速度（使用PID）

}

// 此任务函数主要用于速度控制
static void ChassisControlTask(void *argument) {
    // 在任务开始处添加堆栈检测
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    configASSERT(uxHighWaterMark > 10); // 确保至少有10字剩余堆栈

    constexpr TickType_t xFrequency = pdMS_TO_TICKS(11); // 与MAX_DT对应,最大阻塞时间（11ms）,不要随便修改该数值
    uint32_t notification;
    BaseType_t xResult;

    for(;;) {
        xResult = xTaskNotifyWait(0, ULONG_MAX, &notification, xFrequency);
        // 等待任务通知（带超时保护）
        if(xResult == pdPASS) {
            // printf("ChassisControlTask received notification: %x\r\n", notification);
        } else {
            // printf("ChassisControlTask timed out\r\n");
        }
        //无论如何,都要执行一次控制更新,使实际速度趋近于目标速度
        ChassisControlUpdate();
    }
    vTaskDelete(NULL);
}

static void CarMotionControlTask(void *argument) {
    // 在任务开始处添加堆栈检测
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    configASSERT(uxHighWaterMark > 10); // 确保至少有10字剩余堆栈
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(10); // 任务周期

/*******************************死区测量*******************************************/
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    float duty_cycle = 0.0f;
    const float duty_step = 0.01f; // 每次增加1%
/*******************************************************************************/

    for(;;) {
                                            // // 设置目标速度
                                            // chassis.SetVelocity(0.2f, 0.0f, 0.1f); // 例如：设置前进速度为0.2m/s
                                            // // 小车旋转
                                            // chassis.Rotate(0.1f); // 例如：以0.1rad/s的速度旋转
                                            // // 小车平移
                                            // chassis.MoveRight(0.1f); // 例如：以0.1m/s的速度右移
                                            // chassis.MoveForward(0.1f); // 以0.1m/s的速度前进

/*******************************************************************************/
        Speed_observation();   //打印出速度
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // vTaskDelay(xFrequency); // 延时等待下一个周期,不要随便修改周期,要与定时器中断周期一致
    }
    vTaskDelete(NULL);
}

void FourWheelMotorApp() {
    // 初始化四个轮子
    left_front.Init();
    right_front.Init();
    left_rear.Init();
    right_rear.Init();

    // 设置PID参数（根据实际调试调整）
    left_front.SetPIDParams(0.5f, 0.1f, 0.01f);  //3.9f, 0.001f, 0.0003f
    right_front.SetPIDParams(0.5f, 0.1f, 0.01f);  //0.262f, 0.0001f, 0.001f
    left_rear.SetPIDParams(0.5f, 0.1f, 0.01f);  //0.262f, 0.0001f, 0.001f
    right_rear.SetPIDParams(0.5f, 0.1f, 0.01f);  //0.798f, 0.0001f, 0.002f

    // m/s :0.2f 每秒前进20厘米
    left_front.SetTargetSpeed(0.2f);
    right_front.SetTargetSpeed(0.2f);
    left_rear.SetTargetSpeed(0.2f);
    right_rear.SetTargetSpeed(0.2f);

    // left_front.SetLinearSpeed(0.2f);
    // right_front.SetLinearSpeed(0.2f);
    // left_rear.SetLinearSpeed(0.2f);
    // right_rear.SetLinearSpeed(0.2f);


    xTaskCreate(ChassisControlTask,
              "ChassisCtrl",
              512,//configMINIMAL_STACK_SIZE * 4,
              NULL,
              tskIDLE_PRIORITY + 12,  // 较高优先级
              &chassis_task_handle);
    printf("xTaskCreate:ChassisControlTask created; uxPriority:10; usStackDepth:512\r\n");

    xTaskCreate(CarMotionControlTask,
        "CarMotionCtrl",
        512, // 堆栈大小
        NULL,
        tskIDLE_PRIORITY + 10,  // 优先级
        NULL);
    printf("xTaskCreate:CarMotionControlTask created;  uxPriority:10; usStackDepth:512\r\n");

}


void Speed_observation() {
    // 打印速度
    printf("%d,", (int32_t)(left_front.GetLinearSpeed() * 1000));
    printf("%d,", (int32_t)(right_front.GetLinearSpeed() * 1000));
    printf("%d,", (int32_t)(left_rear.GetLinearSpeed() * 1000));
    printf("%d\r\n", (int32_t)(right_rear.GetLinearSpeed() * 1000));
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