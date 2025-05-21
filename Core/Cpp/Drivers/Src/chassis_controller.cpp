#include "chassis_controller.h"

ChassisController::ChassisController(WheelMotor& left_front, WheelMotor& right_front, 
                                     WheelMotor& left_rear, WheelMotor& right_rear)
    : left_front_(left_front), right_front_(right_front), 
      left_rear_(left_rear), right_rear_(right_rear),
      vx_(0.0f), vy_(0.0f), omega_(0.0f),
      wheel_base_x_(0.1f), wheel_base_y_(0.1f) { // 根据实际车架尺寸调整
}

void ChassisController::SetVelocity(float vx, float vy, float omega) {
    vx_ = vx;
    vy_ = vy;
    omega_ = omega;
    
    // 根据麦轮布局，可能需要调整符号组合，建议验证坐标系方向
    // 麦轮运动学逆解，计算每个轮子的速度
    float v_lf = vy + vx - omega * (wheel_base_x_ + wheel_base_y_); // 左前
    float v_rf = vy - vx + omega * (wheel_base_x_ + wheel_base_y_); // 右前
    float v_lr = vy - vx - omega * (wheel_base_x_ + wheel_base_y_); // 左后
    float v_rr = vy + vx + omega * (wheel_base_x_ + wheel_base_y_); // 右后
    
    // 设置每个轮子的目标速度
    left_front_.SetTargetSpeed(v_lf);
    right_front_.SetTargetSpeed(v_rf);
    left_rear_.SetTargetSpeed(v_lr);
    right_rear_.SetTargetSpeed(v_rr);
}

void ChassisController::MoveForward(float speed) {
    SetVelocity(0.0f, speed, 0.0f); // (vx, vy, ω)=(0,+,0)
}

void ChassisController::MoveBackward(float speed) {
    SetVelocity(0.0f, -speed, 0.0f); // (0,-,0)
}

void ChassisController::MoveLeft(float speed) {
    SetVelocity(-speed, 0.0f, 0.0f); // (-,0,0)
}

void ChassisController::MoveRight(float speed) {
    SetVelocity(speed, 0.0f, 0.0f); // (+,0,0)
}

void ChassisController::Rotate(float angular_speed) {
    SetVelocity(0.0f, 0.0f, angular_speed);  // (0,0,+/-)
}

void ChassisController::Stop() {
    SetVelocity(0.0f, 0.0f, 0.0f);  // (0,0,0)
}

void ChassisController::Update(float dt) {
     // 建议增加dt有效性检查
     if(dt <= 0 || dt > 0.1f) return;
    // 更新每个轮子的速度控制
    left_front_.UpdateSpeedControl(dt);
    right_front_.UpdateSpeedControl(dt);
    left_rear_.UpdateSpeedControl(dt);
    right_rear_.UpdateSpeedControl(dt);
}