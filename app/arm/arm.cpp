//
// Created by guizi on 2026/7/3.
//

#include "arm.h"
#include "arm_math_types.h"
#include "tim.h"
#include "motor/dji.h"
#include "controller/pid.h"
#include "utils/vofa.h"
#include "motion.h"
#include "bsp/buzzer.h"

using namespace controller;

// ---------- 电机与 PID 定义 ----------
motor::dji M_YALL("motor_YALL", motor::dji::GM6020,
                  motor::dji::param_t{.id = 2, .port = E_CAN_1, .mode = motor::dji::CURRENT});
motor::dji M_LIFTER("motor_FILTER", motor::dji::M3508,
                    motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT}, -1, 1);

pid GM6020BasePID(650, 0.7, 0, 7000, 16384); //最大速度17左右
pid GM6020BasePID_position(5, 0, 0, 3000, 10000);
pid M3508_LifterPID(150, 0, 0, 4000, 16384);
pid M3508_LifterPID_position(10, 0, 0, 3000, 400);

static float M3508Position = 0.0;
static float GM6020Position = 0.0;

// ----------- 机械臂定义 -------------
Motion ArmYALL(
    M_YALL,
    GM6020BasePID,
    GM6020BasePID_position,
    4.0f,
    0.5f,
    1.0,
    10,
    0.1);

Motion ArmLIFTER(
    M_LIFTER,
    M3508_LifterPID,
    M3508_LifterPID_position,
    -100.0f,
    0.5f,
    5.0f,
    10,
    0.1);

// ---------- 状态机变量 ----------
static ArmState arm_state = ArmState::IDLE;

//----------- 公开接口 ------------
void Clamp(uint32_t status) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, status);
}

void MoveArm(uint32_t ClampRoll_T2C3, uint32_t ClampPitch_T2C1) {
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, ClampRoll_T2C3);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, ClampPitch_T2C1);
}

void Get_MotorPosition() {
    M3508Position = ArmLIFTER.get_motor_total_position();
    GM6020Position = ArmYALL.get_motor_total_position();
}

float lifter_AimPosition = 0.0;
void lifter_manual_position(float lifter_position) {
    lifter_AimPosition += lifter_position;
    const float AimSpeed = M3508_LifterPID_position.update( M3508Position, lifter_AimPosition);
    const float lifter_output = M3508_LifterPID.update(M_LIFTER.feedback.speed, AimSpeed);
    vofa::send(E_UART_1, lifter_AimPosition, M3508Position, AimSpeed, lifter_output);
    // M_LIFTER.update(lifter_output);
}

void lifter_manual_speed(float lifter_speed) {
    float SpeedOutput =  M3508_LifterPID.update(M_LIFTER.feedback.speed, lifter_speed);
    // vofa::send(E_UART_1, M_LIFTER.feedback.timestamp, static_cast<float>(M_LIFTER.feedback.round) * 2.0f * static_cast<float>(M_PI)+ M_LIFTER.feedback.angle,
    //     lifter_speed, M_LIFTER.feedback.speed, SpeedOutput);
    M_LIFTER.update(SpeedOutput);
}

void yall_manual_speed(float yall_speed) {
    float output = GM6020BasePID.update(M_YALL.feedback.speed, yall_speed);
    M_YALL.update(output);
}

void arm_offline_protect() {
    ArmLIFTER.motor_offline_protect();
    ArmYALL.motor_offline_protect();
}

void arm_init() {
    M_YALL.init();
    M_LIFTER.init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void ArmTriggerLoad() {
    if (arm_state == ArmState::IDLE) {
        arm_state = ArmState::HOMING;
        ArmLIFTER.resetHome();
        ArmLIFTER.resetPID();
        ArmYALL.resetHome();
        ArmYALL.resetPID();
    }
}

ArmState arm_get_state() {
    return arm_state;
}

// ---------- 自动装修复模块状态机 ----------
void arm_auto_load() {

    // vofa::send(E_UART_1,
    //     arm_state,+
    //     M_LIFTER.feedback.current, M_LIFTER.feedback.speed,
    //     M_YALL.feedback.current, M_YALL.feedback.speed);

    switch (arm_state) {
        case ArmState::IDLE:
            break;

        case ArmState::HOMING: {
            ArmLIFTER.homeMotor();
            ArmYALL.homeMotor();

            if (ArmLIFTER.isHomed() && ArmYALL.isHomed()) {
                // Clamp(111); //打开夹爪
                bsp_buzzer_flash(1000, 0.8f, 200);
                ArmLIFTER.resetHome();
                ArmYALL.resetHome();
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                // ArmLIFTER.startTrajectory(1.0);
                ArmYALL.startTrajectory(0.0);
                arm_state = ArmState::IDLE;
            }
            break;
        }

        case ArmState::MOVE_TO_REPAIR: {
            ArmLIFTER.update(0.00125);
            ArmYALL.update(0.00125);

            if (ArmLIFTER.isArrived() && ArmYALL.isArrived()) {
                Clamp(222); //关闭夹爪
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                ArmLIFTER.startTrajectory(1.0);
                ArmYALL.startTrajectory(1.0);
                arm_state = ArmState::MOVE_TO_LOAD;
            }
            break;
        }

        case ArmState::MOVE_TO_LOAD: {
            ArmLIFTER.update(0.00125);
            ArmYALL.update(0.00125);

            if (ArmLIFTER.isArrived() && ArmYALL.isArrived()) {
                Clamp(111); //打开夹爪
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                arm_state = ArmState::RETURN_HOME;
            }
            break;
        }

        case ArmState::RETURN_HOME: {
            ArmLIFTER.homeMotor();
            ArmYALL.homeMotor();

            if (ArmLIFTER.isHomed() && ArmYALL.isHomed()) {
                ArmLIFTER.resetHome();
                ArmYALL.resetHome();
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                arm_state = ArmState::IDLE;
            }
            break;
        }

        case ArmState::SAFE: {
            break;
        }
    }
}