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
#include "bsp/time.h"

using namespace controller;

// ---------- 电机与 PID 定义 ----------
motor::dji M_YALL("motor_YALL", motor::dji::GM6020,
                  motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT});
motor::dji M_LIFTER("motor_FILTER", motor::dji::M3508,
                    motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT}, -1, 1);

pid GM6020BasePID(650, 0.7, 0, 7000, 16384); //最大速度17左右
pid GM6020BasePID_position(30, 0, 0.1, 3000, 10000);
pid M3508_LifterPID(150, 0, 0, 4000, 16384);
pid M3508_LifterPID_position(15, 0, 0, 3000, 400);

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
    0.05,
    0.65f);

Motion ArmLIFTER(
    M_LIFTER,
    M3508_LifterPID,
    M3508_LifterPID_position,
    -200.0f,
    0.5f,
    5.0f,
    10,
    1.5,
    0.1f);

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

void arm_init() {
    M_YALL.init();
    M_LIFTER.init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void ArmTriggerLoad() {
    if (arm_state == ArmState::LOADING_IS_OK || arm_state == ArmState::IDLE) {
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

void lifter_manual_speed(float lifter_speed) {
    float SpeedOutput =  M3508_LifterPID.update(M_LIFTER.feedback.speed, lifter_speed);
    M_LIFTER.update(SpeedOutput);
    // vofa::send(E_UART_1, M_LIFTER.output);
}

float lifter_AimPosition = 0.0;
void lifter_manual_position(float lifter_position) {
    if (arm_state != ArmState::IDLE)
        return;
    lifter_AimPosition += lifter_position;
    const float AimSpeed = M3508_LifterPID_position.update(ArmLIFTER.getCurrentPosition(), lifter_AimPosition);
    const float lifter_output = M3508_LifterPID.update(M_LIFTER.feedback.speed, AimSpeed);
    // vofa::send(E_UART_1,
    //     lifter_AimPosition, ArmLIFTER.getCurrentPosition(), AimSpeed, lifter_output, arm_state);
    M_LIFTER.update(lifter_output);
}

void yall_manual_speed(float yall_speed) {
    float output = GM6020BasePID.update(M_YALL.feedback.speed, yall_speed);
    M_YALL.update(output);
}

float yall_AimPosition = 0.0;
void yall_manual_position(float yall_position) {
    yall_AimPosition += yall_position;
    const float AimSpeed = GM6020BasePID_position.update(ArmYALL.getCurrentPosition(), yall_AimPosition);
    const float yall_output = GM6020BasePID.update(M_YALL.feedback.speed, AimSpeed);
    M_YALL.update(yall_output);
    // vofa::send(E_UART_1,
    //     yall_AimPosition, ArmYALL.getCurrentPosition(), AimSpeed, yall_output);
}

void arm_offline_protect() {
    ArmLIFTER.motor_offline_protect();
    ArmYALL.motor_offline_protect();
}

void Reset_arm_state() {
    arm_state = ArmState::IDLE;
    ArmLIFTER.resetPID();
    ArmYALL.resetPID();
    lifter_AimPosition = ArmLIFTER.getCurrentPosition();
    yall_AimPosition = ArmYALL.getCurrentPosition();
}

// ---------- 自动装修复模块状态机 ----------

#define clamp_0 950
#define clamp_90 1600
#define arm_0 1850
#define arm_1 1500

void arm_auto_load() {

    // vofa::send(E_UART_1, ArmYALL.getCurrentPosition(), ArmYALL.target_position_,
    //                             ArmLIFTER.getCurrentPosition(), ArmLIFTER.target_position_,
    //                             arm_state, M_LIFTER.feedback.speed);

    switch (arm_state) {
        case ArmState::IDLE:
            ArmYALL.update(0.0f);
            break;

        case ArmState::HOMING: {
            ArmLIFTER.homeMotor();
            ArmYALL.homeMotor();

            if (ArmLIFTER.isHomed() && ArmYALL.isHomed()) {
                bsp_buzzer_flash(1000, 0.8f, 200);
                Clamp(1400); //打开夹爪
                ArmLIFTER.resetHome();
                ArmYALL.resetHome();
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                ArmLIFTER.startTrajectory(300.0);
                ArmYALL.startTrajectory(0.0);
                arm_state = ArmState::MOVE_TO_REPAIR;
            }
            break;
        }

        case ArmState::MOVE_TO_REPAIR: {
            ArmLIFTER.update(0.3);
            ArmYALL.update(0.0f);
            MoveArm(clamp_0, arm_0);

            if (ArmLIFTER.isArrived()) {
                bsp_buzzer_flash(1000, 0.8f, 200);
                Clamp(1200); //关闭夹爪
                bsp_time_delay(300);
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                ArmLIFTER.startTrajectory(0.0);
                ArmYALL.startTrajectory(-M_PI);
                arm_state = ArmState::MOVE_TO_LOAD;
            }
            break;
        }

        case ArmState::MOVE_TO_LOAD: {
            ArmLIFTER.update(0.3);
            ArmYALL.update(0.01);

            if (ArmLIFTER.isArrived() && ArmYALL.isArrived()) {
                bsp_time_delay(1000);
                bsp_buzzer_flash(1000, 0.8f, 200);
                MoveArm(clamp_90, arm_1);
                Clamp(1400);    //打开夹爪
                bsp_time_delay(1000);
                arm_state = ArmState::LOADING_IS_OK;
            }
            break;
        }

        case ArmState::LOADING_IS_OK: {
            ArmLIFTER.update(0.0f);
            ArmYALL.update(0.0f);
            break;
        }

        case ArmState::SAFE: {
            break;
        }
    }
}