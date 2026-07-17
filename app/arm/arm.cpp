//
// Created by guizi on 2026/7/3.
//

#include "arm.h"
#include "arm_math_types.h"
#include "tim.h"
#include "motor/dji.h"
#include "controller/pid.h"
#include "utils/vofa.h"
#include "bsp/buzzer.h"
#include "bsp/time.h"

using namespace controller;

// ---------- 电机与 PID 定义 ----------
motor::dji M_YALL("motor_YALL", motor::dji::GM6020,
                  motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT});
motor::dji M_LIFTER("motor_FILTER", motor::dji::M3508,
                    motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT}, -1, 1);

pid GM6020BasePID(900, 0.7, 0, 7000, 16384); //最大速度17左右
pid GM6020BasePID_position(15, 0, 0.1, 3000, 18);
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
    0.1,
    0.2f);

Motion ArmLIFTER(
    M_LIFTER,
    M3508_LifterPID,
    M3508_LifterPID_position,
    -200.0f,
    0.5f,
    5.0f,
    10,
    15,
    0.5f);

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

void lifter_manual_speed(float lifter_speed) {
    float SpeedOutput =  M3508_LifterPID.update(M_LIFTER.feedback.speed, lifter_speed);
    M_LIFTER.update(SpeedOutput);
    // vofa::send(E_UART_1, M_LIFTER.output);
}

float lifter_AimPosition = 0.0;
static bool lifter_manual_synced_ = false; // 手动模式首次同步标志，防止 lifter_AimPosition 默认值 0 导致冲目标

void lifter_set_target(float target) {
    lifter_AimPosition = target;
}

void lifter_update() {
    const float AimSpeed = M3508_LifterPID_position.update(ArmLIFTER.get_motor_total_position(), lifter_AimPosition);
    const float lifter_output = M3508_LifterPID.update(M_LIFTER.feedback.speed, AimSpeed);
    M_LIFTER.update(lifter_output);
}

void lifter_manual_position(float lifter_position) {
    if (arm_state != ArmState::IDLE)
        return;
    // 首次同步手动目标到当前实际位置
    if (!lifter_manual_synced_ && M_LIFTER.feedback.timestamp != 0) {
        lifter_AimPosition = ArmLIFTER.getCurrentPosition();
        lifter_manual_synced_ = true;
    }
    lifter_AimPosition += lifter_position;
    lifter_update();
}

void yall_manual_speed(float yall_speed) {
    float output = GM6020BasePID.update(M_YALL.feedback.speed, yall_speed);
    M_YALL.update(output);
}

float yall_AimPosition = 0.0;

bool yall_set_target(float target) {
    yall_AimPosition = target;
    return std::abs(target - ArmYALL.getCurrentPosition()) < 0.05;
}

void yall_update() {
    const float AimSpeed = GM6020BasePID_position.update(ArmYALL.get_motor_total_position(), yall_AimPosition);
    const float yall_output = GM6020BasePID.update(M_YALL.feedback.speed, AimSpeed);
    M_YALL.update(yall_output);
}

void yall_manual_position(float yall_position) {
    if (arm_state != ArmState::IDLE)
        return;
    yall_AimPosition += yall_position;
    yall_update();
}

void arm_offline_protect() {
    ArmLIFTER.motor_offline_protect();
    ArmYALL.motor_offline_protect();
}

void Reset_arm_state() {
    arm_state = ArmState::IDLE;
    ArmLIFTER.resetPID();
    ArmYALL.resetPID();
    lifter_AimPosition = ArmLIFTER.get_motor_total_position();
    yall_AimPosition = ArmYALL.get_motor_total_position();
}

void ArmDebug() {
    vofa::send(E_UART_1, M_YALL.feedback.speed, M_YALL.output);
}

// ---------- 自动装修复模块状态机 ----------

#define clamp_0 950
#define clamp_90 1600
#define arm_0 1850
#define arm_1 1500

static uint8_t loading_step = 0;
static uint32_t loading_timer = 0;

void arm_auto_load() {

    switch (arm_state) {
        case ArmState::IDLE:
            MoveArm(clamp_0, arm_0);
            break;

        case ArmState::HOMING: {
            ArmLIFTER.homeMotor();
            ArmYALL.homeMotor();

            if (ArmLIFTER.isHomed() && ArmYALL.isHomed()) {
                bsp_buzzer_flash(3000, 0.8f, 100);
                Clamp(1600); //打开夹爪
                ArmLIFTER.resetHome();
                ArmYALL.resetHome();
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                ArmLIFTER.startTrajectory(293.0);
                arm_state = ArmState::MOVE_TO_REPAIR;
            }
            break;
        }

        case ArmState::MOVE_TO_REPAIR: {
            ArmLIFTER.update(0.15);

            if (ArmLIFTER.isArrived()) {
                bsp_buzzer_flash(3000, 0.8f, 100);
                Clamp(1000); //关闭夹爪
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
            ArmLIFTER.update(0.15);

            if (ArmLIFTER.isArrived()) {
                ArmYALL.update(0.002);
                if (ArmYALL.isArrived()) {
                    bsp_buzzer_flash(3000, 0.8f, 100);
                    loading_step = 0;
                    loading_timer = bsp_time_get_ms();
                    arm_state = ArmState::LOADING_IS_OK;
                }
            }
            break;
        }

        case ArmState::LOADING_IS_OK: {
            ArmLIFTER.update(0.0f);
            ArmYALL.setPosition(ArmYALL.getCurrentPosition());

            switch (loading_step) {
            case 0:
                MoveArm(clamp_90, arm_1);
                loading_timer = bsp_time_get_ms();
                loading_step = 1;
                break;
            case 1:
                if (bsp_time_get_ms() - loading_timer >= 1000) {
                    Clamp(1600);
                    loading_timer = bsp_time_get_ms();
                    loading_step = 2;
                }
                break;
            case 2:
                if (bsp_time_get_ms() - loading_timer >= 2000) {
                    bsp_buzzer_flash(3000, 0.8f, 100);
                    loading_step = 3;
                }
                break;
            case 3:
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                arm_state = ArmState::RETURN_TO_ZERO;
                break;
            }
            break;
        }

        case ArmState::RETURN_TO_ZERO: {
            ArmYALL.homeMotor();
            ArmLIFTER.homeMotor();

            if (ArmLIFTER.isHomed() && ArmYALL.isHomed()) {
                bsp_buzzer_flash(3000, 0.8f, 100);
                ArmLIFTER.resetHome();
                ArmYALL.resetHome();
                ArmLIFTER.resetPID();
                ArmYALL.resetPID();
                arm_state = ArmState::IDLE;
            }
        }

        case ArmState::SAFE: {
            break;
        }
    }
}