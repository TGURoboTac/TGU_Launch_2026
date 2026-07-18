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
        ArmLIFTER.manual_reset_sync();
        ArmYALL.manual_reset_sync();
    }
}

ArmState arm_get_state() {
    return arm_state;
}

void lifter_manual_speed(float lifter_speed) {
    ArmLIFTER.manual_speed_update(lifter_speed);
}

void lifter_manual_position(float lifter_position) {
    if (arm_state != ArmState::IDLE)
        return;
    ArmLIFTER.manual_delta_position(lifter_position);
    ArmLIFTER.manual_position_update();
}

void yall_manual_speed(float yall_speed) {
    ArmYALL.manual_speed_update(yall_speed);
}

bool yall_set_position(float target) {
    ArmYALL.manual_set_position(target);
    return std::abs(target - ArmYALL.getCurrentPosition()) < 0.05;
}

void yall_manual_position(float yall_position) {
    if (arm_state != ArmState::IDLE)
        return;
    ArmYALL.manual_delta_position(yall_position);
    ArmYALL.manual_position_update();
}

void arm_offline_protect() {
    ArmLIFTER.motor_offline_protect();
    ArmYALL.motor_offline_protect();
}

void Reset_arm_state() {
    arm_state = ArmState::IDLE;
    ArmLIFTER.resetPID();
    ArmYALL.resetPID();
}

void arm_manual_sync() {
    ArmLIFTER.manual_sync_position();
    ArmYALL.manual_sync_position();
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
            ArmYALL.setTarget(ArmYALL.getCurrentPosition());

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