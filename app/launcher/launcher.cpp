//
// Created by guizi on 2026/7/2.
//

#include "launcher.h"
#include "controller/pid.h"
#include "motor/dji.h"
#include "tim.h"
#include "bsp/sys.h"
#include "utils/vofa.h"
#include "motion.h"
#include "bsp/buzzer.h"

using namespace controller;

// ---------- 电机与 PID 定义 ----------
motor::dji M_SwitchLeft("motor_SwitchLeft", motor::dji::M3508,
                        motor::dji::param_t{.id = 4, .port = E_CAN_1, .mode = motor::dji::CURRENT}, 50);
motor::dji M_SwitchRight("motor_SwitchRight", motor::dji::M3508,
                         motor::dji::param_t{.id = 3, .port = E_CAN_1, .mode = motor::dji::CURRENT}, 50);

pid M3508_SwitchLeftPID(1000, 0.3, 0, 4000, 16384);              // 左电机速度环
pid M3508_SwitchLeftPID_position(30, 0, 0, 3000, 20);           // 左电机位置环
pid M3508_SwitchRightPID(1000, 0.3, 0, 4000, 16384);             // 右电机速度环
pid M3508_SwitchRightPID_position(30, 0, 0, 3000, 20);          // 右电机位置环

// ---------- 发射机构定义 ----------
Motion LeftLauncher(
    M_SwitchLeft,
    M3508_SwitchLeftPID,
    M3508_SwitchLeftPID_position,
    -3.0f,
    1.0f,
    2.5,
    10,
    0.5);

Motion RightLauncher(
    M_SwitchRight,
    M3508_SwitchRightPID,
    M3508_SwitchRightPID_position,
    3.0f,
    1.0f,
    2.5,
    10,
    0.5);

// ---------- 发射机构电机位置 ----------
static float GM3508LeftPosition = 0.0;
static float GM3508RightPosition = 0.0;

// ---------- 状态机变量 ----------
static LauncherState launcher_state = LauncherState::IDLE;

//----------- 公开接口 ------------
void Get_3508Position() {
    GM3508LeftPosition = LeftLauncher.get_motor_total_position();
    GM3508RightPosition = RightLauncher.get_motor_total_position();
}

void launcher_init() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    M_SwitchLeft.init();
    M_SwitchRight.init();
}

void LauncherTriggerLoad() {
    if (launcher_state == LauncherState::IDLE) {
        launcher_state = LauncherState::HOMING;
        LeftLauncher.resetHome();
        RightLauncher.resetHome();
        LeftLauncher.resetPID();
        RightLauncher.resetPID();
    }
}

LauncherState launcher_get_state() {
    return launcher_state;
}

void launcher_manual_speed(float left_speed, float right_speed) {
    if (launcher_state != LauncherState::IDLE)
        return;
    M_SwitchLeft.update(
        M3508_SwitchLeftPID.update(M_SwitchLeft.feedback.speed, left_speed));
    M_SwitchRight.update(
        M3508_SwitchRightPID.update(M_SwitchRight.feedback.speed, right_speed));
}

float left_AimPosition, right_AimPosition;
void launcher_manual_position(float left_position, float right_position) {
    if (launcher_state != LauncherState::IDLE)
        return;
    left_AimPosition += left_position;
    const float left_AimSpeed = M3508_SwitchLeftPID_position.update(GM3508LeftPosition, left_AimPosition);
    const float left_output = M3508_SwitchLeftPID.update(M_SwitchLeft.feedback.speed, left_AimSpeed);
    M_SwitchLeft.update(left_output);

    right_AimPosition += right_position;
    const float right_AimSpeed = M3508_SwitchRightPID_position.update(GM3508RightPosition, right_AimPosition);
    const float right_output = M3508_SwitchRightPID.update(M_SwitchRight.feedback.speed, right_AimSpeed);
    M_SwitchRight.update(right_output);
}

void Fire(uint32_t status) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, status);
}

void Launcher_offline_protect() {
    LeftLauncher.motor_offline_protect();
    RightLauncher.motor_offline_protect();
}

// ---------- 自动上膛状态机 ----------
void launcher_auto_load() {
    // vofa::send(E_UART_1,
    //     GM3508LeftPosition,
    //     GM3508RightPosition
    //     );
    vofa::send(E_UART_1, M_SwitchLeft.output, M_SwitchRight.output, M_SwitchLeft.feedback.raw.temp, M_SwitchRight.feedback.raw.temp, M_SwitchLeft.feedback.timestamp,  M_SwitchRight.feedback.timestamp);
    switch (launcher_state) {

        case LauncherState::IDLE:
            break;

        // ---- 归零阶段：两电机各自朝限位方向转动找堵转 ----
        case LauncherState::HOMING: {
            LeftLauncher.homeMotor();
            RightLauncher.homeMotor();

            // 两电机都归零完成 → 进入上膛阶段
            if (LeftLauncher.isHomed() && RightLauncher.isHomed()) {
                bsp_buzzer_flash(1000, 0.8f, 200);
                LeftLauncher.resetHome();
                RightLauncher.resetHome();
                // 归零阶段结束，清零 PID 准备位置环
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                //设定此时位置零点
                LeftLauncher.startTrajectory(12.0);
                RightLauncher.startTrajectory(-12.0);
                launcher_state = LauncherState::IDLE;
            }
            break;
        }

        // ---- 上膛阶段：两电机从零点朝各自上膛方向转动固定距离 ----
        case LauncherState::LOADING_REVERSE: {
            // 开始反向上膛
            LeftLauncher.update(0.005f);
            RightLauncher.update(0.005f);

            // 两电机都到位后完成
            if (LeftLauncher.isArrived() && RightLauncher.isArrived())
            {
                bsp_buzzer_flash(3000, 0.8f, 200);
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_state = LauncherState::RETURN_TO_ZERO;
            }
            break;
        }

        // ---- 回零阶段：上膛完成后两电机回到零点待命 ----
        case LauncherState::RETURN_TO_ZERO: {
            LeftLauncher.homeMotor();
            RightLauncher.homeMotor();

            if (LeftLauncher.isHomed() && RightLauncher.isHomed())
            {
                bsp_buzzer_flash(1000, 0.8f, 200);
                LeftLauncher.resetHome();
                RightLauncher.resetHome();
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_state = LauncherState::IDLE;
            }
            break;
        }
        case LauncherState::SAFE: {
            //TO DO:

            break;
        }
    }
}
