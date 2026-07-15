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
#include "bsp/time.h"
#include "arm.h"

using namespace controller;

// ---------- 双电机电流均衡 PID ----------
// 两电机通过同步带机械耦合，运动方向相反。
// I_left + I_right ≈ 0 表示负载均匀分配。
// 该 PID 以电流差为误差，输出位置偏置来重新分配负载。
// 同时兼容自动状态机 (LOADING_REVERSE) 和遥控器手动位置模式。
static pid current_balance_pid(0.02f, 0.01f, 0.0f, 0.3f, 0.5f);

static float launcher_compute_balance();

void launcher_balance_reset() {
    current_balance_pid.clear();
}

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
static float M3508LeftPosition = 0.0;
static float M3508RightPosition = 0.0;

static float launcher_compute_balance() {
    float current_error = M_SwitchLeft.feedback.current + M_SwitchRight.feedback.current;
    return current_balance_pid.update(current_error, 0.0f);
}

// ---------- 状态机变量 ----------
static LauncherState launcher_state = LauncherState::IDLE;

//----------- 公开接口 ------------
void Get_3508Position() {
    M3508LeftPosition = LeftLauncher.get_motor_total_position();
    M3508RightPosition = RightLauncher.get_motor_total_position();
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
        launcher_balance_reset();
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

void launcher_set_target(float left_target, float right_target) {
    left_AimPosition = left_target;
    right_AimPosition = right_target;
}

void launcher_update() {
    float correction = launcher_compute_balance();

    const float left_AimSpeed = M3508_SwitchLeftPID_position.update(M3508LeftPosition, left_AimPosition + correction);
    const float left_output = M3508_SwitchLeftPID.update(M_SwitchLeft.feedback.speed, left_AimSpeed);
    M_SwitchLeft.update(left_output);

    const float right_AimSpeed = M3508_SwitchRightPID_position.update(M3508RightPosition, right_AimPosition + correction);
    const float right_output = M3508_SwitchRightPID.update(M_SwitchRight.feedback.speed, right_AimSpeed);
    M_SwitchRight.update(right_output);
}

void launcher_manual_position(float left_position, float right_position) {
    if (launcher_state != LauncherState::IDLE)
        return;

    left_AimPosition += left_position;
    right_AimPosition += right_position;
    launcher_update();
}

void Fire(uint32_t status) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, status);
}

void Launcher_offline_protect() {
    LeftLauncher.motor_offline_protect();
    RightLauncher.motor_offline_protect();
}

void Reset_launcher_state() {
    launcher_state = LauncherState::IDLE;
    LeftLauncher.resetPID();
    RightLauncher.resetPID();
    launcher_balance_reset();
    left_AimPosition = M3508LeftPosition;
    right_AimPosition = M3508RightPosition;
}

void DebugSend() {
    // vofa::send(E_UART_1,
    //     launcher_state,
    //     M_SwitchLeft.output, M_SwitchRight.output,
    //     M_SwitchLeft.feedback.current, M_SwitchRight.feedback.current
    //     );
}

void SetGate(uint32_t status) {
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, status);
}

// ---------- 自动上膛状态机 ----------
void launcher_auto_load() {

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
                LeftLauncher.startTrajectory(11.5);
                RightLauncher.startTrajectory(-11.5);
                launcher_balance_reset();
                launcher_state = LauncherState::LOADING_REVERSE;
            }
            break;
        }

        // ---- 上膛阶段：两电机从零点朝各自上膛方向转动固定距离 ----
        case LauncherState::LOADING_REVERSE: {
            static int loading_stall_count = 0;
            float correction = launcher_compute_balance();

            // 两电机运动方向相反，均施加 +correction 偏置可使高负载侧滞后、低负载侧超前
            LeftLauncher.update(0.005f, correction);
            RightLauncher.update(0.005f, correction);

            // 两电机都到位后完成
            if (LeftLauncher.isArrived() && RightLauncher.isArrived())
            {
                yall_set_target(-1.0f);
                loading_stall_count = 0;
                bsp_buzzer_flash(3000, 0.8f, 200);
                launcher_balance_reset();
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_state = LauncherState::RETURN_TO_ZERO;
            } else if (abs(M_SwitchLeft.output) > 16000 && abs(M_SwitchRight.output) > 16000) {
                loading_stall_count++;
                if (loading_stall_count >= 10) {
                    loading_stall_count = 0;
                    bsp_buzzer_flash(2000, 1.0f, 1000);
                    launcher_balance_reset();
                    LeftLauncher.resetPID();
                    RightLauncher.resetPID();
                    LeftLauncher.startTrajectory(0.0f);
                    RightLauncher.startTrajectory(0.0f);
                    launcher_state = LauncherState::SAFE;
                }
            } else {
                loading_stall_count = 0;
            }
            break;
        }

        // ---- 回零阶段：上膛完成后两电机回到零点待命 ----
        case LauncherState::RETURN_TO_ZERO: {
            LeftLauncher.homeMotor();
            RightLauncher.homeMotor();

            if (LeftLauncher.isHomed() && RightLauncher.isHomed())
            {
                LeftLauncher.resetHome();
                RightLauncher.resetHome();
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_balance_reset();
                bsp_buzzer_flash(1000, 0.8f, 50);      bsp_time_delay(100);
                bsp_buzzer_flash(2000, 0.8f, 50);      bsp_time_delay(100);
                bsp_buzzer_flash(3000, 0.8f, 50);
                launcher_state = LauncherState::IDLE;
            }
            break;
        }
        case LauncherState::SAFE: {
            float correction = launcher_compute_balance();
            LeftLauncher.update(0.005f, correction);
            RightLauncher.update(0.005f, correction);
            if (LeftLauncher.isArrived() && RightLauncher.isArrived()) {
                bsp_buzzer_flash(3000, 0.8f, 200);
                launcher_balance_reset();
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_state = LauncherState::IDLE;
            }
            break;
        }
    }
}
