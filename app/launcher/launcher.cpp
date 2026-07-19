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
#include "gimbal.h"

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
    -5.0f,
    0.5f,
    4.0f,
    50,
    0.6f,
    0.6f,
    0.09f);

Motion RightLauncher(
    M_SwitchRight,
    M3508_SwitchRightPID,
    M3508_SwitchRightPID_position,
    5.0f,
    0.5f,
    4.0f,
    50,
    0.6f,
    0.6f,
    0.09f);

// ---------- 发射机构电机位置 ----------
static float launcher_compute_balance() {
    float current_error = M_SwitchLeft.feedback.current + M_SwitchRight.feedback.current;
    return current_balance_pid.update(current_error, 0.0f);
}

// ---------- 状态机变量 ----------
static LauncherState launcher_state = LauncherState::IDLE;

//----------- 公开接口 ------------
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
        // LeftLauncher.manual_reset_sync();
        // RightLauncher.manual_reset_sync();
        // gimbal_manual_reset_sync();
    }
}

LauncherState launcher_get_state() {
    return launcher_state;
}

void launcher_manual_speed(float left_speed, float right_speed) {
    if (launcher_state != LauncherState::IDLE)
        return;
    LeftLauncher.manual_speed_update(left_speed);
    RightLauncher.manual_speed_update(right_speed);
}

void launcher_update() {
    const float correction = launcher_compute_balance();
    LeftLauncher.manual_position_update(correction);
    RightLauncher.manual_position_update(correction);
}

void launcher_manual_position(float left_position, float right_position) {
    if (launcher_state != LauncherState::IDLE)
        return;
    LeftLauncher.manual_delta_position(left_position);
    RightLauncher.manual_delta_position(right_position);
    launcher_update();
}

void Fire(uint32_t status) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, status);
}

void Launcher_offline_protect() {
    LeftLauncher.motor_offline_protect();
    RightLauncher.motor_offline_protect();
}

void launcher_emergency_stop() {
    LeftLauncher.stop();
    RightLauncher.stop();
}

void Reset_launcher_state() {
    launcher_state = LauncherState::IDLE;
    LeftLauncher.resetPID();
    RightLauncher.resetPID();
    launcher_balance_reset();
}

void launch_manual_sync() {
    LeftLauncher.manual_sync_position();
    RightLauncher.manual_sync_position();
}

void launcher_reset_sync() {
    LeftLauncher.manual_reset_sync();
    RightLauncher.manual_reset_sync();
}

void DebugSend() {
    // vofa::send(E_UART_1,
    //     launcher_state,
    //     RightLauncher.getCurrentPosition(), LeftLauncher.getCurrentPosition(),
    //     -(RightLauncher.aim_position_ - RightLauncher.target_position_), LeftLauncher.aim_position_ - LeftLauncher.target_position_
    //     );
}

void SetGate(uint32_t status) {
    // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, status);
}

// ---------- 自动上膛状态机 ----------
uint8_t homing_counter = 0;
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
                if (homing_counter == 0) {
                    bsp_buzzer_flash(1000, 0.8f, 200);
                }
                LeftLauncher.resetHome();
                RightLauncher.resetHome();
                // 归零阶段结束，清零 PID 准备位置环
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                //设定此时位置零点
                LeftLauncher.startTrajectory(11.8);
                RightLauncher.startTrajectory(-11.8);
                launcher_balance_reset();
                if (homing_counter == 1) {
                    homing_counter = 0;
                    bsp_buzzer_flash(1000, 0.8f, 50);      bsp_time_delay(100);
                    bsp_buzzer_flash(2000, 0.8f, 50);      bsp_time_delay(100);
                    bsp_buzzer_flash(3000, 0.8f, 50);
                    launcher_state = LauncherState::IDLE;
                } else if (homing_counter == 0) {
                    launcher_state = LauncherState::LOADING_REVERSE;
                }
            }
            break;
        }

        // ---- 上膛阶段：两电机从零点朝各自上膛方向转动固定距离 ----
        case LauncherState::LOADING_REVERSE: {
            static int loading_stall_count = 0;
            float correction = launcher_compute_balance();

            // 两电机运动方向相反，均施加 +correction 偏置可使高负载侧滞后、低负载侧超前
            LeftLauncher.update(0.015f, correction);
            RightLauncher.update(0.015f, correction);

            // 两电机都到位后完成
            if (LeftLauncher.isArrived() && RightLauncher.isArrived())
            {
                gimbal_home = true;
                loading_stall_count = 0;
                launcher_balance_reset();
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                LeftLauncher.startTrajectory(1.0);
                RightLauncher.startTrajectory(-1.0);
                bsp_buzzer_flash(3000, 0.8f, 200);
                launcher_state = LauncherState::RETURN_TO_ZERO;
            } else if (abs(M_SwitchLeft.output) > 16500 && abs(M_SwitchRight.output) > 16500) {     //堵转无法到达目标位置
                loading_stall_count++;
                if (loading_stall_count >= 10) {
                    loading_stall_count = 0;
                    bsp_buzzer_flash(8000, 1.0f, 2000);
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
            float correction = launcher_compute_balance();

            LeftLauncher.update(0.03f, correction);
            RightLauncher.update(0.03f, correction);

            if (LeftLauncher.isArrived() && RightLauncher.isArrived())
            {
                homing_counter += 1;
                LeftLauncher.resetPID();
                RightLauncher.resetPID();
                launcher_balance_reset();
                // bsp_buzzer_flash(5000, 0.3f, 200);
                launcher_state = LauncherState::HOMING;
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
