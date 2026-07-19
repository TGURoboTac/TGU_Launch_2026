//
// Created by guizi on 2026/7/2.
//

#ifndef TROBOT_LAUNCHER_H
#define TROBOT_LAUNCHER_H
#include "utils/vofa.h"
#include "motion.h"

// 自动上膛状态机
enum class LauncherState {
    IDLE,             // 空闲，等待触发
    HOMING,           // 两电机各自朝限位转动，独立检测堵转并置零
    LOADING_REVERSE,  // 两电机归零后，位置环上膛
    RETURN_TO_ZERO,    // 上膛完成后，两电机回到零点待命
    SAFE //电机堵转，触发安全模式
};

extern Motion LeftLauncher;
extern Motion RightLauncher;
extern uint8_t Gimbal_homing_flag;
extern uint8_t Gimbal_moving_flag;
extern uint8_t Gimbal_moved_flag;
extern bool gimbal_trajectory_init;

void DebugSend();
void SetGate(uint32_t status);
void launcher_init();
void launcher_auto_load();           // 状态机更新，在 task 循环中每周期调用
void LauncherTriggerLoad();        // 触发自动上膛（从 IDLE 或 COMPLETED 状态）
void launcher_manual_speed(float left_speed, float right_speed); // 手动速度控制 (rad/s)
void launcher_manual_position(float left_position, float right_position);
void launcher_update();
LauncherState launcher_get_state();  // 获取当前状态
void Fire(uint32_t status);
void launcher_balance_reset();        // 清除电流均衡 PID 积分
void Launcher_offline_protect();    //离线保护
void launcher_emergency_stop();
void Reset_launcher_state();
void launch_manual_sync();
void launcher_reset_sync();

#endif //TROBOT_LAUNCHER_H