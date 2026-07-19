//
// Created by guizi on 2026/7/7.
//

#ifndef TROBOT_MOTION_H
#define TROBOT_MOTION_H
#include <cstdint>
#include "controller/pid.h"
#include "motor/dji.h"
#include "utils/vofa.h"

class Motion
{
public:

    Motion(motor::dji& motor, controller::pid& speed_pid, controller::pid& position_pid,
                  float homing_speed, float stall_speed,
                  float stall_curent,
                  int stall_count_threshold,
                  float position_tolerance,
                  float decel_ratio = 0.0f,
                  float homing_ramp_step = 0.0f)
        : motor_(motor), speed_pid_(speed_pid), position_pid_(position_pid),
          homing_speed_(homing_speed), stall_speed_(stall_speed),
          stall_current_(stall_curent),
          stall_count_threshold_(stall_count_threshold),
          position_tolerance_(position_tolerance),
          decel_ratio_(decel_ratio),
          homing_ramp_step_(homing_ramp_step)
    {
    }
    /*
     *  获取电机位置
     */
    [[nodiscard]] float get_motor_total_position() const;

    /*
     *  回零相关
     */
    bool homeMotor();   //电机回零

    [[nodiscard]] bool isHomed() const; //判断电机是否回零标志位

    void resetHome();   //重置电机回零标志位

    /*
     *  轨迹相关
     */

    [[nodiscard]] float getCurrentPosition() const; //获取电机现在位置

    void startTrajectory(float load_target); //开启一条新轨迹

    float update(float step, float position_offset = 0.0f);    //更新电机位置，position_offset 用于双电机电流均衡的外部位置偏置

    [[nodiscard]] bool isArrived() const;   //判断电机是否达到目标位置标志位

    void resetPID() const;  //重置电机 PID

    void setTarget(float position) const; //固定目标位置 (使用getcurrentPosition坐标系)

    /*
     *  手动控制（使用 motor total position 坐标系）
     */
    void manual_set_position(float target);                       // 设置绝对目标位置

    void manual_delta_position(float delta);                       // 增量调整目标

    void manual_position_update(float position_offset = 0.0f) const; // 位置环 PID 更新，position_offset 用于双电机电流均衡

    void manual_speed_update(float speed) const;                    // 直接速度环控制

    [[nodiscard]] bool manual_is_arrived(float tolerance) const; // 判断是否到达手动目标

    void manual_sync_position();                                 // 同步目标到当前实际位置

    void manual_reset_sync();                                    // 重置手动同步标志，下次 manual_sync_position 时重新对齐

    /*
     *  急停
     */
    void stop() const;

    /*
     *  离线保护
     */
    void motor_offline_protect() const;

    motor::dji& motor_;
    controller::pid& speed_pid_;
    controller::pid& position_pid_;

    float zero_position_ = 0.0f;
    float aim_position_ = 0.0f;
    float target_position_ = 0.0f;
    float homing_speed_ = 0.0f;     //回零点转速
    float stall_speed_ = 0.0f;      // 堵转速度阈值 (rad/s)，低于此速度才判定堵转
    float stall_current_ = 0.0f;     // 堵转电流阈值 (A)
    bool homed_ = false;    //判断是否回零点标志位
    int stall_count_ = 0;      //连续超过阈值次数
    int stall_count_threshold_ = 0;      // 连续超过阈值次数（1ms/次 → 5ms 去抖）
    float position_tolerance_ = 0.0f;
    float total_travel_ = 0.0f;     // 本次轨迹的总行程 (rad)，startTrajectory() 时计算，用于减速距离计算
    float decel_ratio_ = 0.0f;      // 减速段占比 (0~1)，剩余距离 < total_travel_ * decel_ratio_ 时开始线性降速；0 表示不减速
    float homing_ramp_step_ = 0.0f; // 回零速度斜坡步长 (rad/s 每次调用)，0 表示不使用斜坡直接给目标速度
    float homing_cmd_speed_ = 0.0f; // 回零当前速度指令，斜坡从 0 逐步加速到 homing_speed_
    float manual_target_ = 0.0f;     // 手动控制目标位置 (raw motor position)
    bool manual_synced_ = false;
private:

};

void motor_offline_protect(motor::dji* motor, controller::pid* pid);

#endif //TROBOT_MOTION_H
