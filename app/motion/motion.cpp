//
// Created by guizi on 2026/7/7.
//

#include "motion.h"
#include <cstdint>

#include "bsp/time.h"
#include "controller/pid.h"
#include "motor/dji.h"
#include "utils/vofa.h"

float Motion::get_motor_total_position() const {
    return static_cast<float>(motor_.feedback.round) * 2.0f * static_cast<float>(M_PI)
           + motor_.feedback.angle;
}

bool Motion::homeMotor()
{
    if (homed_)
        return true;

    // ---- 速度斜坡：指令从 0 逐步逼近 homing_speed_ ----
    // 避免启动瞬间大电流冲击被误判为堵转
    bool ramp_done = true;
    if (homing_ramp_step_ > 0.0f) {
        if (homing_cmd_speed_ < homing_speed_) {
            homing_cmd_speed_ += homing_ramp_step_;
            if (homing_cmd_speed_ > homing_speed_)
                homing_cmd_speed_ = homing_speed_;
        } else if (homing_cmd_speed_ > homing_speed_) {
            homing_cmd_speed_ -= homing_ramp_step_;
            if (homing_cmd_speed_ < homing_speed_)
                homing_cmd_speed_ = homing_speed_;
        }
        ramp_done = homing_cmd_speed_ == homing_speed_;
    } else {
        homing_cmd_speed_ = homing_speed_;
    }

    // 速度环控制
    float current_cmd = speed_pid_.update(motor_.feedback.speed, homing_cmd_speed_);

    motor_.update(current_cmd);

    // 堵转检测（斜坡加速阶段不检测，防止启动电流误判）
    if (ramp_done
        && std::abs(motor_.feedback.current) > stall_current_
        && std::abs(motor_.feedback.speed) < stall_speed_){
        stall_count_++;
    } else {
        stall_count_ = 0;
    }

    // 连续满足堵转
    if (stall_count_ >= stall_count_threshold_)
    {
        stall_count_ = 0;
        homed_ = true;
        homing_cmd_speed_ = 0.0f;
        zero_position_ = get_motor_total_position();
        motor_.update(0);
        speed_pid_.clear();

        return true;
    }
    return false;
}

bool Motion::isHomed() const  //判断回零标志位
{
    return homed_;
}

void Motion::resetHome()    //重置回零标志位
{
    homed_ = false;
    stall_count_ = 0;
    homing_cmd_speed_ = 0.0f;
}

float Motion::getCurrentPosition() const{     //获取当前电机机械位置
    return get_motor_total_position() - zero_position_;
}

void Motion::startTrajectory(float load_target){
    aim_position_ = load_target;
    target_position_ = getCurrentPosition();
    total_travel_ = aim_position_ - target_position_;
}

void Motion::update(float step, float position_offset){
    float current = getCurrentPosition();

    // ---- 目标接近终点时的线性降速 (deceleration ramp) ----
    // 动机：匀速轨迹在终点瞬间停住会导致电机因惯性超调。
    // 当前目标与终点的距离 < 总行程 × decel_ratio_ 时，
    // step 按 remaining/decel_distance 等比缩小，目标平缓减速至接近零。
    float effective_step = step;
    float remaining = aim_position_ - target_position_;
    float decel_distance = decel_ratio_ * std::abs(total_travel_);
    constexpr float min_step_ratio = 0.02f; // 最小 step 比例，防止减到零导致永不到达

    if (decel_distance > 1e-6f) {
        float abs_remaining = std::abs(remaining);
        if (abs_remaining < decel_distance) {
            effective_step = step * (abs_remaining / decel_distance);
            if (effective_step < step * min_step_ratio)
                effective_step = step * min_step_ratio;
        }
    }

    // ---- 匀速 ramp：target 以 step 向 aim 靠拢 ----
    if (target_position_ < aim_position_){
        target_position_ += effective_step;

        if (target_position_ > aim_position_)
            target_position_ = aim_position_;
    } else {
        target_position_ -= effective_step;

        if (target_position_ < aim_position_)
            target_position_ = aim_position_;
    }

    // ---- 级联 PID：位置环 → 速度环 → 电流环 ----
     float effective_target = target_position_ + position_offset;
     float speed_cmd = position_pid_.update(current, effective_target);
     float current_cmd = speed_pid_.update(motor_.feedback.speed, speed_cmd);
     motor_.update(current_cmd);
}

bool Motion::isArrived() const {      //判断电机是否到达目标位置
    return std::abs(aim_position_ - getCurrentPosition()) < position_tolerance_;
}

void Motion::resetPID() const {     //重置 PID
    position_pid_.clear();
    speed_pid_.clear();
}

void Motion::setTarget(float position) const {
    resetPID();
    const float aim_speed = position_pid_.update(getCurrentPosition(), position);
    const float output = speed_pid_.update(motor_.feedback.speed, aim_speed);
    motor_.update(output);
}

void Motion::manual_set_position(float target) {
    manual_target_ = target;
}

void Motion::manual_delta_position(float delta) {
    manual_target_ += delta;
}

void Motion::manual_position_update(float position_offset) const {
    const float current = get_motor_total_position();
    const float effective_target = manual_target_ + position_offset;
    const float speed_cmd = position_pid_.update(current, effective_target); // 位置环 → 速度环
    const float current_cmd = speed_pid_.update(motor_.feedback.speed, speed_cmd);
    if (manual_synced_) {
        motor_.update(current_cmd);
    }
}

void Motion::manual_speed_update(float speed) const {
    const float current_cmd = speed_pid_.update(motor_.feedback.speed, speed); // 速度环 → 电流环
    motor_.update(current_cmd);
}

bool Motion::manual_is_arrived(float tolerance) const {
    return std::abs(manual_target_ - get_motor_total_position()) < tolerance;
}

void Motion::manual_sync_position() {
    if (!manual_synced_ && motor_.feedback.timestamp != 0) {
        manual_target_ = get_motor_total_position(); // 对齐到当前实际位置}
        manual_synced_ = true;
    }
}

void Motion::manual_reset_sync() {
    manual_synced_ = false;
}

void Motion::stop() const {
    speed_pid_.clear();
    motor_.update(0); // 发送零电流指令
}

void Motion::motor_offline_protect() const {
    if (bsp_time_get_ms() - motor_.feedback.timestamp >= 3) {
        speed_pid_.clear();
        motor_.update(0);
    }
}
