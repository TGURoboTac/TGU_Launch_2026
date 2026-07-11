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

    // 速度环控制
    float current_cmd = speed_pid_.update(motor_.feedback.speed, homing_speed_);

    motor_.update(current_cmd);

    // 堵转检测
    if (std::abs(motor_.feedback.current) > stall_current_ && std::abs(motor_.feedback.speed) < stall_speed_){
        stall_count_++;
    } else {
        stall_count_ = 0;
    }

    // 连续满足堵转
    if (stall_count_ >= stall_count_threshold_)
    {
        stall_count_ = 0;
        homed_ = true;
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
}

float Motion::getCurrentPosition() const{     //获取当前电机机械位置
    return get_motor_total_position() - zero_position_;
}

void Motion::startTrajectory(float load_target){     //开启一条新轨迹
    aim_position_ = load_target;
    target_position_ = getCurrentPosition();
}

void Motion::update(float step, float position_offset){
    float current = getCurrentPosition();

    if (target_position_ < aim_position_){
        target_position_ += step;

        if (target_position_ > aim_position_)
            target_position_ = aim_position_;
    } else {
        target_position_ -= step;

        if (target_position_ < aim_position_)
            target_position_ = aim_position_;
    }

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

void Motion::motor_offline_protect() const {
    if (bsp_time_get_ms() - motor_.feedback.timestamp >= 3) {
        speed_pid_.clear();
        motor_.update(0);
    }
}
