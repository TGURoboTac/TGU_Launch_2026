//
// Created by guizi on 2026/7/17.
//

#include "gimbal.h"

#include "motion.h"
#include "controller/pid.h"
#include  "motor/dji.h"

using namespace controller;

motor::dji M_gimbal("motor_Gimbal", motor::dji::M2006,
                        motor::dji::param_t{.id = 1, .port = E_CAN_1, .mode = motor::dji::CURRENT}, 50);

pid M2006PID(800, 1.5, 0, 3000, 10000); //最大速度27左右
pid M2006PID_position(40, 0, 20, 3000, 10000);

Motion Gimbal {
    M_gimbal,
    M2006PID,
    M2006PID_position,
    0.0,
    0.0,
    0.0,
    0,
    0.0,
    0.0,
    0.0
};

// ----- 公开接口 -----//
void gimbal_init() {
    M_gimbal.init();
}

void gimbal_manual_speed(float gimbal_speed) {
    Gimbal.manual_speed_update(gimbal_speed);
}

void gimbal_manual_sync() {
    Gimbal.manual_sync_position();
}

void gimbal_manual_position(float gimbal_position) {
    Gimbal.manual_delta_position(gimbal_position);
    Gimbal.manual_position_update();
}

void Reset_gimbal() {
    Gimbal.resetPID();
}

void gimbal_offline_protect() {
    Gimbal.motor_offline_protect();
}

bool gimbal_set_position(float target) {
    Gimbal.manual_set_position(target);
    return std::abs(target - Gimbal.getCurrentPosition()) < 0.05;
}

void gimbal_manual_reset_sync() {
    Gimbal.manual_reset_sync();
}