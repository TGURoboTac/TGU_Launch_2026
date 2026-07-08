//
// Created by guizi on 2026/7/3.
//

#ifndef TROBOT_ARM_H
#define TROBOT_ARM_H

#include <cstdint>

enum class ArmState {
    IDLE,
    HOMING,
    MOVE_TO_REPAIR,
    MOVE_TO_LOAD,
    RETURN_HOME,
    SAFE
};

void arm_init();
void ArmTriggerLoad();
void arm_auto_load();
ArmState arm_get_state();
void Clamp(uint32_t status);
void MoveArm(uint32_t ClampRoll_T2C3, uint32_t ClampPitch_T2C1);
void Get_MotorPosition();
void lifter_manual_position(float lifter_position);
void lifter_manual_speed(float lifter_speed);
void yall_manual_speed(float yall_speed);
void arm_offline_protect();

#endif //TROBOT_ARM_H
