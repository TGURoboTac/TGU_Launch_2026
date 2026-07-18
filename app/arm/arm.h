//
// Created by guizi on 2026/7/3.
//

#ifndef TROBOT_ARM_H
#define TROBOT_ARM_H

#include <cstdint>
#include "motion.h"

enum class ArmState {
    IDLE,   //空闲
    HOMING, //返回零点
    MOVE_TO_REPAIR, //移动到火种夹取区
    MOVE_TO_LOAD,   //移动到装弹区
    LOADING_IS_OK,  //装弹完成
    RETURN_TO_ZERO,
    SAFE    //触发电机保护
};

extern Motion ArmYALL;

void arm_init();
void ArmTriggerLoad();
void arm_auto_load();
ArmState arm_get_state();
void Clamp(uint32_t status);
void MoveArm(uint32_t ClampRoll_T2C3, uint32_t ClampPitch_T2C1);
void lifter_manual_position(float lifter_position);
void lifter_manual_speed(float lifter_speed);
void yall_manual_speed(float yall_speed);
void yall_manual_position(float yall_position);
bool yall_set_position(float target);
void arm_offline_protect();
void Reset_arm_state();
void arm_manual_sync();
void ArmDebug();

#endif //TROBOT_ARM_H
