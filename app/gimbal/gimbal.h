//
// Created by guizi on 2026/7/17.
//

#ifndef TROBOT_GIMBAL_H
#define TROBOT_GIMBAL_H
#include "motion.h"

extern Motion Gimbal;
inline bool gimbal_move = false;
inline bool gimbal_home = false;

void gimbal_debug();
void gimbal_init();
void gimbal_manual_speed(float gimbal_speed);
void gimbal_manual_sync();
void gimbal_reset_sync();
void gimbal_manual_position(float gimbal_position);
void Reset_gimbal();
void gimbal_offline_protect();
bool gimbal_SetTarget(float target);
void gimbal_manual_reset_sync();
void gimbal_update(float position);
void gimbal_stop();

#endif //TROBOT_GIMBAL_H
