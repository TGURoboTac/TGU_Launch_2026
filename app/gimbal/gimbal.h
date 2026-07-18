//
// Created by guizi on 2026/7/17.
//

#ifndef TROBOT_GIMBAL_H
#define TROBOT_GIMBAL_H

void gimbal_init();
void gimbal_manual_speed(float gimbal_speed);
void gimbal_manual_sync();
void gimbal_manual_position(float gimbal_position);
void Reset_gimbal();
void gimbal_offline_protect();
bool gimbal_set_position(float target);
void gimbal_manual_reset_sync();

#endif //TROBOT_GIMBAL_H
