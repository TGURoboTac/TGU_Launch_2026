//
// Created by guizi on 2026/7/2.
//

#ifndef TROBOT_CHASSIS_H
#define TROBOT_CHASSIS_H

void print_speed_chassis();
void chassis_Debug();
void motor_update(double vx_t, double vy_t, double r_t);
void chassis_init();
void chassis_stop();
void chassis_disable();

#endif //TROBOT_CHASSIS_H
