//
// Created by guizi on 2026/6/11.
//

#ifndef TROBOT_CHASSIS_POWER_H
#define TROBOT_CHASSIS_POWER_H
#include "power_manager.h"

motor_power_init_t motor_3508_power_data(
    0.65213,
    -0.15659,
    0.00041660,
    0.00235415,
    0.20022,
    1.08e-7,
    1000
);
MotorPower m3508_1_power(motor_3508_power_data);
MotorPower m3508_2_power(motor_3508_power_data);
MotorPower m3508_3_power(motor_3508_power_data);
MotorPower m3508_4_power(motor_3508_power_data);
ChassisPowerManager chassis(
    &m3508_1_power,
    &m3508_2_power,
    &m3508_3_power,
    &m3508_4_power
);
// extern ChassisPowerManager chassis;
#endif //TROBOT_CHASSIS_POWER_H
