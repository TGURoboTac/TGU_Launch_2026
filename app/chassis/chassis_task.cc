//
// Created by fish on 2026/1/24.
//

#include "utils/os.h"
#include "rc/ht10.h"
#include "chassis.h"
/*
*  麦克纳姆轮
*  ^ vy
*  |       LU              RU
*  |           O ------ O
*  |           |        |
*  |           |        |
*  |           O ------ O
*  |       LD              RD
*  O------------------------------> vx
*
*  定义每个轮子的正速度为 vy 方向的速度，故 RU、RD 需要 reverse 一下
*
*  v_LU =  rotate * sqrt(2) + vy + vx * sqrt(2)
*  v_LD =  rotate * sqrt(2) + vy + vx * sqrt(2)
*  v_RU = -rotate * sqrt(2) + vy - vx * sqrt(2)
*  v_RD = -rotate * sqrt(2) + vy - vx * sqrt(2)
*/

[[noreturn]] void chassis_task(void *args) {
    chassis_init();
    const auto rc_ht10_data = rc::ht10::data();
    for (;;) {
        // chassis_Debug();
        if (rc_ht10_data->swd == 1) {
            if (rc_ht10_data->swc == 1) {
                motor_update(rc_ht10_data->rc_r[0] / 40.0, rc_ht10_data->rc_r[1] / 40.0, rc_ht10_data->rc_l[0] / 40.0);
            } else if (rc_ht10_data->swc == -1) {
                motor_update(rc_ht10_data->rc_r[0] / 400.0, rc_ht10_data->rc_r[1] / 400.0, rc_ht10_data->rc_l[0] / 400.0);
            }
        } else {
            motor_update(0, 0, 0);
        }
        os::task::sleep(1);
    }
}
