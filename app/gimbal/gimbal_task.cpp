//
// Created by guizi on 2026/7/19.
//

#include "utils/os.h"
#include "gimbal.h"
#include "launcher.h"
#include "bsp/buzzer.h"
#include "rc/ht10.h"

[[noreturn]] void gimbal_task(void *args) {

    gimbal_init();
    bool buzzer_flag = false;

    // const auto rc_ht10_data = rc::ht10::data();

    for (;;) {
        if (Gimbal.isHomed()) {
            if (!buzzer_flag) {
                buzzer_flag = true;
                bsp_buzzer_flash(1000, 0.8f, 200);
            }
        } else {
            Gimbal.homeMotor();
            SetGate(1850);
        }
        if (gimbal_move) {
            Gimbal.startTrajectory(1.62f);
            Gimbal.update(1.2);
            if (Gimbal.isArrived()) {
                bsp_buzzer_flash(1000, 0.8f, 200);
                SetGate(500);
                gimbal_move = false;
            }
        }else if (gimbal_home) {
            Gimbal.startTrajectory(0.0f);
            Gimbal.update(1.2);
            if (Gimbal.isArrived()) {
                bsp_buzzer_flash(3000, 0.2f, 200);
                SetGate(1850);
                gimbal_home = false;
            }
        }

        os::task::sleep(1);
    }
}