//
// Created by guizi on 2026/7/3.
//
#include <algorithm>
#include "arm.h"
#include "rc/ht10.h"
#include <cstdio>
#include "bsp/time.h"
#include "utils/os.h"
#include "utils/vofa.h"

static float debug;

[[noreturn]] void arm_task(void *args) {

     bsp_uart_set_callback(E_UART_1, [](bsp_uart_e, const uint8_t *data, size_t len) {
         sscanf((const char *) data, "%f", &debug);
     });

    arm_init();
    const auto rc_ht10_data = rc::ht10::data();

    int8_t last_swa = rc_ht10_data->swa;
    uint8_t ArmTrigCount = 0;
    uint16_t trig_timeout = 0;
    constexpr uint16_t TRIG_TIMEOUT_MS = 800;

     for (;;) {
         Get_MotorPosition();
         if (rc_ht10_data->swd == 0) {
             // ---- 遥控器边沿检测 ----
             if (last_swa == 0 && rc_ht10_data->swa != 0 && rc_ht10_data->swb == 1 && ArmTrigCount++ == 1) {
                 ArmTrigCount = 0;
                 trig_timeout = 0;
                 ArmTriggerLoad();
             }
             last_swa = rc_ht10_data->swa;

             if (ArmTrigCount > 0 && (rc_ht10_data->swb != 1 || ++trig_timeout > TRIG_TIMEOUT_MS)) {
                 ArmTrigCount = 0;
                 trig_timeout = 0;
             }

             // ---- 自动装弹状态机：非空闲时持续运行 ----
             arm_auto_load();

             // ---- 手动模式：空闲时响应遥控器 ----
             auto state = arm_get_state();
             if (state == ArmState::IDLE && rc_ht10_data->swb == -1) {
                 lifter_manual_speed(static_cast<float>(rc_ht10_data->rc_l[1]) / 1.f);
                 yall_manual_speed(static_cast<float>(rc_ht10_data->rc_r[0]) / 20.0f);
             }
         } else {
             ArmTrigCount = 0;
             trig_timeout = 0;
             lifter_manual_speed(0.0f);
             yall_manual_speed(0.0f);
         }
         arm_offline_protect();
         os::task::sleep(1);
     }
}