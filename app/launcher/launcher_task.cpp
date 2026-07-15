//
// Created by guizi on 2026/7/2.
//
#include "bsp/uart.h"
#include "utils/os.h"
#include "launcher.h"
#include "arm.h"
#include "bsp/time.h"
#include "rc/ht10.h"

[[noreturn]] void launcher_task(void *args) {

    // bsp_uart_set_callback(E_UART_1, [](bsp_uart_e, const uint8_t *data, size_t len) {
    //     sscanf((const char *) data, "%f", &debug);
    //     vofa::send(E_UART_1, debug);
    // });

    launcher_init();
    const auto rc_ht10_data = rc::ht10::data();

    // 记录 swa/swb 上一拍状态，用于边沿检测
    int8_t last_swa = rc_ht10_data->swa;
    int8_t last_swb = rc_ht10_data->swb;
    uint8_t LauncherTrigCount = 0;      //检测到两次
    uint16_t trig_timeout = 0;
    constexpr uint16_t TRIG_TIMEOUT_MS = 800;

    for (;;) {
        DebugSend();
        Get_3508Position();
        if (rc_ht10_data->swd == -1) {
            yall_update();
            Fire(1000);
            if (launcher_get_state() == LauncherState::IDLE
                && rc_ht10_data->rc_l[1] * 1.25 + 1500 < 1000)
                Fire(rc_ht10_data->rc_l[1] * 1.25 + 1500);

            // ---- 自动上膛触发：swa 回中按键，检测边沿（0 → -1）----
            if (rc_ht10_data->swb == 1) {
                if (last_swa == 0 && rc_ht10_data->swa == -1 && LauncherTrigCount++ == 1) {
                    LauncherTrigCount = 0;
                    trig_timeout = 0;
                    LauncherTriggerLoad();
                }
                if (rc_ht10_data->swc == -1) {
                    if(yall_set_target(0.363f)) {   //机械臂yall肘挪到发射台上方
                        SetGate(500);
                    }
                } else if (rc_ht10_data->swc == 1) {
                    SetGate(1850);
                }
            }
            last_swa = rc_ht10_data->swa;

            if (LauncherTrigCount > 0 && (rc_ht10_data->swb != 1 || ++trig_timeout > TRIG_TIMEOUT_MS)) {
                LauncherTrigCount = 0;
                trig_timeout = 0;
            }

            // ---- 手动模式：空闲或完成时响应遥控器 ----
            if (rc_ht10_data->swb == -1) {
                if (last_swb != -1)
                    Reset_launcher_state();
                if (rc_ht10_data->swc == -1) {
                    launcher_manual_position(static_cast<float>(rc_ht10_data->rc_l[0]) / 80000.0f, static_cast<float>(rc_ht10_data->rc_r[0]) / 80000.0f);
                } else if (rc_ht10_data->swc == 1) {
                    launcher_manual_position(static_cast<float>(rc_ht10_data->rc_r[0]) / 80000.0f, -(static_cast<float>(rc_ht10_data->rc_r[0]) / 80000.0f));
                }
            } else {
            // ---- 自动上膛状态机：非空闲时持续运行 ----
                launcher_auto_load();
            }
            last_swb = rc_ht10_data->swb;
        } else {
            LauncherTrigCount = 0;
            trig_timeout = 0;
            launcher_emergency_stop();
        }
        Launcher_offline_protect();

        os::task::sleep(1);
    }
}