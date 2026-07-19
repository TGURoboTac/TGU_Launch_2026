//
// Created by fish on 2025/9/13.
//

#include "bsp/bsp.h"
#include "bsp/led.h"
#include "bsp/time.h"
#include "bsp/adc.h"

#include "bsp/buzzer.h"
#include "bsp/can.h"
#include "bsp/uart.h"

#include <cstring>
#include <iso646.h>

#include "ins/ins.h"
#include "rc/dr16.h"
#include "rc/ht10.h"
#include "utils/os.h"
#include "utils/vofa.h"

extern void chassis_task(void *args);
extern void launcher_task(void *args);
extern void arm_task(void *args);
extern void gimbal_task(void *args);

extern "C" [[noreturn]] void app_entrance(void *args) {
    bsp_hw_init();

    bsp_led_set(0, 50, 0);
    bsp_buzzer_flash(4500, 0.2f, 100);
    bsp_time_delay(100);
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);

    // Init Basic Components

    // logger::init(E_UART_1, logger::INFO);
    // terminal::init(E_UART_1, 921600);
    // rc::dr16::init(E_UART_5);
    rc::ht10::init(E_UART_5);

    // ins::init();
    // while (!ins::inited) os::task::sleep(5), bsp_iwdg_refresh();

    bsp_buzzer_flash(4500, 0.2f, 75);
    bsp_time_delay(50);
    bsp_buzzer_flash(4500, 0.2f, 75);

    // Init Application Tasks
    os::task::static_create(chassis_task, nullptr, "chassis_task", 1024, os::task::Priority::HIGH);
    os::task::static_create(launcher_task, nullptr, "launcher_task", 1024, os::task::Priority::HIGH);
    os::task::static_create(arm_task, nullptr, "arm_task", 1024, os::task::Priority::MEDIUM);
    os::task::static_create(gimbal_task, nullptr, "gimbal_task", 128, os::task::Priority::MEDIUM);

    int count = 0;
    uint8_t CarTrigCount = 0;
    static float can_current = 0.0f;
    static float can_voltage = 0.0f;
    static bool can_new_data = false;

    bsp_can_set_callback(E_CAN_3, 0x7FF, [](bsp_can_e device, uint32_t id, const uint8_t *data, size_t len) {
        if (len < 8) return;
        memcpy(&can_current, data, 4);
        memcpy(&can_voltage, data + 4, 4);
        can_new_data = true;
    });

    const auto rc_ht10_data = rc::ht10::data();

    int8_t last_swa = rc_ht10_data->swa;
    uint32_t trig_start_time = 0;
    constexpr uint32_t TRIG_TIMEOUT_MS = 800;

    HAL_GPIO_WritePin(BIU_GPIO_Port, BIU_Pin, GPIO_PIN_SET);    //激光笔

    for (;;) {
        if (can_new_data) {
            can_new_data = false;
        //     vofa::send(E_UART_1, static_cast<double>(can_current), static_cast<double>(can_voltage),
        //         static_cast<double>(can_current) * static_cast<double>(can_voltage));
        }

        if (last_swa == 0 && rc_ht10_data->swa == 1) {
            if (CarTrigCount++ == 1) {
                bsp_buzzer_flash(4500, 0.8f, 100);
                CarTrigCount = 0;
                trig_start_time = 0;
                rc::ht10::set_reverse(rc::ht10::REVERSE_RC_R0 | rc::ht10::REVERSE_RC_R1);
            } else {
                trig_start_time = bsp_time_get_ms();
            }
        }
        if (CarTrigCount > 0 && bsp_time_get_ms() - trig_start_time > TRIG_TIMEOUT_MS) {
            CarTrigCount = 0;
            trig_start_time = 0;
        }

        last_swa = rc_ht10_data->swa;

        // vofa::send(E_UART_1, bsp_adc_vbus());
        if (bsp_adc_vbus() < 11.4) {
            if (++count == 10) count = 0, bsp_buzzer_flash(2500, 0.5f, 50);
            bsp_led_set(static_cast<uint8_t>(std::abs(255 * ((static_cast<float>(bsp_time_get_ms() % 600) - 300) / 300.f))), 0, 0);
            bsp_iwdg_refresh();
            os::task::sleep(5);
        } else {
            bsp_led_set_hsv(static_cast<float>(bsp_time_get_ms() % 3000) / 3000.0f, 1.0f, 0.3f);
            bsp_iwdg_refresh();
            os::task::sleep(5);
        }
    }
}
