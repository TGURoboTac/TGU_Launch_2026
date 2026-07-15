//
// Created by guizi on 2026/7/2.
//

#include "chassis.h"
#include "controller/pid.h"
#include "motor/dji.h"
#include "chassis_power.h"
#include <algorithm>
#include "utils/os.h"
#include "bsp/bsp.h"
#include "chassis_power.h"

#include "utils/vofa.h"
using namespace controller;
pid M3508_ChassisPID_LU(1000, 0.3, 0, 5000, 16384);     //最大速度25左右
pid M3508_ChassisPID_RU(1000, 0.3, 0, 5000, 16384);
pid M3508_ChassisPID_RD(1000, 0.3, 0, 5000, 16384);
pid M3508_ChassisPID_LD(1000, 0.3, 0, 5000, 16384);

motor::dji M_LU("motor_LU", motor::dji::M3508,
                motor::dji::param_t{.id = 1, .port = E_CAN_2, .mode = motor::dji::CURRENT});
motor::dji M_RU("motor_RU", motor::dji::M3508,
                motor::dji::param_t{.id = 4, .port = E_CAN_2, .mode = motor::dji::CURRENT});
motor::dji M_RD("motor_RD", motor::dji::M3508,
                motor::dji::param_t{.id = 3, .port = E_CAN_2, .mode = motor::dji::CURRENT});
motor::dji M_LD("motor_LD", motor::dji::M3508,
                motor::dji::param_t{.id = 2, .port = E_CAN_2, .mode = motor::dji::CURRENT});

static constexpr double wheel_speed_limit = 25.0;

void chassis_Debug() {
    M_LU.update(M3508_ChassisPID_LU.update(M_LU.feedback.speed, 5));
    // vofa::send(E_UART_1, M_LU.output, M_LU.feedback.speed);
}
void print_speed_chassis() {
    // vofa::send(E_UART_1, M_LU.feedback.speed, M_RU.feedback.speed, M_RD.feedback.speed, M_LD.feedback.speed);
    // vofa::send(E_UART_1, M_LU.feedback.timestamp, M_LD.feedback.timestamp, M_RD.feedback.timestamp, M_RU.feedback.timestamp);
}
// void motor_update(double vx_t, double vy_t, double r_t) {
//     //底盘解算
//     double w1 = r_t + vy_t * M_SQRT2 + vx_t * M_SQRT2;
//     double w2 = r_t - vy_t * M_SQRT2 + vx_t * M_SQRT2;
//     double w3 = r_t - vy_t * M_SQRT2 - vx_t * M_SQRT2;
//     double w4 = r_t + vy_t * M_SQRT2 - vx_t * M_SQRT2;
//
//     w1 = std::clamp(w1, -wheel_speed_limit, wheel_speed_limit);
//     w2 = std::clamp(w2, -wheel_speed_limit, wheel_speed_limit);
//     w3 = std::clamp(w3, -wheel_speed_limit, wheel_speed_limit);
//     w4 = std::clamp(w4, -wheel_speed_limit, wheel_speed_limit);
//
//     //PID
//     float lu_output = M3508_ChassisPID_LU.update(M_LU.feedback.speed, w1);
//     float ru_output = M3508_ChassisPID_RU.update(M_RU.feedback.speed, w2);
//     float rd_output = M3508_ChassisPID_RD.update(M_RD.feedback.speed, w3);
//     float ld_output = M3508_ChassisPID_LD.update(M_LD.feedback.speed, w4);
//
//     //速度输出
//     M_LU.update(lu_output);
//     M_RU.update(ru_output);
//     M_RD.update(rd_output);
//     M_LD.update(ld_output);
// }

void motor_update(double vx_t, double vy_t, double r_t) {
    //底盘解算
    double w1 = r_t + vy_t * M_SQRT2 + vx_t * M_SQRT2;
    double w2 = r_t - vy_t * M_SQRT2 + vx_t * M_SQRT2;
    double w3 = r_t - vy_t * M_SQRT2 - vx_t * M_SQRT2;
    double w4 = r_t + vy_t * M_SQRT2 - vx_t * M_SQRT2;

    w1 = std::clamp(w1, -wheel_speed_limit, wheel_speed_limit);
    w2 = std::clamp(w2, -wheel_speed_limit, wheel_speed_limit);
    w3 = std::clamp(w3, -wheel_speed_limit, wheel_speed_limit);
    w4 = std::clamp(w4, -wheel_speed_limit, wheel_speed_limit);

    //PID
    float lu_output = M3508_ChassisPID_LU.update(M_LU.feedback.speed, w1);
    float ru_output = M3508_ChassisPID_RU.update(M_RU.feedback.speed, w2);
    float rd_output = M3508_ChassisPID_RD.update(M_RD.feedback.speed, w3);
    float ld_output = M3508_ChassisPID_LD.update(M_LD.feedback.speed, w4);
    //功率限制
    chassis.updateMotorError(0, 1 * (w1 - static_cast<float>(M_LU.feedback.speed))); // 控制循环中更新每个电机的 error
    chassis.updateMotorError(1, 1 * (w2 - static_cast<float>(M_RU.feedback.speed))); // 这里的 error 为 PID 目标速度与当前反馈速度的差值
    chassis.updateMotorError(2, 1 * (w3 - static_cast<float>(M_RD.feedback.speed)));
    chassis.updateMotorError(3, 1 * (w4 - static_cast<float>(M_LD.feedback.speed)));

    chassis.allocatePower(1000); // target 填入希望限制的底盘总功率

    m3508_1_power.limiter(&lu_output, M_LU.feedback.raw.speed, m3508_1_power.power_limit);      // 在 PID 输出已经算出、但还未发送给电调之前调用 limiter()
    m3508_2_power.limiter(&ru_output, M_RU.feedback.raw.speed, m3508_2_power.power_limit);
    m3508_3_power.limiter(&rd_output, M_RD.feedback.raw.speed, m3508_3_power.power_limit);
    m3508_4_power.limiter(&ld_output, M_LD.feedback.raw.speed, m3508_4_power.power_limit);

    //速度输出
    M_LU.update(lu_output);
    M_RU.update(ru_output);
    M_RD.update(rd_output);
    M_LD.update(ld_output);
}

void chassis_init() {
    M_LU.init();
    M_RU.init();
    M_RD.init();
    M_LD.init();
}
void chassis_stop() {
    M_LU.update(0), M_RU.update(0), M_RD.update(0), M_LD.update(0);
}
void chassis_disable() {
    M_LU.disable(), M_RU.disable(), M_RD.disable(), M_LD.disable();
}
void chassis_pid_clear() {
    M3508_ChassisPID_LD.clear(), M3508_ChassisPID_LU.clear(), M3508_ChassisPID_RD.clear(), M3508_ChassisPID_RU.clear();
}