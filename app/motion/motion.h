//
// Created by guizi on 2026/7/7.
//

#ifndef TROBOT_MOTION_H
#define TROBOT_MOTION_H
#include <cstdint>
#include "controller/pid.h"
#include "motor/dji.h"
#include "utils/vofa.h"

class Motion
{
public:

    Motion(motor::dji& motor, controller::pid& speed_pid, controller::pid& position_pid,
                  float homing_speed, float stall_speed,
                  float stall_curent,
                  int stall_count_threshold,
                  float position_tolerance)
        : motor_(motor), speed_pid_(speed_pid), position_pid_(position_pid),
          homing_speed_(homing_speed), stall_speed_(stall_speed),
          stall_current_(stall_curent),
          stall_count_threshold_(stall_count_threshold),
          position_tolerance_(position_tolerance)
    {
    }
    /*
     *  获取电机位置
     */
    [[nodiscard]] float get_motor_total_position() const;

    /*
     *  回零相关
     */
    bool homeMotor();   //电机回零

    [[nodiscard]] bool isHomed() const; //判断电机是否回零标志位

    void resetHome();   //重置电机回零标志位

    /*
     *  轨迹相关
     */

    [[nodiscard]] float getCurrentPosition() const; //获取电机现在位置

    void startTrajectory(float load_target); //开启一条新轨迹

    void update(float step);    //更新电机位置

    [[nodiscard]] bool isArrived() const;   //判断电机是否达到目标位置标志位

    void resetPID() const;  //重置电机 PID

    /*
     *  离线保护
     */
    void motor_offline_protect() const;

    motor::dji& motor_;
    controller::pid& speed_pid_;
    controller::pid& position_pid_;

    float zero_position_ = 0.0f;
    float aim_position_ = 0.0f;
    float target_position_ = 0.0f;
    float homing_speed_ = 0.0f;     //回零点转速
    float stall_speed_ = 0.0f;      // 堵转速度阈值 (rad/s)，低于此速度才判定堵转
    float stall_current_ = 0.0f;     // 堵转电流阈值 (A)
    bool homed_ = false;    //判断是否回零点标志位
    int stall_count_ ;      //连续超过阈值次数
    int stall_count_threshold_;      // 连续超过阈值次数（1ms/次 → 5ms 去抖）
    float position_tolerance_ = 0.0f;
private:

};

void motor_offline_protect(motor::dji* motor, controller::pid* pid);

#endif //TROBOT_MOTION_H
