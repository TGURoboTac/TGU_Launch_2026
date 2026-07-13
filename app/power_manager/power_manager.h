//
// Created by 3545 on 25-11-1.
//

#ifndef POWERCTRL_FORFRAMEWORK_H
#define POWERCTRL_FORFRAMEWORK_H

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#define M_Too_Small_AllErrors 500.0
#define M_Enable_PowerCompensation
#define M_Power_Compensation_Alpha 0.02
#define M_Motor_ReservedPower_Border 54.0
#define M_PerMotor_ReservedPower 8.0

typedef enum {
    E_disabled_negative,E_enable_negative
}E_CalMotorPower_Negative_Status_Type;
typedef enum {
    E_disabled_predict,E_enable_predict,E_enable_not_limit_predict
}E_Predict_Status_Type;

typedef struct{
    float k0,k1,k2,k3,k4,k5;
    float real_current_conversion;
}motor_power_init_t;

class MotorPower{
protected:
    const float K0,K1,K2,K3,K4,K5,Current_Conversion;
public:
    float feedback_power = 0;
    float predict_not_limit_power = 0;
    float predict_power = 0;
    float power_limit = 0;
    explicit MotorPower(const motor_power_init_t &motor_power_init_data);
    [[nodiscard]] float getMotorRealCurrent(float current) const;
    float update(float current, float speed,E_Predict_Status_Type Predict_status = E_disabled_predict, E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);
    float limiter(float *desired_current, float current_speed, float motor_power_limit);
};

class ChassisPowerManager {
protected:
    std::array<MotorPower*, 4> motors_;
    std::array<float, 4> motor_errors_;
public:
    ChassisPowerManager(MotorPower* motor1, MotorPower* motor2, MotorPower* motor3, MotorPower* motor4);
    void updateMotorError(size_t index, float error);
    void allocatePower(float total_power_limit, float buffer_power_attenuation = 1.0);
    [[nodiscard]] float getTotalPredictNotLimitPower() const;
    [[nodiscard]] float getTotalPowerLimit() const;
    [[nodiscard]] float getTotalPredictPower() const;
};

std::vector<float> power_allocation_by_error(std::vector<float>& motor_errors_vector, float total_power_limit,float buffer_power_attenuation = 1.0);
std::vector<float> allocate_SW_power( float total_power, float servo_rate, float servo_predict_want_power);
float rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, float alpha);
void rotate_theta_forwardfeed(float* theta,float rotate,float translation,float kp);

class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t size);
    float update(float new_value);

private:
    std::vector<float> buffer;
    size_t size;
    size_t index;
    size_t count;
    float sum;
};

#endif //POWERCTRL_FORFRAMEWORK_H
